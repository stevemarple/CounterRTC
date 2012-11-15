#include <util/atomic.h>

#include "CounterRTC.h"


CounterRTC cRTC;

// volatile CounterRTC::Time time;
volatile CounterRTC::time_t seconds = 0;
volatile uint16_t fraction = 0;

// Number of 1/32768 fractions for every clock tick
uint16_t fractionsPerTick;
int8_t fractionsPerTickLog2;
// Number of 1/32768 fractions when timer overflow occurs
uint32_t overflowFractions;
int8_t overflowFractionsLog2;
CounterRTC::Time overflowInterval;

CounterRTC::Time alarmTime[CounterRTC::numAlarms];
CounterRTC::Time alarmBlockTime[CounterRTC::numAlarms];
uint8_t alarmCounter[CounterRTC::numAlarms];

void (*callback[CounterRTC::numAlarms])(uint8_t alarmNum, bool late,
					const void *context);
bool alarmActive[CounterRTC::numAlarms];
const void* context[CounterRTC::numAlarms];


static void calcAlarmParams(const CounterRTC::Time &t,
			    CounterRTC::Time &blockStart,
			    uint8_t &counter)
{
  uint32_t tmp;
  if (overflowInterval.getSeconds()) {
    blockStart.setFraction(0);
    tmp = t.getSeconds() % overflowInterval.getSeconds();
    blockStart.setSeconds(t.getSeconds() - tmp);

    // Convert the remainder seconds into fractionsPerSecond and add
    // the fraction. Divide by fractionsPerTick and take the LSB to
    // get the counter value for the desired alarm time.
    tmp <<= CounterRTC::fractionsPerSecondLog2; // Convert to fractions per second
    tmp |= (uint32_t)t.getFraction(); // Add the fraction part

  }
  else {
    blockStart.setSeconds(t.getSeconds());
    tmp = t.getFraction() % overflowInterval.getFraction();
    blockStart.setFraction(t.getFraction() - tmp);
  }

  tmp >>= fractionsPerTickLog2; // Work in clock ticks
  counter = tmp; // LSB is the counter value
}
  

// TIMER2_OVF_vect has lower priority than TIMER2_COMPA_vect or
// TIMER2_COMPB_vect
ISR(TIMER2_OVF_vect)
{
  uint32_t tmp = fraction + overflowFractions;
  fraction = tmp & (CounterRTC::fractionsPerSecond - 1);
  seconds += (tmp >> CounterRTC::fractionsPerSecondLog2);


  
  CounterRTC::Time now(seconds, fraction);
  // Schedule future alarms. Do this before running any callbacks.
  if (alarmActive[0] && now == alarmBlockTime[0] && alarmCounter[0]) {
    while ((ASSR & (1 << OCR2AUB)) != 0)
      ; // Wait
    OCR2A = alarmCounter[0];
    TIMSK2 |= (1 << OCIE2A);
  }
  if (alarmActive[1] && now == alarmBlockTime[1] && alarmCounter[1]) {
    while ((ASSR & (1 << OCR2BUB)) != 0)
      ; // Wait
    OCR2B = alarmCounter[1];
    TIMSK2 |= (1 << OCIE2B);
  }
     
  // Check for active alarms that must be run now.
  for (uint8_t i = 0; i < CounterRTC::numAlarms; ++i)
    if (alarmActive[i] && now == alarmBlockTime[i] && alarmCounter[i] == 0)
      cRTC.runAlarm(i, false);
    
  
  // Check for active alarms which have expired. They might have been
  // called in the near future but without sufficient time for setting
  // the alarm.
  for (uint8_t i = 0; i < CounterRTC::numAlarms; ++i)
    if (alarmActive[i] && now > alarmBlockTime[i])
      cRTC.runAlarm(i, true);
}

// TIMER2_COMPA_vect has higher priority than either TIMER2_COMPB_vect
// or TIMER2_OVF_vect
ISR(TIMER2_COMPA_vect)
{
  cRTC.runAlarm(0, false);
}


ISR(TIMER2_COMPB_vect)
{
  cRTC.runAlarm(1, false);
}


CounterRTC::CounterRTC(void) 
{
  for (uint8_t i = 0; i < numAlarms; ++i) {
    alarmActive[i] = false;
    context[i] = NULL;
    callback[i] = NULL;
  }
}


/* Initialise the real-time clock and begin counting.
 *
 */
bool CounterRTC::begin(uint16_t freq, bool extClock, uint8_t prescaler)
{
  frequency = freq;
  frequencyLog2 = log2(frequency);
  if (frequencyLog2 < 0)
    return false;

  fractionsPerTick = fractionsPerSecond / frequency;
  fractionsPerTickLog2 = CounterRTC::log2(fractionsPerTick);
  overflowFractions = 256 * (time_t)fractionsPerTick;
  overflowFractionsLog2 = log2(overflowFractions);
  overflowInterval
    = Time(overflowFractions >> CounterRTC::fractionsPerSecondLog2,
		  overflowFractions & (CounterRTC::fractionsPerSecond - 1));
    
  TIMSK2 = 0; // Disable timer-related interrupts before setting callback
  externalClock = extClock;
  for (uint8_t i = 0; i < numAlarms; ++i) {
    alarmActive[i] = false;
    context[i] = NULL;
    callback[i] = NULL;
  }

  // If the prescaler is off then waiting for the timer update will not
  // happen since it needs an edge. Force to 1.
  if (prescaler == 0)
    prescaler = _BV(CS20);
  
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    // Time must always be modified with interrupts disabled
    seconds = 0;
    fraction = 0;

    // Set up timer2

    // Disable asynchronous operation before enabling the clock
    ASSR = 0;
    // ASSR &= ~(1 << AS2);

    if (extClock)
      // Must be done before asynchronous operation enabled
      ASSR |= (1 << EXCLK);

    // Enable asynchronous operaiton
    ASSR |= (1 << AS2);
    
    TCCR2A = 0;
    TCCR2B = 0;
    TCNT2 = 0; // Clear any count
  
    OCR2A = 0;
    OCR2B = 0;

    // A and B compare modes are normal operation. Waveform generation
    // mode set to normal.
    TCCR2A = 0;

    // Don't force output compare, waveform generation mode normal,
    // use user-defined prescalar values.
    TCCR2B = (_BV(CS22) | _BV(CS21) | _BV(CS20)) & prescaler;

    while((ASSR & ((1 << TCN2UB) | (1 << OCR2AUB) | (1 << OCR2BUB) | (1 << TCR2AUB) | (1 << TCR2BUB))) != 0)
      ; // Wait for changes to latch

    TIFR2 = 0;
    // No alarms, interrupt only on overflow.
    TIMSK2 = (1 << TOIE2);
  }
  return true;
}


void CounterRTC::getTime(Time &t) const
{
  uint8_t count;
  CounterRTC::time_t s;
  uint16_t f;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    count = TCNT2;
    s = seconds;
    f = fraction;
  }
  uint32_t tmp = f + (count * fractionsPerTick);
  t.setSeconds(s + (tmp >> CounterRTC::fractionsPerSecondLog2)); // divide
  t.setFraction(tmp & (CounterRTC::fractionsPerSecond - 1)); // mod
}

void CounterRTC::setTime(const Time &t)
{
  // To make implementing alarms more efficient the stored seconds and
  // fraction values must be an exact multiple of
  // overflowInterval. Also the time that is set must be a multiple of
  // fractionsPerTick.

  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    uint32_t tmpFraction;
    if (overflowInterval.getSeconds()) {
      time_t mask = overflowInterval.getSeconds() - 1;

       // round down to overflowInterval.seconds
      seconds = (t.getSeconds() & ~mask);
      // Add the remainder of (t.seconds / overflowInterval.seconds) to the
      // fractions
      tmpFraction = (((t.getSeconds() & mask) << fractionsPerSecondLog2)
		     | t.getFraction()); // in 32768ths of second
    }
    else {
      seconds = t.getSeconds();
      tmpFraction = t.getFraction();
    }
    
    // Work in units of clock ticks
    tmpFraction >>= fractionsPerTickLog2;
    
    // The counter must now take the lower 8 bits and the upper 3
    // bytes goes into fraction
    fraction = (tmpFraction & 0xFFF0) << fractionsPerTickLog2;

    while (ASSR & _BV(TCN2UB))
      ; // Wait until TCNT2 can be updated.

    TCNT2 = (uint8_t)tmpFraction; // Take lowest byte only
    GTCCR |= _BV(PSRASY); // Reset prescaler
  }
}

void CounterRTC::setTime(const Time &t, Time &oldTime)
{
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    oldTime.setSeconds(seconds);
    oldTime.setFraction(fraction);
    seconds = t.getSeconds();
    fraction = t.getFraction();

    while (ASSR & TCN2UB)
      ; // Wait
    TCNT2 = 0;
  }
}

bool CounterRTC::isAlarmActive(uint8_t alarmNum) const
{
  bool r;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    r = alarmActive[alarmNum];
  }
  return r;
}

bool CounterRTC::isAlarmExpired(uint8_t alarmNum) const
{
  bool r;
  Time t;
  getTime(t);
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    r = (alarmActive[alarmNum] && t >= alarmTime[alarmNum]);
  }
  return r;
}

bool CounterRTC::getAlarm(uint8_t alarmNum, Time &t,
		       void(**cb)(uint8_t alarmNum, bool late,
				  const void *context),
		       const void **cont) const
{
  bool active;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    t = alarmTime[alarmNum];
    if (cb)
      *cb = callback[alarmNum];
    if (cont)
      *cont = context[alarmNum];
    active = alarmActive[alarmNum]; 
  }
  return active;
}

bool CounterRTC::setAlarm(uint8_t alarmNum, const Time &t,
		       void(*cb)(uint8_t alarmNum, bool late,
				 const void *context),
		       const void *cont)
{
  if (alarmNum > numAlarms)
    return false;
  
  if (cb == NULL) {
    alarmActive[alarmNum] = false;
    return false;
  }

  alarmTime[alarmNum] = t;
  calcAlarmParams(alarmTime[alarmNum], alarmBlockTime[alarmNum],
		  alarmCounter[alarmNum]);
  callback[alarmNum] = cb;
  context[alarmNum] = cont;
  alarmActive[alarmNum] = true;  

  // Fetch the time, see if the alarm is expired or whether it should
  // be scheduled now
  Time now;
  getTime(now);
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    Time counterZero(seconds, fraction); // Time when TCNT2 was zero
    if (alarmBlockTime[alarmNum] == counterZero
	&& alarmCounter[alarmNum] > TCNT2) {
      // In the future but due this cycle, set interrupt.
      if (alarmNum == 0) {
	while (ASSR & _BV(OCR2AUB))
	  ;
	OCR2A = alarmCounter[0];
	TIMSK2 |= (1 << OCIE2A);
      }
      else {
	while (ASSR & _BV(OCR2BUB))
	  ;
	OCR2B = alarmCounter[1];
	TIMSK2 |= (1 << OCIE2B);
      }

      // After setting the interrupt check if alarm is in the past. If
      // so and it is still active must have just missed the
      // time. Call manually
      getTime(now);
      if (t > now || !alarmActive[alarmNum])
	// Still in future, or has run
	return true;
      // Otherwise fall-through and run the alarm from here.
    }
    else if (t > now) {
      // Not expired
      return true;
    }

    // Must have expired, run, but with interrupts enabled if they
    // already were
  }

  runAlarm(alarmNum, true);
  return true;
}


void CounterRTC::clearAlarm(uint8_t alarmNum)
{
  if (alarmNum > numAlarms)
    return;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    // Remove the interrupt
    if (alarmNum == 0)
      TIMSK2 &= ~(1 << OCIE2A);
    else
      TIMSK2 &= ~(1 << OCIE2B);
    alarmActive[alarmNum] = false;
    alarmTime[alarmNum] = alarmBlockTime[alarmNum] = Time();
    callback[alarmNum] = NULL;
    context[alarmNum] = NULL;
  }
}

void CounterRTC::runAlarm(uint8_t alarmNum, bool late) {
  // The decision on whether to run, and if so marking the alarm as
  // done, should be atomic to prevent the possibility of it running
  // twice.
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    // Remove the interrupt
    if (alarmNum == 0)
      TIMSK2 &= ~(1 << OCIE2A);
    else
      TIMSK2 &= ~(1 << OCIE2B);
    if (!alarmActive[alarmNum])
      return;
    alarmActive[alarmNum] = false; // Ensure the timer won't run it
  }
  NONATOMIC_BLOCK(NONATOMIC_RESTORESTATE) {
    (*(callback[alarmNum]))(alarmNum, late, context[alarmNum]);
  }
}

CounterRTC::Time::Time(void) : seconds(0), fraction(0)
{
  ;
}

CounterRTC::Time::Time(const Time &t) : seconds(t.seconds), fraction(t.fraction)
{
  ;
}

CounterRTC::Time::Time(time_t sec, uint16_t frac)
  : seconds(sec), fraction(frac)
{
  ;
}

bool CounterRTC::Time::operator==(const Time& rhs) const
{
  return (seconds == rhs.getSeconds() && fraction == rhs.getFraction());
}

bool CounterRTC::Time::operator<(const Time& rhs) const
{
  return (seconds < rhs.getSeconds() ||
	  (seconds == rhs.getSeconds() && fraction < rhs.getFraction()));
}

