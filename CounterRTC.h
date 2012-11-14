#ifndef COUNTERRTC_H
#define COUNTERRTC_H

#include <Arduino.h>

#include <stdint.h>
#include <stdlib.h>

class CounterRTC;
extern CounterRTC cRTC;

class CounterRTC {
public:
  typedef int32_t time_t;

  class Time {
  public:
    Time(void);
    Time(time_t sec, uint16_t frac);
    
    inline time_t getSeconds(void) const {
      return seconds;
    }
    inline uint16_t getFraction(void) const {
      return fraction;
    }
    
    inline void setSeconds(time_t sec) {
      seconds = sec;
    }

    inline void setFraction(uint16_t frac) {
      fraction = frac;
    }

    bool operator==(const Time& rhs) const;
    bool operator<(const Time& rhs) const;

  private:
    time_t seconds;
    uint16_t fraction;
  };

  static const uint8_t numAlarms = 2;
  // fractionsPerSecond must be 2^n; some algebraic operations assume
  // this is the case and are implemented using bitwise AND and shift
  // for speed.
  static const uint16_t fractionsPerSecond = 32768;
  static const int8_t fractionsPerSecondLog2 = 15;

  //static inline int8_t bitWidth(uint16_t a);
  //static inline int8_t log2(uint16_t a);

  template <typename T>
  static inline int8_t bitWidth(T a);

  template <typename T>
  static inline int8_t log2(T a);

  CounterRTC(void);
  // freq must be 2^n (where n=0..15); some algebraic operations
  // assume this is the case and are implemented using bitwise AND and
  // shift for speed.
  bool begin(uint16_t freq, bool extClock, uint8_t prescaler = 1);
  void getTime(Time &t) const;
  void setTime(const Time &t);
  void setTime(const Time &t, Time &oldTime);
  
  bool isAlarmActive(uint8_t alarmNum) const;
  bool isAlarmExpired(uint8_t alarmNum) const;
  bool getAlarm(uint8_t alarmNum, Time &t,
		void(**callback)(uint8_t alarmNum, bool late,
				 const void *context) = NULL,
		const void **cont = NULL) const;
  bool setAlarm(uint8_t alarmNum, const Time &t,
		void(*callback)(uint8_t alarmNum, bool late,
				const void *context),
		const void *cont);
  void clearAlarm(uint8_t alarmNum);
  void runAlarm(uint8_t alarmNum, bool late);
  
  // TODO:
  
  // Have a compare operator set in Time struct to make time
  // comparisons easy. Refactor as subclass.

  // When setting the time check if any timers are expired, if so run
  // callbacks (earliest timer first if both expired).

  //inline void timerOverflowISR(void);
  // inline void alarm0ISR(void);
  // inline void alarm1ISR(void);
  
  
private:
  // alarmBlockTime holds the time used for comparison inside the ISR
  // to determine when to set up the match on TCNT2.
  // Time alarmTime[numAlarms];
  // Time alarmBlockTime[numAlarms];
  // uint8_t alarmCounter[numAlarms];
  
  uint16_t frequency;
  int8_t frequencyLog2;
  bool externalClock;

};

inline bool operator!=(const CounterRTC::Time& lhs,
		       const CounterRTC::Time& rhs)
{
  return !(lhs == rhs);
} 

inline bool operator>(const CounterRTC::Time& lhs,
		      const CounterRTC::Time& rhs)
{
  return (rhs < lhs);
} 

inline bool operator<=(const CounterRTC::Time& lhs,
		       const CounterRTC::Time& rhs)
{
  return !(lhs > rhs);
} 

inline bool operator>=(const CounterRTC::Time& lhs,
		       const CounterRTC::Time& rhs)
{
  return !(lhs < rhs);
}

template <typename T>
int8_t CounterRTC::bitWidth(T a)
{
  uint8_t width = 0;
  while (a) {
    a >>= 1;
    ++width;
  }
  return width;
}

template <typename T>
int8_t CounterRTC::log2(T a)
{
  return bitWidth(a) - (T)1;
}

#endif
