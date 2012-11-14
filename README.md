# CounterRTC

Arduino library to implement a real-time clock using the asynchronous
timer. Either a crystal or an external clock signal can be used. The
library counts seconds and fractions of a second (in units of
1/32768th of second). The epoch is determined by the user. No facility
is provided for converting between calendar time and elapsed seconds,
use the static functions in
[RTCx](https://github.com/stevemarple/RTCx) to do this.




