// Rollover-safe millisecond timebase.
//
// millis() wraps every ~49.7 days.  A plain "deadline < now" comparison
// fails across the wrap and can leave a solenoid stuck open until the
// clock catches up.  All timing in this firmware uses the signed-difference
// idiom below; raw </> comparisons of timestamps are forbidden.
#pragma once
#include <stdint.h>

using TimeMs = uint32_t;

// True once timestamp `deadline` has been reached (wrap-safe).
inline bool timeReached(TimeMs deadline, TimeMs now) {
  return (int32_t)(now - deadline) >= 0;
}

// Milliseconds elapsed since `since` (wrap-safe for spans < ~24.8 days).
inline uint32_t elapsedMs(TimeMs since, TimeMs now) {
  return now - since;
}
