// Per-channel physical duty limiting — the hard backstop that holds no
// matter what any decoder or the sequencer asks for:
//   * MAX_OPEN_MS   — absolute cap on continuous open time
//   * MIN_CLOSE_MS  — forced close between opens
//   * duty budget   — at most MAX_OPEN_PER_WINDOW_MS of open time in any
//                     sliding DUTY_WINDOW_MS window (bucketed, conservative)
//
// Owned by the SafetySupervisor; evaluate() must be called every loop with
// the channel's desired state so open time is accounted continuously.
#pragma once
#include "config.h"
#include "timebase.h"

class DutyLimiter {
 public:
  // Returns whether the channel may be open right now, updating accounting
  // and open/close state.  `want` is the requested state this instant.
  bool evaluate(bool want, TimeMs now);

  bool isOpen() const { return open_; }
  uint32_t windowOpenMs() const;

 private:
  void resetAt(TimeMs now);
  void rotateBuckets(TimeMs now);

  bool inited_ = false;
  bool open_ = false;
  TimeMs opened_at_ = 0;
  TimeMs closed_at_ = 0;
  TimeMs last_eval_ = 0;
  TimeMs bucket_start_ = 0;
  uint8_t bucket_i_ = 0;
  uint16_t buckets_[cfg::DUTY_BUCKETS] = {};
};
