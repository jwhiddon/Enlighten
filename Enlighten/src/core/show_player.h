// Standalone .show playback (SD card): streams the text timeline format
// written by tools/seqgen ("<ms> <mask_hex>" lines) and replays it as a
// requested output mask.
//
// The player is a REQUEST source only: its mask is OR-ed with the
// sequencer's and everything still passes through SafetySupervisor,
// so playback fires nothing unless the rig is armed (MIDI handshake +
// keyswitch + E-stop), and duty limits still clamp every channel.
//
// Portable core: file bytes are pumped in by the sketch (feed/feedEof);
// a small ring of parsed upcoming changes keeps RAM use tiny while the
// card streams.
#pragma once
#include "config.h"
#include "timebase.h"

class ShowPlayer {
 public:
  void start(TimeMs now);  // begin playback; expects a fresh byte stream
  void stop();

  bool playing() const { return active_; }
  bool wantsData() const { return active_ && !eof_ && count_ < RING; }
  void feed(char c);
  void feedEof();

  void update(TimeMs now);  // advance the playback clock
  uint16_t mask() const { return mask_; }
  uint32_t positionMs() const { return pos_; }

 private:
  void parseLine();

  static constexpr uint8_t RING = 8;
  struct Change {
    uint32_t t;
    uint16_t mask;
  };
  Change ring_[RING];
  uint8_t head_ = 0;
  uint8_t count_ = 0;

  char line_[24];
  uint8_t len_ = 0;

  bool active_ = false;
  bool eof_ = false;
  TimeMs t0_ = 0;
  uint32_t pos_ = 0;
  uint16_t mask_ = 0;
};
