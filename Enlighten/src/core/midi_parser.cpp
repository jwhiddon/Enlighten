#include "midi_parser.h"

namespace {
inline uint8_t dataBytesFor(uint8_t status) {
  uint8_t type = status & 0xF0;
  return (type == 0xC0 || type == 0xD0) ? 1 : 2;
}
}  // namespace

bool MidiParser::feed(uint8_t b, MidiEvent* out) {
  // Real-time: emit immediately, never disturbs message assembly.
  if (b >= 0xF8) {
    out->status = b;
    out->data1 = 0;
    out->data2 = 0;
    return true;
  }

  if (b == 0xF0) {  // SysEx start
    in_sysex_ = true;
    running_ = 0;
    have_ = 0;
    return false;
  }
  if (b == 0xF7) {  // SysEx end
    in_sysex_ = false;
    return false;
  }

  if (b & 0x80) {  // status byte
    in_sysex_ = false;
    have_ = 0;
    if (b >= 0xF0) {
      running_ = 0;  // system common cancels running status
      return false;
    }
    running_ = b;
    return false;
  }

  // data byte
  if (in_sysex_ || running_ == 0) return false;
  data_[have_++] = b;
  if (have_ >= dataBytesFor(running_)) {
    out->status = running_;
    out->data1 = data_[0];
    out->data2 = (dataBytesFor(running_) == 2) ? data_[1] : 0;
    have_ = 0;  // running status: next data bytes reuse the status
    return true;
  }
  return false;
}
