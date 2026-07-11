#include "status_led.h"

bool statusLedOn(SafetyState state, FaultCode fault, bool firing, TimeMs now) {
  switch (state) {
    case SafetyState::BOOT_SELFTEST:
      return true;

    case SafetyState::SAFE:
      return (now / 500) % 2 == 0;

    case SafetyState::ARM_PENDING: {
      uint32_t t = now % 1000;
      return t < 100 || (t >= 200 && t < 300);
    }

    case SafetyState::ARMED:
      return firing || (now / 125) % 2 == 0;

    case SafetyState::FAULT_LOCKOUT: {
      uint8_t n = (uint8_t)fault;
      if (n == 0) n = 1;
      uint32_t cycle = (uint32_t)n * 400 + 1000;  // N blinks + 1 s gap
      uint32_t t = now % cycle;
      return (t / 400) < n && (t % 400) < 200;
    }
  }
  return false;
}
