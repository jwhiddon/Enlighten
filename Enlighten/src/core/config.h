// Every tunable in one place.  Values that affect flame-effect safety are
// guarded by static_asserts; do not raise them without a safety review.
#pragma once
#include <stdint.h>

namespace cfg {

constexpr uint8_t  NUM_POOFERS       = 16;
constexpr uint8_t  NUM_CHASE_POOFERS = 8;   // chase patterns run on poofers 1-8

// ---- Choreography timing (requests are clamped to these) ----------------
constexpr uint16_t MIN_POOF_MS = 30;
constexpr uint16_t MAX_POOF_MS = 500;
constexpr uint16_t MIN_REST_MS = 45;
constexpr uint16_t MAX_REST_MS = 2000;

// ---- Hard physical limits (DutyLimiter; independent of any input) -------
constexpr uint16_t MAX_OPEN_MS            = 500;    // absolute continuous-open cap
constexpr uint16_t MIN_CLOSE_MS           = 50;     // forced close between opens
constexpr uint16_t DUTY_WINDOW_MS         = 10000;  // rolling budget window
constexpr uint16_t MAX_OPEN_PER_WINDOW_MS = 3000;   // 30% duty budget
constexpr uint16_t DUTY_BUCKET_MS         = 1000;
// One extra bucket so the bucketed history always covers AT LEAST the full
// window; enforcement is therefore conservative for any sliding window.
constexpr uint8_t  DUTY_BUCKETS = DUTY_WINDOW_MS / DUTY_BUCKET_MS + 1;

// ---- Arming --------------------------------------------------------------
constexpr uint16_t ARM_HOLD_MS = 500;   // handshake must be held this long
constexpr uint8_t  MIDI_ARM_A  = 85;    // CC20 (MIDI data is 7-bit: 0-127)
constexpr uint8_t  MIDI_ARM_B  = 106;   // CC21

// ---- Signal supervision (deadman) ----------------------------------------
constexpr uint16_t MIDI_AS_TIMEOUT_MS = 330;   // once active sensing is seen
constexpr uint16_t MIDI_KEEPALIVE_MS  = 2000;  // otherwise: any byte required

// ---- Mode selection (CC22 scaled x2 onto 0-255 bands) ----------------------
constexpr uint16_t MODE_DEBOUNCE_MS  = 150;
constexpr uint8_t  MODE_BAND_WIDTH   = 20;
constexpr uint8_t  MODE_GUARD        = 2;   // hold-current values at band edges

// ---- MIDI mapping ----------------------------------------------------------
constexpr uint8_t  MIDI_CHANNEL    = 0;    // MIDI channel 1 (0-based)
constexpr uint8_t  MIDI_NOTE_FIRST = 36;   // C2 -> poofer 1
constexpr uint8_t  MIDI_VEL_GATE   = 64;   // note-on velocity >= this fires

// ---- Bench mode -------------------------------------------------------------
constexpr uint32_t BENCH_KEEPALIVE_MS = 120000;  // console silence -> disarm

// ---- Operator panel ----------------------------------------------------------
constexpr uint32_t DISPLAY_PAGE_TIMEOUT_MS = 30000;  // aux pages auto-return

// ---- Loop supervision -------------------------------------------------------
constexpr uint16_t LOOP_BUDGET_MS = 5;   // longer gap between filters = fault

static_assert(MAX_POOF_MS <= MAX_OPEN_MS,
              "choreography cap must not exceed the hard open cap");
static_assert(MAX_OPEN_MS <= 500,
              "raising the hard open cap requires a safety review");
static_assert(MAX_OPEN_PER_WINDOW_MS < DUTY_WINDOW_MS,
              "duty budget must be a fraction of the window");
static_assert(DUTY_WINDOW_MS % DUTY_BUCKET_MS == 0,
              "buckets must divide the duty window evenly");
static_assert(MIN_CLOSE_MS > 0, "a nonzero forced close is required");
static_assert(NUM_POOFERS <= 16, "trigger/output masks are 16-bit");
static_assert(MIDI_ARM_A <= 127 && MIDI_ARM_B <= 127,
              "MIDI CC data bytes are 7-bit");

}  // namespace cfg
