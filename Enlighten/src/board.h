// The hardware-abstraction contract.  Everything under src/core/ is
// portable and touches hardware ONLY through these free functions; the
// active board implementation is linked in from src/boards/<board>/.
//
// Porting to a new MCU (e.g. RP2040) = implement this header in a new
// src/boards/<mcu>/board.cpp.  The core is untouched.
#pragma once
#include <stdint.h>

#include "core/faults.h"
#include "core/hw_inputs.h"
#include "core/timebase.h"

// NOTE: enumerator names avoid Arduino.h macros (EXTERNAL, etc.).
enum class ResetCause : uint8_t {
  POWER_ON,
  EXTERNAL_RESET,  // reset button / programmer
  BROWNOUT,
  WATCHDOG,
};

namespace Board {

// Must run before anything else; forces every solenoid output to the
// closed (safe) state before configuring anything further.
void init();

TimeMs millisNow();

// bit i of open_mask = poofer i OPEN.  This is the ONLY path to the
// solenoid outputs, and its only caller is the sketch loop, with the
// SafetySupervisor's filtered mask.
void writeOutputs(uint16_t open_mask);

HwInputs readHwInputs();

// Sampled once at boot and latched for the session.
Protocol readProtocolSelect();

ResetCause resetCause();

// Output-port readback check for the boot self-test.
bool selfTestOutputs();

void watchdogEnable();
void watchdogFeed();

// Fault code preserved across a watchdog reset (checksummed .noinit RAM).
void persistFault(FaultCode f);
FaultCode recoverPersistedFault();

void setStatusLed(bool on);

// 16 per-solenoid panel LEDs (bit i = LED i lit).  Indicators only.
void writeChannelLeds(uint16_t mask);

// Operator display (16x2 character LCD, optional hardware).
// displayBegin() probes and initializes; the rig runs headless if absent.
// displayService() converges the LCD toward the 32-char desired screen at
// most ONE character per call, bounding its cost well under the loop
// budget.  A display/bus failure disables the display, never the show.
void displayBegin();
void displayService(const char* screen32);

// Protocol glue
void dmxBegin();
uint8_t dmxRead(uint16_t channel);  // 1-based DMX channel
uint32_t dmxAgeMs();                // ms since last valid packet
void midiBegin();
int16_t midiReadByte();  // next raw byte or -1

// SD-card standalone show playback (optional hardware, SPI + CS on D53).
bool sdBegin();  // true if a card is present and readable
// Opens the next *.SHW file in the root (cycling); false if none.
bool showOpenNext(char* name, uint8_t cap);
int16_t showReadByte();  // next file byte or -1 at EOF / no file
void showClose();
bool playButtonPressed();  // raw level of the PLAY button (true = pressed)
bool dispButtonPressed();  // display-page cycle button
bool selButtonPressed();   // page-context select/input button

// Bench mode: jumper sampled once at boot.  In bench mode the USB serial
// port (free, since DMX is not started) is an interactive console.
bool benchSelected();
void benchBegin();
int16_t benchReadByte();  // next console byte or -1
void benchPrint(const char* s);

// Blocking wait, used ONLY before the watchdog is enabled (boot signaling).
void bootDelayMs(uint16_t ms);

}  // namespace Board
