// Pin assignments for the Arduino Mega 2560 build.
// Solenoid outputs are direct port writes: PORTA = poofers 1-8,
// PORTC = poofers 9-16, ACTIVE LOW (bit 1 = closed/safe).
#pragma once

namespace pins {
constexpr uint8_t STATUS_LED = 13;
constexpr uint8_t DMX_SHIELD_POWER = 9;  // legacy shield power gate
constexpr uint8_t ESTOP = 2;             // NC loop to GND: LOW = circuit intact
constexpr uint8_t ARM_KEY = 3;           // keyswitch to GND: LOW = arm permitted
constexpr uint8_t PROTOCOL_SELECT = 4;   // open = MIDI (primary), GND = DMX
constexpr uint8_t BENCH_SELECT = 5;      // jumper to GND at boot = bench mode
constexpr uint8_t PLAY_BUTTON = 6;       // momentary to GND: SD show play/stop
constexpr uint8_t DISP_BUTTON = 7;       // momentary to GND: cycle display pages
constexpr uint8_t SEL_BUTTON = 8;        // momentary to GND: page-context input
constexpr uint8_t SD_CS = 53;            // SD card module chip select (SPI)

// 16x2 LCD on a PCF8574 I2C backpack, D20 (SDA) / D21 (SCL).
// Common addresses: 0x27 (PCF8574) or 0x3F (PCF8574A) — set to match yours.
constexpr uint8_t LCD_I2C_ADDR = 0x27;
}  // namespace pins
