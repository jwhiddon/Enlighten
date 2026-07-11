// Arduino Mega 2560 board layer.  The ONLY file that touches PORTA/PORTC,
// the watchdog, or reset-cause plumbing.
//
// Guard so the Arduino builder ignores this file when a different board is
// selected (future boards add their own guarded implementation).
#if defined(ARDUINO_AVR_MEGA2560) || defined(__AVR_ATmega2560__) || defined(__AVR_ATmega1280__)

#include <Arduino.h>
#include <DMXSerial.h>
#include <SdFat.h>
#include <Wire.h>
#include <avr/wdt.h>

#include "../../board.h"
#include "board_pins.h"

// ---- Reset-cause capture ----------------------------------------------------
// Captured in .init3, BEFORE main()/setup(): the stock Mega bootloader does
// not reliably clear WDRF, and a still-enabled watchdog could re-fire during
// startup.  This is the standard AVR idiom.
static uint8_t g_mcusr __attribute__((section(".noinit")));
extern "C" void captureMcusr() __attribute__((naked, used, section(".init3")));
extern "C" void captureMcusr() {
  g_mcusr = MCUSR;
  MCUSR = 0;
  wdt_disable();
}

// ---- Fault persistence across watchdog resets -------------------------------
struct NoinitFault {
  uint8_t magic;
  uint8_t code;
  uint8_t check;
};
static NoinitFault g_fault __attribute__((section(".noinit")));
static constexpr uint8_t FAULT_MAGIC = 0xA5;

namespace Board {

void init() {
  // Solenoid ports FIRST: all outputs, all closed (active low: 1 = closed).
  PORTA = 0xFF;
  PORTC = 0xFF;
  DDRA = 0xFF;
  DDRC = 0xFF;
  PORTA = 0xFF;  // re-assert after direction change
  PORTC = 0xFF;

  pinMode(pins::STATUS_LED, OUTPUT);
  digitalWrite(pins::STATUS_LED, LOW);

  // Per-solenoid panel LEDs: PORTL = LEDs 1-8 (D49..D42), PORTK = LEDs
  // 9-16 (A8..A15).  Active high, all off at boot.  (SPI D50-D53 is
  // deliberately left free for a future SD-card show player.)
  PORTL = 0x00;
  PORTK = 0x00;
  DDRL = 0xFF;
  DDRK = 0xFF;

  pinMode(pins::ESTOP, INPUT_PULLUP);
  pinMode(pins::ARM_KEY, INPUT_PULLUP);
  pinMode(pins::PROTOCOL_SELECT, INPUT_PULLUP);
  pinMode(pins::BENCH_SELECT, INPUT_PULLUP);
  pinMode(pins::PLAY_BUTTON, INPUT_PULLUP);

  // Hold the DMX shield unpowered until dmxBegin().
  pinMode(pins::DMX_SHIELD_POWER, OUTPUT);
  digitalWrite(pins::DMX_SHIELD_POWER, LOW);
}

TimeMs millisNow() { return (TimeMs)millis(); }

void writeOutputs(uint16_t open_mask) {
  // Active low: invert.  bit i of open_mask = poofer i OPEN.
  PORTA = (uint8_t)~(open_mask & 0xFF);
  PORTC = (uint8_t)~((open_mask >> 8) & 0xFF);
}

HwInputs readHwInputs() {
  HwInputs hw;
  // NC loop to GND: LOW = intact.  Assertion is instant; release is
  // debounced 10 ms so a bouncing contact can't flap the state machine.
  bool raw_ok = digitalRead(pins::ESTOP) == LOW;
  static TimeMs last_bad = 0;
  static bool ever_bad = false;
  TimeMs now = millisNow();
  if (!raw_ok) {
    last_bad = now;
    ever_bad = true;
  }
  hw.estop_ok = raw_ok && (!ever_bad || elapsedMs(last_bad, now) >= 10);
  hw.arm_key = digitalRead(pins::ARM_KEY) == LOW;
  return hw;
}

Protocol readProtocolSelect() {
  // Open (pull-up high) = MIDI, the rig's primary protocol; jumper to GND
  // for a DMX console.
  return digitalRead(pins::PROTOCOL_SELECT) == LOW ? Protocol::DMX
                                                   : Protocol::MIDI;
}

ResetCause resetCause() {
  if (g_mcusr & _BV(WDRF)) return ResetCause::WATCHDOG;
  if (g_mcusr & _BV(BORF)) return ResetCause::BROWNOUT;
  if (g_mcusr & _BV(EXTRF)) return ResetCause::EXTERNAL_RESET;
  return ResetCause::POWER_ON;
}

bool selfTestOutputs() {
  // Both ports were driven to 0xFF (closed) in init(); read them back.
  // A shorted driver or solder bridge shows up as a stuck-low bit.
  return PINA == 0xFF && PINC == 0xFF;
}

void watchdogEnable() { wdt_enable(WDTO_120MS); }
void watchdogFeed() { wdt_reset(); }

void persistFault(FaultCode f) {
  g_fault.magic = FAULT_MAGIC;
  g_fault.code = (uint8_t)f;
  g_fault.check = (uint8_t)(FAULT_MAGIC ^ g_fault.code ^ 0x5A);
}

FaultCode recoverPersistedFault() {
  bool valid = g_fault.magic == FAULT_MAGIC &&
               g_fault.check == (uint8_t)(FAULT_MAGIC ^ g_fault.code ^ 0x5A);
  FaultCode f = valid ? (FaultCode)g_fault.code : FaultCode::NONE;
  g_fault.magic = 0;  // consume
  return f;
}

void setStatusLed(bool on) { digitalWrite(pins::STATUS_LED, on ? HIGH : LOW); }

void writeChannelLeds(uint16_t mask) {
  PORTL = (uint8_t)(mask & 0xFF);         // LEDs 1-8: PL0..PL7 = D49..D42
  PORTK = (uint8_t)((mask >> 8) & 0xFF);  // LEDs 9-16: PK0..PK7 = A8..A15
}

// ---- 20x4 LCD on a PCF8574 I2C backpack -------------------------------------
// Own minimal HD44780 driver: budgeted (max one character per service call,
// ~1.8 ms worst case at 100 kHz), with a Wire bus timeout and fail-open
// behavior — any NACK/timeout permanently disables the display for the
// session so a flaky panel can never stall the safety loop.
namespace {
constexpr uint8_t LCD_RS = 0x01;  // PCF8574 P0
constexpr uint8_t LCD_E = 0x04;   // P2 (P1 = R/W, kept low = write)
constexpr uint8_t LCD_BL = 0x08;  // P3 backlight
constexpr uint8_t LCD_COLS = 20;  // 20x4 panel
constexpr uint8_t LCD_CHARS = 80;

bool lcd_present = false;
char lcd_shadow[LCD_CHARS];
uint8_t lcd_scan = 0;      // round-robin scan position
uint8_t lcd_addr = 0xFF;   // current DDRAM cursor (0xFF = unknown)

bool pcfWrite(uint8_t v) {
  Wire.beginTransmission(pins::LCD_I2C_ADDR);
  Wire.write(v);
  if (Wire.endTransmission() != 0) {
    lcd_present = false;  // display fails open; the show goes on
    return false;
  }
  return true;
}

bool lcdPulse(uint8_t v) {
  if (!pcfWrite((uint8_t)(v | LCD_E))) return false;
  delayMicroseconds(1);
  if (!pcfWrite((uint8_t)(v & ~LCD_E))) return false;
  delayMicroseconds(50);  // HD44780 execution time
  return true;
}

bool lcdSend(uint8_t value, bool rs) {
  uint8_t flags = (uint8_t)(LCD_BL | (rs ? LCD_RS : 0));
  return lcdPulse((uint8_t)((value & 0xF0) | flags)) &&
         lcdPulse((uint8_t)(((value << 4) & 0xF0) | flags));
}

uint8_t ddramFor(uint8_t index) {
  // HD44780 20x4 row bases (rows are interleaved in DDRAM).
  static const uint8_t ROW_BASE[4] = {0x00, 0x40, 0x14, 0x54};
  return (uint8_t)(ROW_BASE[index / LCD_COLS] + index % LCD_COLS);
}
}  // namespace

void displayBegin() {
  Wire.begin();
  Wire.setClock(100000);            // PCF8574 is a 100 kHz part
  Wire.setWireTimeout(3000, true);  // a wedged bus can never hang the loop

  // Probe: headless rigs are fine.
  Wire.beginTransmission(pins::LCD_I2C_ADDR);
  if (Wire.endTransmission() != 0) {
    lcd_present = false;
    return;
  }
  lcd_present = true;

  // HD44780 4-bit init sequence (blocking; runs before the watchdog).
  delay(50);
  lcdPulse(0x30 | LCD_BL);
  delay(5);
  lcdPulse(0x30 | LCD_BL);
  delayMicroseconds(150);
  lcdPulse(0x30 | LCD_BL);
  delayMicroseconds(150);
  lcdPulse(0x20 | LCD_BL);  // 4-bit mode
  lcdSend(0x28, false);     // 2 lines, 5x8 font
  lcdSend(0x08, false);     // display off
  lcdSend(0x01, false);     // clear
  delay(2);
  lcdSend(0x06, false);  // entry mode: increment, no shift
  lcdSend(0x0C, false);  // display on, cursor off
  for (uint8_t i = 0; i < LCD_CHARS; ++i) lcd_shadow[i] = ' ';
  lcd_addr = 0;
}

void displayService(const char* screen32) {
  if (!lcd_present) return;
  // Find the next character that differs and write just that one.
  for (uint8_t k = 0; k < LCD_CHARS; ++k) {
    uint8_t i = (uint8_t)((lcd_scan + k) % LCD_CHARS);
    if (screen32[i] == lcd_shadow[i]) continue;
    uint8_t want_addr = ddramFor(i);
    if (lcd_addr != want_addr) {
      if (!lcdSend((uint8_t)(0x80 | want_addr), false)) return;
    }
    if (!lcdSend((uint8_t)screen32[i], true)) return;
    lcd_shadow[i] = screen32[i];
    lcd_addr = (uint8_t)(want_addr + 1);
    // Rows are non-contiguous in DDRAM: cursor position is unknown once a
    // row's last column is written.
    if (i % LCD_COLS == LCD_COLS - 1) lcd_addr = 0xFF;
    lcd_scan = (uint8_t)((i + 1) % LCD_CHARS);
    return;
  }
}

void dmxBegin() {
  DMXSerial.maxChannel(24);
  DMXSerial.init(DMXReceiver);
  // Power-cycle timing the legacy shield needs before it is enabled.
  bootDelayMs(1000);
  digitalWrite(pins::DMX_SHIELD_POWER, HIGH);
}

uint8_t dmxRead(uint16_t channel) { return DMXSerial.read((int)channel); }

uint32_t dmxAgeMs() { return DMXSerial.noDataSince(); }

void midiBegin() {
  Serial1.begin(31250);  // DIN MIDI on RX1 (pin 19)
}

int16_t midiReadByte() {
  int v = Serial1.read();
  return v < 0 ? (int16_t)-1 : (int16_t)v;
}

// ---- SD-card standalone show playback ---------------------------------------
// SdFat (not the stock SD library): the stock library's dead debug code
// references the `Serial` object, which drags in HardwareSerial0 and its
// USART0 interrupt vectors — colliding with DMXSerial's.  SdFat's core is
// Serial-free (and MIT licensed).
namespace {
bool g_sd_ok = false;
SdFat g_sd;
SdFile g_show_dir;
SdFile g_show_file;

bool nameIsShw(const char* n) {
  uint8_t len = 0;
  while (n[len]) ++len;
  return len > 4 && n[len - 4] == '.' &&
         (n[len - 3] == 'S' || n[len - 3] == 's') &&
         (n[len - 2] == 'H' || n[len - 2] == 'h') &&
         (n[len - 1] == 'W' || n[len - 1] == 'w');
}
}  // namespace

bool sdBegin() {
  g_sd_ok = g_sd.begin(pins::SD_CS, SD_SCK_MHZ(8));
  if (g_sd_ok) g_sd_ok = g_show_dir.open("/", O_RDONLY);
  return g_sd_ok;
}

bool showOpenNext(char* name, uint8_t cap) {
  if (!g_sd_ok) return false;
  if (g_show_file.isOpen()) g_show_file.close();

  // Cycle the root directory (wrapping once) for the next *.SHW file.
  for (uint8_t pass = 0; pass < 2; ++pass) {
    while (g_show_file.openNext(&g_show_dir, O_RDONLY)) {
      char n[16];
      if (!g_show_file.isDir() && g_show_file.getName(n, sizeof(n)) &&
          nameIsShw(n)) {
        uint8_t i = 0;
        while (n[i] && i + 1 < cap) {
          name[i] = n[i];
          ++i;
        }
        name[i] = 0;
        return true;
      }
      g_show_file.close();
    }
    g_show_dir.rewind();
  }
  return false;
}

int16_t showReadByte() {
  if (!g_show_file.isOpen()) return -1;
  int v = g_show_file.read();
  return v < 0 ? (int16_t)-1 : (int16_t)v;
}

void showClose() {
  if (g_show_file.isOpen()) g_show_file.close();
}

bool playButtonPressed() { return digitalRead(pins::PLAY_BUTTON) == LOW; }

bool benchSelected() { return digitalRead(pins::BENCH_SELECT) == LOW; }

// Bench console: polled register-level USART0 driver.  The Arduino
// `Serial` object cannot be linked here — DMXSerial defines the USART0
// interrupt vectors and the two collide.  In bench mode DMXSerial is never
// initialized, so we drive the UART directly with no interrupts: RX is
// polled each loop, TX drains from a ring buffer a byte at a time so a
// long reply can never stall the safety loop.
namespace {
constexpr uint16_t BENCH_TXBUF = 384;
uint8_t bench_tx[BENCH_TXBUF];
uint16_t bench_tx_head = 0;
uint16_t bench_tx_tail = 0;

void benchTxDrain() {
  while (bench_tx_head != bench_tx_tail && (UCSR0A & _BV(UDRE0))) {
    UDR0 = bench_tx[bench_tx_tail];
    bench_tx_tail = (uint16_t)((bench_tx_tail + 1) % BENCH_TXBUF);
  }
}
}  // namespace

void benchBegin() {
  UCSR0A = _BV(U2X0);
  UBRR0 = 16;  // 115200 baud at 16 MHz (U2X), same setup Arduino core uses
  UCSR0B = _BV(RXEN0) | _BV(TXEN0);  // no interrupts — polled on purpose
  UCSR0C = _BV(UCSZ01) | _BV(UCSZ00);  // 8N1
}

int16_t benchReadByte() {
  benchTxDrain();  // piggyback: called at least once per loop
  return (UCSR0A & _BV(RXC0)) ? (int16_t)UDR0 : (int16_t)-1;
}

void benchPrint(const char* s) {
  while (*s) {
    uint16_t next = (uint16_t)((bench_tx_head + 1) % BENCH_TXBUF);
    while (next == bench_tx_tail) {  // ring full: block for one slot
      while (!(UCSR0A & _BV(UDRE0))) {
      }
      UDR0 = bench_tx[bench_tx_tail];
      bench_tx_tail = (uint16_t)((bench_tx_tail + 1) % BENCH_TXBUF);
    }
    bench_tx[bench_tx_head] = (uint8_t)*s++;
    bench_tx_head = next;
    benchTxDrain();
  }
}

void bootDelayMs(uint16_t ms) { delay(ms); }

}  // namespace Board

#endif  // Mega 2560 / 1280
