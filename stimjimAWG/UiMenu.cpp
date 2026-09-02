//    stimjimAWG — UiMenu implementation. GPL-3.0-or-later; see Config.h header.

#include "UiMenu.h"
#include "UiInput.h"
#include "Config.h"
#include "Engine.h"
#include "TrainStore.h"
#include <stdio.h>
#include <stdarg.h>
#include <string.h>

#if SJ_USE_DISPLAY
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
static Adafruit_SSD1306 display(SJ_OLED_WIDTH, SJ_OLED_HEIGHT, &Wire, -1);
static bool displayOk = false;
#endif

namespace UiMenu {

// The rendered state, kept as text so a redraw can be skipped when nothing
// changed. An I2C frame push costs ~13 ms at 400 kHz — harmless to the
// waveforms (they run from ISRs that preempt loop()) but pointless to repeat.
static const uint8_t NROWS = SJ_OLED_ROWS;          // 4 rows of 8 px
static const uint8_t NCOLS = SJ_OLED_COLS;          // 21 chars of 6 px
static char    rowText[NROWS][NCOLS + 1];
static char    prevText[NROWS][NCOLS + 1];
static int8_t  barPct    = -1;                      // -1 = no progress bar
static int8_t  prevBarPct = -1;
static uint8_t cursor    = 0;                       // browsed slot
static uint32_t lastRender = 0;

uint8_t browseSlot() { return cursor; }

// Format into a row, clipping at the panel width. Composition deliberately
// over-fills (a 100 us delay and a 1000.000s one need different room), so
// clipping is the designed behaviour, not an accident — doing it here keeps
// every call site free of snprintf truncation guesswork.
static void setRow(uint8_t r, const char* fmt, ...) __attribute__((format(printf, 2, 3)));
static void setRow(uint8_t r, const char* fmt, ...) {
  char scratch[80];
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(scratch, sizeof scratch, fmt, ap);
  va_end(ap);
  uint8_t i = 0;
  while (i < NCOLS && scratch[i]) { rowText[r][i] = scratch[i]; i++; }
  rowText[r][i] = '\0';
}

// ------------------------------------------------------------- formatting
//
// Microseconds as the shortest readable form that still shows three
// significant figures: "850us", "10.00ms", "1.500s". Fixed-point only —
// these run in loop context, but keeping the UI float-free costs nothing.
static void fmtUs(char* out, size_t n, uint32_t us) {
  if (us < 1000)          snprintf(out, n, "%luus", (unsigned long)us);
  else if (us < 1000000)  snprintf(out, n, "%lu.%02lums",
                                   (unsigned long)(us / 1000), (unsigned long)(us % 1000) / 10);
  else                    snprintf(out, n, "%lu.%03lus",
                                   (unsigned long)(us / 1000000), (unsigned long)(us % 1000000) / 1000);
}

// Output mode as one character: the original 0-3 numbering (protocol §2).
static char modeChar(uint8_t m) {
  static const char c[4] = {'V', 'I', '-', 'G'};
  return m < 4 ? c[m] : '?';
}

static char typeChar(uint8_t t) {
  return t == SINE ? 'W' : t == PIECEWISE_RAMP ? 'L' : 'S';
}

// ------------------------------------------------------------- composition

static void composeHeader() {
  // Engine tags are the one thing always worth screen space: they answer
  // "is this box driving my preparation right now?" without the serial port.
  char t[8], u[8];
  int16_t s0 = Engine::activeSlot(0), s1 = Engine::activeSlot(1);
  if (s0 < 0) strcpy(t, "--"); else snprintf(t, sizeof t, "%02u", (unsigned)(uint8_t)s0);
  if (s1 < 0) strcpy(u, "--"); else snprintf(u, sizeof u, "%02u", (unsigned)(uint8_t)s1);
  setRow(0, "AWG %s T%s U%s", SJ_FW_VERSION, t, u);
}

// Line describing a slot: type+index, per-channel modes, repetition period.
static void composeSlotLine(uint8_t row, uint8_t idx) {
  const TrainDef& d = TrainStore::slotConst(idx);
  char per[16];
  fmtUs(per, sizeof per, d.period_us);
  setRow(row, "%c%02u %c/%c %s", typeChar(d.type), idx,
         modeChar(d.mode0), modeChar(d.mode1), per);
}

static void composeBrowse() {
  const TrainDef& d = TrainStore::slotConst(cursor);
  composeSlotLine(1, cursor);

  char dur[16], dly[16];
  fmtUs(dur, sizeof dur, d.duration_us);
  fmtUs(dly, sizeof dly, d.delay_us);
  setRow(2, "dur %s dly %s", dur, dly);

  if (d.type == SINE) {
    // Frequency to one decimal is enough to recognise a setting at a glance.
    unsigned long f = d.sine.freq0_mHz ? d.sine.freq0_mHz : d.sine.freq1_mHz;
    setRow(3, "sine %lu.%luHz  <>slot", f / 1000, (f % 1000) / 100);
  } else {
    setRow(3, "%u stage%s     <>slot", d.nStages, d.nStages == 1 ? "" : "s");
  }
  barPct = -1;
}

static void composeRunning(uint8_t eng, const Engine::EngineStatus& s) {
  composeSlotLine(1, (uint8_t)s.slot);

  // Row 2 is the progress bar in this view, so the phase word and the numbers
  // share row 3. The engine is not repeated here -- the header already shows
  // which of T/U carries the slot, and 21 characters is a tight budget.
  (void)eng;
  setRow(2, "%s", "");
  char a[16], b[16];
  if (s.waiting) {
    // Armed but still counting out the slot's delay: outputs are grounded and
    // the bar tracks how much of the delay is gone, not the train.
    barPct = (s.delay_us == 0) ? 0
           : (int8_t)(100 - (uint64_t)s.remaining_delay_us * 100 / s.delay_us);
    fmtUs(a, sizeof a, s.remaining_delay_us);
    fmtUs(b, sizeof b, s.delay_us);
    setRow(3, "WAIT %s/%s", a, b);
    return;
  }
  barPct = (s.duration_us == 0) ? 100
         : (int8_t)((uint64_t)s.elapsed_us * 100 / s.duration_us);
  fmtUs(a, sizeof a, s.elapsed_us);
  fmtUs(b, sizeof b, s.duration_us);
  setRow(3, "n=%lu %s/%s", (unsigned long)s.nPulses, a, b);
}

static void compose() {
  composeHeader();
  Engine::EngineStatus s0, s1;
  Engine::status(0, s0);
  Engine::status(1, s1);
  if (s0.slot >= 0)      composeRunning(0, s0);
  else if (s1.slot >= 0) composeRunning(1, s1);
  else                   composeBrowse();
}

// ---------------------------------------------------------------- rendering

#if SJ_USE_DISPLAY
static void paint() {
  display.clearDisplay();
  // Row 0 is a title bar: white background, black text — it separates the
  // machine's identity and engine state from the slot detail below.
  display.fillRect(0, 0, SJ_OLED_WIDTH, 8, SSD1306_WHITE);
  display.setTextSize(1);
  display.setTextColor(SSD1306_BLACK, SSD1306_WHITE);
  display.setCursor(1, 0);
  display.print(rowText[0]);

  display.setTextColor(SSD1306_WHITE, SSD1306_BLACK);
  for (uint8_t r = 1; r < NROWS; r++) {
    // The progress bar replaces row 2's text when a train is running.
    if (r == 2 && barPct >= 0) {
      display.drawRect(0, 17, SJ_OLED_WIDTH, 6, SSD1306_WHITE);
      int w = (SJ_OLED_WIDTH - 4) * (barPct > 100 ? 100 : barPct) / 100;
      if (w > 0) display.fillRect(2, 19, w, 2, SSD1306_WHITE);
      continue;
    }
    display.setCursor(1, r * 8);
    display.print(rowText[r]);
  }
  display.display();
}
#endif

static bool changed() {
  if (barPct != prevBarPct) return true;
  for (uint8_t r = 0; r < NROWS; r++)
    if (strcmp(rowText[r], prevText[r])) return true;
  return false;
}

static void commitPrev() {
  for (uint8_t r = 0; r < NROWS; r++) strcpy(prevText[r], rowText[r]);
  prevBarPct = barPct;
}

// -------------------------------------------------------------------- API

void begin() {
  memset(rowText, 0, sizeof rowText);
  memset(prevText, 0xFF, sizeof prevText);   // force the first paint
#if SJ_USE_DISPLAY
  displayOk = display.begin(SSD1306_SWITCHCAPVCC, SJ_OLED_ADDR);
  if (!displayOk) {
    Serial.println("WARN display: SSD1306 init failed");
    return;
  }
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.printf("%s %s\n", SJ_FW_NAME, SJ_FW_VERSION);
  display.println(SJ_HW_NAME);
  display.println("ready");
  display.display();
#endif
}

void tick() {
  UiInput::Event e;
  bool moved = false;
  while ((e = UiInput::pop()) != UiInput::EV_NONE) {
    switch (e) {
      case UiInput::EV_PREV: cursor = (uint8_t)((cursor + SJ_NUM_SLOTS - 1) % SJ_NUM_SLOTS); moved = true; break;
      case UiInput::EV_NEXT: cursor = (uint8_t)((cursor + 1) % SJ_NUM_SLOTS);                moved = true; break;
      case UiInput::EV_OK:   cursor = 0; moved = true; break;   // jump home; start stays serial-only
      default: break;
    }
  }
  (void)moved;

  uint32_t now = millis();
  if (now - lastRender < SJ_UI_MIN_MS) return;
  compose();
  if (!changed()) return;
  lastRender = now;
  commitPrev();
#if SJ_USE_DISPLAY
  if (displayOk) paint();
#endif
}

void dumpScreen() {
#if SJ_USE_DISPLAY
  if (!displayOk) { Serial.println("ERR SCREEN: SSD1306 not present"); return; }
  compose();
  commitPrev();
  paint();
  const uint8_t* b = display.getBuffer();
  Serial.printf("# SCREEN %dx%d\n", SJ_OLED_WIDTH, SJ_OLED_HEIGHT);
  // The composed text next to the pixels: a capture is then readable without
  // decoding the bitmap font, and a layout regression names itself.
  for (uint8_t r = 0; r < NROWS; r++)
    Serial.printf("# row%u: \"%s\"%s\n", r, rowText[r],
                  (r == 2 && barPct >= 0) ? "  (replaced by the progress bar)" : "");
  if (barPct >= 0) Serial.printf("# bar: %d%%\n", barPct);
  for (int y = 0; y < SJ_OLED_HEIGHT; y++) {
    char line[SJ_OLED_WIDTH + 3];
    line[0] = '|';
    // SSD1306 buffer is page-major: byte (y/8)*width + x holds 8 stacked rows,
    // bit (y & 7) is this row's pixel.
    for (int x = 0; x < SJ_OLED_WIDTH; x++)
      line[x + 1] = ((b[(y / 8) * SJ_OLED_WIDTH + x] >> (y & 7)) & 1) ? '#' : '.';
    line[SJ_OLED_WIDTH + 1] = '|';
    line[SJ_OLED_WIDTH + 2] = '\0';
    Serial.println(line);
  }
  Serial.println("OK");
#else
  Serial.println("ERR SCREEN: display disabled at build time (SJ_USE_DISPLAY)");
#endif
}

} // namespace UiMenu
