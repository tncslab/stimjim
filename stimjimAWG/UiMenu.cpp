//    stimjimAWG — UiMenu implementation. GPL-3.0-or-later; see Config.h header.

#include "UiMenu.h"
#include "UiInput.h"
#include "UiFmt.h"
#include "Config.h"
#include "Clock.h"
#include "Engine.h"
#include "FastIO.h"
#include "Measure.h"
#include "SdLog.h"
#include "TrainStore.h"
#include "Triggers.h"
#include <stdio.h>
#include <stdarg.h>
#include <string.h>

#if SJ_USE_DISPLAY
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
static Adafruit_SSD1306 display(SJ_OLED_WIDTH, SJ_OLED_HEIGHT, &Wire, -1);
static bool framebufferOk = false;   // display.begin() allocated its 512-byte buffer
static bool panelPresent  = false;   // something answered at SJ_OLED_ADDR
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
static uint8_t page      = 0;
static uint32_t lastRender = 0;

// The last completed train's numbers, filled by noteResult from the completion
// drain. Zero-initialised, so `valid` is false until a train has finished.
static Measure::ResultSet result;

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
  // Trailing blanks carry no information and would make `changed()` fire on a
  // field that only moved within its column, so they are dropped here.
  while (i && rowText[r][i - 1] == ' ') i--;
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

// Seconds since boot as the shortest readable form: "45s", "12m", "3h07",
// "5d04" — always four characters or fewer.
static void fmtUptime(char* out, size_t n, uint32_t sec) {
  if (sec < 60)         snprintf(out, n, "%lus", (unsigned long)sec);
  else if (sec < 3600)  snprintf(out, n, "%lum", (unsigned long)(sec / 60));
  else if (sec < 86400) snprintf(out, n, "%luh%02lu", (unsigned long)(sec / 3600),
                                 (unsigned long)((sec / 60) % 60));
  else                  snprintf(out, n, "%lud%02lu", (unsigned long)(sec / 86400),
                                 (unsigned long)((sec / 3600) % 24));
}

// Output mode as one character: the original 0-3 numbering (protocol §2).
static char modeChar(uint8_t m) {
  static const char c[4] = {'V', 'I', '-', 'G'};
  return m < 4 ? c[m] : '?';
}

static char typeChar(uint8_t t) {
  return t == SINE ? 'W' : t == PIECEWISE_RAMP ? 'L' : 'S';
}

// A slot as the display names it: "S07". "--" when a route leaves it empty.
static void slotTag(int8_t slot, char* out, size_t n) {
  if (slot < 0) { snprintf(out, n, "--"); return; }
  snprintf(out, n, "%c%02u", typeChar(TrainStore::slotConst((uint8_t)slot).type),
           (unsigned)(uint8_t)slot);
}

// ---------------------------------------------------------------- the pages

// Result pages the current snapshot offers, capped so one button still walks
// the whole list. `MSUM` carries the points beyond the cap.
static uint8_t resultPages() {
  if (!result.valid) return 0;
  return (result.nPoints > SJ_UI_RESULT_PAGES) ? (uint8_t)SJ_UI_RESULT_PAGES : result.nPoints;
}

uint8_t pageCount() { return (uint8_t)(2 + resultPages()); }

static const char* pageName(uint8_t p) {
  if (p == 0)                return "STATUS";
  if (p <= resultPages())    return "RESULT";
  return "SYSTEM";
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

static void composeRunning(const Engine::EngineStatus& s) {
  composeSlotLine(1, (uint8_t)s.slot);

  // Row 2 is the progress bar in this view, so the phase word and the numbers
  // share row 3. The engine is not repeated here -- the header already shows
  // which of T/U carries the slot, and 21 characters is a tight budget.
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

// What one trigger button would fire: the slots its input routes to, or why it
// would fire nothing. With Btn1 and Btn2 wired to the routes this is the useful
// thing to show when nothing is playing, and it is what the slot browse the
// buttons used to scroll was for.
static void routeText(uint8_t input, char* out, size_t n) {
  const TriggerRoute& r = Triggers::route(input);
  char a[8], b[8];
  switch (r.mode) {
    case 1:                                        // joint: one slot, one engine
      slotTag(r.slot0, a, sizeof a);
      snprintf(out, n, "%s", a);
      break;
    case 2:                                        // independent: one per engine
      slotTag(r.slot0, a, sizeof a);
      slotTag(r.slot1, b, sizeof b);
      snprintf(out, n, "%s+%s", a, b);
      break;
    case 3:  snprintf(out, n, "marker"); break;    // pin is an output, not an input
    default: snprintf(out, n, "off");    break;
  }
}

static void composeStatusIdle() {
  char r0[16], r1[16];
  routeText(0, r0, sizeof r0);
  routeText(1, r1, sizeof r1);
  setRow(1, "B1 IN0 %s", r0);
  setRow(2, "B2 IN1 %s", r1);
  if (SdLog::isOpen())          setRow(3, "%s %luB", SdLog::name(), (unsigned long)SdLog::bytes());
  else if (SdLog::cardPresent()) setRow(3, "card in, LOG1 logs");
  else                          setRow(3, "no card");
  barPct = -1;
}

static void composeStatus() {
  composeHeader();
  Engine::EngineStatus s0, s1;
  Engine::status(0, s0);
  Engine::status(1, s1);
  if (s0.slot >= 0)      composeRunning(s0);
  else if (s1.slot >= 0) composeRunning(s1);
  else                   composeStatusIdle();
}

// One result row: a three-character tag, then each channel's seven-character
// value followed by its limit marker, then the unit. 3 + 8 + 8 + 2 = 21, the
// whole panel width — the marker column comes out of the gap between the two
// channels, not out of the numbers.
static void setResultRow(uint8_t row, const char* tag,
                         const char* v0, char f0, const char* v1, char f1,
                         const char* unit) {
  setRow(row, "%-3s%7s%c%7s%c%s", tag, v0, f0, v1, f1, unit);
}

static void composeResult(uint8_t k) {
  const Measure::ResultChan& c0 = result.ch[k][0];
  const Measure::ResultChan& c1 = result.ch[k][1];

  // Title bar: which train, which engine, how many repetitions are behind the
  // numbers, which measurement point, and where this page sits in the set. A
  // sine train's points are peaks, so they are labelled in degrees.
  char lbl[8];
  if (result.type == SINE) snprintf(lbl, sizeof lbl, "%ud", (unsigned)result.label[k]);
  else                     snprintf(lbl, sizeof lbl, "s%u", (unsigned)result.label[k]);
  setRow(0, "#%lu %c n=%lu %s %u/%u", (unsigned long)result.trainNo,
         result.eng ? 'U' : 'T', (unsigned long)result.nMax, lbl,
         (unsigned)(k + 1), (unsigned)resultPages());

  char a[SJ_UI_FIELD_MAX], b[SJ_UI_FIELD_MAX];
  const char fv0 = c0.nV ? UiFmt::voltFlag(c0.uV) : ' ';
  const char fv1 = c1.nV ? UiFmt::voltFlag(c1.uV) : ' ';
  const char fi0 = c0.nI ? UiFmt::currFlag(c0.nA) : ' ';
  const char fi1 = c1.nI ? UiFmt::currFlag(c1.nA) : ' ';

  UiFmt::fmtVolt(a, sizeof a, c0.nV != 0, c0.uV);
  UiFmt::fmtVolt(b, sizeof b, c1.nV != 0, c1.uV);
  setResultRow(1, "V", a, fv0, b, fv1, "mV");

  UiFmt::fmtCurr(a, sizeof a, c0.nI != 0, c0.nA);
  UiFmt::fmtCurr(b, sizeof b, c1.nI != 0, c1.nA);
  setResultRow(2, "I", a, fi0, b, fi1, "uA");

  UiFmt::fmtOhm(a, sizeof a, c0.nV != 0, c0.uV, c0.seUV, c0.nI != 0, c0.nA, c0.seNA);
  UiFmt::fmtOhm(b, sizeof b, c1.nV != 0, c1.uV, c1.seUV, c1.nI != 0, c1.nA, c1.seNA);
  // A resistance derived from a reading at a hardware limit is suspect for the
  // same reason the reading is, so the marker carries through.
  const char fr0 = (fv0 == '*' || fi0 == '*') ? '*' : ' ';
  const char fr1 = (fv1 == '*' || fi1 == '*') ? '*' : ' ';
  setResultRow(3, "R", a, fr0, b, fr1, "");
  barPct = -1;
}

static void composeSystem() {
  setRow(0, "SYSTEM %s %s", SJ_HW_NAME, SJ_FW_VERSION);

  char up[12];
  fmtUptime(up, sizeof up, (uint32_t)(SJ_CYC_TO_US(FastIO::cycles64()) / 1000000u));
  setRow(1, "%s code  up %s", SJ_HOT_NAME, up);

#if SJ_USE_SD
  const char* card = SdLog::cardPresent() ? "yes" : "no";
#else
  const char* card = "n/a";                  // no socket on this board
#endif
#if SJ_USE_DISPLAY
  const char* panel = panelPresent ? "yes" : "no";
#else
  const char* panel = "off";                 // compiled out
#endif
  setRow(2, "SD %s  OLED %s", card, panel);

  uint32_t s;
  uint16_t ms;
  if (Clock::read(s, ms)) {
    Clock::Civil c;
    Clock::civilFromUnix(s, ms, c);
    // Two-digit year and no seconds: the panel is a glance, and the source word
    // is the part that decides whether the rest means anything.
    setRow(3, "%02u-%02u-%02u %02u:%02u %s", (unsigned)(c.year % 100u), (unsigned)c.mon,
           (unsigned)c.day, (unsigned)c.hour, (unsigned)c.min,
           Clock::sourceName(Clock::source()));
  } else {
    setRow(3, "no RTC");
  }
  barPct = -1;
}

static void compose() {
  const uint8_t nres = resultPages();
  if (page >= (uint8_t)(2 + nres)) page = 0;   // the result set shrank under us
  if (page == 0)          composeStatus();
  else if (page <= nres)  composeResult((uint8_t)(page - 1));
  else                    composeSystem();
}

// ---------------------------------------------------------------- rendering

#if SJ_USE_DISPLAY
// One raw transaction to the panel's address. Adafruit_SSD1306::begin() returns
// false only when its framebuffer malloc fails (2.5.17,
// Adafruit_SSD1306.cpp:496-638): it never probes the bus and every I2C write it
// makes is unchecked, so this is the only thing that can tell a missing panel
// from a present one. With nothing connected there are no pull-ups on the bus
// and the Kinetis Wire driver falls back on its timeouts, ~20 ms for this one
// transaction.
static bool probePanel() {
  Wire.beginTransmission(SJ_OLED_ADDR);
  return Wire.endTransmission() == 0;
}

static void paint() {
  display.clearDisplay();
  // Row 0 is a title bar: white background, black text — it separates the
  // machine's identity and engine state from the detail below.
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
  // Everything above is RAM work on the framebuffer; this is the only bus
  // traffic, and it is the one thing a missing panel must not be asked for.
  if (panelPresent) display.display();
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

// -------------------------------------------------------------------- PAGE

void pageStatus() {
  Serial.printf("PAGE,%u,%u,%s\n", (unsigned)page, (unsigned)pageCount(), pageName(page));
}

void pageNext() {
  page = (uint8_t)((page + 1) % pageCount());
  pageStatus();
}

void pageSelect(uint8_t idx) {
  if (idx >= pageCount()) {
    Serial.printf("ERR PAGE: page %u does not exist — there are %u (0..%u)\n",
                  (unsigned)idx, (unsigned)pageCount(), (unsigned)(pageCount() - 1));
    return;
  }
  page = idx;
  pageStatus();
}

void noteResult(uint32_t trainNo, uint8_t eng, uint8_t slot) {
  Measure::resultSnapshot(eng, slot, trainNo, result);
  // A completion that measured something is what the operator is waiting for,
  // so the panel goes to it by itself. One that measured nothing leaves
  // whatever page was being read alone.
  if (result.valid) page = 1;
}

// -------------------------------------------------------------------- API

// A trigger button dispatches from loop(), not from its own ISR: a press is a
// human action, so a loop()-scale delay (microseconds normally, tens of
// milliseconds while the card flushes) costs nothing that matters, and it keeps
// train arming to a single context — no second ISR path to reason about, and
// no need to raise the button ports' NVIC priority so that a trigger edge
// cannot preempt a button's arm. Delivered waveform timing is unchanged either
// way, because t0 is anchored at the start request, which is this call.
static void buttonFire(uint8_t input) {
  // The same masking handleStart() takes for `T`/`U`: a trigger edge can
  // preempt loop() and arm the same engine, and the busy check plus the
  // arm-time writes are not atomic against it.
  FastIO::busLock();
  const bool routed = Triggers::fireRoute(input, 0);
  FastIO::busUnlock();
  if (!routed)
    Serial.printf("WARN button: input %u has no train routed — set TRIG%u\n",
                  (unsigned)input, (unsigned)input);
}

void begin() {
  memset(rowText, 0, sizeof rowText);
  memset(prevText, 0xFF, sizeof prevText);   // force the first paint
  memset(&result, 0, sizeof result);
#if SJ_USE_DISPLAY
  Wire.begin();                              // probePanel needs the bus up
  panelPresent = probePanel();
  // display.begin() runs either way: it is what allocates the framebuffer, and
  // there is no other way to get one. With no panel its ~25 unchecked
  // transactions cost up to half a second — once, at boot.
  framebufferOk = display.begin(SSD1306_SWITCHCAPVCC, SJ_OLED_ADDR);
  if (!framebufferOk) {
    Serial.println("WARN display: framebuffer allocation failed — no rendering, "
                   "SCREEN will report it");
    return;
  }
  if (!panelPresent) {
    // No automatic retry: a periodic probe would cost a 20 ms loop() stall for
    // ever on a board that is meant to run without a panel. `SCREEN` re-probes.
    Serial.println("# display: no panel — rendering to the framebuffer only "
                   "(SCREEN still works; it re-probes)");
    return;
  }
  Serial.printf("# display: SSD1306 at 0x%02X\n", SJ_OLED_ADDR);
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
  while ((e = UiInput::pop()) != UiInput::EV_NONE) {
    switch (e) {
      case UiInput::EV_PAGE:  page = (uint8_t)((page + 1) % pageCount()); break;
      case UiInput::EV_TRIG0: buttonFire(0); break;
      case UiInput::EV_TRIG1: buttonFire(1); break;
      default: break;
    }
  }

  uint32_t now = millis();
  if (now - lastRender < SJ_UI_MIN_MS) return;
  compose();
  if (!changed()) return;
  lastRender = now;
  commitPrev();
#if SJ_USE_DISPLAY
  if (framebufferOk) paint();
#endif
}

void dumpScreen() {
#if SJ_USE_DISPLAY
  if (!framebufferOk) { Serial.println("ERR SCREEN: no framebuffer"); return; }
  // An explicit, human-initiated command, so one 20 ms bus timeout is
  // irrelevant — and this is how a panel plugged in after boot gets picked up.
  panelPresent = probePanel();
  compose();
  commitPrev();
  paint();
  const uint8_t* b = display.getBuffer();
  Serial.printf("# SCREEN %dx%d\n", SJ_OLED_WIDTH, SJ_OLED_HEIGHT);
  Serial.printf("# page %u/%u %s, panel %s\n", (unsigned)page, (unsigned)pageCount(),
                pageName(page), panelPresent ? "present" : "absent");
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
