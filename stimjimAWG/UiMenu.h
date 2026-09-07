//    stimjimAWG — UiMenu: non-blocking status/result display for the SSD1306,
//    loop-only, redrawn at most every SJ_UI_MIN_MS and only when the rendered
//    text actually changed. One flat page list, advanced by the page button and
//    by the `PAGE` command, wrapping at the end:
//
//      STATUS   a running train's progress, or -- when both engines are idle --
//               what each trigger button would fire and what the log is doing.
//      RESULT   the last completed train, one page per measurement point (at
//               most SJ_UI_RESULT_PAGES of them): V, I and the load resistance
//               per channel. A completion that measured something jumps here.
//      SYSTEM   board, firmware, card and panel presence, the clock and its
//               source, uptime.
//
//    The panel is optional. `begin()` probes the I2C bus itself, because
//    Adafruit_SSD1306::begin() never does and would otherwise spend up to a
//    third of a second per frame in bus timeouts with nothing connected. With no
//    panel the framebuffer is still composed and `SCREEN` still works, so a
//    headless board is fully reviewable over the serial port.
//
//    Buttons never edit a slot (there is no editing FSM); Btn1 and Btn2 start
//    the two trigger routes, which is a start request like any other.
//
//    GPL-3.0-or-later; see Config.h header.

#ifndef STIMJIMAWG_UIMENU_H
#define STIMJIMAWG_UIMENU_H

#include <stdint.h>

namespace UiMenu {
void begin();
void tick();          // drain UiInput events, re-render if changed (loop only)

// `SCREEN`: render now and print the framebuffer as ASCII art, one line per
// pixel row, ending with OK. The panel is small and unphotographable over a
// serial link, so this is how display changes get reviewed and regression-
// checked — the captures in docs/progress/ are its raw output. It re-probes the
// bus first, so a panel plugged in after boot is picked up by sending `SCREEN`.
void dumpScreen();

// ------------------------------------------------------------ `PAGE` backend
// Every entry point prints one `PAGE,<index>,<count>,<name>` status line.
void    pageStatus();
void    pageNext();                  // same action as the page button
void    pageSelect(uint8_t idx);     // out of range: ERR, page unchanged
uint8_t pageCount();

// Hand over a finished train's numbers, from the completion drain in loop()
// context and immediately *before* Measure::printSummary — which is what
// releases the buffer the snapshot reads. A train that measured something jumps
// the panel to its first result page; one that measured nothing leaves the page
// alone.
void noteResult(uint32_t trainNo, uint8_t eng, uint8_t slot);
}

#endif // STIMJIMAWG_UIMENU_H
