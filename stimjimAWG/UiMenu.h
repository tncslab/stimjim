//    stimjimAWG — UiMenu: non-blocking status/browse display for the SSD1306,
//    loop-only, redrawn at most every SJ_UI_MIN_MS and only when the rendered
//    text actually changed (plan §5). Two views:
//      BROWSE  — no train running: the slot under the cursor, scrolled with
//                Btn1/Btn2, so the panel says what the box would do if started.
//      RUN     — a train is playing (or waiting out its start delay): which
//                slot on which engine, a progress bar and the pulse count.
//    There is no editing FSM yet (HOME -> SELECT -> ARMED -> RESULT is planned);
//    buttons never start or stop anything (plan §2 decision 7).
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
// checked — the captures in docs/progress/ are its raw output.
void dumpScreen();

uint8_t browseSlot();          // slot the cursor sits on (BROWSE view)
}

#endif // STIMJIMAWG_UIMENU_H
