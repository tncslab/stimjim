//    stimjimAWG — UiMenu: non-blocking menu FSM + SSD1306 rendering, loop-only,
//    dirty-flag with >= 50 ms render throttle (plan §5). The FSM
//    (HOME -> SELECT -> ARMED -> RUNNING -> RESULT) arrives in Phase 8;
//    Phase 1 shows a splash and proves the input chain.
//    GPL-3.0-or-later; see Config.h header.

#ifndef STIMJIMAWG_UIMENU_H
#define STIMJIMAWG_UIMENU_H

namespace UiMenu {
void begin();
void tick();   // drain UiInput events, render if dirty (loop context only)
}

#endif // STIMJIMAWG_UIMENU_H
