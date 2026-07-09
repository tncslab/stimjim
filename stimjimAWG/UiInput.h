//    stimjimAWG — UiInput: debounced button ISRs feeding a lock-free SPSC event
//    ring (plan §5). Buttons are menu navigation only: Btn0 = OK, Btn1 = prev,
//    Btn2 = next; BACK is synthesized from OK long-press by UiMenu. The event
//    set maps 1:1 onto the future rotary+back/ok module.
//    GPL-3.0-or-later; see Config.h header.

#ifndef STIMJIMAWG_UIINPUT_H
#define STIMJIMAWG_UIINPUT_H

#include <stdint.h>

namespace UiInput {

enum Event : uint8_t {
  EV_NONE = 0,
  EV_OK,        // Btn0 — future: OK / encoder click
  EV_PREV,      // Btn1 — future: encoder CCW
  EV_NEXT,      // Btn2 — future: encoder CW
  EV_BACK,      // synthesized (OK long-press); future: dedicated back button
};

void begin();
Event pop();    // EV_NONE when the ring is empty (loop context)

} // namespace UiInput

#endif // STIMJIMAWG_UIINPUT_H
