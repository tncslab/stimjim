//    stimjimAWG — UiInput: armed-once button ISRs feeding a lock-free SPSC
//    event ring. Btn0 switches the display page; Btn1 and Btn2 fire the two
//    trigger inputs' routes, which is what stimjimPulser wired them to
//    (stimjimPulser.ino:888-896) and what the `// TODO: protection against
//    rolling buttons` above those lines asked for.
//
//    Bounce is suppressed by arming rather than by a lockout. The ISR fires only
//    while its button is armed, pushes one event and disarms it; poll() re-arms
//    a button once its pin has read low continuously for SJ_BTN_DEBOUNCE_MS. One
//    press therefore yields exactly one event whatever the contact does on make
//    *or* break — a 25 ms lockout from the last accepted edge suppressed only
//    the first of those, so a switch held for 300 ms and released with a bouncy
//    break used to start a second train. A stalled loop() only delays re-arming,
//    which is the safe direction.
//
//    GPL-3.0-or-later; see Config.h header.

#ifndef STIMJIMAWG_UIINPUT_H
#define STIMJIMAWG_UIINPUT_H

#include <stdint.h>

namespace UiInput {

enum Event : uint8_t {
  EV_NONE  = 0,
  EV_PAGE,      // Btn0 — next display page; future: encoder click
  EV_TRIG0,     // Btn1 — fire input 0's route
  EV_TRIG1,     // Btn2 — fire input 1's route
};

void begin();
void poll();    // re-arm released buttons (loop context; three pin reads)
Event pop();    // EV_NONE when the ring is empty (loop context)

} // namespace UiInput

#endif // STIMJIMAWG_UIINPUT_H
