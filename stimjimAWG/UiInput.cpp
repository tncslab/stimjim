//    stimjimAWG — UiInput implementation. GPL-3.0-or-later; see Config.h header.

#include "UiInput.h"
#include "Config.h"

namespace UiInput {

// SPSC ring: button ISRs (producers, mutually exclusive at same NVIC priority)
// push, loop pops. Power-of-two size, head/tail free-running.
static volatile uint8_t ring[16];
static volatile uint8_t head = 0, tail = 0;

static void push(Event e) {
  uint8_t h = head;
  if ((uint8_t)(h - tail) >= sizeof(ring)) return;   // full: drop (UI events are lossy-ok)
  ring[h % sizeof(ring)] = e;
  head = h + 1;
}

Event pop() {
  if (tail == head) return EV_NONE;
  Event e = (Event)ring[tail % sizeof(ring)];
  tail = tail + 1;
  return e;
}

// 25 ms last-edge debounce per input, done entirely in the ISR (~1 us body).
static volatile uint32_t lastMs[3];

template <uint8_t IDX, Event EV>
static void btnIsr() {
  uint32_t now = millis();
  if (now - lastMs[IDX] < SJ_BTN_DEBOUNCE_MS) return;
  lastMs[IDX] = now;
  push(EV);
}

void begin() {
  pinMode(SJ_BTN_OK,   INPUT);   // board has external pulldowns; buttons pull high
  pinMode(SJ_BTN_PREV, INPUT);
  pinMode(SJ_BTN_NEXT, INPUT);
  attachInterrupt(SJ_BTN_OK,   btnIsr<0, EV_OK>,   RISING);
  attachInterrupt(SJ_BTN_PREV, btnIsr<1, EV_PREV>, RISING);
  attachInterrupt(SJ_BTN_NEXT, btnIsr<2, EV_NEXT>, RISING);
}

} // namespace UiInput
