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

static const uint8_t PIN[3] = {SJ_BTN_PAGE, SJ_BTN_TRIG0, SJ_BTN_TRIG1};

// Set here, cleared in the ISR: a disarmed button generates nothing at all,
// which is what makes the whole of a bouncing make *and* break one event.
static volatile bool armed[3] = {true, true, true};
// When poll() last saw the pin high. Only poll() writes it, so no barrier.
static uint32_t lastHighMs[3] = {0, 0, 0};

// Whole ISR body: a load, a store and a ring push.
template <uint8_t IDX, Event EV>
static void btnIsr() {
  if (!armed[IDX]) return;
  armed[IDX] = false;
  push(EV);
}

void begin() {
  for (uint8_t i = 0; i < 3; i++) pinMode(PIN[i], INPUT);   // board has external pulldowns
  attachInterrupt(SJ_BTN_PAGE,  btnIsr<0, EV_PAGE>,  RISING);
  attachInterrupt(SJ_BTN_TRIG0, btnIsr<1, EV_TRIG0>, RISING);
  attachInterrupt(SJ_BTN_TRIG1, btnIsr<2, EV_TRIG1>, RISING);
}

void poll() {
  const uint32_t now = millis();
  for (uint8_t i = 0; i < 3; i++) {
    if (armed[i]) continue;
    // digitalReadFast falls back to digitalRead for a non-constant pin, which
    // is what a loop over the table gets; three of those per pass is nothing
    // next to the render this shares loop() with.
    if (digitalReadFast(PIN[i])) { lastHighMs[i] = now; continue; }
    if ((uint32_t)(now - lastHighMs[i]) >= SJ_BTN_DEBOUNCE_MS) armed[i] = true;
  }
}

} // namespace UiInput
