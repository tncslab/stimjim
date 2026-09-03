//    stimjimAWG — Triggers implementation: IN0/IN1 routing, edge ISRs and the
//    stimulus-marker output. GPL-3.0-or-later; see Config.h header.
//
//    An edge ISR does the whole arm-and-start in place (Engine::startTrain is
//    copy-on-arm and sets t0 = edge - CAL TRIGCOMP + CAL STARTLAT + the slot's
//    delay), so the trigger-to-output latency is a fixed constant rather than
//    something that depends on how busy loop() is, on how complex the train is,
//    or on how long interrupt entry took. The ISRs run at SJ_TRIG_PRIO (80),
//    below the players (64): a trigger can never delay a waveform already
//    playing, and a player always preempts the arming work.
//
//    Nothing here prints. A rejected trigger (channel busy) only bumps a
//    counter; poll() emits the WARN from loop context (plan §3.3).

#include "Triggers.h"
#include "Config.h"
#include "Engine.h"
#include "TrainStore.h"
#include "FastIO.h"

namespace Triggers {

static TriggerRoute routes[2];
static volatile uint32_t rejects = 0;
static uint32_t reportedRejects = 0;
static volatile uint8_t markerMask = 0;   // bit0 = IN0 drives a marker, bit1 = IN1

static const uint8_t PIN[2] = {IN0, IN1};

// Start one slot on one engine from ISR context, counting a refusal.
// `at` is the edge timestamp, so the delivered latency is measured from the
// edge and not from the end of the arm (Engine::startTrain).
static inline void fire(uint8_t eng, int8_t slot, uint64_t at) {
  if (slot < 0) return;
  char scratch[SJ_MSG_MAX];
  if (!Engine::startTrain(eng, (uint8_t)slot, TrainStore::slotConst((uint8_t)slot),
                          scratch, sizeof scratch, at))
    rejects++;
}

static void edge(uint8_t input) {
  // First thing in the ISR: the edge's own timebase reading. Everything after
  // it — dispatch, the arm's precomputation, a second engine's arm — is then
  // subtracted from the start latency instead of added to it, so two engines
  // started by one edge also share one t0 grid.
  const uint64_t at = FastIO::cycles64();
  const TriggerRoute& r = routes[input];
  if (r.mode == 1) {
    // joint: one slot, one engine — the slot's own modes decide which physical
    // channels it drives, so "both channels synchronized" is a property of the
    // waveform definition, not of the routing.
    fire(0, r.slot0, at);
  } else if (r.mode == 2) {
    // independent: slot0 on engine 0, slot1 on engine 1. Each refusal counts
    // separately, so a half-served trigger is visible in the reject count.
    fire(0, r.slot0, at);
    fire(1, r.slot1, at);
  }
}

static void trig0Isr() { edge(0); }
static void trig1Isr() { edge(1); }

// Apply a route to the hardware: detach whatever was there, then set the pin up
// for its new job. Called from loop context only.
static void applyRoute(uint8_t input) {
  const TriggerRoute& r = routes[input];
  uint8_t pin = PIN[input];
  detachInterrupt(pin);
  markerMask &= (uint8_t)~(1u << input);

  if (r.mode == 3) {
    // Output marker: the same pin becomes an output driven high during a
    // stimulus (legacy behaviour, protocol §3).
    pinMode(pin, OUTPUT);
    digitalWriteFast(pin, LOW);
    markerMask |= (uint8_t)(1u << input);
    return;
  }

  pinMode(pin, INPUT);
  if (r.mode == 1 || r.mode == 2) {
    attachInterrupt(pin, input ? trig1Isr : trig0Isr, r.edge ? FALLING : RISING);
#if SJ_MCU_KINETISK
    // IN0 = pin 22 (PTC1) and IN1 = pin 23 (PTC2) share IRQ_PORTC; the buttons
    // live on PORTA/PORTB, so this priority applies to the triggers alone.
    NVIC_SET_PRIORITY(IRQ_PORTC, SJ_TRIG_PRIO);
#endif
  }
}

void begin() {
  // legacy boot state: both triggers are output markers
  routes[0] = {3, -1, -1, 0};
  routes[1] = {3, -1, -1, 0};
  applyRoute(0);
  applyRoute(1);
}

void poll() {
  uint32_t n = rejects;
  if (n != reportedRejects) {
    Serial.printf("WARN trigger: %lu start(s) dropped, target engine or channel busy\n",
                  (unsigned long)(n - reportedRejects));
    reportedRejects = n;
  }
}

void marker(bool on) {
  uint8_t m = markerMask;
  if (m & 1) digitalWriteFast(IN0, on ? HIGH : LOW);
  if (m & 2) digitalWriteFast(IN1, on ? HIGH : LOW);
}

const TriggerRoute& route(uint8_t input) { return routes[input]; }

void setRoute(uint8_t input, const TriggerRoute& r) {
  // Detaching and reattaching an interrupt is not atomic against an edge that
  // arrives mid-change; the bus lock masks the trigger ISRs (priority 80) for
  // the few microseconds it takes.
  FastIO::busLock();
  routes[input] = r;
  applyRoute(input);
  FastIO::busUnlock();
}

const char* validateRoute(const TriggerRoute& r) {
  if (r.mode > 3)  return "mode must be 0 (off), 1 (joint), 2 (independent) or 3 (marker)";
  if (r.edge > 1)  return "edge must be 0 (rising) or 1 (falling)";
  if (r.slot0 >= (int8_t)SJ_NUM_SLOTS || r.slot1 >= (int8_t)SJ_NUM_SLOTS ||
      r.slot0 < -1 || r.slot1 < -1)
    return "slots must be -1 (none) or a valid slot index";
  if (r.mode == 1 && r.slot0 < 0) return "joint mode needs a slot in slot0";
  if (r.mode == 1 && r.slot1 != -1) return "joint mode drives one slot — slot1 must be -1";
  if (r.mode == 2 && r.slot0 < 0 && r.slot1 < 0)
    return "independent mode needs at least one slot";
  if (r.mode == 3 && (r.slot0 != -1 || r.slot1 != -1))
    return "marker mode drives no train — both slots must be -1";
  if (r.mode == 0 && (r.slot0 != -1 || r.slot1 != -1))
    return "disabled mode takes no slots — both must be -1";
  return nullptr;
}

uint32_t rejectCount() { return rejects; }

} // namespace Triggers
