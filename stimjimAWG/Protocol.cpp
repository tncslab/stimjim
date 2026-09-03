//    stimjimAWG — Protocol implementation (protocol §1). GPL-3.0-or-later.

#include "Protocol.h"
#include "Config.h"
#include "Engine.h"
#include "Cal.h"

namespace Protocol {

static char     lineBuf[SJ_LINE_MAX + 1];
static uint16_t lineLen    = 0;
static bool     discarding = false;   // overflow: swallow until next '\n'

// Identity + build configuration. Printed at boot and by `IDN`, so a session
// that attached after boot can still ask which backends this binary uses —
// that decides whether the Config.h timing constants apply as written.
// Everything after the IDN line is a `#` comment, which protocol §1 tells
// machine parsers to skip.
void printIdentity(Print& out) {
  out.printf("IDN,%s,%s,fw=%s,proto=%d\n",
             SJ_FW_NAME, SJ_HW_NAME, SJ_FW_VERSION, SJ_PROTO_VERSION);
  out.printf("# build: %s, F_CPU=%lu MHz, fastio=%s, timer=%s, sd=%s\n",
             SJ_HW_NAME, (unsigned long)(F_CPU / 1000000),
             SJ_FASTIO_REGISTER ? "registers" : "Arduino-SPI",
             SJ_TIMER_REGISTER  ? "raw-PIT"   : "IntervalTimer",
             SJ_USE_SD ? "yes" : "no");
  // `cal` says whether the timing budgets are still this build's defaults;
  // a board calibrated by hand answers `custom`, and `CAL?` prints the values.
  const char* calState = Cal::isDefault() ? "default" : "custom";
  if (Engine::pitChannelOf(0) == 0xFE)
    out.printf("# engine: IntervalTimer-managed channels, K_RELOAD=%lu cycles, cal=%s\n",
               (unsigned long)Engine::kReloadCycles(), calState);
  else
    out.printf("# engine: PIT channels %u/%u, K_RELOAD=%lu cycles, cal=%s\n",
               Engine::pitChannelOf(0), Engine::pitChannelOf(1),
               (unsigned long)Engine::kReloadCycles(), calState);
}

// Digits produced back to front, so no 64-bit division-by-10 loop appears in
// any printf format string.
void u64str(uint64_t v, char* buf) {
  char tmp[21];
  uint8_t n = 0;
  do { tmp[n++] = (char)('0' + (v % 10u)); v /= 10u; } while (v);
  for (uint8_t i = 0; i < n; i++) buf[i] = tmp[n - 1 - i];
  buf[n] = '\0';
}

void begin() {
  Serial.printf("# %s %s - S/L/W waveform slots, trigger routing and T/U playback "
                "with a per-slot start delay; HELP lists commands\n",
                SJ_FW_NAME, SJ_FW_VERSION);
  printIdentity(Serial);
}

void poll() {
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n') {
      if (discarding) {
        discarding = false;
        lineLen = 0;
        Serial.printf("ERR line too long (max %d)\n", SJ_LINE_MAX);
        continue;
      }
      while (lineLen && (lineBuf[lineLen - 1] == '\r' || lineBuf[lineLen - 1] == ' '))
        lineLen--;
      lineBuf[lineLen] = '\0';
      if (lineLen && lineBuf[0] != '#')
        Commands::handleLine(lineBuf);
      lineLen = 0;
    } else if (!discarding) {
      if (lineLen >= SJ_LINE_MAX) discarding = true;
      else                        lineBuf[lineLen++] = c;
    }
  }
}

} // namespace Protocol
