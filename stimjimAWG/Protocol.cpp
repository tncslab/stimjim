//    stimjimAWG — Protocol implementation (protocol §1). GPL-3.0-or-later.

#include "Protocol.h"
#include "Config.h"
#include "Engine.h"

namespace Protocol {

static char     lineBuf[SJ_LINE_MAX + 1];
static uint16_t lineLen    = 0;
static bool     discarding = false;   // overflow: swallow until next '\n'

// Identity + build configuration. Printed at boot and by `IDN`, so a session
// that attached after boot can still ask which backends this binary uses —
// that decides whether the Config.h timing constants apply as written.
// Everything after the IDN line is a `#` comment, which protocol §1 tells
// machine parsers to skip.
void printIdentity() {
  Serial.printf("IDN,%s,%s,fw=%s,proto=%d\n",
                SJ_FW_NAME, SJ_HW_NAME, SJ_FW_VERSION, SJ_PROTO_VERSION);
  Serial.printf("# build: %s, F_CPU=%lu MHz, fastio=%s, timer=%s\n",
                SJ_HW_NAME, (unsigned long)(F_CPU / 1000000),
                SJ_FASTIO_REGISTER ? "registers" : "Arduino-SPI",
                SJ_TIMER_REGISTER  ? "raw-PIT"   : "IntervalTimer");
  if (Engine::pitChannelOf(0) == 0xFE)
    Serial.printf("# engine: IntervalTimer-managed channels, K_RELOAD=%lu cycles\n",
                  (unsigned long)Engine::kReloadCycles());
  else
    Serial.printf("# engine: PIT channels %u/%u, K_RELOAD=%lu cycles\n",
                  Engine::pitChannelOf(0), Engine::pitChannelOf(1),
                  (unsigned long)Engine::kReloadCycles());
}

void begin() {
  Serial.printf("# %s %s - S/L/W waveform slots, trigger routing and T/U playback "
                "with a per-slot start delay; HELP lists commands\n",
                SJ_FW_NAME, SJ_FW_VERSION);
  printIdentity();
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
