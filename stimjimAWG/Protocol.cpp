//    stimjimAWG — Protocol implementation (protocol §1). GPL-3.0-or-later.

#include "Protocol.h"
#include "Config.h"
#include "Engine.h"

namespace Protocol {

static char     lineBuf[SJ_LINE_MAX + 1];
static uint16_t lineLen    = 0;
static bool     discarding = false;   // overflow: swallow until next '\n'

void begin() {
  Serial.printf("# %s %s (Phase 2) — waveform definition + queries live; "
                "HELP lists commands; T/U start arrives in Phase 3\n",
                SJ_FW_NAME, SJ_FW_VERSION);
  Serial.printf("IDN,%s,%s,fw=%s,proto=%d\n",
                SJ_FW_NAME, SJ_HW_NAME, SJ_FW_VERSION, SJ_PROTO_VERSION);
  Serial.printf("# engine: PIT channels %u/%u, K_RELOAD=%lu cycles\n",
                Engine::pitChannelOf(0), Engine::pitChannelOf(1),
                (unsigned long)Engine::kReloadCycles());
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
