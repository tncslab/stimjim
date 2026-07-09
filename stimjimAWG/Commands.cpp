//    stimjimAWG — command handlers. Phase 1: IDN/HELP/STAT + the BENCH group;
//    legacy waveform commands are stubbed until Phase 2/3. GPL-3.0-or-later.

#include "Protocol.h"
#include "Config.h"
#include "FastIO.h"
#include "Engine.h"
#include <ctype.h>
#include <string.h>
#include <stdlib.h>

namespace Commands {

// ------------------------------------------------------------------ helpers

// Parse up to `maxN` comma-separated integers after the command word.
// Accepts an optional leading comma before each value. Returns count parsed.
static uint8_t parseLongs(const char* p, long* out, uint8_t maxN) {
  uint8_t n = 0;
  while (n < maxN) {
    while (*p == ',' || *p == ' ') p++;
    if (!*p) break;
    char* end;
    long v = strtol(p, &end, 10);
    if (end == p) break;
    out[n++] = v;
    p = end;
  }
  return n;
}

static inline uint32_t cycToNs(uint32_t cyc) {          // 1 cycle = 25/3 ns
  return (cyc * 25u + 1u) / 3u;
}

static void printStat(const char* name, uint32_t n, uint32_t mn, uint64_t sum, uint32_t mx) {
  uint32_t avg = (uint32_t)(sum / n);
  Serial.printf("BENCH,%s,n=%lu,cycles(min/avg/max)=%lu/%lu/%lu,ns=%lu/%lu/%lu\n",
                name, (unsigned long)n,
                (unsigned long)mn, (unsigned long)avg, (unsigned long)mx,
                (unsigned long)cycToNs(mn), (unsigned long)cycToNs(avg),
                (unsigned long)cycToNs(mx));
}

// Time n invocations of op(i); print min/avg/max in cycles and ns.
template <typename F>
static void benchRun(const char* name, uint32_t n, F op) {
  uint32_t mn = UINT32_MAX, mx = 0;
  uint64_t sum = 0;
  for (uint32_t i = 0; i < n; i++) {
    uint32_t t0 = ARM_DWT_CYCCNT;
    op(i);
    uint32_t dt = ARM_DWT_CYCCNT - t0;
    if (dt < mn) mn = dt;
    if (dt > mx) mx = dt;
    sum += dt;
  }
  printStat(name, n, mn, sum, mx);
}

static void restoreDacOffsets(uint8_t mask) {
  if (mask & 1) FastIO::dacProgram(0, (int16_t)Stimjim.currentOffsets[0]);
  if (mask & 2) FastIO::dacProgram(1, (int16_t)Stimjim.currentOffsets[1]);
  FastIO::dacLatch(mask);
}

static void spinUntil(uint64_t deadlineCyc) {
  while ((int64_t)(FastIO::cycles64() - deadlineCyc) < 0) ;
}

static void ok()                              { Serial.println("OK"); }
static void err(const char* cmd, const char* msg) { Serial.printf("ERR %s: %s\n", cmd, msg); }

// -------------------------------------------------------------------- BENCH

static void benchList() {
  Serial.println("# BENCH group (Phase 1) — timing in CPU cycles (120/us) and ns");
  Serial.println("# BENCHDAC[,n]                 dacProgram single channel (no latch)");
  Serial.println("# BENCHDAC2[,n]                dacProgramBoth (no latch)");
  Serial.println("# BENCHLATCH[,n]               dacLatch(0b11) pulse");
  Serial.println("# BENCHADC[,ch,line,n]         adcRead, line pre-selected");
  Serial.println("# BENCHSW[,ch,n]               adcSelectLine+adcRead pair (line-switch cost)");
  Serial.println("# BENCHMISO[,n]                alternating ch0/ch1 reads (MISO mux swap)");
  Serial.println("# BENCHCYC[,n]                 cycles64() overhead");
  Serial.println("# BENCHK[,n]                   K_RELOAD recalibration, residual stats");
  Serial.println("# BENCHPIT,period_us,n[,preload_us]  PIT wake/latch jitter vs deadline");
  Serial.println("# BENCHSQ,ch,code,half_us,n    square wave, FastIO path (scope A/B)");
  Serial.println("# BENCHSQL,ch,code,half_us,n   square wave, legacy Stimjim.writeToDac path");
  ok();
}

static void benchPit(const char* args) {
  long v[3] = {0, 0, 0};
  uint8_t n = parseLongs(args, v, 3);
  if (n < 2) { err("BENCHPIT", "need period_us,n[,preload_us]"); return; }
  uint32_t period = (uint32_t)v[0], reps = (uint32_t)v[1];
  uint32_t preload = (n >= 3) ? (uint32_t)v[2] : 0;
  if (period < 20 || preload >= period) { err("BENCHPIT", "period >= 20 us and > preload"); return; }
  if (reps < 1 || (uint64_t)period * reps > 10000000ull) { err("BENCHPIT", "total run must be <= 10 s"); return; }

  Engine::PitBenchResult r;
  if (!Engine::benchPitLatency(period, reps, preload, &r)) {
    err("BENCHPIT", "bench already active or timed out");
    return;
  }
  int32_t avg = (int32_t)(r.sumErr / (int64_t)r.n);
  Serial.printf("BENCH,PIT,n=%lu,period=%luus,preload=%luus,err_cycles(min/avg/max)=%ld/%ld/%ld\n",
                (unsigned long)r.n, (unsigned long)period, (unsigned long)preload,
                (long)r.minErr, (long)avg, (long)r.maxErr);
  Serial.println("# histogram: bin0=early, then 0.5us bins of lateness");
  for (uint32_t b = 0; b < SJ_BENCH_BINS; b++)
    if (r.hist[b])
      Serial.printf("#   bin%lu: %lu\n", (unsigned long)b, (unsigned long)r.hist[b]);
  ok();
}

static void benchSquare(const char* args, bool legacyPath) {
  long v[4];
  if (parseLongs(args, v, 4) < 4) { err("BENCHSQ", "need ch,code,half_us,n"); return; }
  uint8_t  ch = (uint8_t)v[0];
  int16_t  code = (int16_t)v[1];
  uint32_t half = (uint32_t)v[2], n = (uint32_t)v[3];
  if (ch > 1) { err("BENCHSQ", "ch must be 0/1"); return; }
  if (half < 5 || (uint64_t)half * n * 2 > 10000000ull) { err("BENCHSQ", "5 us <= half_us, total <= 10 s"); return; }
  Serial.println("WARN BENCHSQ: drives the DAC — make sure the output mode is safe (grounded)");

  if (legacyPath) {
    for (uint32_t i = 0; i < n; i++) {
      Stimjim.writeToDac(ch, code);
      delayMicroseconds(half);
      Stimjim.writeToDac(ch, (int16_t)-code);
      delayMicroseconds(half);
    }
    FastIO::acquireBus();               // legacy SPI transactions clobbered the CTARs
    restoreDacOffsets(1 << ch);
  } else {
    uint64_t t = FastIO::cycles64();
    for (uint32_t i = 0; i < n; i++) {
      FastIO::dacProgram(ch, code);
      FastIO::dacLatch(1 << ch);
      t += SJ_US_TO_CYC(half);
      spinUntil(t);
      FastIO::dacProgram(ch, (int16_t)-code);
      FastIO::dacLatch(1 << ch);
      t += SJ_US_TO_CYC(half);
      spinUntil(t);
    }
    restoreDacOffsets(1 << ch);
  }
  ok();
}

// sub = command word after "BENCH", args = rest of line
static void benchDispatch(const char* sub, const char* args) {
  FastIO::acquireBus();   // benches assume our CTAR config regardless of history

  if (!*sub) { benchList(); return; }   // "BENCH" and "BENCH?" both list

  long v[4];
  if (!strcmp(sub, "DAC")) {
    uint32_t n = parseLongs(args, v, 1) ? (uint32_t)v[0] : 1000;
    int16_t c = (int16_t)Stimjim.currentOffsets[0];
    benchRun("DAC", n, [c](uint32_t) { FastIO::dacProgram(0, c); });   // no latch: output unchanged
    ok();
  } else if (!strcmp(sub, "DAC2")) {
    uint32_t n = parseLongs(args, v, 1) ? (uint32_t)v[0] : 1000;
    int16_t c0 = (int16_t)Stimjim.currentOffsets[0];
    int16_t c1 = (int16_t)Stimjim.currentOffsets[1];
    benchRun("DAC2", n, [c0, c1](uint32_t) { FastIO::dacProgramBoth(c0, c1); });
    ok();
  } else if (!strcmp(sub, "LATCH")) {
    uint32_t n = parseLongs(args, v, 1) ? (uint32_t)v[0] : 1000;
    restoreDacOffsets(0b11);            // make sure latching only re-asserts the offsets
    benchRun("LATCH", n, [](uint32_t) { FastIO::dacLatch(0b11); });
    ok();
  } else if (!strcmp(sub, "ADC")) {
    uint8_t np = parseLongs(args, v, 3);
    uint8_t ch = np >= 1 ? (uint8_t)v[0] : 0;
    uint8_t line = np >= 2 ? (uint8_t)v[1] : 0;
    uint32_t n = np >= 3 ? (uint32_t)v[2] : 1000;
    if (ch > 1 || line > 1) { err("BENCHADC", "ch,line must be 0/1"); return; }
    FastIO::adcSelectLine(ch, line);
    benchRun("ADC", n, [ch, line](uint32_t) { (void)FastIO::adcRead(ch, line); });
    ok();
  } else if (!strcmp(sub, "SW")) {
    uint8_t np = parseLongs(args, v, 2);
    uint8_t ch = np >= 1 ? (uint8_t)v[0] : 0;
    uint32_t n = np >= 2 ? (uint32_t)v[1] : 500;
    if (ch > 1) { err("BENCHSW", "ch must be 0/1"); return; }
    // alternate lines so every read pays the control-register switch
    benchRun("SW", n, [ch](uint32_t i) {
      uint8_t line = i & 1;
      FastIO::adcSelectLine(ch, line);
      (void)FastIO::adcRead(ch, line);
    });
    FastIO::adcSelectLine(ch, 0);
    ok();
  } else if (!strcmp(sub, "MISO")) {
    uint32_t n = parseLongs(args, v, 1) ? (uint32_t)v[0] : 1000;
    FastIO::adcSelectLine(0, 0);
    FastIO::adcSelectLine(1, 0);
    benchRun("MISO", n, [](uint32_t i) { (void)FastIO::adcRead(i & 1, 0); });
    ok();
  } else if (!strcmp(sub, "CYC")) {
    uint32_t n = parseLongs(args, v, 1) ? (uint32_t)v[0] : 10000;
    benchRun("CYC", n, [](uint32_t) { (void)FastIO::cycles64(); });
    ok();
  } else if (!strcmp(sub, "K")) {
    uint32_t n = parseLongs(args, v, 1) ? (uint32_t)v[0] : 65;
    int32_t mn, med, mx;
    Engine::calibrateKReload((uint16_t)n, &mn, &med, &mx);
    Serial.printf("BENCH,K,n=%lu,residual_cycles(min/med/max)=%ld/%ld/%ld,K_RELOAD=%lu\n",
                  (unsigned long)n, (long)mn, (long)med, (long)mx,
                  (unsigned long)Engine::kReloadCycles());
    ok();
  } else if (!strcmp(sub, "PIT")) {
    benchPit(args);
  } else if (!strcmp(sub, "SQ")) {
    benchSquare(args, false);
  } else if (!strcmp(sub, "SQL")) {
    benchSquare(args, true);
  } else {
    err("BENCH", "unknown benchmark — BENCH? lists them");
  }
}

// --------------------------------------------------------------------- HELP

static void help() {
  Serial.println("# stimjimAWG Phase 1 scaffold — implemented commands:");
  Serial.println("#   IDN            firmware identification line");
  Serial.println("#   STAT           engine status (both engines idle in Phase 1)");
  Serial.println("#   BENCH?         list hardware benchmarks (BENCHDAC, BENCHPIT, ...)");
  Serial.println("#   HELP or ?      this text");
  Serial.println("# Legacy commands (S/L/W/T/U/R/M/V/A/E/B/C/D/P) and ENV/MEAS/TRIG/DUMP/LOG");
  Serial.println("# arrive in Phase 2+ — see docs/serial-protocol.md. Until then use stimjimPulser.");
  ok();
}

// ------------------------------------------------------------------ dispatch

void handleLine(const char* line) {
  if (!strcmp(line, "?")) { help(); return; }

  size_t wl = 0;
  while (isalpha((unsigned char)line[wl])) wl++;
  if (wl == 0) { err("parse", "expected command word"); return; }
  const char* args = line + wl;

  if (wl == 1) {
    // Single-letter namespace = legacy + `L`; all deferred to Phase 2/3.
    static const char known[] = "SLWTURMVAEBCDP";
    if (strchr(known, line[0])) {
      Serial.printf("ERR %c: not implemented in Phase 1 scaffold (Phase 2+); "
                    "BENCH/IDN/HELP available\n", line[0]);
    } else {
      Serial.printf("ERR %c: unknown command\n", line[0]);
    }
    return;
  }

  char word[16];
  if (wl >= sizeof(word)) { err("parse", "unknown command"); return; }
  memcpy(word, line, wl);
  word[wl] = '\0';

  if (!strcmp(word, "IDN")) {
    Serial.printf("IDN,%s,%s,fw=%s,proto=%d\n",
                  SJ_FW_NAME, SJ_HW_NAME, SJ_FW_VERSION, SJ_PROTO_VERSION);
  } else if (!strcmp(word, "HELP")) {
    help();
  } else if (!strcmp(word, "STAT")) {
    Serial.println("STAT,-1,0,0,0,-1,0,0,0");   // both engines idle (Phase 1)
  } else if (!strncmp(word, "BENCH", 5)) {
    const char* sub = word + 5;
    // allow "BENCH?" — the '?' lands in args, sub is empty
    benchDispatch(sub, args);
  } else {
    Serial.printf("ERR %s: unknown command\n", word);
  }
}

} // namespace Commands
