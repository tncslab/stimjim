//    stimjimAWG — host-side tests for TrainStore: S/L/W body parser, ENV/MEAS
//    validation, round-trip serializers (docs/serial-protocol.md §2/§4).
//    No Arduino dependencies — build & run on the development machine:
//
//      g++ -std=c++17 -Wall -Wextra -I stimjimAWG tests/host/test_trainstore.cpp
//          stimjimAWG/TrainStore.cpp -o test_trainstore     (one command line)
//      ./test_trainstore        (exit code = number of failed checks)
//
//    GPL-3.0-or-later; see stimjimAWG/Config.h header.

#include "TrainStore.h"
#include <cstdio>
#include <cstring>

static int failures = 0;

#define CHECK(cond) do { \
    if (!(cond)) { failures++; printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond); } \
  } while (0)

#define CHECK_STREQ(a, b) do { \
    if (strcmp((a), (b)) != 0) { \
      failures++; printf("FAIL %s:%d  \"%s\" != \"%s\"\n", __FILE__, __LINE__, (a), (b)); } \
  } while (0)

// Convenience wrapper: parse `body` (everything after the slot index) against
// a default current slot unless one is supplied.
static bool parse(char letter, const char* body, TrainDef& staged,
                  char* err, char* warn, const TrainDef* current = nullptr) {
  TrainDef cur;
  if (current) cur = *current;
  else         TrainStore::slotDefault(cur);
  err[0] = warn[0] = '\0';
  return TrainStore::parseTrainBody(letter, body, cur, staged,
                                    err, SJ_MSG_MAX, warn, SJ_MSG_MAX);
}

int main() {
  TrainStore::begin();
  TrainDef t;
  char err[SJ_MSG_MAX], warn[SJ_MSG_MAX], line[SJ_SERIALIZE_MAX], line2[SJ_SERIALIZE_MAX];

  // ---------------------------------------------------- README example line
  // "S0,0,1,2000,1000000; 100,0,150; -100,-100,200"
  CHECK(parse('S', ",0,1,2000,1000000; 100,0,150; -100,-100,200", t, err, warn));
  CHECK(t.type == PIECEWISE_HOLD && t.mode0 == 0 && t.mode1 == 1);
  CHECK(t.period_us == 2000 && t.duration_us == 1000000);
  CHECK(t.nStages == 2);
  CHECK(t.stages[0].a0 == 100 && t.stages[0].a1 == 0 && t.stages[0].dur_us == 150);
  CHECK(t.stages[1].a0 == -100 && t.stages[1].a1 == -100 && t.stages[1].dur_us == 200);
  CHECK(warn[0] == '\0');
  TrainStore::serializeTrain(0, t, line, sizeof line);
  CHECK_STREQ(line, "S0,0,1,2000,1000000;100,0,150;-100,-100,200");
  // canonical form must re-parse to the identical canonical form
  CHECK(parse('S', line + 2, t, err, warn));   // skip "S0"
  TrainStore::serializeTrain(0, t, line2, sizeof line2);
  CHECK_STREQ(line, line2);

  // -------------------------------------- stimjimPulser header example lines
  // Fonoff: exactly 3000 uA in current mode — at the limit, no warning
  CHECK(parse('S', ",1,1,10000,1000000;800,3000,100;-800,-3000,100", t, err, warn));
  CHECK(warn[0] == '\0');
  // iTBS: 5 stages. NOTE: the lab header used modes 3,3 meaning "current, no
  // measurement" — under the restored original numbering 3 = not driven
  // (protocol §6.3); the line still parses, the 90/91 replay is below.
  CHECK(parse('S', ",3,3,200000,2000000;800,2000,280;0,0,19720;800,2000,280;0,0,19720;800,2000,280", t, err, warn));
  CHECK(t.nStages == 5);
  CHECK(t.mode0 == 3 && t.mode1 == 3);

  // ------------------------------------------- 90/91: V/I without measurement
  // iTBS as the lab intended it: current output, measurement off
  CHECK(parse('S', ",91,91,200000,2000000;800,2000,280;0,0,19720;800,2000,280;0,0,19720;800,2000,280", t, err, warn));
  CHECK(t.mode0 == 1 && t.mode1 == 1);                 // stored as plain current mode
  CHECK(t.meas.what0 == 0 && t.meas.what1 == 0);        // measurement disabled
  TrainStore::serializeTrain(3, t, line, sizeof line);  // ... and rendered back as 91
  CHECK(strncmp(line, "S3,91,91,", 9) == 0);
  CHECK(parse('S', line + 2, t, err, warn));            // canonical form round-trips
  TrainStore::serializeTrain(3, t, line2, sizeof line2);
  CHECK_STREQ(line, line2);
  // plain 0/1 re-enables measurement: a stored what=0 promotes back to 3 ...
  TrainDef noMeas = t;
  CHECK(parse('S', ",0,1,10000,500000;100,100,50", t, err, warn, &noMeas));
  CHECK(t.meas.what0 == 3 && t.meas.what1 == 3);
  // ... but an explicit MEAS refinement (what=1/2) is preserved
  noMeas.meas.what0 = 2;
  CHECK(parse('S', ",0,1,10000,500000;100,100,50", t, err, warn, &noMeas));
  CHECK(t.meas.what0 == 2 && t.meas.what1 == 3);
  // mixed: ch0 measured voltage, ch1 unmeasured current
  CHECK(parse('S', ",0,91,10000,500000;100,100,50", t, err, warn));
  CHECK(t.mode0 == 0 && t.mode1 == 1);
  CHECK(t.meas.what0 == 3 && t.meas.what1 == 0);
  TrainStore::serializeTrain(4, t, line, sizeof line);
  CHECK(strncmp(line, "S4,0,91,", 8) == 0);

  // legacy W line — but with the header's negative-frequency typo: rejected
  CHECK(!parse('W', ",1,1,100000,300000; 100,-100,10000; -500,150,0; 0,0,0", t, err, warn));
  CHECK(strstr(err, "negative frequency") != nullptr);
  // corrected sign: parses, stores mHz, envelope resets to 0,0,0
  CHECK(parse('W', ",1,1,100000,300000; 100,-100,10000; 500,150,0; 0,0,0", t, err, warn));
  CHECK(t.type == SINE);
  CHECK(t.sine.amp0 == 100 && t.sine.amp1 == -100 && t.sine.burst_us == 10000);
  CHECK(t.sine.freq0_mHz == 500000 && t.sine.freq1_mHz == 150000);
  CHECK(t.sine.phase0_mdeg == 0 && t.sine.phase1_mdeg == 0);
  CHECK(t.env.rampIn_us == 0 && t.env.rampOut_us == 0 && t.env.shape == 0);
  TrainStore::serializeTrain(1, t, line, sizeof line);
  CHECK_STREQ(line, "W1,1,1,100000,300000;100,-100,10000;500,150,0;0,0,0;0,0,0");

  // ------------------------------------------------ decimals and W envelope
  CHECK(parse('W', ",0,0,1000000,2000000;1000,0,500000;12.5,0.25,0;90,-0.25,0;1000,2000,0", t, err, warn));
  CHECK(t.sine.freq0_mHz == 12500 && t.sine.freq1_mHz == 250);
  CHECK(t.sine.phase0_mdeg == 90000 && t.sine.phase1_mdeg == -250);
  CHECK(t.env.rampIn_us == 1000 && t.env.rampOut_us == 2000);
  TrainStore::serializeTrain(2, t, line, sizeof line);
  CHECK_STREQ(line, "W2,0,0,1000000,2000000;1000,0,500000;12.5,0.25,0;90,-0.25,0;1000,2000,0");
  CHECK(parse('W', line + 2, t, err, warn));
  TrainStore::serializeTrain(2, t, line2, sizeof line2);
  CHECK_STREQ(line, line2);

  // trailing zeros trimmed, sub-milli rejected
  char mbuf[16];
  TrainStore::milliToStr(500000, mbuf);  CHECK_STREQ(mbuf, "500");
  TrainStore::milliToStr(250, mbuf);     CHECK_STREQ(mbuf, "0.25");
  TrainStore::milliToStr(-250, mbuf);    CHECK_STREQ(mbuf, "-0.25");
  TrainStore::milliToStr(-90000, mbuf);  CHECK_STREQ(mbuf, "-90");
  TrainStore::milliToStr(12100, mbuf);   CHECK_STREQ(mbuf, "12.1");
  CHECK(!parse('W', ",0,0,1000,10000;1,1,10;1.0001,0,0;0,0,0", t, err, warn));

  // ------------------------------------------------------- rejected inputs
  CHECK(!parse('S', ",6,0,1000,10000;1,1,10", t, err, warn));          // mode outside {0-3,90,91}
  CHECK(strstr(err, "mode") != nullptr);
  CHECK(!parse('S', ",4,0,1000,10000;1,1,10", t, err, warn));          // lab firmware's old hi-Z code
  CHECK(!parse('S', ",92,0,1000,10000;1,1,10", t, err, warn));
  CHECK(!parse('S', ",0,0,0,10000;1,1,10", t, err, warn));             // period 0
  CHECK(!parse('S', ",0,0,1000", t, err, warn));                       // missing duration
  CHECK(!parse('S', ",0,0,1000,10000;1,1", t, err, warn));             // incomplete triplet
  CHECK(!parse('S', ",0,0,1000,10000;1,1,10 x", t, err, warn));        // trailing garbage
  CHECK(!parse('S', ",0,0,1000,10000"
               ";1,1,1;1,1,1;1,1,1;1,1,1;1,1,1;1,1,1;1,1,1;1,1,1;1,1,1;1,1,1;1,1,1",
               t, err, warn));                                         // 11 stages
  CHECK(strstr(err, "stages") != nullptr);
  CHECK(!parse('W', ",0,0,1000,10000;1,1,10;5,5;0,0,0", t, err, warn));   // freq triplet 3rd field missing
  CHECK(!parse('W', ",0,0,1000,10000;1,1,10;5,5,0;0,0,0;100,100,1", t, err, warn)); // shape 1 reserved
  CHECK(!parse('W', ",0,0,1000,10000;1,1,10;5,5,0;0,0,0;6000,6000,0", t, err, warn)); // env > duration
  CHECK(!parse('S', ",0,0,1000,-5;1,1,10", t, err, warn));             // negative duration

  // 0 stages stays legal (legacy "empty train" semantics + boot default)
  CHECK(parse('S', ",3,3,10000,500000", t, err, warn));
  CHECK(t.nStages == 0);
  CHECK(TrainStore::isDefaultTrain(t));
  // trailing semicolon tolerated like the legacy strtok parser
  CHECK(parse('S', ",0,1,2000,1000000;100,0,150;", t, err, warn));
  CHECK(t.nStages == 1);

  // --------------------------------------------------------------- warnings
  CHECK(parse('S', ",1,1,10000,1000000;800,3500,100", t, err, warn));  // >3000 uA, current mode
  CHECK(strstr(warn, "3000 uA") != nullptr);
  CHECK(parse('S', ",0,0,10000,1000000;20000,0,100", t, err, warn));   // voltage beyond full scale
  CHECK(strstr(warn, "full scale") != nullptr);
  CHECK(parse('S', ",0,0,100,10000;1,1,80;1,1,80", t, err, warn));     // stages exceed period
  CHECK(strstr(warn, "exceed") != nullptr);
  CHECK(parse('W', ",0,0,1000,10000;1,1,2000;5,5,0;0,0,0", t, err, warn)); // burst > period
  CHECK(strstr(warn, "burst") != nullptr);

  // ------------------------------------ ENV/MEAS preservation and coercion
  TrainDef cur;
  TrainStore::slotDefault(cur);
  cur.env = {100, 100, 0};
  cur.meas = {1, 2, 0, 0, 2, 0};   // what0=V, what1=I, when=stage end, stage 0 only, report=SD
  CHECK(parse('L', ",0,0,1000,10000;500,500,400", t, err, warn, &cur));
  CHECK(t.type == PIECEWISE_RAMP);
  CHECK(t.env.rampIn_us == 100 && t.env.rampOut_us == 100);            // ENV preserved for S/L
  CHECK(t.meas.what0 == 1 && t.meas.what1 == 2 && t.meas.when == 0);
  CHECK(t.meas.stage == 0 && t.meas.report == 2);
  // type change to sine: when auto-coerces to 3 (both peaks), stage to -1,
  // and the W line resets the envelope
  CHECK(parse('W', ",0,0,1000,10000;1,1,500;5,5,0;0,0,0", t, err, warn, &cur));
  CHECK(t.meas.when == 3 && t.meas.stage == -1);
  CHECK(t.env.rampIn_us == 0 && t.env.rampOut_us == 0);
  // an explicit peak choice survives a same-family redefinition ...
  TrainDef curW = t;
  curW.meas.when = 2;                                    // -peak only
  CHECK(parse('W', ",0,0,1000,10000;1,1,500;5,5,0;0,0,0", t, err, warn, &curW));
  CHECK(t.meas.when == 2);
  // ... and back to S: sine-peak codes are invalid for S/L, auto-coerces to 0
  CHECK(parse('S', ",0,0,1000,10000;1,1,10", t, err, warn, &curW));
  CHECK(t.meas.when == 0);
  // preserved per-stage selection must fit a shortened stage list
  cur.meas.stage = 1;
  CHECK(parse('S', ",0,0,1000,10000;1,1,10;2,2,10", t, err, warn, &cur));   // 2 stages: fits
  CHECK(t.meas.stage == 1);
  CHECK(!parse('S', ",0,0,1000,10000;1,1,10", t, err, warn, &cur));         // 1 stage: stage 1 gone
  CHECK(strstr(err, "MEAS stage") != nullptr);
  cur.meas.stage = -1;
  // preserved ENV no longer fitting a shortened duration fails loudly
  cur.env = {400, 400, 0};
  CHECK(!parse('S', ",0,0,1000,500;1,1,10", t, err, warn, &cur));
  CHECK(strstr(err, "ENV") != nullptr);

  // ---------------------------------------------------- validateEnv / Meas
  TrainStore::slotDefault(cur);                    // duration 500000
  CHECK(TrainStore::validateEnv(cur, {250000, 250000, 0}) == nullptr);
  CHECK(TrainStore::validateEnv(cur, {250001, 250000, 0}) != nullptr);
  CHECK(TrainStore::validateEnv(cur, {0, 0, 1}) != nullptr);           // shape reserved

  cur.mode0 = cur.mode1 = 0;              // driven modes (the default 3/3 has nothing to measure)
  CHECK(TrainStore::validateMeas(cur, {3, 3, 0, -1, 0, 0}, warn, sizeof warn) == nullptr);
  CHECK(warn[0] == '\0');
  CHECK(TrainStore::validateMeas(cur, {4, 3, 0, -1, 0, 0}, warn, sizeof warn) != nullptr);  // what > 3
  CHECK(TrainStore::validateMeas(cur, {3, 3, 1, -1, 0, 0}, warn, sizeof warn) != nullptr);  // sine-peak code on S slot
  CHECK(TrainStore::validateMeas(cur, {3, 3, 0, -1, 8, 0}, warn, sizeof warn) != nullptr);  // report > 7
  CHECK(TrainStore::validateMeas(cur, {3, 3, 0,  0, 0, 0}, warn, sizeof warn) != nullptr);  // stage 0 on a 0-stage slot
  // All three report bits are implemented, so none of them warns.
  for (uint8_t r = 0; r <= SJ_REPORT_MAX; r++) {
    CHECK(TrainStore::validateMeas(cur, {3, 3, 0, -1, r, 0}, warn, sizeof warn) == nullptr);
    CHECK(warn[0] == '\0');
  }

  // ------------------------------------------- default report (protocol §2/§4)
  //
  // A channel that is both driven and measured writes its summary to the card
  // without being asked; 90/91 (V/I, measurement off) ask for nothing, and so
  // does an undriven channel.
  TrainDef dr;
  TrainStore::slotDefault(dr);                       // modes 3/3: not driven
  CHECK(!TrainStore::isMeasured(dr));
  CHECK(TrainStore::defaultReport(dr) == 0);
  dr.mode0 = 0;                                      // driven and measured (what0 = 3)
  CHECK(TrainStore::isMeasured(dr));
  CHECK(TrainStore::defaultReport(dr) == SJ_REPORT_SD_SUM);
  dr.meas.what0 = 0;                                 // what 90 stores
  CHECK(!TrainStore::isMeasured(dr));
  CHECK(TrainStore::defaultReport(dr) == 0);
  dr.mode1 = 1; dr.meas.what1 = 3;                   // the other channel carries it
  CHECK(TrainStore::isMeasured(dr));
  CHECK(TrainStore::defaultReport(dr) == SJ_REPORT_SD_SUM);
  // The promotion never clears a bit, so a slot switched back to undriven modes
  // keeps the report it was given -- and must then not count as "configured to
  // log", because it can write no rows. That is what isMeasured is for.
  dr.mode0 = dr.mode1 = 3;
  dr.meas.report = SJ_REPORT_SD_SUM;
  CHECK(!TrainStore::isMeasured(dr));
  CHECK((dr.meas.report & SJ_REPORT_SD) != 0);

  // The promotion happens at parse time, on the same rule normalizeMode uses
  // for `what`: an unset report takes the default, a deliberate one survives.
  TrainDef pr;
  TrainStore::slotDefault(pr);
  CHECK(parse('S', ",0,1,1000,10000;1,1,10", t, err, warn, &pr));
  CHECK(t.meas.report == SJ_REPORT_SD_SUM);          // plain 0/1 promotes
  CHECK(TrainStore::isDefaultMeas(t));               // ... and stays "default"
  CHECK(parse('S', ",90,91,1000,10000;1,1,10", t, err, warn, &pr));
  CHECK(t.meas.what0 == 0 && t.meas.what1 == 0);
  CHECK(t.meas.report == 0);                         // nothing measured, nothing logged
  // 90/91 is not a default MEAS and never was: `what` is 0 rather than 3, which
  // round-trips through the mode field of the S line and a MEAS line both.
  CHECK(!TrainStore::isDefaultMeas(t));
  CHECK(parse('S', ",2,3,1000,10000;1,1,10", t, err, warn, &pr));
  CHECK(t.meas.report == 0);                         // undriven channels ask for nothing
  pr.meas.report = SJ_REPORT_STREAM;                 // an explicit refinement ...
  CHECK(parse('S', ",0,0,1000,10000;1,1,10", t, err, warn, &pr));
  CHECK(t.meas.report == SJ_REPORT_STREAM);          // ... survives a redefinition
  CHECK(!TrainStore::isDefaultMeas(t));              // ... and so serializes in DUMP
  pr.meas.report = 0;

  // A DUMP round-trip in both directions: a slot whose owner turned the card
  // off must serialize a MEAS line, or replaying its S line would turn it back
  // on. This is what the default-report rule in isDefaultMeas is there for.
  TrainDef off = t;
  off.meas.report = 0;
  CHECK(!TrainStore::isDefaultMeas(off));
  cur.nStages = 2;                                                     // per-stage selection in range
  CHECK(TrainStore::validateMeas(cur, {3, 3, 0,  1, 0, 0}, warn, sizeof warn) == nullptr);
  CHECK(TrainStore::validateMeas(cur, {3, 3, 0,  2, 0, 0}, warn, sizeof warn) != nullptr);
  cur.nStages = 0;
  cur.mode0 = 2;                                                       // hi-Z: not driven
  CHECK(TrainStore::validateMeas(cur, {3, 0, 0, -1, 0, 0}, warn, sizeof warn) == nullptr);
  CHECK(strstr(warn, "nothing to measure") != nullptr);
  cur.mode0 = 0;
  cur.type = SINE;
  CHECK(TrainStore::validateMeas(cur, {3, 3, 0, -1, 0, 0}, warn, sizeof warn) != nullptr);  // sine needs when 1-3
  CHECK(TrainStore::validateMeas(cur, {3, 3, 2, -1, 0, 0}, warn, sizeof warn) == nullptr);  // -peak only: fine
  CHECK(TrainStore::validateMeas(cur, {3, 3, 3,  0, 0, 0}, warn, sizeof warn) != nullptr);  // sine has no stages

  // ------------------------------------------------- ENV/MEAS serializers
  TrainStore::serializeEnv(7, {1000, 2000, 0}, line, sizeof line);
  CHECK_STREQ(line, "ENV7,1000,2000,0");
  TrainStore::serializeMeas(7, {3, 3, 3, -1, 1, 1}, line, sizeof line);
  CHECK_STREQ(line, "MEAS7,3,3,3,-1,1,1");
  // fit is the 7th field and round-trips; 0 = refuse a point that does not fit
  TrainStore::serializeMeas(7, {3, 3, 3, -1, 1, 0}, line, sizeof line);
  CHECK_STREQ(line, "MEAS7,3,3,3,-1,1,0");
  CHECK(TrainStore::validateMeas(cur, {3, 3, 3, -1, 0, 1}, warn, sizeof warn) == nullptr);
  CHECK(TrainStore::validateMeas(cur, {3, 3, 3, -1, 0, 2}, warn, sizeof warn) != nullptr);  // fit > 1

  // --------------------------------------- optional post-trigger delay field
  // A legacy header (5 fields) must keep meaning exactly what it meant before:
  // no delay, and a canonical line that carries no 6th field either.
  CHECK(parse('S', ",0,1,2000,1000000;100,0,150", t, err, warn));
  CHECK(t.delay_us == 0);
  TrainStore::serializeTrain(0, t, line, sizeof line);
  CHECK_STREQ(line, "S0,0,1,2000,1000000;100,0,150");

  // 6th header field = delay, on each of the three waveform letters
  CHECK(parse('S', ",0,1,2000,1000000,2500;100,0,150", t, err, warn));
  CHECK(t.delay_us == 2500 && t.nStages == 1 && t.period_us == 2000);
  TrainStore::serializeTrain(4, t, line, sizeof line);
  CHECK_STREQ(line, "S4,0,1,2000,1000000,2500;100,0,150");
  CHECK(parse('S', line + 2, t, err, warn));            // canonical form round-trips
  TrainStore::serializeTrain(4, t, line2, sizeof line2);
  CHECK_STREQ(line, line2);

  CHECK(parse('L', ",0,0,5000,100000,750;1000,1000,200", t, err, warn));
  CHECK(t.type == PIECEWISE_RAMP && t.delay_us == 750);
  CHECK(parse('W', ",0,0,10000,1000000,30000;1000,1000,5000;100,100,0;0,0,0", t, err, warn));
  CHECK(t.type == SINE && t.delay_us == 30000 && t.sine.freq0_mHz == 100000);
  TrainStore::serializeTrain(2, t, line, sizeof line);
  CHECK_STREQ(line, "W2,0,0,10000,1000000,30000;1000,1000,5000;100,100,0;0,0,0;0,0,0");
  CHECK(parse('W', line + 2, t, err, warn));
  TrainStore::serializeTrain(2, t, line2, sizeof line2);
  CHECK_STREQ(line, line2);

  // Omitting the field resets the delay: an S/L/W line fully defines its header,
  // so replaying an old script cannot inherit a delay set earlier.
  TrainDef withDelay;
  TrainStore::slotDefault(withDelay);
  withDelay.delay_us = 12345;
  CHECK(parse('S', ",0,1,2000,1000000;100,0,150", t, err, warn, &withDelay));
  CHECK(t.delay_us == 0);

  CHECK(!parse('S', ",0,1,2000,1000000,-5;100,0,150", t, err, warn));   // negative
  CHECK(!parse('S', ",0,1,2000,1000000,2000000001;100,0,150", t, err, warn));  // over the cap
  CHECK(parse('S', ",0,1,2000,1000000,2000000000;100,0,150", t, err, warn));   // exactly the cap
  CHECK(t.delay_us == 2000000000u);
  CHECK(!parse('S', ",0,1,2000,1000000,;100,0,150", t, err, warn));      // empty field

  TrainStore::serializeDelay(9, 4200, line, sizeof line);
  CHECK_STREQ(line, "DELAY9,4200");

  // -------------------------------------- optional ramp sample interval field
  // 7th header field, `L` only. It is positional, so a slot that wants an
  // interval and no delay writes the delay as 0 — and serializes back that way.
  CHECK(parse('L', ",0,1,2000,1000000,0,50;100,0,150", t, err, warn));
  CHECK(t.dt_us == 50 && t.delay_us == 0);
  TrainStore::serializeTrain(4, t, line, sizeof line);
  CHECK_STREQ(line, "L4,0,1,2000,1000000,0,50;100,0,150");
  CHECK(parse('L', line + 2, t, err, warn));            // canonical form round-trips
  TrainStore::serializeTrain(4, t, line2, sizeof line2);
  CHECK_STREQ(line, line2);

  CHECK(parse('L', ",0,1,2000,1000000,300,25;100,0,150", t, err, warn));
  CHECK(t.dt_us == 25 && t.delay_us == 300);
  // Omitting it resets the interval to the build default, like the delay.
  TrainDef withDt;
  TrainStore::slotDefault(withDt);
  withDt.dt_us = 200;
  CHECK(parse('L', ",0,1,2000,1000000;100,0,150", t, err, warn, &withDt));
  CHECK(t.dt_us == 0);

  // ------------------------------------------- 0-duration L stages (jumps)
  // One is the documented instant jump — legal anywhere, including first and
  // last — and it means "shift the level here, then ramp on from it".
  CHECK(parse('L', ",0,1,2000,1000000;100,0,0;200,0,150", t, err, warn));
  CHECK(t.nStages == 2 && t.stages[0].dur_us == 0);
  CHECK(parse('L', ",0,1,2000,1000000;100,0,150;200,0,0", t, err, warn));
  CHECK(parse('L', ",0,1,2000,1000000;100,0,150;200,0,0;300,0,150", t, err, warn));
  // Two in a row ask for two levels at one instant: the first could never be
  // delivered, so the line is refused rather than silently losing a level.
  CHECK(!parse('L', ",0,1,2000,1000000;100,0,0;200,0,0;300,0,150", t, err, warn));
  CHECK(strstr(err, "0-duration") != nullptr);
  // `S` stages are rectangular steps, not jumps — a 0-duration one there is
  // legacy-legal and stays legal (bit-exact S semantics).
  CHECK(parse('S', ",0,1,2000,1000000;100,0,0;200,0,0;300,0,150", t, err, warn));

  CHECK(!parse('L', ",0,1,2000,1000000,0,1;100,0,150", t, err, warn));        // below the floor
  CHECK(!parse('L', ",0,1,2000,1000000,0,1000001;100,0,150", t, err, warn));  // above the ceiling
  CHECK(!parse('L', ",0,1,2000,1000000,0,0;100,0,150", t, err, warn));        // 0 = omit the field
  // Only a ramp has a sample interval: on S and W the field is an error, not a
  // silently ignored number.
  CHECK(!parse('S', ",0,1,2000,1000000,0,50;100,0,150", t, err, warn));
  CHECK(strstr(err, "7th header field") != nullptr);
  CHECK(!parse('W', ",0,0,10000,1000000,0,50;1,1,500;5,5,0;0,0,0", t, err, warn));

  TrainStore::serializeDt(9, 40, line, sizeof line);
  CHECK_STREQ(line, "DT9,40");

  // ------------------------------------------------------- default helpers
  TrainStore::slotDefault(cur);
  CHECK(TrainStore::isDefaultTrain(cur) && TrainStore::isDefaultEnv(cur.env));
  CHECK(TrainStore::isDefaultMeas(cur));
  cur.type = SINE;                        // auto-when: default MEAS on a W slot is when=3
  cur.meas.when = 3;
  CHECK(TrainStore::isDefaultMeas(cur));
  CHECK(TrainStore::defaultWhen(SINE) == 3 && TrainStore::defaultWhen(PIECEWISE_HOLD) == 0);

  // ------------------------------------------------------- minLatchGapUs
  // The shortest nonzero interval a definition puts between two consecutive DAC
  // latches. The S/L/W handler compares it with Cal::minLatchUs and warns about
  // stage boundaries this board cannot deliver.
  CHECK(parse('S', ",0,0,10000,500000;5000,5000,1000;5000,5000,40", t, err, warn));
  CHECK(TrainStore::minLatchGapUs(t, 20) == 40u);      // S: the stage duration itself
  // a 0-duration S stage is a coincident pair by definition, not a short gap
  CHECK(parse('S', ",0,0,10000,500000;5000,5000,0;5000,5000,300", t, err, warn));
  CHECK(TrainStore::minLatchGapUs(t, 20) == 300u);
  // an undriven train latches nothing at all
  CHECK(parse('S', ",3,3,10000,500000;5000,5000,40", t, err, warn));
  CHECK(TrainStore::minLatchGapUs(t, 20) == UINT32_MAX);
  // L: a long stage is sampled at dt, so it reports dt and not its duration
  CHECK(parse('L', ",0,0,10000,500000,0,50;5000,5000,1000", t, err, warn));
  CHECK(TrainStore::minLatchGapUs(t, 50) == 50u);
  // L: a stage under dt/2 gets one sample, at its *end* -- the case the warning
  // exists for, and there the gap is the whole stage duration
  CHECK(parse('L', ",0,0,10000,500000,0,20;5000,5000,1000;0,0,3", t, err, warn));
  CHECK(TrainStore::minLatchGapUs(t, 20) == 3u);
  // W is exempt: its sample grid comes from the engine's Fs policy
  CHECK(parse('W', ",0,0,10000,500000;5000,5000,5000;1000,1000,0;0,0,0", t, err, warn));
  CHECK(TrainStore::minLatchGapUs(t, 20) == UINT32_MAX);

  if (failures == 0) printf("all checks passed\n");
  else               printf("%d check(s) FAILED\n", failures);
  return failures;
}
