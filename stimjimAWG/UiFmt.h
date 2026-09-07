//    stimjimAWG — UiFmt: the integer number formatting the OLED result pages
//    use. No Arduino dependency, so tests/host/test_uifmt.cpp exercises every
//    rule below directly. GPL-3.0-or-later; see Config.h header.
//
//    Two jobs, and the second is the interesting one.
//
//    Voltage and current are printed in their measurement unit (mV, uA) with a
//    decimal below 1000 and none above, which fits the panel's seven-character
//    field at full scale (+-15 V, +-3.33 mA) and reads the way the `MSUM` line
//    does.
//
//    Resistance is printed with the number of digits the measurement actually
//    supports, and nothing more. R = V/I, so its relative uncertainty follows
//    from the two standard errors the accumulators carry, floored at
//    SJ_UI_REL_FLOOR_PPM because this project has never characterised the ADC
//    path's gain accuracy -- without that floor a 500-repetition train would
//    claim five significant figures it does not have. The floored uncertainty
//    then chooses the unit (ohm/kilohm/megohm) and the decimal place, capped at
//    three significant figures, so a short or noisy train automatically loses
//    digits. Two readings get a word instead of a number: `open` when the
//    current is not distinguishable from zero, and `--` when the line was never
//    read or when neither V nor I differs from zero.
//
//    Readings large enough to be the hardware's ceiling rather than the
//    requested amplitude are marked with '*' (see voltFlag/currFlag).

#ifndef STIMJIMAWG_UIFMT_H
#define STIMJIMAWG_UIFMT_H

#include <stdint.h>
#include <stddef.h>

namespace UiFmt {

// Relative-uncertainty floor, parts per million: 1 %. It stands in for the
// analog path's gain accuracy, which has not been measured on this board. A
// measured figure belongs here instead, and nothing else has to change.
#define SJ_UI_REL_FLOOR_PPM 10000u

// Quantisation deviation of one reading, from the converter step and the
// uniform distribution: 2.44 mV / sqrt(12) = 704 uV on the voltage line,
// 0.85 uA / sqrt(12) = 245 nA on the current line. A standard error is floored
// at these -- a handful of identical codes gives sd = 0, which would otherwise
// claim a precision the converter cannot deliver.
#define SJ_UI_QUANT_UV 704u
#define SJ_UI_QUANT_NA 245u

// Readings this large or larger are marked '*'. The output stage runs off a
// +-15 V rail but its driver saturates below that, so an absolute output
// voltage of 9 V or more is more likely the driver IC's ceiling than the
// amplitude that was asked for; the current pump is designed to 3 mA and the
// DAC conversion above 3000 uA is known bad (TrainStore's parse-time warning
// uses the same number). Either way the value is suspect and the panel says so
// -- the same kind of feedback `open` gives on the resistance row.
#define SJ_UI_VLIMIT_UV 9000000L
#define SJ_UI_ILIMIT_NA 3000000L

// Widest string any formatter below produces, terminator included.
#define SJ_UI_FIELD_MAX 12

// '*' at or above the limit, ' ' otherwise.
char voltFlag(int32_t uV);
char currFlag(int32_t nA);

// Microvolts as millivolts, nanoamps as microamps: one decimal below 1000 of
// the printed unit, none above. `have` false renders "--".
void fmtVolt(char* out, size_t n, bool have, int32_t uV);
void fmtCurr(char* out, size_t n, bool have, int32_t nA);

// R = V/I in ohms, kilohms or megohms, to the precision the two readings and
// their standard errors support. See the header comment for `open` and `--`.
void fmtOhm(char* out, size_t n, bool haveV, int32_t uV, uint32_t seUV,
            bool haveI, int32_t nA, uint32_t seNA);

// ------------------------------------------------- exposed for the host tests

// R in milliohms from microvolts and nanoamps. Peaks near 2e10 at the smallest
// current this module will still divide by, so the intermediate is int64.
int64_t ohmMilli(int32_t uV, int32_t nA);
// Relative uncertainty of a mean, ppm, floored by SJ_UI_REL_FLOOR_PPM and
// capped at 100 %. A zero mean gets the cap: it carries no digits at all.
uint32_t relPpm(int32_t value, uint32_t se);
// floor(sqrt(v)) by Newton iteration on integers.
uint32_t isqrt64(uint64_t v);

} // namespace UiFmt

#endif // STIMJIMAWG_UIFMT_H
