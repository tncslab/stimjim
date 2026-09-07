//    stimjimAWG — UiFmt implementation. GPL-3.0-or-later; see Config.h header.

#include "UiFmt.h"
#include <stdio.h>

namespace UiFmt {

static inline uint32_t absU(int32_t v) {
  return (uint32_t)((v < 0) ? -(int64_t)v : (int64_t)v);
}

char voltFlag(int32_t uV) { return (absU(uV) >= (uint32_t)SJ_UI_VLIMIT_UV) ? '*' : ' '; }
char currFlag(int32_t nA) { return (absU(nA) >= (uint32_t)SJ_UI_ILIMIT_NA) ? '*' : ' '; }

uint32_t isqrt64(uint64_t v) {
  if (v == 0) return 0;
  // Start above the root (bit_width/2 rounded up) so the Newton iteration
  // descends monotonically and the `x1 >= x0` test terminates it.
  uint64_t x0 = v, x1 = (v + 1) / 2;
  while (x1 < x0) { x0 = x1; x1 = (x1 + v / x1) / 2; }
  return (uint32_t)x0;
}

int64_t ohmMilli(int32_t uV, int32_t nA) {
  // V[uV] / I[nA] is already ohms; the 1e6 puts the result in milliohms without
  // ever forming a fraction. 15 V over the smallest current this module divides
  // by is about 2e10 milliohms, so int64 has three orders of margin.
  return (int64_t)uV * 1000000LL / (int64_t)nA;
}

uint32_t relPpm(int32_t value, uint32_t se) {
  const uint32_t a = absU(value);
  if (a == 0) return 1000000u;                     // no digits at all
  uint64_t r = ((uint64_t)se * 1000000ull) / a;
  if (r < SJ_UI_REL_FLOOR_PPM) r = SJ_UI_REL_FLOOR_PPM;
  if (r > 1000000ull)          r = 1000000ull;
  return (uint32_t)r;
}

// Thousandths of a unit as that unit: one decimal below 1000, none above. The
// wide branch is what keeps -15000 mV and -3333 uA inside a seven-character
// field; the narrow one is what makes 100.2 mV readable.
static void fmtThousandths(char* out, size_t n, int32_t v) {
  const bool neg = v < 0;
  const uint32_t a = absU(v);
  if (a < 1000000u) {
    const uint32_t t = (a + 50u) / 100u;           // tenths, rounded
    snprintf(out, n, "%s%lu.%lu", neg ? "-" : "",
             (unsigned long)(t / 10u), (unsigned long)(t % 10u));
  } else {
    snprintf(out, n, "%s%lu", neg ? "-" : "", (unsigned long)((a + 500u) / 1000u));
  }
}

void fmtVolt(char* out, size_t n, bool have, int32_t uV) {
  if (!have) { snprintf(out, n, "--"); return; }
  fmtThousandths(out, n, uV);
}

void fmtCurr(char* out, size_t n, bool have, int32_t nA) {
  if (!have) { snprintf(out, n, "--"); return; }
  fmtThousandths(out, n, nA);
}

// Round to one significant figure. Rounding at every decade rather than once at
// the end costs a digit of exactness the caller does not have anyway, and keeps
// the whole thing in integers.
static int64_t oneSigFig(int64_t v) {
  int64_t pw = 1;
  while (v >= 10) { v = (v + 5) / 10; pw *= 10; }
  if (v > 9) { v = 1; pw *= 10; }                  // 9.6 rounded up to 10
  return v * pw;
}

void fmtOhm(char* out, size_t n, bool haveV, int32_t uV, uint32_t seUV,
            bool haveI, int32_t nA, uint32_t seNA) {
  if (!haveV || !haveI) { snprintf(out, n, "--"); return; }

  // The standard errors floored at the converter's own quantisation deviation:
  // a run of identical codes gives sd = 0, which is not a claim the hardware
  // supports.
  const uint32_t seV = (seUV > SJ_UI_QUANT_UV) ? seUV : (uint32_t)SJ_UI_QUANT_UV;
  const uint32_t seI = (seNA > SJ_UI_QUANT_NA) ? seNA : (uint32_t)SJ_UI_QUANT_NA;
  const bool vZero = absU(uV) < 3u * seV;
  const bool iZero = absU(nA) < 3u * seI;
  if (vZero && iZero) { snprintf(out, n, "--");   return; }   // undriven or grounded
  if (iZero)          { snprintf(out, n, "open"); return; }   // no current path

  const int64_t Rm = ohmMilli(uV, nA);
  const bool    neg = Rm < 0;                      // V and I disagreeing in sign
  int64_t am = neg ? -Rm : Rm;

  // Relative uncertainty of the quotient: the two contributions in quadrature,
  // each already floored, so at the floor this is 1.4 %.
  const uint64_t rV = relPpm(uV, seV), rI = relPpm(nA, seI);
  uint64_t rR = isqrt64(rV * rV + rI * rI);
  if (rR > 1000000ull) rR = 1000000ull;

  int64_t dRm = (int64_t)((uint64_t)am * rR / 1000000ull);
  if (dRm < 1) dRm = 1;                            // a zero window would pick every digit
  const int64_t dr1 = oneSigFig(dRm);

  // Unit: the mantissa belongs in [1, 1000). The boundaries are the values that
  // round to 1000 of the smaller unit, so 999.6 ohm reads as 1.00k rather than
  // as a four-digit ohm value.
  int64_t     unit;
  const char* suffix;
  if      (am < 999500LL)       { unit = 1000LL;       suffix = "";  }
  else if (am < 999500000LL)    { unit = 1000000LL;    suffix = "k"; }
  else                          { unit = 1000000000LL; suffix = "M"; }

  // Decimals: enough to reach the decade the uncertainty sits in, then capped
  // so the whole number never claims more than three significant figures.
  int     d     = 0;
  int64_t place = unit;
  while (place > dr1 && d < 3) { place /= 10; d++; }
  const int64_t ip     = am / unit;                              // 0..999
  const int     intDig = (ip >= 100) ? 3 : (ip >= 10) ? 2 : 1;
  if (d > 3 - intDig) d = 3 - intDig;

  static const int64_t POW10[4] = {1, 10, 100, 1000};
  const int64_t scaled = (am * POW10[d] + unit / 2) / unit;
  // Both parts are at most three digits after the scaling above, so the printf
  // stays in unsigned long and no %lld appears in a format string.
  const unsigned long whole = (unsigned long)(scaled / POW10[d]);
  const unsigned long frac  = (unsigned long)(scaled % POW10[d]);
  if (d == 0) snprintf(out, n, "%s%lu%s", neg ? "-" : "", whole, suffix);
  else        snprintf(out, n, "%s%lu.%0*lu%s", neg ? "-" : "", whole, d, frac, suffix);
}

} // namespace UiFmt
