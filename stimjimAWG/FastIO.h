//    stimjimAWG — FastIO: split-phase register-level hardware access (plan §3.4).
//    GPL-3.0-or-later; see Config.h header.
//
//    Hot-path DAC/ADC operations driving SPI0 registers directly (no SPI-library
//    transactions), the 64-bit CYCCNT timebase and the bus lock. `lib/stimjim`
//    stays untouched and is still used for all non-realtime work (boot init,
//    calibration, output-mode switching); after any legacy SPI-library call the
//    caller must re-assert our SPI0 configuration with acquireBus().

#ifndef STIMJIMAWG_FASTIO_H
#define STIMJIMAWG_FASTIO_H

#include <stdint.h>

namespace FastIO {

// Enable DWT CYCCNT (NOT enabled by the Teensy 3 core startup — verified in
// core 1.59: only AudioStream sets it), configure SPI0 CTARs/pins and the MISO
// mux default. Call after Stimjim.begin() so the legacy boot init keeps working.
void begin();

// Re-assert MCR/CTAR0/CTAR1 after legacy SPI-library use (SPISettings clobber both CTARs).
void acquireBus();

// --- AD5752 DAC, split program/execute ------------------------------------
// dacProgram shifts the 24-bit write (reg 0 = DAC A, two's complement code)
// while CS is low; the output only changes on dacLatch (NLDAC pulse).
void dacProgram(uint8_t ch, int16_t code);
void dacProgramBoth(int16_t c0, int16_t c1);   // sequential CS transactions (shared MOSI)
void dacLatch(uint8_t chMask);                 // bit0 = ch0, bit1 = ch1; ~100 ns GPIO pulse

// --- AD7321 ADC ------------------------------------------------------------
// Select which MISO pin feeds SPI0_SIN (isolators don't tristate). Tracked
// internally; cheap no-op when already selected. adcRead calls it itself.
void adcSetMiso(uint8_t ch);
// Write the AD7321 control register to select input line (0 = output voltage,
// 1 = current sense). Pre-issue during slack so adcRead is ~2 us, not ~4.5 us.
void adcSelectLine(uint8_t ch, uint8_t line);
// Read one conversion. The line MUST already be selected; `line` is only used
// to strip the channel-ID bit (byte-exact with legacy StimJim::readAdc math).
int16_t adcRead(uint8_t ch, uint8_t line);

// --- timebase ----------------------------------------------------------------
// 64-bit DWT cycle counter (8.33 ns resolution). Must be called at least once
// per 35.8 s to catch wrap-around — loop() and the engine's slice chunking
// guarantee that. Safe from any context (brief PRIMASK critical section).
uint64_t cycles64();

// --- bus lock ------------------------------------------------------------------
// Raise BASEPRI to SJ_PLAYER_PRIO so player ISRs cannot run while loop-context
// code touches the SPI bus. Nestable. Keep sections <= a few us.
void busLock();
void busUnlock();

} // namespace FastIO

#endif // STIMJIMAWG_FASTIO_H
