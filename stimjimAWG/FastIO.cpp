//    stimjimAWG — FastIO implementation (plan §3.4). GPL-3.0-or-later.

#include "FastIO.h"
#include "Config.h"

namespace FastIO {

// ---------------------------------------------------------------- SPI0 setup
//
// CTAR0 — AD5752 DAC: 30 MHz, SPI mode 1 (CPOL=0, CPHA=1), 12-bit frames.
//   baud = (F_BUS/PBR)·(1+DBR)/BR = (60 MHz/2)·2/2 = 30 MHz
//   The 24-bit DAC word is sent as two back-to-back 12-bit frames from the
//   4-deep TX FIFO (both frames must share one CTAR, and CTAR1 belongs to the
//   ADC — the plan's "8+16" is realized as 12+12; the AD5752 only counts SCLK
//   falling edges while SYNC is low, so the inter-frame pause is harmless).
static const uint32_t CTAR_DAC =
    SPI_CTAR_FMSZ(11) | SPI_CTAR_CPHA | SPI_CTAR_DBR | SPI_CTAR_PBR(0) | SPI_CTAR_BR(0);

// CTAR1 — AD7321 ADC: 10 MHz, SPI mode 2 (CPOL=1, CPHA=0), 16-bit frames.
//   baud = (60 MHz/3)/2 = 10 MHz
static const uint32_t CTAR_ADC =
    SPI_CTAR_FMSZ(15) | SPI_CTAR_CPOL | SPI_CTAR_PBR(1) | SPI_CTAR_BR(0);

static const uint32_t MCR_RUN =
    SPI_MCR_MSTR | SPI_MCR_PCSIS(0x1F);   // master, FIFOs enabled, hardware PCS unused

static void reassertMiso();   // defined with the ADC section below

void acquireBus() {
  // CTARs may only be written while the module is halted.
  KINETISK_SPI0.MCR   = MCR_RUN | SPI_MCR_CLR_TXF | SPI_MCR_CLR_RXF | SPI_MCR_HALT;
  KINETISK_SPI0.CTAR0 = CTAR_DAC;
  KINETISK_SPI0.CTAR1 = CTAR_ADC;
  KINETISK_SPI0.SR    = 0xFF0F0000;        // clear all w1c status flags
  KINETISK_SPI0.MCR   = MCR_RUN;
  // Legacy SPI-library calls (Stimjim.readAdc) also re-mux MISO via
  // SPI.setMISO, invalidating our cached selection — re-assert it.
  reassertMiso();
}

// ------------------------------------------------------------ frame helpers
//
// Discipline: every transaction clears EOQF, pushes its frame(s) with EOQ on
// the last one, spins on EOQF (module halts itself on EOQ), then drains RX.
// Worst case spin: 24 bits @30 MHz = 0.8 us / 16 bits @10 MHz = 1.6 us.

static inline void dac24(uint32_t word24) {   // CS must already be low
  KINETISK_SPI0.SR    = SPI_SR_EOQF | SPI_SR_TCF;
  KINETISK_SPI0.PUSHR = ((word24 >> 12) & 0xFFF) | SPI_PUSHR_CTAS(0);
  KINETISK_SPI0.PUSHR = ( word24        & 0xFFF) | SPI_PUSHR_CTAS(0) | SPI_PUSHR_EOQ;
  while (!(KINETISK_SPI0.SR & SPI_SR_EOQF)) ;
  (void)KINETISK_SPI0.POPR;                   // discard the two RX frames
  (void)KINETISK_SPI0.POPR;
  KINETISK_SPI0.SR = SPI_SR_EOQF;             // resume module
}

static inline uint16_t adc16(uint16_t tx) {   // CS must already be low
  KINETISK_SPI0.SR    = SPI_SR_EOQF | SPI_SR_TCF;
  KINETISK_SPI0.PUSHR = tx | SPI_PUSHR_CTAS(1) | SPI_PUSHR_EOQ;
  while (!(KINETISK_SPI0.SR & SPI_SR_EOQF)) ;
  uint16_t r = KINETISK_SPI0.POPR;
  KINETISK_SPI0.SR = SPI_SR_EOQF;
  return r;
}

// ---------------------------------------------------------------------- DAC

void dacProgram(uint8_t ch, int16_t code) {
  // 24-bit input word: [23] R/W=0(write) [21:19] reg=000(DAC) [18:16] addr=000(DAC A)
  const uint32_t w = (uint16_t)code;
  if (ch == 0) {
    digitalWriteFast(CS0_0, LOW);  dac24(w);  digitalWriteFast(CS0_0, HIGH);
  } else {
    digitalWriteFast(CS0_1, LOW);  dac24(w);  digitalWriteFast(CS0_1, HIGH);
  }
}

void dacProgramBoth(int16_t c0, int16_t c1) {
  // The channels share MOSI/SCK with separate CS, so the two 24-bit shifts are
  // inherently sequential; the FIFO already overlaps each word's two frames
  // with the CPU (callers may compute between program and latch). ~2 us total.
  dacProgram(0, c0);
  dacProgram(1, c1);
}

void dacLatch(uint8_t chMask) {
  if (chMask & 1) digitalWriteFast(NLDAC_0, LOW);
  if (chMask & 2) digitalWriteFast(NLDAC_1, LOW);
  // AD5752 LDAC min low pulse is ~20 ns; hold ~100 ns (12 cycles) for margin.
  // With chMask=0b11 the two falling edges are 2 CPU cycles (~17 ns) apart —
  // effectively hardware-synchronous dual update.
  uint32_t t0 = ARM_DWT_CYCCNT;
  while ((uint32_t)(ARM_DWT_CYCCNT - t0) < 12) ;
  if (chMask & 1) digitalWriteFast(NLDAC_0, HIGH);
  if (chMask & 2) digitalWriteFast(NLDAC_1, HIGH);
}

// ---------------------------------------------------------------------- ADC

static volatile uint8_t misoCh = 0xFF;   // which channel's MISO pin feeds SPI0_SIN

void adcSetMiso(uint8_t ch) {
  if (ch == misoCh) return;
  // Swap the SPI0_SIN PORT mux between pin 12 (PTC7, ch0) and pin 8 (PTD3, ch1).
  // The deselected pin falls back to plain GPIO input (both pins are configured
  // as inputs in begin(), so nothing ever drives against an isolator output).
  if (ch == 0) {
    CORE_PIN8_CONFIG  = PORT_PCR_MUX(1);
    CORE_PIN12_CONFIG = PORT_PCR_MUX(2);
  } else {
    CORE_PIN12_CONFIG = PORT_PCR_MUX(1);
    CORE_PIN8_CONFIG  = PORT_PCR_MUX(2);
  }
  misoCh = ch;
}

static void reassertMiso() {
  uint8_t ch = (misoCh == 0xFF) ? 0 : misoCh;
  misoCh = 0xFF;      // force adcSetMiso to rewrite both PORT muxes
  adcSetMiso(ch);
}

void adcSelectLine(uint8_t ch, uint8_t line) {
  // AD7321 control register: [15]=write [12:11]=00(control) [10]=ADD0(line) [4]=internal ref
  // — identical to legacy StimJim::setAdcLine (0x8000 + 1024*line + 16).
  const uint16_t ctrl = 0x8000 | (line ? 0x0400 : 0) | 0x0010;
  if (ch == 0) {
    digitalWriteFast(CS1_0, LOW);  (void)adc16(ctrl);  digitalWriteFast(CS1_0, HIGH);
  } else {
    digitalWriteFast(CS1_1, LOW);  (void)adc16(ctrl);  digitalWriteFast(CS1_1, HIGH);
  }
}

int16_t adcRead(uint8_t ch, uint8_t line) {
  adcSetMiso(ch);
  int32_t data;
  if (ch == 0) {
    digitalWriteFast(CS1_0, LOW);  data = adc16(0);  digitalWriteFast(CS1_0, HIGH);
  } else {
    digitalWriteFast(CS1_1, LOW);  data = adc16(0);  digitalWriteFast(CS1_1, HIGH);
  }
  // Byte-exact with legacy readAdc: bit 13 is the AD7321 channel-ID (set when
  // reading line 1), then 13-bit two's-complement sign extension.
  data -= line ? 8192 : 0;
  if (data > 4095) data -= 8192;
  return (int16_t)data;
}

// ------------------------------------------------------------------ timebase

static volatile uint32_t cycHi   = 0;
static volatile uint32_t cycLast = 0;

uint64_t cycles64() {
  uint32_t primask;
  __asm__ volatile("mrs %0, primask" : "=r"(primask));
  __disable_irq();
  uint32_t now = ARM_DWT_CYCCNT;
  if (now < cycLast) cycHi++;    // 32-bit wrap (every 35.8 s) — callers keep us alive
  cycLast = now;
  uint32_t hi = cycHi;
  if (!primask) __enable_irq();
  return ((uint64_t)hi << 32) | now;
}

// ------------------------------------------------------------------ bus lock

static volatile uint8_t  lockDepth   = 0;
static uint32_t          savedBasepri = 0;

void busLock() {
  uint32_t primask;
  __asm__ volatile("mrs %0, primask" : "=r"(primask));
  __disable_irq();
  if (lockDepth++ == 0) {
    __asm__ volatile("mrs %0, basepri" : "=r"(savedBasepri));
    uint32_t v = SJ_PLAYER_PRIO;   // masks priority values >= 64 (players and below)
    __asm__ volatile("msr basepri, %0" :: "r"(v) : "memory");
  }
  if (!primask) __enable_irq();
}

void busUnlock() {
  uint32_t primask;
  __asm__ volatile("mrs %0, primask" : "=r"(primask));
  __disable_irq();
  if (lockDepth && --lockDepth == 0) {
    __asm__ volatile("msr basepri, %0" :: "r"(savedBasepri) : "memory");
  }
  if (!primask) __enable_irq();
}

// --------------------------------------------------------------------- begin

void begin() {
  // DWT cycle counter — NOT enabled by the Teensy 3 core startup.
  ARM_DEMCR    |= ARM_DEMCR_TRCENA;
  ARM_DWT_CTRL |= ARM_DWT_CTRL_CYCCNTENA;

  SIM_SCGC6 |= SIM_SCGC6_SPI0;   // defensive — SPI.begin() normally did this already

  // SCK / MOSI muxed to SPI0 with high drive strength (SPI.begin() also does
  // this, but FastIO must be self-sufficient if the legacy init is skipped).
  CORE_PIN13_CONFIG = PORT_PCR_MUX(2) | PORT_PCR_DSE;   // SCK  = PTC5
  CORE_PIN11_CONFIG = PORT_PCR_MUX(2) | PORT_PCR_DSE;   // MOSI = PTC6

  // Both MISO candidates as plain inputs; adcSetMiso() then muxes one to SIN.
  pinMode(MISO_0, INPUT);
  pinMode(MISO_1, INPUT);
  misoCh = 0xFF;
  adcSetMiso(0);

  // Chip selects and latch pins idle high (defensive re-assert).
  const uint8_t highPins[] = {CS0_0, CS0_1, CS1_0, CS1_1, NLDAC_0, NLDAC_1};
  for (uint8_t p : highPins) {
    pinMode(p, OUTPUT);
    digitalWriteFast(p, HIGH);
  }

  acquireBus();
}

} // namespace FastIO
