# StimJim hardware notes (firmware developer reference)

Distilled from `lib/stimjim/src/Stimjim.{h,cpp}`, the root `README.md`, `PCB/` (schematic
`PCB/stimjim.pdf`, BOM `PCB/manufacturing/bom/stimjim BOM.csv`) and `StimJimBIST/`. The
`datasheets/` folder referenced by `firmware-spec.md` was removed by the upstream Rev C PCB
merges — this file replaces it for firmware purposes.

## MCU

**Firmware target: Teensy 3.5** — NXP MK64FX512VMD12, 120 MHz Cortex-M4F, F_BUS 60 MHz, 256 KB
RAM, 512 KB flash, 4 KB EEPROM, 4 PIT channels (`IntervalTimer`), DWT cycle counter, eDMA, SPI0
with 4-deep FIFO and CTAR0/1, onboard micro-SD socket on native SDIO (independent of SPI0).

Caution: the Rev C PCB files in this repo (Open Ephys redesign, commits `f462c28`, `b67df72`) use
a **Raspberry Pi RP2354B** — a different MCU, pin map and HAL. The stimjimAWG firmware targets the
Teensy-based boards; a port would replace `FastIO`/`Engine` internals.

## Per-channel analog chain (2 identical isolated channels)

| Part | Function |
|---|---|
| AD5752 (U6/U17) | 16-bit dual bipolar DAC; SPI 30 MHz max, mode 1/2; 24-bit frames; two's complement; output latched by NLDAC pin |
| AD7321 (U7/U18) | 12-bit-plus-sign (13-bit signed) 2-input SAR ADC; SPI 10 MHz, mode 2; input selected via control register (line 0 = output voltage, line 1 = current sense); has intrinsic "bipolar zero error" → boot offset calibration |
| AD8421 (U24/U29) | instrumentation amp, current sense |
| DG409 (U12/U23) | 4:1 analog mux — output mode steering via `OE0_x`/`OE1_x` (voltage / current / hi-Z / ground) |
| OPA197 (×6) | Howland current pump + buffers |
| LT1995 (U27/U28) | difference/gain amp (voltage path, gain 1.505) |
| AD1582 (U8/U19) | 2.5 V reference |
| ISO7760 + ISO7741 (U3/U4, U14/U15) | digital isolators (SPI + control). **They do not tristate** → each channel has its own MISO pin; firmware must mux SPI0_SIN between pins 12 and 8 per read |
| CC3-0512DF-E (U2/U13) | isolated ±15 V DC-DC per channel |

## Output specifications (README)

- Voltage mode: −15…+15 V (DAC ±10 V span × gain 1.505).
- Current mode: −3.33…+3.33 mA (Howland transimpedance ≈ 3000 V/A, 0.33 mA/V), compliance ±13.7 V
  (±3.33 mA into ≤4 kΩ; ±137 µA into ≤100 kΩ).
- Channels isolated from USB supply and from each other.

## Unit conversion constants (`Stimjim.h:31-34`)

```
MICROAMPS_PER_DAC  0.1017   // 20 V span · (1/3000 V/A) / 2^16
MICROAMPS_PER_ADC  0.85     // (1/(100·(1+49.9k/1.8k)) V/A) · 20 V / 2^13
MILLIVOLTS_PER_DAC 0.4574   // 20 V / 2^16 · gain 1.505
MILLIVOLTS_PER_ADC 2.44     // 20 V / 2^13
```

DAC code = `amp / *_PER_DAC + {voltage|current}Offsets[ch]`; ADC reading =
`(raw − adcOffset10[ch]) · *_PER_ADC`. Offsets measured at boot (`StimJim::begin`) and on
`B`/`C` commands. Note: `getVoltageOffsets()` sweeps a DAC ramp onto the output — dangerous with
a load attached (it is commented out of `begin()` for that reason, `Stimjim.cpp:50`).

## Pin map (`Stimjim.h:36-62`, Teensy pin numbers)

| Signal | ch0 | ch1 | Notes |
|---|---|---|---|
| DAC chip select `CS0_x` | 3 | 9 | plain GPIO (no hardware PCS) |
| ADC chip select `CS1_x` | 2 | 7 | plain GPIO |
| DAC latch `NLDAC_x` | 4 | 10 | low pulse latches; both pulsed together = synchronous dual update |
| Output mode `OE0_x`/`OE1_x` | 1/0 | 6/5 | 2-bit mode: 0 V, 1 I, 2 hi-Z, 3 GND |
| MISO | 12 | 8 | SPI0_SIN muxed per read (`SPI.setMISO`) |
| Trigger input `INx` | 22 | 23 | BNC |
| LED | 21 | 20 | |
| Buttons | Btn0=17, Btn1=39, Btn2=16 | | RISING, no external debounce |
| Display | SSD1306 128×32, I²C addr 0x3C on `Wire` | | future: taller display + rotary |

**Known header bug** (`Stimjim.h:52-62`): `GPIO_1`…`GPIO_11` are *all* defined as pin 36
(`GPIO_7` twice, `GPIO_8` missing). Verify real GPIO header pins in the PCB netlist before use.

## SPI timing (measured/calibrated in stimjimPulser)

- DAC write (24 bits @ 30 MHz incl. CS + latch overhead): **2.75 µs** per channel.
- ADC read (control write + 16-bit read @ 10 MHz incl. MISO mux + line switch): **4.5 µs**;
  ≈2 µs when the input line is already selected.
- AD5752 power-up quirk: range register written twice ("first write may be ignored"), 10 µs delay
  after the power-control write (`Stimjim.cpp:71-81`).
- Calibration routines allow 30–50 µs settling after DAC steps before averaging ADC reads; first
  ADC reads after reconfiguration can show a transient (first 50 of 150 reads discarded).

## Known analog limitations (stimjimPulser.ino:26-31)

- Amplitudes above **3000 µA are converted incorrectly** on the DAC.
- Into a 10 kΩ load the output converges more slowly and measurements interfere with the load:
  use stages ≥ 100 µs and expect ~10 % error (1 kΩ loads are fine down to ~10 µs).

## Bench instruments available

- Tektronix TDS 2004B oscilloscope (timing/jitter/settling verification).
- Rigol DG800 Pro function generator (trigger-latency stimulus into IN0/IN1).
- PicoScope 2204A two-channel USB scope with one AWG (versatile verification during development).
