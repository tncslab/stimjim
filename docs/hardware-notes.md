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
| DG409 (U12/U23) | **dual** 4:1 analog mux, addressed by `OE1:OE0` = the mode bits. Bank B picks what `CHANNEL_OUT` sees (`V_OUT` / `I_OUT` / open / `GRND`); bank A picks what `I_OUT` is tied to — `CHANNEL_OUT` in current mode, and R14, a 1 kΩ on-board dummy to ground, in all three others. That second bank is why the current sense reads nothing useful outside current mode |
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
MICROAMPS_PER_ADC  0.85     // (1/(100·(1+9.9k/360)) V/A) · 20 V / 2^13   [R12 shunt, R13 gain]
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
| Buttons | Btn0=17, Btn1=39, Btn2=16 | | RISING, no external debounce (firmware arms once per press). Btn0 = next display page, Btn1 = fire input 0's trigger route, Btn2 = fire input 1's |
| Display | SSD1306 128×32, I²C addr 0x3C on `Wire` | | future: taller display + rotary |

**Known header bug** (`Stimjim.h:52-62`): `GPIO_1`…`GPIO_11` are *all* defined as pin 36
(`GPIO_7` twice, `GPIO_8` missing). Verify real GPIO header pins in the PCB netlist before use.

## SPI timing (measured/calibrated in stimjimPulser)

- DAC write (24 bits @ 30 MHz incl. CS + latch overhead): **2.75 µs** per channel.
- ADC read (control write + 16-bit read @ 10 MHz incl. MISO mux + line switch): **4.5 µs**;
  ≈2 µs when the input line is already selected.
- AD5752 power-up quirk: range register written twice ("first write may be ignored"), 10 µs delay
  after the power-control write (`Stimjim.cpp:71-81`).
- **The AD5752 is double-buffered**: an SPI write lands in the input register and does nothing at
  the output until `NLDAC` is pulsed low (≥20 ns; `FastIO::dacLatch` holds ~100 ns and costs
  0.44 µs measured). A code can therefore be programmed arbitrarily long before it is executed, and
  two channels latched a couple of CPU cycles apart. `stimjimAWG` uses this for every event
  (program during the preload window, latch on the deadline) and could use it to preload a first
  sample before a trigger edge — see [timing.md](timing.md) §3.
- Output settling after a latch: **8–9 µs** to the final value, measured with `BENCHSETTLE` on
  either channel, either polarity, at 2000 and 8000 codes of step. This is the analog floor under
  every latency figure in the firmware.
- Calibration routines allow 30–50 µs settling after DAC steps before averaging ADC reads; first
  ADC reads after reconfiguration can show a transient (first 50 of 150 reads discarded).

## Measured on the board in hand (Teensy 3.5, fw 0.8.0, 1 kΩ load)

Three figures from the configuration B session (`tests/device/trigcomp.py`,
[bench-wiring.md](bench-wiring.md)). All three are properties of this board, not of the firmware,
and none of them has been checked on a second one.

- **In voltage mode the load current is not measurable at all**, and the firmware no longer
  reports one. The DG409 (U23/U12) is a dual 4:1 mux addressed by `OE1:OE0` = the mode bits: in
  current mode (01) it ties `I_OUT` to `CHANNEL_OUT`, but in voltage mode (00) it steers
  `CHANNEL_OUT` from `V_OUT` and parks `I_OUT` on R14, an on-board 1 kΩ dummy to the channel's
  ground. The 100 Ω shunt the AD8421 measures (R12) is in the `I_OUT` branch, between the Howland
  pump's R2 and that mux input — so in voltage mode it carries the pump's own current into the
  dummy, and there is no shunt anywhere in the `V_OUT` path. The pump is driven by the same DAC
  code whatever the mux does, so the reading was real, linear and completely unrelated to the
  load: it is the DAC code reinterpreted as a current, and matches
  `commanded_mV / MILLIVOLTS_PER_DAC × MICROAMPS_PER_DAC` to 0.8 % at every amplitude. Measure
  current in current mode. (Modes 2 and 3 park `I_OUT` on the same dummy, so nothing is measurable
  there either — but those channels are not driven anyway.)
- **The voltage output is accurate unloaded and droops under load: its source impedance is about
  135 Ω.** An unloaded channel reads 1996 mV for 2000 commanded (−0.2 %); the same command into
  1 kΩ gives 1766 mV (−12 %), confirmed on a scope. That is (2000 − 1766) mV over the 1.77 mA
  drawn. Earlier revisions of this document read the droop as a gain error, which the unloaded
  measurement rules out. Plan for it: a preparation of a few hundred ohms will see noticeably less
  than the commanded voltage, and the board's own voltage readback reports the delivered value
  rather than the requested one.
- **The analog output settles far faster than a reading of it does.** On a scope the output reaches
  90 % of an 8 V step 1.91 µs after it starts moving (2.93 µs for a 1.8 V step). `BENCHSETTLE`
  reports 8–9 µs for the same board, so most of `CAL SETTLE` is the ADC path — conversion, the
  input-line switch, the isolator — and not the output stage. `CAL SETTLE` is still the right
  budget for *when a reading means anything*, which is what it is used for; it is not the time the
  output takes to arrive.

## Known analog limitations (stimjimPulser.ino:26-31)

- Amplitudes above **3000 µA are converted incorrectly** on the DAC. The firmware warns at parse
  time when a stage asks for more, and the OLED result pages mark a *measured* current of 3 mA or
  more with `*` for the same reason — from the extremes of the train, so one repetition that got
  there is enough.
- The output driver saturates below the ±15 V the DAC span implies, so a measured output voltage
  of 9 V or more is more likely the driver's ceiling than the amplitude that was requested. The
  result pages mark those with `*` too. **9 V is a working threshold, not a measured limit:** it
  has not been characterised on this board, and it lives in `UiFmt.h` as `SJ_UI_VLIMIT_UV` so a
  measured figure can replace it.
- Into a 10 kΩ load the output converges more slowly and measurements interfere with the load:
  use stages ≥ 100 µs and expect ~10 % error (1 kΩ loads are fine down to ~10 µs).

## Bench instruments available

- Tektronix TDS 2004B oscilloscope (timing/jitter/settling verification).
- Rigol DG800 Pro function generator (trigger-latency stimulus into IN0/IN1).
- PicoScope 2204A two-channel USB scope with one AWG (versatile verification during development).
