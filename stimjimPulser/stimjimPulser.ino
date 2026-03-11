//    stimjimWaver (c) 2025- TNCS, Dept of Comp Sci, HUN-REN Wigner RCP, Hungary
//
//    Authors:
//      Gergely Gaal, Zsofia Tasnadi, Marcell Stippinger
//    
//    Developed during the Summer Student Week in 2025 and thereafter.
//    Based on the work of Nathan Cermak and original authors.
//
//    This file is part of stimjimPulser.
//
//    stimjimWaver is free software: you can redistribute it and/or modify
//    it under the terms of the GNU General Public License as published by
//    the Free Software Foundation, either version 3 of the License, or
//    (at your option) any later version.
//
//    This program is distributed in the hope that it will be useful,
//    but WITHOUT ANY WARRANTY; without even the implied warranty of
//    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//    GNU General Public License for more details.
//
//    You should have received a copy of the GNU General Public License
//    along with this program.  If not, see <https://www.gnu.org/licenses/>.
//

// Valid serial commands:
//    S, W - Set pulseTrain parameters. Example:
//        S0,0,1,2000,1000000; 100,-100,150; -100,0,200
//        1st argument (0) means set parameters for pulseTrain 0.
//        2nd argument (0) - mode 0 (voltage) on output channel 0 (see modes below under M)
//        3rd argument (1) - mode 1 (current) on output channel 1
//        4th argument (1000) - period of pulsetrain in microseconds. In example, run 1 pulse every ms.
//        5th argument (100000) - duration of pulsetrain in microseconds. In example, duration is 100 ms.
//        6th, 7th and 8th arguments - pulse stage 0 parameters
//            amplitudes for both channels (in uA and mV, depending on mode), and duration in usec.
//            In this case, sets amplitudes to 100mV, -100uA, for 150 microseconds
//        9th, 10th and 11th arguments - pulse stage 1 parameters
//            amplitudes for both channels (in uA and mV, depending on mode), and duration in usec.
//            In this case, sets amplitudes to -100mV, 0uA, for 200 microseconds
//        etc... for trios of arguments, up to 10 stages total.
//        Alternatively, for sine wave, example:
//        W1,1,1,100000,300000; 100,-100,10000; -500,150,0; 0,0,0
//        6th, 7th and 8th arguments - sine amplitude
//            amplitudes for both channels (in uA or mV, depending on mode), 8th argument is duration,
//            In this case, sets amplitudes to 100uA, -100mV, for 10 seconds
//        9th, 10th and 11th arguments - sine frequency
//            frequencies for both channel (in Hz), 11th argument omitted
//            In this case, sets frequencies to 500Hz and 150Hz
//        12th, 13th and 14th arguments - phase offset
//            phase offset for both channels (in degrees), 14th argument omitted
//            In this case, sets no phase offset
// 
//
//    T, U - T0 means start PulseTrain[0]. U0 also means start PulseTrain[0]. T and U can be
//           used to run two pulse train simultaneously.
//    B - measure ADC offset value (by grounding output and measuring ADC value on output).
//    C - measure current and voltage offsets by sweeping DAC values and reading output.
//    D - Print current values of all offsets (ADC, current, voltage)
//    R - R0,<n>,0 means that logic high on "input" 0 starts PulseTrain n.
//        R0,0,1 means that "input" 0 is reprogrammed as an output that marks stimulus start time of whatever pulsetrain is being delivered
//    P - save current definitions to EEPROM as defaults loaded on next boot
//    M - M0,0 means set output mode for channel 0 to 0. output modes are as follows:
//        0 - voltage
//        1 - current
//        2 - disconnected (hi-z)
//        3 - grounded
//    A - A0,1000 means set amplitude on channel 0 to 1000 (dac units; -32,768 to +32,767)
//    V - V0,100 means set amplitude on channel 0 to 100mV
//    E - E0,1 means read channel zero, line 1. Line 0 is voltage out, line 1 is current sense.
//        Returns (prints over serial) value in raw adc units.
//    V - V0 means default serial reporting, V1 means verbose


#include <Stimjim.h>
#include <math.h>
#include <EEPROM.h>
#define USE_DISPLAY

// For display based on https://github.com/adafruit/Adafruit_SSD1306/blob/master/examples/ssd1306_128x32_i2c/ssd1306_128x32_i2c.ino
// Install Arduino library "Adafruit SSD1306" by Adafruit, make sure it is not the emulator.
#ifdef USE_DISPLAY
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 32 // OLED display height, in pixels
#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
#endif

#define PT_ARRAY_LENGTH 100
#define MAX_NUM_STAGES 10
#define PT_EEPROM_LENGTH 10
#define pi 3.141592653
#define VOLTAGE_LIMIT_MV 8000  // value for detecting voltage-limited stimulaion
// TODO int approximation of macros MICROAMPS_PER_DAC, MICROAMPS_PER_ADC, MILLIVOLTS_PER_DAC
// and MILLIVOLTS_PER_ADC for higher speed in sine wave
#define Btn0 17
#define Btn1 39
#define Btn2 16

// ------------- Serial setup ---------------------------------- //
char comBuf[1000];
int32_t train_count = 0;
int bytesRecvd;
bool verbose = false;

struct Wave {
    int amplitude;
    int frequency;
    int phase;
    int _unused[MAX_NUM_STAGES-3];
};


// ------------- PulseTrain parameter setup -------------------- //
struct PulseTrain {
    unsigned int mode[2];
    unsigned long period;                          // usec
    unsigned long duration;                       // usec

    int nStages;
    union {
    int amplitude[2][MAX_NUM_STAGES];             // mV or uA, depending on mode
    Wave wave[2];
    };

    unsigned int stageDuration[MAX_NUM_STAGES];   // usec

    unsigned long trainStartTime;                 // usec
    int nPulses;
    int voltage[2][MAX_NUM_STAGES];     // Ch0_V, Ch1_V
    int current[2][MAX_NUM_STAGES];     // Ch0_I, Ch1_I

    // Todo we could encode being a pulse into frequency being 0, and allow independent setup on both channels.
    // For now, we use a single separate flag for simplicity.
    unsigned int isWave;
};

float sinetable[8192];


volatile PulseTrain PTs[PT_ARRAY_LENGTH];
volatile PulseTrain *activePT0, *activePT1;
IntervalTimer IT0, IT1;

// ------------ Globals for trigger status -------------------- //
int triggerTargetPTs[2];
bool trigOutput[2];

// ------------ Function prototypes --------------------------- //
int pulse (volatile PulseTrain* PT);
void pulse0();
void pulse1();
void startIT0(int ptIndex);
void startIT0ViaInputTrigger();
void startIT1(int ptIndex);
void startIT1ViaInputTrigger();

void printPulseTrainParameters(int i);
void printResultSummary(volatile PulseTrain* PT);
void displayResultSummary(volatile PulseTrain* PT);
volatile PulseTrain* clearPulseTrainHistory(volatile PulseTrain* PT);


struct myEEPROMdata {
    struct checksum {
        uint32_t xr, sm;
    };
    struct payload {
        PulseTrain PTs[PT_EEPROM_LENGTH];
        int triggerTargetPTs[2];
        bool trigOutput[2];
    };

    // todo add program version number to be able to detect incompatibilities in case of future changes in the structure
    checksum chk;
    payload data;
    uint32_t _padding;
};

static_assert(sizeof(myEEPROMdata) < 4096, "myEEPROMdata size exceeds EEPROM limit of 4096 bytes");


struct ResistanceResult{
    int voltage;
    int voltageLimited;
    int current;
    int currentLimited;
    int resistance;
    bool isKOhm;
};

myEEPROMdata::checksum do_checksum(const myEEPROMdata& my) {
    int n = (sizeof(myEEPROMdata::payload) + 3) / 4;
    uint32_t *ptr = (uint32_t*)(my.data.PTs);
    myEEPROMdata::checksum chk = {0xDEADBEEF, 0xFEEDBEEF};
    for(int i = 0; i < n; i++) {
        chk.xr ^= (*ptr);
        chk.sm += (*ptr);
    }
    return chk;
}

void setTriggers(const int ptIndex, const int trigSrc, const bool output) {
    if (ptIndex >= 0 && !output) {
            Serial.print("Attaching interrupt to IN"); Serial.print(trigSrc);
            Serial.print(" to run PulseTrain["); Serial.print(ptIndex); Serial.println("]");
            pinMode((trigSrc) ? IN1 : IN0, INPUT);
            triggerTargetPTs[trigSrc] = ptIndex;
            attachInterrupt( (trigSrc) ? IN1 : IN0, (trigSrc) ? startIT1ViaInputTrigger : startIT0ViaInputTrigger, RISING);
            trigOutput[trigSrc] = false;
    } else {
            Serial.print("Detaching interrupt to IN"); Serial.println(trigSrc);
            Serial.print("Programming IN"); Serial.print(trigSrc); Serial.print(" as output that indicates activity on output ");Serial.println(trigSrc);
            detachInterrupt( (trigSrc) ? IN1 : IN0);
            triggerTargetPTs[trigSrc] = -1;  // todo requires value for buttons to work, is there any collision?
            pinMode((trigSrc) ? IN1 : IN0, OUTPUT);
            trigOutput[trigSrc] = true;
    }
}

int loadTriggersEEPROM(){
    int eeAddress = 0;   //Location we want the data to be put.
    myEEPROMdata retrieved;
    EEPROM.get(eeAddress, retrieved);
    myEEPROMdata::checksum chk = do_checksum(retrieved);
    if (memcmp(&retrieved.chk, &chk, sizeof(myEEPROMdata::chk)) == 0) {
        Serial.print("Restored first "); Serial.print(PT_EEPROM_LENGTH);
        Serial.println(" pulse train definitions and the triggers.");
        memcpy((void*)(PTs), retrieved.data.PTs, PT_EEPROM_LENGTH * sizeof(PulseTrain));
        for(int i=0; i < 2; i++) {
            triggerTargetPTs[i] = retrieved.data.triggerTargetPTs[i];
            trigOutput[i] = retrieved.data.trigOutput[i];
        }
        for(int i=0; i < 2; i++) {
            if (triggerTargetPTs[i] < PT_EEPROM_LENGTH) {
                setTriggers(triggerTargetPTs[i], i, trigOutput[i]);
            }
        }
        return 0;
    } else {
        Serial.println("Failed to restore train definitions due to checksum error.");
        return 1;
    }
}

int saveTriggersEEPROM(){
    int eeAddress = 0;   //Location we want the data to be put.
    myEEPROMdata captured;
    memcpy(captured.data.PTs, (void*)(PTs), PT_EEPROM_LENGTH * sizeof(PulseTrain));
    for(int i=0; i < 2; i++) {
        captured.data.triggerTargetPTs[i] = triggerTargetPTs[i];
        captured.data.trigOutput[i] = trigOutput[i];
    }
    for(int i=0; i < 2; i++) {
        if (PT_EEPROM_LENGTH <= captured.data.triggerTargetPTs[i]) {
            Serial.print("Trigger "); Serial.print(i);
            Serial.print(" pointing to train "); Serial.print(captured.data.triggerTargetPTs[i]);
            Serial.print(" will not be stored as the corresponding train is out of the first ");
            Serial.print(PT_EEPROM_LENGTH); Serial.println(" saved to EEPROM");
            captured.data.triggerTargetPTs[i] = -1;
        }
    }
    captured.chk = do_checksum(captured);
    EEPROM.put(eeAddress, captured);
    Serial.print("First "); Serial.print(PT_EEPROM_LENGTH); Serial.println(" saved to EEPROM.");
    return 0;
}

int pulse (volatile PulseTrain* PT)
{
    //check if the pulseTrain is finished; if so, exit
    if (micros() - PT->trainStartTime >= PT->duration)
        return 0;

    if (PT->nStages == 0) {
        PT->nPulses++;
        return 1;
    }

    int dac0val, dac1val;
    float adcReadTime =  4.50 * ((PT->mode[0] < 2) + (PT->mode[1] < 2));  //16 bits at 10MHz, calibrated time is 4.5us
    float dacWriteTime = 2.75 * ((PT->mode[0] < 2) + (PT->mode[1] < 2));  //24 bits at 30MHz, calibrated time is 2.75us
    float totalDelayTime = dacWriteTime + adcReadTime + adcReadTime + 0.5;
    dac0val = PT->amplitude[0][0] / ((!PT->mode[0]) ? MILLIVOLTS_PER_DAC : MICROAMPS_PER_DAC) + ((PT->mode[0]) ? Stimjim.currentOffsets[0] : Stimjim.voltageOffsets[0]);
    dac1val = PT->amplitude[1][0] / ((!PT->mode[1]) ? MILLIVOLTS_PER_DAC : MICROAMPS_PER_DAC) + ((PT->mode[1]) ? Stimjim.currentOffsets[1] : Stimjim.voltageOffsets[1]);

    if (PT->mode[0] < 2 && PT->mode[1] < 2) {
        Stimjim.writeToDacs(dac0val, dac1val);
    } else if (PT->mode[0] < 2) {
        Stimjim.writeToDac(0, dac0val);
    } else if (PT->mode[1] < 2) {
        Stimjim.writeToDac(1, dac1val);
    }
    if (PT->mode[0] < 2)
        Stimjim.setOutputMode(0, PT->mode[0]);

    if (PT->mode[1] < 2)
        Stimjim.setOutputMode(1, PT->mode[1]);
    for (int i = 0; i < PT->nStages; i++) {
        delayMicroseconds(PT->stageDuration[i] - totalDelayTime); // empirically calibrated!

        // read ADCs  TODO measure both current and voltage, not only that corresponding to mode
        if (PT->mode[0] < 2) {
            PT->voltage[0][i] += (Stimjim.readAdc(0, 0)-Stimjim.adcOffset10[0]) * MILLIVOLTS_PER_ADC;
            PT->current[0][i] += (Stimjim.readAdc(0, 1)-Stimjim.adcOffset10[0]) * MICROAMPS_PER_ADC;
        }
        if (PT->mode[1] < 2) {
            PT->voltage[1][i] += (Stimjim.readAdc(1, 0)-Stimjim.adcOffset10[1]) * MILLIVOLTS_PER_ADC;
            PT->current[1][i] += (Stimjim.readAdc(1, 1)-Stimjim.adcOffset10[1]) * MICROAMPS_PER_ADC;
        }

        if ( i + 1 < PT->nStages) {
            dac0val = PT->amplitude[0][i + 1] / ((!PT->mode[0]) ? MILLIVOLTS_PER_DAC : MICROAMPS_PER_DAC) + ((PT->mode[0]) ? Stimjim.currentOffsets[0] : Stimjim.voltageOffsets[0]);
            dac1val = PT->amplitude[1][i + 1] / ((!PT->mode[1]) ? MILLIVOLTS_PER_DAC : MICROAMPS_PER_DAC) + ((PT->mode[1]) ? Stimjim.currentOffsets[1] : Stimjim.voltageOffsets[1]);
        } else { // we're in the last stage, set DACs back to zero
            dac0val = (PT->mode[0]) ? Stimjim.currentOffsets[0] : Stimjim.voltageOffsets[0];
            dac1val = (PT->mode[1]) ? Stimjim.currentOffsets[1] : Stimjim.voltageOffsets[1];
        }

        // write to dacs
        if (PT->mode[0] < 2 && PT->mode[1] < 2) {
            Stimjim.writeToDacs(dac0val, dac1val);
        } else if (PT->mode[0] < 2) {
            Stimjim.writeToDac(0, dac0val);
        } else if (PT->mode[1] < 2) {
            Stimjim.writeToDac(1, dac1val);
        }
    }

    // switch outputs to ground
    if (PT->mode[0] < 2)
        Stimjim.setOutputMode(0, 3);

    if (PT->mode[1] < 2)
        Stimjim.setOutputMode(1, 3);

    PT->nPulses++;

    return 1;
}
int sinewave(volatile PulseTrain* PT)
{
    //check if the pulseTrain is finished; if so, exit
    if (micros() - PT->trainStartTime >= PT->duration)
        return 0;
    
    uint32_t t0, t=0;
    t0 = micros();

    int dac0val, dac1val;
    //float adcReadTime =  4.50 * ((PT->mode[0] < 2) + (PT->mode[1] < 2));  //16 bits at 10MHz, calibrated time is 4.5us
    //float dacWriteTime = 2.75 * ((PT->mode[0] < 2) + (PT->mode[1] < 2));  //24 bits at 30MHz, calibrated time is 2.75us
    //float totalDelayTime = dacWriteTime + adcReadTime + 0.5;
    dac0val = PT->wave[0].amplitude / ((!PT->mode[0]) ? MILLIVOLTS_PER_DAC : MICROAMPS_PER_DAC) + ((PT->mode[0]) ? Stimjim.currentOffsets[0] : Stimjim.voltageOffsets[0]);
    dac1val = PT->wave[1].amplitude / ((!PT->mode[1]) ? MILLIVOLTS_PER_DAC : MICROAMPS_PER_DAC) + ((PT->mode[1]) ? Stimjim.currentOffsets[1] : Stimjim.voltageOffsets[1]);

    float f1 = 8192*(PT-> wave[1].frequency)/1000000.;
    float f0 = 8192*(PT-> wave[0].frequency)/1000000.;
    float meresido0 = 0.25 / f0;
    float meresido1 = 0.25 / f1;
    int merendo0 = 1, merendo1 = 1;
    //8192-tablazat hossza(periodus)
    // /1000000- us-> s

    if (PT->mode[0] < 2 && PT->mode[1] < 2) {
        Stimjim.writeToDacs(dac0val, dac1val);
    } else if (PT->mode[0] < 2) {
        Stimjim.writeToDac(0, dac0val);
    } else if (PT->mode[1] < 2) {
        Stimjim.writeToDac(1, dac1val);
    }
    if (PT->mode[0] < 2)
        Stimjim.setOutputMode(0, PT->mode[0]);

    if (PT->mode[1] < 2)
        Stimjim.setOutputMode(1, PT->mode[1]);

    int c;
    for (c = 0; t < PT->stageDuration[0]; c++){
        t = micros()-t0;
    
        //delayMicroseconds(PT->stageDuration[i] - totalDelayTime); // empirically calibrated!

        // TODO start phase? offset phase?
        if (t < PT->stageDuration[0]) {
            dac0val = int(round(PT->wave[0].amplitude*sinetable[int(round(t*f0))&8191] / ((!PT->mode[0]) ? MILLIVOLTS_PER_DAC : MICROAMPS_PER_DAC))) + ((PT->mode[0]) ? Stimjim.currentOffsets[0] : Stimjim.voltageOffsets[0]);
            dac1val = int(round(PT->wave[1].amplitude*sinetable[int(round(t*f1))&8191] / ((!PT->mode[1]) ? MILLIVOLTS_PER_DAC : MICROAMPS_PER_DAC))) + ((PT->mode[1]) ? Stimjim.currentOffsets[1] : Stimjim.voltageOffsets[1]);
        } else { // we're in the last stage, set DACs back to zero
            dac0val = (PT->mode[0]) ? Stimjim.currentOffsets[0] : Stimjim.voltageOffsets[0];
            dac1val = (PT->mode[1]) ? Stimjim.currentOffsets[1] : Stimjim.voltageOffsets[1];
        }
        
        // read ADCs (this should be far from last DAC set, i.e., just before new DAC set)
        if (merendo0 && (meresido0 < t) && (PT->mode[0] < 2)) {
            PT->voltage[0][0] += (Stimjim.readAdc(0, 0)-Stimjim.adcOffset10[0]) * MILLIVOLTS_PER_ADC;
            PT->current[0][0] += (Stimjim.readAdc(0, 1)-Stimjim.adcOffset10[0]) * MICROAMPS_PER_ADC;
            merendo0 = 0;
        }
        if (merendo1 && (meresido1 < t) && (PT->mode[1] < 2)) {
            PT->voltage[1][0] += (Stimjim.readAdc(1, 0)-Stimjim.adcOffset10[1]) * MILLIVOLTS_PER_ADC;
            PT->current[1][0] += (Stimjim.readAdc(1, 1)-Stimjim.adcOffset10[1]) * MICROAMPS_PER_ADC;
            merendo1 = 0;
        }

        // write to dacs
        if (PT->mode[0] < 2 && PT->mode[1] < 2) {
            Stimjim.writeToDacs(dac0val, dac1val);
        } else if (PT->mode[0] < 2) {
            Stimjim.writeToDac(0, dac0val);
        } else if (PT->mode[1] < 2) {
            Stimjim.writeToDac(1, dac1val);
        }
    }
    
    // switch outputs to ground
    if (PT->mode[0] < 2)
        Stimjim.setOutputMode(0, 3);

    if (PT->mode[1] < 2)
        Stimjim.setOutputMode(1, 3);

    PT->nPulses++;

    return 1;
}

inline int abs(int x) {
    return (x < 0) ? -x : x;
}

void calculateResistance(volatile PulseTrain* PT, int stageIndex, int channelIndex, ResistanceResult& result) {
    result.voltage = PT->voltage[channelIndex][stageIndex] / PT->nPulses; // assuming we're interested in channel 0 for resistance calculation
    result.current = PT->current[channelIndex][stageIndex] / PT->nPulses;
    result.voltageLimited = (abs(result.voltage) > VOLTAGE_LIMIT_MV) || ((PT->mode[channelIndex] == 0) && (abs(result.voltage) * 8 < abs(PT->amplitude[channelIndex][stageIndex]) * 7));
    result.currentLimited = (PT->mode[channelIndex] == 1) && (abs(result.current) * 8 < abs(PT->amplitude[channelIndex][stageIndex]) * 7);
    if (result.current == 0) {
        result.resistance = -1; // indicate infinite resistance
        result.isKOhm = false;
    } else {
        result.isKOhm = abs(result.voltage) >= 100 * abs(result.current);
        if (result.isKOhm) {
            result.resistance = result.voltage / result.current; // resistance in kilo-ohms
        } else {
            result.resistance = result.voltage * 1000 / result.current; // resistance in ohms
        }
    }
}

void printResultSummary(volatile PulseTrain* PT)
{
    Serial.print("Train #"); Serial.print(train_count);
    Serial.print(" complete. Delivered "); Serial.print(PT->nPulses);
    if (PT->isWave) {
        Serial.println(" waves.");
    } else {
        Serial.println(" pulses.");
    }
    Serial.println("Current/Voltage by stage: ");
    Serial.println("           Ch0                Ch1 ");
    char str[200];
    int n = (PT->isWave ? 1 : PT->nStages);
    ResistanceResult res[2];
    for (int i = 0; i < n; i++) {
        calculateResistance(PT, i, 0, res[0]);
        calculateResistance(PT, i, 1, res[1]);
        sprintf(str, "Stage %d%6dmV%s,          %6dmV%s", i,
            res[0].voltage, res[0].voltageLimited ? "*" : " ",
            res[1].voltage, res[1].voltageLimited ? "*" : " ");
        Serial.println(str);
        sprintf(str, "       %6duA%s,          %6duA%s",
            res[0].current, res[0].currentLimited ? "*" : " ",
            res[1].current, res[1].currentLimited ? "*" : " ");
        Serial.println(str);
        sprintf(str, "       %6d%sOhm,       %6d%sOhm",
            res[0].resistance, res[0].isKOhm ? "k" : " ",
            res[1].resistance, res[1].isKOhm ? "k" : " ");
        Serial.println(str);
    }
}

#ifdef USE_DISPLAY
inline void setTextColor(bool inverted) {
    if (inverted) {
        display.setTextColor(SSD1306_BLACK, SSD1306_WHITE);
    } else {
        display.setTextColor(SSD1306_WHITE, SSD1306_BLACK);
    }
}

void displayResultSummary(volatile PulseTrain* PT)
{
  display.clearDisplay();

  display.setTextSize(1);             // Normal 1:1 pixel scale
  display.setCursor(0,0);             // Start at top-left corner
  setTextColor(true);
  display.print("#"); display.print(train_count);
  
  setTextColor(false);        // Draw white text
  display.print(" > "); display.print(PT->nPulses);
  if (PT->isWave) {
    display.println(" waves.");
  } else {
    display.println(" pulses.");
  }
  char str[200];
  ResistanceResult res[2];
  for (int i = 0; i < 1; i++) {  // single stage display
    calculateResistance(PT, i, 0, res[0]);
    calculateResistance(PT, i, 1, res[1]);
    //sprintf(str, "%-2d %6d%s %6d%s", i,
    //    PT->voltage[0][i] / PT->nPulses, "mV",
    //    PT->voltage[1][i] / PT->nPulses, "mV");
    sprintf(str, "%-2d", i); display.print(str);
    sprintf(str, " %6dmV", res[0].voltage);
    setTextColor(res[0].voltageLimited); display.print(str);
    sprintf(str, " %6dmV", res[1].voltage);
    setTextColor(res[1].voltageLimited); display.print(str);
    setTextColor(false); display.println("");

    //sprintf(str, "%2s %6d%s %6d%s", "",
    //    PT->current[0][i] / PT->nPulses, "uA",
    //    PT->current[1][i] / PT->nPulses, "uA");
    //display.println(str);
    sprintf(str, "   %6duA", res[0].current);
    setTextColor(res[0].currentLimited); display.print(str);
    sprintf(str, " %6duA", res[1].current);
    setTextColor(res[1].currentLimited); display.print(str);
    setTextColor(false); display.println("");

    sprintf(str, "   %6d%sO %6d%sO",
        res[0].resistance, res[0].isKOhm ? "k" : " ",
        res[1].resistance, res[1].isKOhm ? "k" : " ");
    display.println(str);
  }
  display.display();
}
#else
void displayResultSummary(volatile PulseTrain* PT) {}
#endif

void pulse0()
{
    // todo: it is not a good idea to keep IT busy while generating signal
    int ret = activePT0->isWave ? sinewave(activePT0) : pulse(activePT0);
    if (!ret) {

        IT0.end();
        train_count++;
        printResultSummary(activePT0);
        displayResultSummary(activePT0);

        if (activePT0->mode[0] < 2) {
            digitalWriteFast(LED0, LOW);
            digitalWriteFast(GPIO_10, LOW);
            if (trigOutput[0])
                digitalWriteFast(IN0, LOW);
        }

        if (activePT0->mode[1] < 2) {
            digitalWriteFast(LED1, LOW);
            digitalWriteFast(GPIO_11, LOW);
            if (trigOutput[1])
                digitalWriteFast(IN1, LOW);
        }
    }
}

void pulse1()
{
    // todo: it is not a good idea to keep IT busy while generating signal
    int ret = activePT1->isWave ? sinewave(activePT1) : pulse(activePT1);
    if (!ret) {

        IT1.end();
        train_count++;
        printResultSummary(activePT1);
        displayResultSummary(activePT1);

        if (activePT1->mode[0] < 2) {
            digitalWriteFast(LED0, LOW);
            digitalWriteFast(GPIO_10, LOW);
            if (trigOutput[0])
               digitalWriteFast(IN0, LOW);
        }

        if (activePT1->mode[1] < 2)
        {
            digitalWriteFast(LED1, LOW);
            digitalWriteFast(GPIO_11, LOW);
            if (trigOutput[1])
                digitalWriteFast(IN1, LOW);
        }
    }
}

void startIT0ViaInputTrigger()
{
    if (triggerTargetPTs[0] >= 0)
        startIT0(triggerTargetPTs[0]);
}

void startIT1ViaInputTrigger()
{
    if (triggerTargetPTs[1] >= 0)
        startIT1(triggerTargetPTs[1]);
}

void startIT0(int ptIndex)
{
    if (ptIndex < 0) {

        Serial.println("Forcing T train to stop");
        IT0.end();

        if (activePT0->mode[0] < 2) {
            digitalWriteFast(LED0, LOW);
            digitalWriteFast(GPIO_10, LOW);
            if (trigOutput[0])
              digitalWriteFast(IN0, LOW);
        }

        if (activePT0->mode[1] < 2) {
            digitalWriteFast(LED1, LOW);
            digitalWriteFast(GPIO_11, LOW);
            if (trigOutput[1])
                digitalWriteFast(IN1, LOW);
        }

        return;
    }

    activePT0 = clearPulseTrainHistory(&PTs[ptIndex]);
    activePT0->trainStartTime = micros();

    if (!IT0.begin(pulse0, activePT0->period))
        Serial.println("startIT0: failure to initiate IntervalTimer IT0");
    Serial.print("\r\nStarted T train with parameters of PulseTrain ");
    Serial.println(ptIndex);

    if (activePT0->mode[0] < 2) {
        digitalWriteFast(LED0, HIGH);
        digitalWriteFast(GPIO_10, HIGH);
        if (trigOutput[0])
          digitalWriteFast(IN0, HIGH);
    }

    if (activePT0->mode[1] < 2){
        digitalWriteFast(LED1, HIGH);
        digitalWriteFast(GPIO_11, HIGH);
        if (trigOutput[1])
          digitalWriteFast(IN1, HIGH);
    }

    pulse0(); //intervalTimer starts with delay - we want to start with pulse!
}

void startIT1(int ptIndex)
{
    if (ptIndex < 0) {

        Serial.println("Forcing U train to stop");

        if (activePT1->mode[0] < 2) {
            digitalWriteFast(LED0, LOW);
            digitalWriteFast(GPIO_10, LOW);
            if (trigOutput[0])
                digitalWriteFast(IN0, LOW);

        }

        if (activePT1->mode[1] < 2) {
            digitalWriteFast(LED1, LOW);
            digitalWriteFast(GPIO_11, LOW);
            if (trigOutput[1])
                digitalWriteFast(IN1, LOW);
        }

        IT1.end();
        return;
    }

    activePT1 = clearPulseTrainHistory(&PTs[ptIndex]);
    activePT1->trainStartTime = micros();

    if (!IT1.begin(pulse1, activePT1->period))
        Serial.println("startIT1: failure to initiate IntervalTimer IT1");
    Serial.print("\r\nStarted U train with parameters of PulseTrain "); 
    Serial.println(ptIndex);

    if (activePT1->mode[0] < 2) {
        digitalWriteFast(LED0, HIGH);
        digitalWriteFast(GPIO_10, HIGH);
        if (trigOutput[0])
            digitalWriteFast(IN0, HIGH);

    }
    if (activePT1->mode[1] < 2) {
        digitalWriteFast(LED1, HIGH);
        digitalWriteFast(GPIO_11, HIGH);
        if (trigOutput[1])
            digitalWriteFast(IN1, HIGH);
    }

    pulse1(); //intervalTimer starts with delay - we want to start with pulse!
}

void printPulseTrainParameters(int i)
{
    if (i < 0 || i >= PT_ARRAY_LENGTH) {
        Serial.println("Invalid PulseTrain array index.");
        return;
    }

    const char modeStrings[4][40] = {"Voltage output", "Current output", "No output (high-Z)", "No output (grounded)"};
    Serial.println("----------------------------------");
    char str[200];
    sprintf(str, "Parameters for PulseTrain[%d]\r\n  mode[ch0]: %d (%s)\r\n  mode[ch1]: %d (%s)\r\n",
        i, PTs[i].mode[0], modeStrings[PTs[i].mode[0]], PTs[i].mode[1], modeStrings[PTs[i].mode[1]]);
    Serial.print(str);
    sprintf(str, "  period:    %lu usec (%0.3f sec, %0.3f Hz)\r\n  duration:  %lu usec (%0.3f sec)\r\n",
          PTs[i].period, 0.000001 * PTs[i].period, 1000000.0 / PTs[i].period, PTs[i].duration, 0.000001 * PTs[i].duration);
    Serial.print(str);
    Serial.println("\r\n  stage    duration     output0   output1");
    if(PTs[i].isWave) {
        sprintf(str, "   %2d  %7d usec %8d%s %8d%s\r\n", 0, PTs[i].stageDuration[0],
            PTs[i].wave[0].amplitude, (PTs[i].mode[0] == 0) ? "mV" : "uA",
            PTs[i].wave[1].amplitude, (PTs[i].mode[1] == 0) ? "mV" : "uA");
        Serial.print(str);
        sprintf(str, "   %2s  %7s      %8d%s %8d%s\r\n", "", "",
            PTs[i].wave[0].frequency, "Hz", PTs[i].wave[1].frequency, "Hz");
        Serial.print(str);
        sprintf(str, "   %2s  %7s      %8d%s %8d%s\r\n", "", "",
            PTs[i].wave[0].phase > 0, "° ", PTs[i].wave[1].phase, "° ");
        Serial.print(str);
    } else {
    for (int j = 0; j < PTs[i].nStages; j++) {
        sprintf(str, "   %2d  %7d usec %8d%s %8d%s\r\n", j, PTs[i].stageDuration[j],
            PTs[i].amplitude[0][j], (PTs[i].mode[0] == 0) ? "mV" : "uA",
            PTs[i].amplitude[1][j], (PTs[i].mode[1] == 0) ? "mV" : "uA");
        Serial.print(str);
    }}

    Serial.println("----------------------------------\r\n");
}

volatile PulseTrain* clearPulseTrainHistory(volatile PulseTrain* PT)
{
      PT->nPulses = 0;
      memset((void *) PT->current, 0, 2*MAX_NUM_STAGES*sizeof(int));
      memset((void *) PT->voltage, 0, 2*MAX_NUM_STAGES*sizeof(int));
      return (PT);
}

void sayHello() {
    Serial.println("Hello!");
}

void setup()
{
  
    Serial.begin(112500);
    bytesRecvd = 0;
    delay(1000);
    float reciprok;

    //sine table values
    sinetable[0]= 0;
    reciprok = 1/8192.;

    for(int i=0; i<8192; i++){
      sinetable[i] = sin(2*pi*i*reciprok);
    }

    #ifdef USE_DISPLAY
    // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
    if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
      Serial.println(F("SSD1306 allocation failed"));
    }

    // Show initial display buffer contents on the screen --
    // the library initializes this with an Adafruit splash screen.
    display.display();
    #endif
    
    Serial.println("Booting StimJim on Teensy 3.5!");

    Stimjim.begin();

    Serial.print("User definitions take "); Serial.print(sizeof(PTs)); Serial.println(" bytes");

    for (int i = 0; i < PT_ARRAY_LENGTH; i++) {
        PTs[i].mode[0] = 3;
        PTs[i].mode[1] = 3;
        PTs[i].period = 10000;
        PTs[i].duration = 500000;
        PTs[i].nStages = 0;
    }

      Serial.println("Initializing triggers inputs...");
      pinMode(IN0, OUTPUT);
      pinMode(IN1, OUTPUT);
      triggerTargetPTs[0] = -1; // initialize target to -1 so that triggers do nothing
      triggerTargetPTs[1] = -1;
      trigOutput[0] = true;
      trigOutput[1] = true;

      // GPIO
      pinMode(GPIO_10, OUTPUT);

      IT0.priority(64);
      IT1.priority(64);
      Serial.flush();

      loadTriggersEEPROM();

      // TODO: protection against rolling buttons
      // TODO: change hardware to switcheng logical low and use internal pullup
      pinMode(Btn0, INPUT);
      attachInterrupt(Btn0, sayHello, RISING);
      pinMode(Btn1, INPUT);
      attachInterrupt(Btn1, startIT0ViaInputTrigger, RISING);
      pinMode(Btn2, INPUT);
      attachInterrupt(Btn2, startIT1ViaInputTrigger, RISING);

      // print offset values for user reference
      char str[200];
      sprintf(str, "ADC offsets (+-2.5V): %f, %f\r\nADC offsets (+-10V): %f, %f\r\ncurrent offsets: %d, %d\r\nvoltage offsets: %d, %d\r\n",
        Stimjim.adcOffset25[0],Stimjim.adcOffset25[1], Stimjim.adcOffset10[0],Stimjim.adcOffset10[1],
        Stimjim.currentOffsets[0], Stimjim.currentOffsets[1], Stimjim.voltageOffsets[0], Stimjim.voltageOffsets[1] );
      Serial.println(str);

      Serial.println("Ready to go!\r\n\r\n");

}

void loop()
{
    if (Serial.available() > 0) {

        Serial.readBytes(comBuf + bytesRecvd, 1); // read one byte into the buffer
        bytesRecvd++; // keep track of the number of characters we've read!

        if (comBuf[bytesRecvd - 1] == '\n') { // termination character for string - we received a full command!
            int ptIndex = 0;

            // remove \n and possibly \r from end of comBuf
            comBuf[bytesRecvd - 1] = '\0';
            if (bytesRecvd >= 2 && comBuf[bytesRecvd - 2] == '\r')
                comBuf[bytesRecvd - 2] = '\0';

            if ((comBuf[0] == 'S') or (comBuf[0] == 'W')) {
                // fixme, this overwrites part of parameters in case of corrupted input
                sscanf(comBuf + 1, "%d,", &ptIndex);
                if (ptIndex < 0 || ptIndex >= PT_ARRAY_LENGTH) {
                    Serial.println("Invalid PulseTrain index.");
                    bytesRecvd = 0;
                    return;
                }

                int m = sscanf(comBuf + 1, "%*d,%u,%u,%lu,%lu;",
                    &(PTs[ptIndex].mode[0]),
                    &(PTs[ptIndex].mode[1]),
                    &(PTs[ptIndex].period),
                    &(PTs[ptIndex].duration));
                if (PTs[ptIndex].mode[0] < 0 || PTs[ptIndex].mode[0] > 3)
                    PTs[ptIndex].mode[0] = 3;
                if (PTs[ptIndex].mode[1] < 0 || PTs[ptIndex].mode[1] > 3)
                    PTs[ptIndex].mode[1] = 3;

                if (m == 4) {
                  // valid mode, period, and duration parameters were read, now read stage parameters
                  PTs[ptIndex].isWave = (comBuf[0] == 'W');
                  PTs[ptIndex].nStages = 0;
                  char *token = strtok(comBuf + 1, ";");
                  token = strtok(NULL, ";"); //move to the 2nd segment delimited by ";"

                    while (token != NULL) {
                        m = sscanf(token, "%d,%d,%u",
                            &(PTs[ptIndex].amplitude[0][PTs[ptIndex].nStages]),
                            &(PTs[ptIndex].amplitude[1][PTs[ptIndex].nStages]),
                            &(PTs[ptIndex].stageDuration[PTs[ptIndex].nStages]));
                        if (m != 3)
                            break;
                        PTs[ptIndex].nStages++;
                        token = strtok(NULL, ";");
                    }
                }

                printPulseTrainParameters(ptIndex);

            } else if (comBuf[0] == 'T' || comBuf[0] == 'U') {

                ptIndex = atoi(comBuf + 1);
                if (ptIndex >= PT_ARRAY_LENGTH) {
                    Serial.println("Invalid PulseTrain index.");
                    bytesRecvd = 0;
                    return;
                }

                if (comBuf[0] == 'T')
                    startIT0(ptIndex);
                if (comBuf[0] == 'U')
                    startIT1(ptIndex);
            

            } else if (comBuf[0] == 'B') {

                Stimjim.getAdcOffsets();

            } else if (comBuf[0] == 'C') {

                Stimjim.getCurrentOffsets();
                Stimjim.getVoltageOffsets();

            } else if (comBuf[0] == 'D') { // print offset values for user reference
                char str[1000];
                sprintf(str, "ADC offsets (+-2.5V): %f, %f\r\nADC offsets (+-10V): %f, %f\r\ncurrent offsets: %d, %d\r\nvoltage offsets: %d, %d\r\n",
                    Stimjim.adcOffset25[0],Stimjim.adcOffset25[1], Stimjim.adcOffset10[0],Stimjim.adcOffset10[1],
                    Stimjim.currentOffsets[0], Stimjim.currentOffsets[1], Stimjim.voltageOffsets[0], Stimjim.voltageOffsets[1] );
                Serial.println(str);

            } else if (comBuf[0] == 'P') {
                saveTriggersEEPROM();

            } else if (comBuf[0] == 'R') {

                int trigSrc = 0, output = 0;
                sscanf(comBuf + 1, "%d,%d,%d", &trigSrc, &ptIndex, &output );

                if (ptIndex >= PT_ARRAY_LENGTH) {
                    Serial.println("Invalid PulseTrain index.");
                    bytesRecvd = 0;
                    return;
                }

                setTriggers(ptIndex, trigSrc, output);

            } else if (comBuf[0] == 'M') {

                int channel = 0, mode = 0;
                sscanf(comBuf + 1, "%d,%d", &channel, &mode );
                Stimjim.setOutputMode(channel, mode);
                Serial.print("Set channel "); Serial.print(channel); Serial.print(" to mode "); Serial.println(mode);

            } else if (comBuf[0] == 'V') {

                int channel = 0, amp = 0;
                sscanf(comBuf + 1, "%d,%d", &channel, &amp );
                int dacVal = 1.0 * amp / MILLIVOLTS_PER_DAC + Stimjim.voltageOffsets[channel];
                if (dacVal <= 32767 && dacVal >= -32768) {
                    Stimjim.writeToDac(channel, dacVal);
                    Serial.print("Set channel "); Serial.print(channel); Serial.print(" to amplitude "); Serial.print(amp);
                    Serial.print(" mV (dac value "); Serial.print(dacVal); Serial.println(").");
                } else {
                    Serial.print(dacVal); Serial.println(" is out of range.");
                }

            } else if (comBuf[0] == 'A') {

                int channel = 0, amp = 0;
                sscanf(comBuf + 1, "%d,%d", &channel, &amp );

                if (amp <= 32767 && amp >= -32768) {
                    Stimjim.writeToDac(channel, amp);
                    Serial.print("Set channel "); Serial.print(channel); Serial.print(" to amplitude "); Serial.println(amp);
                } else {
                    Serial.print(amp); Serial.println(" is out of range.");
                }

            } else if (comBuf[0] == 'E') {
                int channel = 0, line = 0;
                sscanf(comBuf + 1, "%d,%d", &channel, &line );
                int val = Stimjim.readAdc(channel, line);
                int valRealUnits = (val - Stimjim.adcOffset10[channel]) * (line ? MICROAMPS_PER_ADC : MILLIVOLTS_PER_ADC);
                Serial.print("Read value: "); Serial.print(val); Serial.print(" ("); Serial.print(valRealUnits);
                Serial.println((line)?"uA)":"mV)");
            }

            bytesRecvd = 0; // reset the pointer!
        }
    }

}
