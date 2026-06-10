// main.cpp — SMA Variable Voltage Controller firmware (Arduino Uno / Mega 2560)
//
// Board: PN 008-300A-0003 (Hybrid Dynamic Robotics Lab, U-Michigan)
// See doc/Schematic PDF_[No Variations].pdf and include/config.h.
//
// Signal chain:
//   MCP4018 digipot wiper -> TPS7A57 LDO REF -> V_LDO (~0.5..5.2 V, monotonic w/ code)
//   V_LDO -> 0.33 ohm shunt (R1) -> SMA -> Q1 (gated by SMA_EN) -> GND
//   INA296A1 (gain 10 V/V) across shunt -> ISNS_OUT -> A0
//   V_LDO via R9/R11 47k/47k divider (/2) -> A1
//
// Serial console @ SERIAL_BAUD. Type `help` for commands.
//
// Calibration maps each wiper code (0..127) to the measured unloaded V_LDO and
// stores the table in EEPROM so it survives power cycles ("keep it onboard").

#include <Arduino.h>
#include <Wire.h>
#include <EEPROM.h>
#include "config.h"
#include "MCP4018.h"

// ----------------------------------------------------------------------------
// Globals
// ----------------------------------------------------------------------------
MCP4018 digipot;

float    g_vref = ADC_VREF_DEFAULT;     // ADC reference voltage actually used
uint16_t g_cal_mv[WIPER_MAX + 1];       // calibration: code -> V_LDO in millivolts
bool     g_cal_valid = false;           // true once a valid table is loaded/built
uint8_t  g_wiper = 0;                   // last commanded wiper code
bool     g_sma_en = false;              // SMA_EN state

// Minimum current (A) below which R_SMA cannot be resolved (avoids divide-by-zero
// and noise blow-up). At ~0.5 LSB of ISNS the current is in the low-mA range.
constexpr float MIN_CURRENT_FOR_R = 0.005f;   // 5 mA

// EEPROM layout
constexpr uint32_t EE_MAGIC   = 0x534D4143UL; // 'S''M''A''C'
constexpr uint8_t  EE_VERSION = 1;
constexpr int      EE_MAGIC_ADDR = 0;   // uint32
constexpr int      EE_VER_ADDR   = 4;   // uint8
constexpr int      EE_VREF_ADDR  = 6;   // float (aligned)
constexpr int      EE_TABLE_ADDR = 16;  // uint16[128]

// ----------------------------------------------------------------------------
// Low-level helpers
// ----------------------------------------------------------------------------
static float analogReadAvg(uint8_t pin) {
    uint32_t sum = 0;
    for (uint8_t i = 0; i < ADC_AVG_SAMPLES; ++i) {
        sum += analogRead(pin);
    }
    return (float)sum / (float)ADC_AVG_SAMPLES;
}

static inline float adcToVolts(float counts) {
    return counts / ADC_COUNTS * g_vref;
}

// V_LDO at the LDO output node (un-does the /2 sense divider).
static float readVldo() {
    return adcToVolts(analogReadAvg(PIN_VLDO_SENSE)) * VLDO_DIVIDER;
}

// INA296A1 output voltage (= I * Rshunt * gain, ref grounded).
static float readIsnsVolts() {
    return adcToVolts(analogReadAvg(PIN_ISNS));
}

// SMA current (A) = Visns / (gain * Rshunt).
static float readCurrent() {
    return readIsnsVolts() / (INA_GAIN * RSHUNT);
}

static void setSmaEnable(bool on) {
    g_sma_en = on;
    digitalWrite(PIN_SMA_EN, on ? HIGH : LOW);
}

// Set the wiper directly by code.
static bool setWiperCode(uint8_t code) {
    if (code > WIPER_MAX) code = WIPER_MAX;
    bool ok = digipot.setWiper(code);
    if (ok) g_wiper = code;
    return ok;
}

// ----------------------------------------------------------------------------
// EEPROM calibration storage
// ----------------------------------------------------------------------------
static void saveCalToEeprom() {
    EEPROM.put(EE_MAGIC_ADDR, EE_MAGIC);
    EEPROM.put(EE_VER_ADDR,   EE_VERSION);
    EEPROM.put(EE_VREF_ADDR,  g_vref);
    int addr = EE_TABLE_ADDR;
    for (int code = 0; code <= WIPER_MAX; ++code) {
        EEPROM.put(addr, g_cal_mv[code]);
        addr += sizeof(uint16_t);
    }
}

static bool loadCalFromEeprom() {
    uint32_t magic = 0; uint8_t ver = 0;
    EEPROM.get(EE_MAGIC_ADDR, magic);
    EEPROM.get(EE_VER_ADDR,   ver);
    if (magic != EE_MAGIC || ver != EE_VERSION) return false;

    float vref = ADC_VREF_DEFAULT;
    EEPROM.get(EE_VREF_ADDR, vref);
    if (vref > 1.0f && vref < 6.0f) g_vref = vref;

    int addr = EE_TABLE_ADDR;
    for (int code = 0; code <= WIPER_MAX; ++code) {
        EEPROM.get(addr, g_cal_mv[code]);
        addr += sizeof(uint16_t);
    }
    g_cal_valid = true;
    return true;
}

// ----------------------------------------------------------------------------
// Calibration sweep
// ----------------------------------------------------------------------------
static void runCalibration() {
    Serial.println(F("# Calibration sweep starting (SMA disabled)."));
    setSmaEnable(false);             // unloaded LDO measurement
    Serial.println(F("code,vldo_v")); // CSV header (capturable)

    for (int code = 0; code <= WIPER_MAX; ++code) {
        setWiperCode((uint8_t)code);
        delay(WIPER_SETTLE_MS);
        float v = readVldo();
        if (v < 0) v = 0;
        g_cal_mv[code] = (uint16_t)(v * 1000.0f + 0.5f);
        Serial.print(code);
        Serial.print(',');
        Serial.println(v, 4);
    }

    g_cal_valid = true;
    saveCalToEeprom();
    setWiperCode(0);                 // return to lowest output
    Serial.println(F("# Calibration complete and stored in EEPROM."));
}

// Find the wiper code whose calibrated V_LDO is closest to `target` volts.
// Works whether the table is increasing or decreasing in code.
static bool codeForVoltage(float target, uint8_t &outCode, float &achievedV) {
    if (!g_cal_valid) return false;
    uint16_t targetMv = (uint16_t)(target * 1000.0f + 0.5f);
    uint8_t best = 0;
    uint32_t bestErr = 0xFFFFFFFF;
    for (int code = 0; code <= WIPER_MAX; ++code) {
        int32_t d = (int32_t)g_cal_mv[code] - (int32_t)targetMv;
        uint32_t err = (d < 0) ? (uint32_t)(-d) : (uint32_t)d;
        if (err < bestErr) { bestErr = err; best = (uint8_t)code; }
    }
    outCode = best;
    achievedV = g_cal_mv[best] / 1000.0f;
    return true;
}

// Set V_LDO to the closest calibrated target. Returns the chosen code or -1 on error.
static int setVoltage(float target) {
    uint8_t code; float achieved;
    if (!codeForVoltage(target, code, achieved)) {
        Serial.println(F("# ERROR: no calibration table. Run `cal` first."));
        return -1;
    }
    setWiperCode(code);
    return code;
}

// ----------------------------------------------------------------------------
// Cycle runner with CSV logging
// ----------------------------------------------------------------------------
// One phase: set `volts`, then sample/log for `durMs`. Returns false if aborted.
static bool runPhase(const char *phase, float volts, uint32_t durMs, uint32_t cycleStart) {
    int code = setVoltage(volts);
    if (code < 0) return false;

    uint32_t phaseStart = millis();
    uint32_t nextLog = phaseStart;
    while ((uint32_t)(millis() - phaseStart) < durMs) {
        if (Serial.available()) {            // any keypress aborts
            while (Serial.available()) Serial.read();
            return false;
        }
        if ((int32_t)(millis() - nextLog) >= 0) {
            float vldo = readVldo();
            float visns = readIsnsVolts();
            float cur = visns / (INA_GAIN * RSHUNT);
            Serial.print(millis() - cycleStart); Serial.print(',');
            Serial.print(phase);               Serial.print(',');
            Serial.print(volts, 3);            Serial.print(',');
            Serial.print(code);                Serial.print(',');
            Serial.print(vldo, 4);             Serial.print(',');
            Serial.print(visns, 4);            Serial.print(',');
            Serial.print(cur, 4);               Serial.print(',');
            // SMA resistance = (V_LDO - I*Rshunt)/I; 'nan' when current too low
            if (cur >= MIN_CURRENT_FOR_R) Serial.println((vldo - cur * RSHUNT) / cur, 3);
            else                          Serial.println(F("nan"));
            nextLog += LOG_INTERVAL_MS;
        }
    }
    return true;
}

static void runCycle(float vHot, uint32_t tHot, float vCool, uint32_t tCool, uint16_t n) {
    if (!g_cal_valid) {
        Serial.println(F("# ERROR: no calibration table. Run `cal` first."));
        return;
    }
    Serial.print(F("# BEGIN cycle heat=")); Serial.print(vHot, 3);
    Serial.print(F("V/"));   Serial.print(tHot);
    Serial.print(F("ms cool=")); Serial.print(vCool, 3);
    Serial.print(F("V/"));   Serial.print(tCool);
    Serial.print(F("ms x")); Serial.println(n);
    Serial.println(F("t_ms,phase,target_v,wiper,vldo_v,isns_v,current_a,r_sma_ohm"));

    setSmaEnable(true);
    // Discard the command line's trailing EOL (e.g. the '\n' after '\r') so it is
    // not mistaken for an abort keypress on the very first phase iteration.
    while (Serial.available()) Serial.read();
    uint32_t cycleStart = millis();
    bool aborted = false;
    for (uint16_t i = 0; i < n && !aborted; ++i) {
        if (!runPhase("HEAT", vHot,  tHot,  cycleStart)) { aborted = true; break; }
        if (!runPhase("COOL", vCool, tCool, cycleStart)) { aborted = true; break; }
    }
    setVoltage(vCool < 0.5f ? 0.5f : vCool); // park low
    setWiperCode(0);
    setSmaEnable(false);
    Serial.println(aborted ? F("# END cycle (aborted by keypress)")
                           : F("# END cycle"));
}

// ----------------------------------------------------------------------------
// Command console
// ----------------------------------------------------------------------------
static void printHelp() {
    Serial.println(F("Commands:"));
    Serial.println(F("  help                  - this list"));
    Serial.println(F("  scan                  - probe digipot on I2C, report addr/variant"));
    Serial.println(F("  cal                   - sweep wiper, measure V_LDO, store table in EEPROM"));
    Serial.println(F("  caldump               - print stored calibration table as CSV"));
    Serial.println(F("  setv <volts>          - set V_LDO to nearest calibrated voltage"));
    Serial.println(F("  setcode <0-127>       - set wiper code directly"));
    Serial.println(F("  read                  - print V_LDO, ISNS, current, R_SMA once"));
    Serial.println(F("  en <0|1>              - SMA enable off/on"));
    Serial.println(F("  cycle <vH> <tH> <vC> <tC> <n>  - heat/cool cycle w/ CSV log (ms)"));
    Serial.println(F("  vref [volts]          - show or set ADC reference voltage"));
    Serial.println(F("Example: cycle 3.5 100 0.5 2000 5"));
}

static void doScan() {
    bool ok = digipot.detect();
    if (ok) {
        Serial.print(F("# Digipot FOUND at 0x"));
        Serial.print(digipot.address(), HEX);
        Serial.print(F(" ("));
        Serial.print(digipot.variantName());
        Serial.println(F(")"));
    } else {
        Serial.println(F("# Digipot NOT FOUND. Check wiring (SDA/SCL, pull-ups), power."));
    }
}

static void doCaldump() {
    if (!g_cal_valid) { Serial.println(F("# No calibration table. Run `cal`.")); return; }
    Serial.print(F("# vref=")); Serial.println(g_vref, 4);
    Serial.println(F("code,vldo_v"));
    for (int code = 0; code <= WIPER_MAX; ++code) {
        Serial.print(code); Serial.print(',');
        Serial.println(g_cal_mv[code] / 1000.0f, 4);
    }
}

static void doRead() {
    float vldo = readVldo();
    float visns = readIsnsVolts();
    float cur = visns / (INA_GAIN * RSHUNT);
    // SMA voltage = shunt low-side node (V_LDO - I*Rshunt); resistance neglects MOSFET Rds(on).
    float vsma = vldo - cur * RSHUNT;
    Serial.print(F("V_LDO=")); Serial.print(vldo, 4); Serial.print(F(" V  "));
    Serial.print(F("ISNS=")); Serial.print(visns, 4); Serial.print(F(" V  "));
    Serial.print(F("I_SMA=")); Serial.print(cur, 4);  Serial.print(F(" A  "));
    Serial.print(F("R_SMA="));
    if (cur >= MIN_CURRENT_FOR_R) { Serial.print(vsma / cur, 3); Serial.print(F(" ohm  ")); }
    else                         { Serial.print(F("n/a  ")); }   // too little current to resolve
    Serial.print(F("wiper=")); Serial.print(g_wiper);
    Serial.print(F("  SMA_EN=")); Serial.println(g_sma_en ? F("on") : F("off"));
}

// Parse a whitespace-delimited command line.
static void handleLine(char *line) {
    char *cmd = strtok(line, " \t\r\n");
    if (!cmd) return;

    if (!strcmp(cmd, "help")) {
        printHelp();
    } else if (!strcmp(cmd, "scan")) {
        doScan();
    } else if (!strcmp(cmd, "cal")) {
        runCalibration();
    } else if (!strcmp(cmd, "caldump")) {
        doCaldump();
    } else if (!strcmp(cmd, "read")) {
        doRead();
    } else if (!strcmp(cmd, "setv")) {
        char *a = strtok(NULL, " \t\r\n");
        if (!a) { Serial.println(F("# usage: setv <volts>")); return; }
        float v = atof(a);
        int code = setVoltage(v);
        if (code >= 0) {
            Serial.print(F("# set ~")); Serial.print(g_cal_mv[code] / 1000.0f, 3);
            Serial.print(F(" V (req ")); Serial.print(v, 3);
            Serial.print(F(", code ")); Serial.print(code); Serial.println(F(")"));
        }
    } else if (!strcmp(cmd, "setcode")) {
        char *a = strtok(NULL, " \t\r\n");
        if (!a) { Serial.println(F("# usage: setcode <0-127>")); return; }
        int code = atoi(a);
        if (code < 0) code = 0;
        if (code > WIPER_MAX) code = WIPER_MAX;
        if (setWiperCode((uint8_t)code)) {
            Serial.print(F("# wiper=")); Serial.println(code);
        } else {
            Serial.println(F("# ERROR: write failed (digipot not found?)"));
        }
    } else if (!strcmp(cmd, "en")) {
        char *a = strtok(NULL, " \t\r\n");
        if (!a) { Serial.println(F("# usage: en <0|1>")); return; }
        setSmaEnable(atoi(a) != 0);
        Serial.print(F("# SMA_EN=")); Serial.println(g_sma_en ? F("on") : F("off"));
    } else if (!strcmp(cmd, "vref")) {
        char *a = strtok(NULL, " \t\r\n");
        if (a) {
            float v = atof(a);
            if (v > 1.0f && v < 6.0f) { g_vref = v; Serial.print(F("# vref set ")); }
            else { Serial.print(F("# vref out of range, unchanged ")); }
        } else {
            Serial.print(F("# vref="));
        }
        Serial.println(g_vref, 4);
    } else if (!strcmp(cmd, "cycle")) {
        char *a1 = strtok(NULL, " \t\r\n");
        char *a2 = strtok(NULL, " \t\r\n");
        char *a3 = strtok(NULL, " \t\r\n");
        char *a4 = strtok(NULL, " \t\r\n");
        char *a5 = strtok(NULL, " \t\r\n");
        if (!a1 || !a2 || !a3 || !a4 || !a5) {
            Serial.println(F("# usage: cycle <vHeat> <tHeat_ms> <vCool> <tCool_ms> <n>"));
            return;
        }
        runCycle(atof(a1), (uint32_t)atol(a2), atof(a3), (uint32_t)atol(a4),
                 (uint16_t)atoi(a5));
    } else {
        Serial.print(F("# unknown command: ")); Serial.println(cmd);
        Serial.println(F("# type `help`"));
    }
}

// Non-blocking line reader.
static char  s_buf[64];
static uint8_t s_len = 0;

static void pollSerial() {
    while (Serial.available()) {
        char c = (char)Serial.read();
        if (c == '\n' || c == '\r') {
            if (s_len > 0) {
                s_buf[s_len] = '\0';
                handleLine(s_buf);
                s_len = 0;
            }
        } else if (s_len < sizeof(s_buf) - 1) {
            s_buf[s_len++] = c;
        }
    }
}

// ----------------------------------------------------------------------------
// Arduino entry points
// ----------------------------------------------------------------------------
void setup() {
    pinMode(PIN_SMA_EN, OUTPUT);
    setSmaEnable(false);             // SMA off until commanded

    Serial.begin(SERIAL_BAUD);
    Wire.begin();

    // Put the LDO at its lowest setting before anything else (power-on default
    // wiper is mid-scale, so this avoids an unexpected mid voltage at boot).
    bool found = digipot.begin();
    setWiperCode(0);

    Serial.println();
    Serial.println(F("# SMA Variable Voltage Controller (PN 008-300A-0003)"));
    if (found) {
        Serial.print(F("# Digipot at 0x")); Serial.print(digipot.address(), HEX);
        Serial.print(F(" (")); Serial.print(digipot.variantName()); Serial.println(F(")"));
    } else {
        Serial.println(F("# WARNING: digipot not detected on I2C bus."));
    }

    if (loadCalFromEeprom()) {
        Serial.print(F("# Loaded calibration from EEPROM (vref="));
        Serial.print(g_vref, 3); Serial.println(F(")"));
    } else {
        Serial.println(F("# No stored calibration. Run `cal`."));
    }
    Serial.println(F("# Type `help` for commands."));
}

void loop() {
    pollSerial();
}
