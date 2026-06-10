// =============================================================================
// MCP4728 DAC -> TPS7A5701 LDO Programmable Voltage Controller (Arduino Uno)
// =============================================================================
//
// Architecture:
//   A 33-point calibration sweep characterizes the DAC-code -> LDO-output
//   transfer curve. At runtime, target voltages are converted to DAC codes via
//   binary search + linear interpolation in the table -- single DAC write per
//   set point, no closed-loop search. Expected resolution ~1 mV at the LDO
//   output (12-bit DAC * VDD * divider, with TPS7A5701 SET-pin gain ~unity).
//
// Wiring:
//   Uno SDA/SCL  -->  MCP4728 SDA/SCL  (I2C, A4/A5)
//   Uno 5V/GND   -->  MCP4728 VDD/GND
//   D2           -->  MOSFET gate (load enable; HIGH except during cal)
//
//   MCP4728 VA --[2k]--+--[10k]-- GND   (divider -> LDO SET pin)
//                       |
//                     V_mid --> TPS7A5701 SET
//
//   LDO_OUT --[R]--+--[R]-- GND   (1:1 divider, halves voltage for ADC)
//                   |
//                   A0
//
// Calibration:
//   `cal`   -- drives MOSFET LOW, sweeps DAC, measures V_LDO at 33 points,
//              auto-trims to the regulating window, stores in RAM (no save).
//   `save`  -- writes the RAM table to EEPROM (explicit; never automatic).
//   `load`  -- reads EEPROM into RAM (also done automatically at boot).
//
// Commands:
//   <voltage>    Set LDO output (table lookup, one DAC write)
//   set <V>      Same as above
//   read         Read LDO output now
//   code <N>     Set raw DAC code 0-4095 (open-loop debug)
//   cal          Run 33-pt calibration sweep (load disconnected during)
//   save         Save RAM cal table to EEPROM
//   load         Reload cal table from EEPROM
//   info         Print current state
//   sweep [mV]   Open-loop sweep, prints DAC code vs LDO output
//   csv   [mV]   Same as sweep, CSV format
//   aref <V>     Set ADC Vref (used to interpret A0 readings)
// =============================================================================

#include <Wire.h>
#include <Adafruit_MCP4728.h>
#include <EEPROM.h>

Adafruit_MCP4728 mcp;

// --- Pins ---
const int MOSFET_PIN = 2;
const int FB_PIN     = A0;

// --- Circuit parameters ---
const float VDD           = 5.5;
const float R_TOP         = 2000.0;
const float R_BOT         = 10000.0;
const float DIVIDER_RATIO = R_BOT / (R_TOP + R_BOT);  // 0.8333
const float ADC_FB_SCALE  = 2.0;                      // x2 to undo 1:1 FB

// --- Debug sweep range (DAC output voltage) ---
const float SWEEP_MIN = 0.5;
const float SWEEP_MAX = 5.0;

// --- ADC ---
const int   ADC_SAMPLES = 64;
const int   SETTLE_MS   = 150;
float       adc_vref    = 5.0;

// --- Calibration ---
const uint8_t CAL_N = 33;
struct CalPoint {
  uint16_t code;
  float    vldo;
};
CalPoint  calTable[CAL_N];
bool      calValid = false;
uint8_t   calStart = 0;            // first idx of regulating window
uint8_t   calEnd   = CAL_N - 1;    // last  idx of regulating window
float     vldoMin  = 0;
float     vldoMax  = 0;
uint16_t  calCount = 0;            // bumps on every save()

// --- EEPROM layout ---
const uint16_t EEPROM_MAGIC   = 0xCAFE;
const uint8_t  EEPROM_VERSION = 1;
const int      EEPROM_ADDR    = 0;
struct EEPROMHeader {
  uint16_t magic;
  uint8_t  version;
  uint8_t  n;
  uint16_t count;
  uint8_t  calStart;
  uint8_t  calEnd;
};
// Layout: header (8 B) + N * CalPoint (6 B) = 8 + 198 = 206 B for N=33.

// --- DAC state ---
uint16_t currentCode = 0;

// ============================================================================
// ADC helpers
// ============================================================================
float readADC(int pin) {
  analogRead(pin);
  delay(2);
  long sum = 0;
  for (int i = 0; i < ADC_SAMPLES; i++) sum += analogRead(pin);
  return ((float)sum / ADC_SAMPLES / 1023.0) * adc_vref;
}

float readLDO() {
  return readADC(FB_PIN) * ADC_FB_SCALE;
}

// ============================================================================
// DAC helpers
// ============================================================================
void setDAC(uint16_t code) {
  if (code > 4095) code = 4095;
  currentCode = code;
  // Force VDD reference + 1x gain on every write. Without explicit args the
  // MCP4728 can run on its EEPROM defaults (often internal 2.048 V ref + 2x
  // gain) which clips the DAC at ~4.04 V regardless of Vdd.
  mcp.setChannelValue(MCP4728_CHANNEL_A, code,
                      MCP4728_VREF_VDD, MCP4728_GAIN_1X);
  delay(SETTLE_MS);
}

float codeToVdac(uint16_t c) { return (float)c / 4095.0 * VDD; }

// ============================================================================
// Identify the regulating window of the cal table.
// Trims leading "below regulation" and trailing "dropout clamp" flat regions
// by walking inward until local slope >= 30% of peak slope across the table.
// ============================================================================
void detectRegulatingWindow() {
  // Peak slope across all adjacent pairs
  float maxSlope = 0;
  for (uint8_t i = 0; i < CAL_N - 1; i++) {
    float dv = calTable[i + 1].vldo - calTable[i].vldo;
    float dc = (float)(calTable[i + 1].code - calTable[i].code);
    if (dc > 0) {
      float s = dv / dc;
      if (s > maxSlope) maxSlope = s;
    }
  }
  float threshold = 0.3 * maxSlope;

  // Walk from start
  calStart = 0;
  for (uint8_t i = 0; i < CAL_N - 1; i++) {
    float s = (calTable[i + 1].vldo - calTable[i].vldo) /
              (float)(calTable[i + 1].code - calTable[i].code);
    if (s >= threshold) { calStart = i; break; }
  }
  // Walk from end
  calEnd = CAL_N - 1;
  for (int8_t i = CAL_N - 2; i >= 0; i--) {
    float s = (calTable[i + 1].vldo - calTable[i].vldo) /
              (float)(calTable[i + 1].code - calTable[i].code);
    if (s >= threshold) { calEnd = i + 1; break; }
  }

  if (calEnd <= calStart) {
    // Degenerate case (flat curve?) -- fall back to full range
    calStart = 0;
    calEnd = CAL_N - 1;
  }

  vldoMin = calTable[calStart].vldo;
  vldoMax = calTable[calEnd].vldo;
}

// ============================================================================
// Calibration sweep
// ============================================================================
void runCalibration() {
  Serial.println(F("\n== Calibration =="));
  Serial.println(F(">> Disconnecting load (D2 LOW)"));

  // Drop DAC to 0 before disconnecting load -- avoids capacitive surprise.
  mcp.setChannelValue(MCP4728_CHANNEL_A, 0,
                      MCP4728_VREF_VDD, MCP4728_GAIN_1X);
  currentCode = 0;
  delay(SETTLE_MS);
  digitalWrite(MOSFET_PIN, LOW);
  delay(200);

  Serial.println(F("idx  code   V_LDO"));
  Serial.println(F("---  ----   ------"));

  for (uint8_t i = 0; i < CAL_N; i++) {
    uint16_t code = (uint16_t)i * 128;          // 0, 128, ..., 3968, 4096
    if (code > 4095) code = 4095;               // clamp last point
    mcp.setChannelValue(MCP4728_CHANNEL_A, code,
                        MCP4728_VREF_VDD, MCP4728_GAIN_1X);
    delay(i == 0 ? 300 : SETTLE_MS);            // extra settle on the big first step
    float v = readLDO();
    calTable[i].code = code;
    calTable[i].vldo = v;

    if (i < 10) Serial.print(' ');
    Serial.print(' '); Serial.print(i); Serial.print(F("   "));
    if (code < 1000) Serial.print(' ');
    if (code < 100)  Serial.print(' ');
    if (code < 10)   Serial.print(' ');
    Serial.print(code); Serial.print(F("   "));
    Serial.println(v, 4);
  }

  // Park DAC at 0 BEFORE re-enabling the load, so reconnection is benign.
  mcp.setChannelValue(MCP4728_CHANNEL_A, 0,
                      MCP4728_VREF_VDD, MCP4728_GAIN_1X);
  currentCode = 0;
  delay(SETTLE_MS);
  Serial.println(F(">> Restoring load (D2 HIGH)"));
  digitalWrite(MOSFET_PIN, HIGH);

  // Monotonicity check (5 mV slack for noise)
  bool mono = true;
  for (uint8_t i = 1; i < CAL_N; i++) {
    if (calTable[i].vldo < calTable[i - 1].vldo - 0.005) {
      Serial.print(F("WARN: non-monotonic at i=")); Serial.print(i);
      Serial.print(F(" ("));   Serial.print(calTable[i - 1].vldo, 4);
      Serial.print(F(" -> ")); Serial.print(calTable[i].vldo, 4);
      Serial.println(')');
      mono = false;
    }
  }

  detectRegulatingWindow();

  // Slope across regulating window, in mV per DAC code
  float dv = calTable[calEnd].vldo - calTable[calStart].vldo;
  uint16_t dc = calTable[calEnd].code - calTable[calStart].code;
  float slope_mV = (dc > 0) ? (dv * 1000.0 / (float)dc) : 0.0;

  Serial.println();
  Serial.print(F("Cal: ")); Serial.print(CAL_N); Serial.print(F(" pts in, "));
  Serial.print(calEnd - calStart + 1); Serial.print(F(" pts kept ["));
  Serial.print(calStart); Serial.print('-'); Serial.print(calEnd);
  Serial.print(F("], V_range = "));
  Serial.print(vldoMin, 3); Serial.print(F(" - "));
  Serial.print(vldoMax, 3); Serial.print(F(" V, slope = "));
  Serial.print(slope_mV, 2); Serial.print(F(" mV/code, "));
  Serial.println(mono ? F("monotonic OK") : F("WARN nonmono"));

  calValid = true;
  Serial.println(F(">> RAM updated. Run 'save' to commit to EEPROM."));
}

// ============================================================================
// Voltage -> DAC code (binary search + linear interpolation in regulating window)
// ============================================================================
uint16_t voltageToCode(float vtarget) {
  uint8_t lo = calStart, hi = calEnd;
  while (hi - lo > 1) {
    uint8_t mid = (lo + hi) / 2;
    if (calTable[mid].vldo <= vtarget) lo = mid;
    else                                hi = mid;
  }
  float    v0 = calTable[lo].vldo, v1 = calTable[hi].vldo;
  uint16_t c0 = calTable[lo].code, c1 = calTable[hi].code;
  if (v1 <= v0) return c0;
  float frac = (vtarget - v0) / (v1 - v0);
  float code = (float)c0 + frac * (float)(c1 - c0);
  if (code < 0)    code = 0;
  if (code > 4095) code = 4095;
  return (uint16_t)(code + 0.5);
}

void setVoltage(float vtarget) {
  if (!calValid) {
    Serial.println(F("ERR: no cal. Run 'cal' first."));
    return;
  }

  float vc = vtarget;
  bool clamped = false;
  if (vtarget < vldoMin) { vc = vldoMin; clamped = true; }
  if (vtarget > vldoMax) { vc = vldoMax; clamped = true; }
  if (clamped) {
    Serial.print(F("WARN: target ")); Serial.print(vtarget, 3);
    Serial.print(F("V out of range ["));
    Serial.print(vldoMin, 3); Serial.print(F(", "));
    Serial.print(vldoMax, 3); Serial.print(F("] -> clamped to "));
    Serial.print(vc, 3); Serial.println('V');
  }

  uint16_t code = voltageToCode(vc);
  setDAC(code);
  float vmeas = readLDO();
  float err = vmeas - vtarget;

  Serial.print(F("Target=")); Serial.print(vtarget, 3);
  Serial.print(F("V  Code=")); Serial.print(code);
  Serial.print(F("  V_LDO=")); Serial.print(vmeas, 3);
  Serial.print(F("V  err="));
  if (err >= 0) Serial.print('+');
  Serial.print(err * 1000.0, 1); Serial.println(F("mV"));
}

// ============================================================================
// EEPROM persistence
// ============================================================================
void saveCal() {
  if (!calValid) {
    Serial.println(F("ERR: no cal in RAM"));
    return;
  }
  calCount++;
  EEPROMHeader hdr = {
    EEPROM_MAGIC, EEPROM_VERSION, CAL_N, calCount, calStart, calEnd
  };
  int addr = EEPROM_ADDR;
  EEPROM.put(addr, hdr); addr += sizeof(hdr);
  for (uint8_t i = 0; i < CAL_N; i++) {
    EEPROM.put(addr, calTable[i]); addr += sizeof(CalPoint);
  }
  Serial.print(F("Saved cal #")); Serial.print(calCount);
  Serial.print(F(" (")); Serial.print(CAL_N);
  Serial.print(F(" pts, ")); Serial.print(addr - EEPROM_ADDR);
  Serial.println(F(" B)"));
}

bool loadCal() {
  EEPROMHeader hdr;
  int addr = EEPROM_ADDR;
  EEPROM.get(addr, hdr); addr += sizeof(hdr);
  if (hdr.magic   != EEPROM_MAGIC)   return false;
  if (hdr.version != EEPROM_VERSION) return false;
  if (hdr.n       != CAL_N)          return false;
  if (hdr.calEnd  >= CAL_N)          return false;
  if (hdr.calStart > hdr.calEnd)     return false;

  // All header sanity passes -- now read table
  for (uint8_t i = 0; i < CAL_N; i++) {
    EEPROM.get(addr, calTable[i]); addr += sizeof(CalPoint);
  }
  calCount = hdr.count;
  calStart = hdr.calStart;
  calEnd   = hdr.calEnd;
  vldoMin  = calTable[calStart].vldo;
  vldoMax  = calTable[calEnd].vldo;
  calValid = true;
  return true;
}

// ============================================================================
// Info dump
// ============================================================================
void printInfo() {
  Serial.println(F("\n== State =="));
  if (calValid) {
    Serial.print(F("Cal: VALID, #")); Serial.print(calCount);
    Serial.print(F(", ")); Serial.print(calEnd - calStart + 1);
    Serial.print(F(" pts kept ["));
    Serial.print(calStart); Serial.print('-'); Serial.print(calEnd);
    Serial.print(F("], V_range = "));
    Serial.print(vldoMin, 3); Serial.print(F(" - "));
    Serial.print(vldoMax, 3); Serial.println('V');
    float dv = calTable[calEnd].vldo - calTable[calStart].vldo;
    uint16_t dc = calTable[calEnd].code - calTable[calStart].code;
    float slope_mV = (dc > 0) ? (dv * 1000.0 / (float)dc) : 0.0;
    Serial.print(F("Slope ~ ")); Serial.print(slope_mV, 2);
    Serial.println(F(" mV/code"));
  } else {
    Serial.println(F("Cal: NOT LOADED. Run 'cal' first."));
  }
  Serial.print(F("DAC: code=")); Serial.print(currentCode);
  Serial.print(F(", V_dac~"));  Serial.print(codeToVdac(currentCode), 3);
  Serial.println('V');
  Serial.print(F("LDO: V_meas=")); Serial.print(readLDO(), 3); Serial.println('V');
  Serial.print(F("MOSFET: "));
  Serial.println(digitalRead(MOSFET_PIN) ? F("HIGH (load on)") : F("LOW (load off)"));
  Serial.print(F("ADC ref: ")); Serial.print(adc_vref, 3); Serial.println('V');
}

// ============================================================================
// Open-loop debug sweep (unchanged behavior, MOSFET stays HIGH)
// ============================================================================
void runSweep(float stepMV, bool csv) {
  float stepV = stepMV / 1000.0;
  int n = (int)((SWEEP_MAX - SWEEP_MIN) / stepV) + 1;

  if (csv) {
    Serial.println(F("dac_code,v_dac,v_mid_nom,v_ldo_meas"));
  } else {
    Serial.println(F("\nCode  V_DAC   V_mid   V_LDO"));
    Serial.println(F("----  ------  ------  ------"));
  }

  for (int i = 0; i < n; i++) {
    float vdac_t = SWEEP_MIN + i * stepV;
    if (vdac_t > SWEEP_MAX) vdac_t = SWEEP_MAX;

    uint16_t code = (uint16_t)(vdac_t / VDD * 4095.0 + 0.5);
    if (code > 4095) code = 4095;
    float vdac = codeToVdac(code);
    float vmid = vdac * DIVIDER_RATIO;

    setDAC(code);
    float vldo = readLDO();

    if (csv) {
      Serial.print(code);    Serial.print(',');
      Serial.print(vdac, 4); Serial.print(',');
      Serial.print(vmid, 4); Serial.print(',');
      Serial.println(vldo, 4);
    } else {
      if (code < 1000) Serial.print(' ');
      if (code < 100)  Serial.print(' ');
      if (code < 10)   Serial.print(' ');
      Serial.print(code); Serial.print(F("  "));
      Serial.print(vdac, 3); Serial.print(F("   "));
      Serial.print(vmid, 3); Serial.print(F("   "));
      Serial.println(vldo, 3);
    }
  }
  setDAC(currentCode);
  if (!csv) Serial.println();
}

// ============================================================================
void setup() {
  Serial.begin(115200);
  delay(2000);

  pinMode(MOSFET_PIN, OUTPUT);
  digitalWrite(MOSFET_PIN, HIGH);

  analogReference(DEFAULT);
  for (int i = 0; i < 10; i++) analogRead(FB_PIN);

  Wire.begin();

  // I2C scan
  Serial.println(F("\nI2C scan..."));
  byte cnt = 0;
  for (byte a = 1; a < 127; a++) {
    Wire.beginTransmission(a);
    if (Wire.endTransmission() == 0) {
      Serial.print(F("  0x"));
      if (a < 16) Serial.print('0');
      Serial.println(a, HEX);
      cnt++;
    }
  }
  Serial.print(cnt); Serial.println(F(" device(s)"));

  if (!mcp.begin(0x60)) {
    Serial.println(F("MCP4728 not found!"));
    while (1);
  }

  // Start at minimum (safe state). Explicitly force VDD ref + 1x gain so the
  // DAC range is 0..Vdd, not 0..4.096 V (internal 2.048 V ref + 2x default).
  mcp.setChannelValue(MCP4728_CHANNEL_A, 0,
                      MCP4728_VREF_VDD, MCP4728_GAIN_1X);
  currentCode = 0;
  delay(SETTLE_MS);

  // Persist VDD-ref / 1x-gain as the chip's power-on defaults. One-time op
  // in practice -- MCP4728 EEPROM is rated ~1M cycles. If it ever fails,
  // setDAC() and runCalibration() still pass explicit args so the runtime
  // behaviour stays correct.
  if (mcp.saveToEEPROM()) {
    Serial.println(F("DAC EEPROM: VDD ref + 1x gain saved as power-on default"));
  } else {
    Serial.println(F("WARN: DAC EEPROM write failed (runtime still OK)"));
  }

  // Auto-load cal from EEPROM
  Serial.println();
  if (loadCal()) {
    Serial.print(F("Cal loaded from EEPROM: "));
    Serial.print(calEnd - calStart + 1); Serial.print(F(" pts ["));
    Serial.print(calStart); Serial.print('-'); Serial.print(calEnd);
    Serial.print(F("], "));
    Serial.print(vldoMin, 3); Serial.print(F(" - "));
    Serial.print(vldoMax, 3); Serial.print(F(" V (cal #"));
    Serial.print(calCount); Serial.println(F(")"));
  } else {
    Serial.println(F("No valid cal in EEPROM. Run 'cal' first."));
  }

  Serial.println();
  Serial.println(F("== Programmable LDO (MCP4728 + TPS7A5701) =="));
  Serial.println(F("Commands:"));
  Serial.println(F("  <voltage>    Set LDO output (table lookup)"));
  Serial.println(F("  set <V>      Same as above"));
  Serial.println(F("  read         Read LDO output now"));
  Serial.println(F("  code <N>     Set raw DAC code 0-4095 (debug)"));
  Serial.println(F("  cal          Run 33-pt calibration (load disconnected)"));
  Serial.println(F("  save         Save RAM cal to EEPROM"));
  Serial.println(F("  load         Reload cal from EEPROM"));
  Serial.println(F("  info         Print current state"));
  Serial.println(F("  sweep [mV]   Open-loop sweep (load stays connected)"));
  Serial.println(F("  csv   [mV]   Same as sweep, CSV format"));
  Serial.println(F("  aref <V>     Set ADC Vref"));
  Serial.println();
}

// ============================================================================
void loop() {
  if (!Serial.available()) return;

  String in = Serial.readStringUntil('\n');
  in.trim();
  if (in.length() == 0) return;

  // read
  if (in.equalsIgnoreCase("read")) {
    float va = readADC(FB_PIN);
    Serial.print(F("A0=")); Serial.print(va, 3);
    Serial.print(F("V  LDO=")); Serial.print(va * ADC_FB_SCALE, 3);
    Serial.print(F("V  code=")); Serial.println(currentCode);
    return;
  }

  // info
  if (in.equalsIgnoreCase("info")) { printInfo(); return; }

  // cal
  if (in.equalsIgnoreCase("cal"))  { runCalibration(); return; }

  // save
  if (in.equalsIgnoreCase("save")) { saveCal(); return; }

  // load
  if (in.equalsIgnoreCase("load")) {
    if (loadCal()) {
      Serial.print(F("Cal loaded: "));
      Serial.print(calEnd - calStart + 1); Serial.print(F(" pts, "));
      Serial.print(vldoMin, 3); Serial.print(F(" - "));
      Serial.print(vldoMax, 3); Serial.print(F(" V (cal #"));
      Serial.print(calCount); Serial.println(F(")"));
    } else {
      Serial.println(F("ERR: no valid cal in EEPROM"));
    }
    return;
  }

  // code <N>
  if (in.startsWith("code ") || in.startsWith("CODE ")) {
    int c = in.substring(5).toInt();
    if (c >= 0 && c <= 4095) {
      setDAC((uint16_t)c);
      float vldo = readLDO();
      Serial.print(F("Code=")); Serial.print(c);
      Serial.print(F("  Vdac=")); Serial.print(codeToVdac(c), 3);
      Serial.print(F("V  LDO=")); Serial.print(vldo, 3);
      Serial.println('V');
    } else {
      Serial.println(F("Range: 0-4095"));
    }
    return;
  }

  // aref <V>
  if (in.startsWith("aref ") || in.startsWith("AREF ")) {
    float r = in.substring(5).toFloat();
    if (r >= 4.0 && r <= 5.5) {
      adc_vref = r;
      Serial.print(F("ADCref=")); Serial.print(adc_vref, 3); Serial.println('V');
    } else {
      Serial.println(F("Range: 4.0-5.5"));
    }
    return;
  }

  // sweep
  if (in.equalsIgnoreCase("sweep")) { runSweep(250, false); return; }
  if (in.startsWith("sweep ") || in.startsWith("SWEEP ")) {
    float s = in.substring(6).toFloat();
    if (s < 10) s = 10; if (s > 2000) s = 2000;
    runSweep(s, false);
    return;
  }

  // csv
  if (in.equalsIgnoreCase("csv")) { runSweep(250, true); return; }
  if (in.startsWith("csv ") || in.startsWith("CSV ")) {
    float s = in.substring(4).toFloat();
    if (s < 10) s = 10; if (s > 2000) s = 2000;
    runSweep(s, true);
    return;
  }

  // set <V> or bare number -> table-lookup voltage set
  float target;
  if (in.startsWith("set ") || in.startsWith("SET ")) {
    target = in.substring(4).toFloat();
  } else {
    target = in.toFloat();
    if (target == 0.0 && in[0] != '0') {
      Serial.print(F("? ")); Serial.println(in);
      return;
    }
  }
  setVoltage(target);
}
