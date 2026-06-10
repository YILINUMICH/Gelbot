// =============================================================================
// MCP4728 DAC -> LDO Closed-Loop Controller  (Arduino Uno)
// =============================================================================
//
// Wiring:
//   Uno SDA/SCL  -->  MCP4728 SDA/SCL  (I2C, uses A4/A5 internally)
//   Uno 5V/GND   -->  MCP4728 VDD/GND
//   D2           -->  MOSFET gate (held HIGH)
//
//   MCP4728 VA --[2kΩ]--+--[10kΩ]-- GND   (divider -> LDO REF pin)
//                        |
//                      V_mid --> LDO REF
//
//   LDO_OUT --[R]--+--[R]-- GND   (1:1 divider, halves voltage for ADC)
//                   |
//                   A0
//
// Closed-loop: type a target LDO output voltage (0.5 - 5.0V).
// The code will binary-search + fine-walk the DAC code until the
// measured LDO output matches the target within tolerance.
//
// Commands:
//   <number>       Set LDO output to <number> volts (closed-loop)
//   set <V>        Same as above
//   read           Read current LDO output
//   code <N>       Directly set raw DAC code (open-loop, for debug)
//   aref <V>       Adjust ADC reference voltage
//   sweep          Open-loop sweep, print DAC code vs LDO output
//   csv            Same sweep but CSV format
// =============================================================================

#include <Wire.h>
#include <Adafruit_MCP4728.h>

Adafruit_MCP4728 mcp;

// --- Pins ---
const int MOSFET_PIN = 2;
const int FB_PIN     = A0;      // LDO output feedback (halved)

// --- Circuit parameters ---
const float VDD           = 5.5;
const float R_TOP         = 2000.0;
const float R_BOT         = 10000.0;
const float DIVIDER_RATIO = R_BOT / (R_TOP + R_BOT);  // 0.8333
const float ADC_FB_SCALE  = 2.0;   // LDO divider: equal R -> x2

// --- Target range ---
const float LDO_TARGET_MIN = 0.5;
const float LDO_TARGET_MAX = 5.0;

// --- Sweep range (DAC output voltage) ---
const float SWEEP_MIN = 0.5;
const float SWEEP_MAX = 5.0;

// --- ADC ---
const int   ADC_SAMPLES = 64;
const int   SETTLE_MS   = 150;
float       adc_vref    = 5.0;

// --- Closed-loop tuning ---
const float TOLERANCE    = 0.015;  // ±15mV convergence target
const int   MAX_ITER     = 30;     // Max iterations before giving up

// --- Current state ---
uint16_t currentCode = 0;

// ---- ADC ----
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

// ---- DAC helpers ----
void setDAC(uint16_t code) {
  if (code > 4095) code = 4095;
  currentCode = code;
  mcp.setChannelValue(MCP4728_CHANNEL_A, code);
  delay(SETTLE_MS);
}

float codeToVdac(uint16_t c) { return (float)c / 4095.0 * VDD; }

// Estimate: what DAC code would produce a given LDO output?
// LDO_out ≈ f(V_mid) and V_mid = V_dac * DIVIDER_RATIO
// As a first guess, assume LDO_out ≈ V_mid (will be refined by closed loop)
uint16_t estimateCode(float vldo_target) {
  float vmid = vldo_target;  // rough initial guess
  float vdac = vmid / DIVIDER_RATIO;
  if (vdac < 0.0) vdac = 0.0;
  if (vdac > VDD) vdac = VDD;
  uint16_t c = (uint16_t)(vdac / VDD * 4095.0 + 0.5);
  return (c > 4095) ? 4095 : c;
}

// ---- Closed-loop: converge DAC code to hit target LDO output ----
// Strategy: binary search on DAC code, measuring actual LDO output each step.
// Assumes LDO output is monotonically increasing with DAC code.
bool closedLoopSet(float target) {
  Serial.print(F("Target: ")); Serial.print(target, 3); Serial.println(F("V"));

  // Phase 1: Binary search for coarse convergence
  uint16_t lo = 0;
  uint16_t hi = 4095;

  // Narrow the search range with an initial estimate
  uint16_t est = estimateCode(target);
  if (est > 200) lo = est - 200; else lo = 0;
  if (est < 3895) hi = est + 200; else hi = 4095;

  // Verify monotonicity: measure at lo, settle, then hi
  // Use extra settle to avoid false fail from LDO transient
  setDAC(lo);
  delay(200);  // extra settle for big step
  float vlo = readLDO();
  setDAC(hi);
  delay(200);
  float vhi = readLDO();

  Serial.print(F("  lo=")); Serial.print(lo); Serial.print(F(" -> ")); Serial.print(vlo, 3); Serial.println('V');
  Serial.print(F("  hi=")); Serial.print(hi); Serial.print(F(" -> ")); Serial.print(vhi, 3); Serial.println('V');

  // If hi reads lower than lo by a significant margin, something is wrong
  if (vhi < vlo - 0.3) {
    Serial.println(F("ERR: LDO not monotonic. Check wiring."));
    return false;
  }

  // Expand range if target is outside [vlo, vhi]
  if (target < vlo) lo = 0;
  if (target > vhi) hi = 4095;

  int iter = 0;
  uint16_t bestCode = est;
  float bestErr = 999.0;

  // Binary search
  while (lo <= hi && iter < MAX_ITER) {
    uint16_t mid = (lo + hi) / 2;
    setDAC(mid);
    float vmeas = readLDO();
    float err = vmeas - target;

    Serial.print(F("  #")); Serial.print(iter);
    Serial.print(F(" code=")); Serial.print(mid);
    Serial.print(F(" LDO=")); Serial.print(vmeas, 3);
    Serial.print(F("V err=")); 
    if (err >= 0) Serial.print('+');
    Serial.print(err * 1000.0, 1); Serial.println(F("mV"));

    if (abs(err) < abs(bestErr)) {
      bestErr = err;
      bestCode = mid;
    }

    if (abs(err) <= TOLERANCE) {
      // Converged
      Serial.print(F("  OK in ")); Serial.print(iter + 1);
      Serial.print(F(" steps. Code=")); Serial.print(mid);
      Serial.print(F(" LDO=")); Serial.print(vmeas, 3);
      Serial.print(F("V (err=")); 
      if (err >= 0) Serial.print('+');
      Serial.print(err * 1000.0, 1); Serial.println(F("mV)"));
      return true;
    }

    if (vmeas < target) {
      lo = mid + 1;
    } else {
      if (mid == 0) break;
      hi = mid - 1;
    }
    iter++;
  }

  // Phase 2: Fine linear walk from best code found
  Serial.println(F("  Fine-tuning..."));
  setDAC(bestCode);
  float vmeas = readLDO();
  int direction = (vmeas < target) ? 1 : -1;

  for (int i = 0; i < 20; i++) {
    uint16_t nextCode = bestCode + direction;
    if (nextCode > 4095) break;

    setDAC(nextCode);
    vmeas = readLDO();
    float err = vmeas - target;

    if (abs(err) < abs(bestErr)) {
      bestErr = err;
      bestCode = nextCode;
    }

    if (abs(err) <= TOLERANCE) {
      Serial.print(F("  OK. Code=")); Serial.print(nextCode);
      Serial.print(F(" LDO=")); Serial.print(vmeas, 3);
      Serial.print(F("V (err="));
      if (err >= 0) Serial.print('+');
      Serial.print(err * 1000.0, 1); Serial.println(F("mV)"));
      return true;
    }

    // If error started growing, we overshot — go back
    if (abs(err) > abs(bestErr) + TOLERANCE) {
      setDAC(bestCode);
      break;
    }

    bestCode = nextCode;
    bestErr = err;
  }

  // Report best effort
  vmeas = readLDO();
  Serial.print(F("  BEST: code=")); Serial.print(bestCode);
  Serial.print(F(" LDO=")); Serial.print(vmeas, 3);
  Serial.print(F("V (err="));
  float ferr = vmeas - target;
  if (ferr >= 0) Serial.print('+');
  Serial.print(ferr * 1000.0, 1); Serial.println(F("mV)"));
  return (abs(ferr) <= TOLERANCE * 2);
}

// ---- Open-loop sweep (print-as-you-go) ----
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
  setDAC(currentCode);  // restore previous
  if (!csv) Serial.println();
}


// =============================================================================
void setup() {
  Serial.begin(115200);
  delay(2000);

  pinMode(MOSFET_PIN, OUTPUT);
  digitalWrite(MOSFET_PIN, HIGH);

  analogReference(DEFAULT);
  for (int i = 0; i < 10; i++) analogRead(FB_PIN);

  Wire.begin();

  // I2C scan
  Serial.println(F("I2C scan..."));
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
  Serial.print(cnt); Serial.println(F(" device(s)\n"));

  if (!mcp.begin(0x60)) {
    Serial.println(F("MCP4728 not found!"));
    while (1);
  }

  // Start at minimum
  uint16_t initCode = estimateCode(LDO_TARGET_MIN);
  setDAC(initCode);

  Serial.println(F("== LDO Closed-Loop Controller =="));
  Serial.print(F("VDD=")); Serial.print(VDD, 1);
  Serial.print(F("V  FB=A0(x")); Serial.print(ADC_FB_SCALE, 0);
  Serial.print(F(")  tol=±")); Serial.print(TOLERANCE * 1000, 0);
  Serial.println(F("mV"));
  Serial.println(F("Range: 0.5V - 5.0V"));
  Serial.println(F("Commands:"));
  Serial.println(F("  <voltage>  Set LDO output (closed-loop)"));
  Serial.println(F("  read       Read LDO output now"));
  Serial.println(F("  code <N>   Set raw DAC code (open-loop)"));
  Serial.println(F("  sweep      Open-loop characterization"));
  Serial.println(F("  csv        Sweep as CSV"));
  Serial.println(F("  aref <V>   Set ADC Vref"));
  Serial.println();
}

// =============================================================================
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

  // set <V> or bare number -> closed-loop target
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

  if (target >= LDO_TARGET_MIN && target <= LDO_TARGET_MAX) {
    closedLoopSet(target);
  } else {
    Serial.print(F("Range: ")); Serial.print(LDO_TARGET_MIN, 1);
    Serial.print(F("-")); Serial.println(LDO_TARGET_MAX, 1);
  }
}
