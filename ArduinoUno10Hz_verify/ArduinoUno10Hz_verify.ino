/*
 * SMA Driver — Timing Verification (microsecond resolution)
 *
 * Same 10-cycle burst at 25 ms ON / 75 ms OFF, but uses micros()
 * instead of millis() so sub-millisecond jitter is visible.
 *
 * Serial (115200 baud):
 *   1  → run burst
 *   s  → abort
 */

#define MOSFET_PIN   3
#define NUM_CYCLES   10

const unsigned long onTime_us  = 25000UL;   // 25 ms
const unsigned long offTime_us = 75000UL;   // 75 ms

enum State { IDLE, ON_STATE, OFF_STATE };
State         state      = IDLE;
unsigned long tStart_us  = 0;
uint8_t       cyclesDone = 0;

unsigned long onDur[NUM_CYCLES];
unsigned long offDur[NUM_CYCLES];

void setup() {
  Serial.begin(115200);
  while (!Serial) { ; }
  delay(100);

  pinMode(MOSFET_PIN, OUTPUT);
  digitalWrite(MOSFET_PIN, LOW);

  Serial.println(F("=== SMA Timing (microsecond res) ==="));
  Serial.print(F("Target: "));
  Serial.print(onTime_us);  Serial.print(F(" us ON / "));
  Serial.print(offTime_us); Serial.print(F(" us OFF  x  "));
  Serial.print(NUM_CYCLES); Serial.println(F(" cycles"));
  Serial.println(F("Send '1' to run"));
  Serial.println();
}

void loop() {
  handleSerial();
  tick();
}

void handleSerial() {
  if (!Serial.available()) return;
  char c = Serial.read();
  if (c == '1' && state == IDLE) {
    digitalWrite(MOSFET_PIN, HIGH);
    state      = ON_STATE;
    tStart_us  = micros();
    cyclesDone = 0;
    Serial.println(F("[RUN] burst started"));
  } else if (c == 's') {
    digitalWrite(MOSFET_PIN, LOW);
    state = IDLE;
    Serial.println(F("[STOP]"));
  }
}

void tick() {
  if (state == IDLE) return;
  unsigned long elapsed = micros() - tStart_us;

  if (state == ON_STATE && elapsed >= onTime_us) {
    digitalWrite(MOSFET_PIN, LOW);
    onDur[cyclesDone] = elapsed;
    state     = OFF_STATE;
    tStart_us = micros();
  }
  else if (state == OFF_STATE && elapsed >= offTime_us) {
    offDur[cyclesDone] = elapsed;
    cyclesDone++;
    if (cyclesDone < NUM_CYCLES) {
      digitalWrite(MOSFET_PIN, HIGH);
      state     = ON_STATE;
      tStart_us = micros();
    } else {
      state = IDLE;
      dumpLog();
    }
  }
}

void dumpLog() {
  Serial.println();
  Serial.println(F("=== Burst complete — timing log (us) ==="));
  Serial.println(F("Cyc\tON(us)\tOFF(us)\tPeriod(us)"));

  unsigned long sumOn = 0, sumOff = 0;
  unsigned long minOn = onDur[0],  maxOn  = onDur[0];
  unsigned long minOff = offDur[0], maxOff = offDur[0];

  for (uint8_t i = 0; i < NUM_CYCLES; i++) {
    Serial.print(i + 1);         Serial.print('\t');
    Serial.print(onDur[i]);      Serial.print('\t');
    Serial.print(offDur[i]);     Serial.print('\t');
    Serial.println(onDur[i] + offDur[i]);

    sumOn  += onDur[i];
    sumOff += offDur[i];
    if (onDur[i]  < minOn)  minOn  = onDur[i];
    if (onDur[i]  > maxOn)  maxOn  = onDur[i];
    if (offDur[i] < minOff) minOff = offDur[i];
    if (offDur[i] > maxOff) maxOff = offDur[i];
  }

  Serial.println();
  Serial.print(F("ON  target=25000us  avg=")); Serial.print(sumOn / NUM_CYCLES);
  Serial.print(F("  min="));    Serial.print(minOn);
  Serial.print(F("  max="));    Serial.print(maxOn);
  Serial.print(F("  jitter=")); Serial.print(maxOn - minOn); Serial.println(F("us"));

  Serial.print(F("OFF target=75000us  avg=")); Serial.print(sumOff / NUM_CYCLES);
  Serial.print(F("  min="));    Serial.print(minOff);
  Serial.print(F("  max="));    Serial.print(maxOff);
  Serial.print(F("  jitter=")); Serial.print(maxOff - minOff); Serial.println(F("us"));
  Serial.println();
}
