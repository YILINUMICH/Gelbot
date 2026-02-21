/*
 * SMA Coil Actuator — Dual MOSFET Driver
 * Arduino UNO | Pin 3 → MOSFET 1 | Pin 4 → MOSFET 2
 *
 * State machine: IDLE → ON → OFF → IDLE
 * Both channels share the same on/off timing.
 * Send serial command to trigger a cycle.
 *
 * Serial Commands (115200 baud):
 *   1     — trigger CH1 only
 *   2     — trigger CH2 only
 *   b     — trigger both CH1 + CH2
 *   s     — stop all (return to IDLE)
 */

// ============ PIN CONFIG ============
#define MOSFET_1  3
#define MOSFET_2  4

// ======= TIMING CONFIG (ms) ========
unsigned long onTime  = 100;    // heating duration (shared)
unsigned long offTime = 3000;    // cooling duration (shared)

// =========== STATE MACHINE ==========
enum State { IDLE, ON_STATE, OFF_STATE };

struct Channel {
  const char    *name;
  uint8_t       pin;
  State         state;
  unsigned long tStart;
};

Channel ch1, ch2;

// ============ SETUP =================
void setup() {
  Serial.begin(115200);
  while (!Serial) { ; }
  delay(100);

  pinMode(MOSFET_1, OUTPUT);
  pinMode(MOSFET_2, OUTPUT);
  digitalWrite(MOSFET_1, LOW);
  digitalWrite(MOSFET_2, LOW);

  ch1 = { "CH1", MOSFET_1, IDLE, 0 };
  ch2 = { "CH2", MOSFET_2, IDLE, 0 };

  Serial.println(F("=== SMA Driver Ready ==="));
  Serial.print(F("on=")); Serial.print(onTime);
  Serial.print(F("ms  off=")); Serial.print(offTime); Serial.println(F("ms"));
  Serial.println(F("Commands: '1'=CH1  '2'=CH2  'b'=both  's'=stop"));
  Serial.println();
}

// ============ LOOP ==================
void loop() {
  handleSerial();
  tick(ch1);
  tick(ch2);
}

// ========= SERIAL COMMANDS ==========
void handleSerial() {
  if (!Serial.available()) return;
  char c = Serial.read();

  switch (c) {
    case '1':
      startChannel(ch1);
      break;
    case '2':
      startChannel(ch2);
      break;
    case 'b':
      startChannel(ch1);
      startChannel(ch2);
      break;
    case 's':
      stopChannel(ch1);
      stopChannel(ch2);
      Serial.println(F("[CMD] All stopped"));
      break;
  }
}

void startChannel(Channel &ch) {
  digitalWrite(ch.pin, HIGH);
  ch.state  = ON_STATE;
  ch.tStart = millis();
  Serial.print(F("["));  Serial.print(ch.name);
  Serial.println(F("] Started → ON"));
}

void stopChannel(Channel &ch) {
  digitalWrite(ch.pin, LOW);
  ch.state = IDLE;
}

// ========= STATE MACHINE TICK =======
void tick(Channel &ch) {
  if (ch.state == IDLE) return;

  unsigned long elapsed = millis() - ch.tStart;

  switch (ch.state) {
    case ON_STATE:
      if (elapsed >= onTime) {
        digitalWrite(ch.pin, LOW);
        ch.state  = OFF_STATE;
        ch.tStart = millis();

        Serial.print(F("["));  Serial.print(ch.name);
        Serial.print(F("] ON→OFF  heated "));
        Serial.print(elapsed); Serial.println(F("ms"));
      }
      break;

    case OFF_STATE:
      if (elapsed >= offTime) {
        ch.state = IDLE;

        Serial.print(F("["));  Serial.print(ch.name);
        Serial.print(F("] OFF→IDLE  cooled "));
        Serial.print(elapsed); Serial.println(F("ms"));
      }
      break;

    default:
      break;
  }
}
