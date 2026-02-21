#include <Wire.h>
#include <Adafruit_MCP4728.h>

Adafruit_MCP4728 mcp;

void setup() {
  Serial.begin(115200);
  delay(2000);
  Wire.begin();

  // --- I2C Scanner ---
  Serial.println("Scanning I2C bus...");
  byte count = 0;
  for (byte addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
      Serial.print("  Found device at 0x");
      if (addr < 16) Serial.print("0");
      Serial.println(addr, HEX);
      count++;
    }
  }
  Serial.print("Found ");
  Serial.print(count);
  Serial.println(" device(s).\n");

  // --- Init MCP4728 ---
  if (!mcp.begin(0x60)) {
    Serial.println("MCP4728 not found!");
    while (1);
  }
  Serial.println("MCP4728 initialized.");
  Serial.println("Type a voltage (0.000 to 5.000) to set Channel A:");
}

void loop() {
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');
    input.trim();
    float target = input.toFloat();

    if (target >= 0.0 && target <= 5.0) {
      uint16_t value = (uint16_t)((target / 5.0) * 4096.0);
      if (value > 4095) value = 4095;
      mcp.setChannelValue(MCP4728_CHANNEL_A, value);
      Serial.print("VA = ");
      Serial.print(target, 3);
      Serial.print("V (DAC: ");
      Serial.print(value);
      Serial.println(")");
    } else {
      Serial.print("Invalid: ");
      Serial.println(input);
      Serial.println("Enter 0.000 to 5.000");
    }
  }
}