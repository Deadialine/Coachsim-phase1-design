#include <Wire.h>

void setup() {
  Serial.begin(19200);
  while (!Serial) {}
  Wire.begin();
  Wire.setClock(400000); // Fast mode
  Serial.println("I2C scan...");
}

void loop() {
  int found = 0;
  for (uint8_t addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    uint8_t err = Wire.endTransmission();
    if (err == 0) {
      Serial.print("Found 0x");
      if (addr < 16) Serial.print("0");
      Serial.println(addr, HEX);
      found++;
    }
  }
  Serial.print("Devices found: ");
  Serial.println(found);
  delay(3000);
}
