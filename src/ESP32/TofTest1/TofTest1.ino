#include <Wire.h>
void setup() {
  Serial.begin(115200);
  delay(500);
  Wire.begin(21, 22);
}
void loop() {
  Serial.print("Encontrados:");
  for (uint8_t d = 1; d < 127; d++) {
    Wire.beginTransmission(d);
    if (Wire.endTransmission() == 0) Serial.printf(" 0x%02X", d);
  }
  Serial.println();
  delay(1000);
}