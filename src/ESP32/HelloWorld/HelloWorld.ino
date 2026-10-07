/*
 * HelloWorld — prueba rápida de que la ESP32 sube código y habla por Serial.
 * Monitor serie a 115200. El LED integrado (GPIO 2 en la mayoría de las
 * DevKit) parpadea cada segundo.
 */

#define LED_PIN 2

void setup() {
  Serial.begin(115200);
  pinMode(LED_PIN, OUTPUT);
  delay(500);
  Serial.println("ESP32 viva");
}

void loop() {
  Serial.print("Hello world  t=");
  Serial.print(millis() / 1000);
  Serial.println(" s");

  digitalWrite(LED_PIN, !digitalRead(LED_PIN));
  delay(1000);
}
