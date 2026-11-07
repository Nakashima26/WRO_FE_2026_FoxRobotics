// Diagnostico: lee pines (pulldown) e imprime cambios; el 16 alterna cada 1 s.
const int P[] = {4,5,12,13,14,15,17,18,19,21,22,23,25,26,27,32,33,34,35,36,39};
const int N = sizeof(P)/sizeof(P[0]);
uint32_t prev = 0xFFFFFFFF; unsigned long tt = 0; bool lvl = false;
uint32_t leer() { uint32_t m = 0; for (int i = 0; i < N; i++) if (digitalRead(P[i])) m |= 1UL << i; return m; }
void setup() {
  Serial.begin(460800);
  for (int i = 0; i < N; i++) pinMode(P[i], P[i] >= 34 ? INPUT : INPUT_PULLDOWN);
  pinMode(16, OUTPUT);
}
void loop() {
  if (millis() - tt > 1000) { tt = millis(); lvl = !lvl; digitalWrite(16, lvl); Serial.printf("%lu TX16=%d\n", millis(), lvl); }
  uint32_t m = leer();
  if (m != prev) { Serial.printf("%lu ALTOS:", millis()); for (int i = 0; i < N; i++) if (m >> i & 1) Serial.printf(" %d", P[i]); Serial.println(); prev = m; }
  delay(5);
}
