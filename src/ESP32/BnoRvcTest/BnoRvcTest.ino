/*
 * BnoRvcTest — lee el heading de un BNO085 en modo UART-RVC (sin I2C).
 * Sin librerías. Monitor a 115200.
 *
 * PINES DEL BNO:
 *   VIN=3.3V, GND
 *   PS0 = 3.3V, PS1 = GND   -> modo UART-RVC
 *   SDA -> GPIO 25           (en RVC el pin SDA es la salida TX del BNO)
 *   SCL, ADO, CS, INT, RST sin conectar
 *
 * El BNO manda solo un paquete cada 10 ms (100 Hz), 19 bytes:
 *   0xAA 0xAA | índice | yaw pitch roll (int16, 0.01°) | acc x y z (int16, mg)
 *   | 3 reservados | checksum (suma de los bytes índice..reservados)
 *
 * Comandos: z -> heading actual en 0 | r -> contadores en 0
 *
 * Imprime cada 100 ms:
 *   yaw       heading relativo al último 'z' (-180..180)
 *   hz        paquetes buenos por segundo (debe andar cerca de 100)
 *   hueco     mayor tiempo sin paquetes en el último segundo (ms)
 *   perdidos  paquetes que faltaron según el índice
 *   malos     paquetes con checksum incorrecto
 */

const int RVC_RX = 25;

uint8_t  buf[19];
int      n = 0;
float    yaw = 0, yawCero = 0;
uint32_t muestras = 0, perdidos = 0, malos = 0;
uint32_t ultimaMs = 0, huecoMax = 0;
int      ultimoIdx = -1;

float normaliza(float a) {
  while (a >  180) a -= 360;
  while (a < -180) a += 360;
  return a;
}

void procesarPaquete() {
  uint8_t suma = 0;
  for (int i = 2; i < 18; i++) suma += buf[i];
  if (suma != buf[18]) { malos++; return; }

  uint8_t idx = buf[2];
  if (ultimoIdx >= 0) perdidos += (uint8_t)(idx - ultimoIdx - 1);
  ultimoIdx = idx;

  int16_t yawRaw = (int16_t)(buf[3] | (buf[4] << 8));
  yaw = yawRaw * 0.01f;

  uint32_t ahora = millis();
  huecoMax = max(huecoMax, ahora - ultimaMs);
  ultimaMs = ahora;
  muestras++;
}

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("Esperando paquetes RVC en GPIO 25...");
  // Se abre al final de setup: si se abre antes de un delay, el buffer se llena
  // y se pierden paquetes. 1024 bytes = ~0.5 s de margen si el loop se tarda.
  Serial1.setRxBufferSize(1024);
  Serial1.begin(115200, SERIAL_8N1, RVC_RX, -1);
  ultimaMs = millis();
}

void loop() {
  if (Serial.available()) {
    char c = Serial.read();
    if (c == 'z') { yawCero = yaw; Serial.println("-- cero --"); }
    if (c == 'r') { perdidos = 0; malos = 0; Serial.println("-- contadores en 0 --"); }
  }

  while (Serial1.available()) {
    uint8_t b = Serial1.read();
    if (n == 0 && b != 0xAA) continue;              // busca el primer 0xAA
    if (n == 1 && b != 0xAA) { n = 0; continue; }   // el segundo también debe ser 0xAA
    buf[n++] = b;
    if (n == 19) { procesarPaquete(); n = 0; }
  }

  static uint32_t tImprime = 0, tSegundo = 0, hz = 0, huecoMostrar = 0;
  uint32_t ahora = millis();
  if (ahora - tSegundo >= 1000) {
    hz = muestras; muestras = 0;
    huecoMostrar = max(huecoMax, ahora - ultimaMs); huecoMax = 0;
    tSegundo = ahora;
  }
  if (ahora - tImprime >= 100) {
    tImprime = ahora;
    Serial.printf("yaw:%8.2f   hz:%3u   hueco:%4u ms   perdidos:%u   malos:%u\n",
                  normaliza(yaw - yawCero), hz, huecoMostrar, perdidos, malos);
  }
}
