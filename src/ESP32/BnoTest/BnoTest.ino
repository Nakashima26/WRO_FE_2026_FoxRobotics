/*
 * BnoTest — lee el heading (yaw) de un BNO085 por I2C.
 * Librería: "Adafruit BNO08x" (sirve para cualquier módulo BNO08x). Monitor a 115200.
 *
 * PINES:
 *   VIN=3.3V, GND, SDA=21, SCL=22
 *   PS0/PS1 en bajo (modo I2C), INT sin conectar
 *   ADO a GND (0x4A), RST a GPIO 25 (pon BNO_RST = -1 si no lo conectas)
 *
 * Usa GAME_ROTATION_VECTOR: gyro + acelerómetro, SIN magnetómetro
 * (los imanes del motor y la LiPo lo desviarían).
 *
 * Comandos por el monitor serie:
 *   z  -> pone el heading actual en 0
 *   r  -> reinicia los contadores
 *
 * Imprime cada 100 ms:
 *   yaw      heading relativo al último 'z', en grados (-180..180)
 *   hz       muestras por segundo recibidas (debe andar cerca de 100)
 *   hueco    el mayor tiempo sin muestras en el último segundo (ms)
 *   resets   veces que el BNO se reinició solo (1 al arrancar es normal)
 *   recups   veces que el watchdog tuvo que despertarlo
 */

#include <Wire.h>
#include <Adafruit_BNO08x.h>

const int      BNO_RST     = 25;      // RST a GPIO 25 (-1 = sin RST)
const uint32_t PERIODO_US  = 10000;   // 100 Hz
const uint32_t I2C_HZ      = 100000;

Adafruit_BNO08x   bno(BNO_RST);
sh2_SensorValue_t valor;

float    yaw       = 0;     // grados, sin referencia
float    yawCero   = 0;
uint32_t muestras  = 0;
uint32_t resets    = 0;
uint32_t ultimaMs  = 0;
uint32_t huecoMax  = 0;
uint32_t recups    = 0;     // veces que el watchdog tuvo que despertarlo
uint8_t  dirBno    = 0;

void activarReporte() {
  if (!bno.enableReport(SH2_GAME_ROTATION_VECTOR, PERIODO_US))
    Serial.println("No pude activar GAME_ROTATION_VECTOR");
}

float normaliza(float a) {
  while (a >  180) a -= 360;
  while (a < -180) a += 360;
  return a;
}

void setup() {
  Serial.begin(115200);
  delay(500);
  Wire.begin(21, 22);

  // Con RST conectado, la librería reinicia el BNO por hardware dentro de begin_I2C.
  // La dirección se busca antes: un begin_I2C fallido deja la librería en mal estado.
  for (uint8_t d : {0x4A, 0x4B}) {
    Wire.beginTransmission(d);
    if (Wire.endTransmission() == 0) { dirBno = d; break; }
  }
  bool ok = dirBno && bno.begin_I2C(dirBno, &Wire);
  if (!ok) {
    Serial.println("BNO085 NO responde en 0x4A ni 0x4B: revisa VIN/GND/SDA/SCL y PS0/PS1 en bajo");
    while (true) delay(1000);
  }
  Wire.setClock(I2C_HZ);
  Serial.printf("BNO085 OK en 0x%02X\n", dirBno);

  activarReporte();
  ultimaMs = millis();
}

void loop() {
  if (Serial.available()) {
    char c = Serial.read();
    if (c == 'z') { yawCero = yaw; Serial.println("-- cero --"); }
    if (c == 'r') { resets = 0; recups = 0; Serial.println("-- contadores en 0 --"); }
  }

  if (bno.wasReset()) {
    resets++;
    Serial.println("!! el BNO se reinició, reactivo el reporte");
    activarReporte();
  }

  while (bno.getSensorEvent(&valor)) {
    if (valor.sensorId != SH2_GAME_ROTATION_VECTOR) continue;
    float r = valor.un.gameRotationVector.real;
    float i = valor.un.gameRotationVector.i;
    float j = valor.un.gameRotationVector.j;
    float k = valor.un.gameRotationVector.k;
    yaw = atan2f(2 * (r * k + i * j), 1 - 2 * (j * j + k * k)) * 180.0f / PI;

    uint32_t ahora = millis();
    huecoMax = max(huecoMax, ahora - ultimaMs);
    ultimaMs = ahora;
    muestras++;
  }

  static uint32_t tImprime = 0, tSegundo = 0, hz = 0, huecoMostrar = 0, tRecup = 0;
  uint32_t ahora = millis();
  uint32_t sinDatos = ahora - ultimaMs;

  // Watchdog: 0.5 s sin datos -> reactivar el reporte; 2 s -> reiniciar el BNO.
  if (sinDatos > 500 && ahora - tRecup > 500) {
    tRecup = ahora;
    recups++;
    if (sinDatos > 2000) {
      Serial.println("!! 2 s sin datos, reinicio el BNO");
      if (bno.begin_I2C(dirBno, &Wire)) Wire.setClock(I2C_HZ);
      else Serial.println("!! el BNO no contesta al reiniciar");
    } else {
      Serial.println("!! 0.5 s sin datos, reactivo el reporte");
    }
    activarReporte();
  }
  if (ahora - tSegundo >= 1000) {
    hz = muestras; muestras = 0;
    huecoMostrar = huecoMax; huecoMax = 0;
    tSegundo = ahora;
  }
  if (ahora - tImprime >= 100) {
    tImprime = ahora;
    Serial.printf("yaw:%8.2f   hz:%3u   hueco:%4u ms   resets:%u   recups:%u\n",
                  normaliza(yaw - yawCero), hz, huecoMostrar, resets, recups);
  }
}
