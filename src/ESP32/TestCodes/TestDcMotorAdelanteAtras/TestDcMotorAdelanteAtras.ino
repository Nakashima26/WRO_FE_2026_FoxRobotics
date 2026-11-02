// Prueba del motor DC con el TB6612: adelante, freno, atras, freno.
// Pines de la PCB RevB.
#define PWMA 23
#define AIN1 19
#define AIN2 18

#define VELOCIDAD 150     // 0-255
#define TIEMPO_MS 2000    // cuanto tiempo gira en cada sentido
#define PAUSA_MS  1000    // parado entre cambios de sentido

void adelante(int vel) {
  digitalWrite(AIN1, HIGH);
  digitalWrite(AIN2, LOW);
  analogWrite(PWMA, vel);
}

void atras(int vel) {
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, HIGH);
  analogWrite(PWMA, vel);
}

// Freno corto: las dos entradas en HIGH frenan el motor en seco
void frenar() {
  digitalWrite(AIN1, HIGH);
  digitalWrite(AIN2, HIGH);
  analogWrite(PWMA, 255);
}

// Suelto: el motor gira libre hasta pararse
void soltar() {
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, LOW);
  analogWrite(PWMA, 0);
}

void setup() {
  Serial.begin(115200);
  pinMode(PWMA, OUTPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  soltar();
}

void loop() {
  Serial.println("Adelante");
  adelante(VELOCIDAD);
  delay(TIEMPO_MS);

  // Parar antes de cambiar de sentido para no meter un pico de corriente
  Serial.println("Freno");
  frenar();
  delay(PAUSA_MS);

  Serial.println("Atras");
  atras(VELOCIDAD);
  delay(TIEMPO_MS);

  Serial.println("Freno");
  frenar();
  delay(PAUSA_MS);
}
