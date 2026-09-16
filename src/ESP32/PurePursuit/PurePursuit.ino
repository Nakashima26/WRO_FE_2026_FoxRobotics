/*
 * PurePursuit.ino — WRO Future Engineers
 * Firmware ESP32 con soporte de protocolo V2 (Pure Pursuit desde Raspberry Pi).
 *
 * Basado en Controller_PI.ino. Diferencias respecto a la versión anterior:
 *   1. Parseo del campo pp= en mensajes V2.
 *   2. Cuando piPurePursuit=true:
 *        - Se suspende el PID de paredes (ultrasonidos).
 *        - La Pi envía obs=steer_deg/35 → ESP32 aplica ppSteerGain=35.
 *        - Solo se mantiene una corrección liviana de gyro para estabilizar heading.
 *   3. Cuando piPurePursuit=false (V1 o pp=0):
 *        - Comportamiento idéntico a Controller_PI.ino.
 *   4. Timeout de Pi (>800 ms sin mensaje):
 *        - Fallback a wall PID + gyro (igual que antes).
 *
 * PINES (sin cambios):
 *   HC-SR04 izq  : TRIG=27, ECHO=32
 *   HC-SR04 der  : TRIG=26, ECHO=35
 *   Motor DC     : PWMA=23, A1=18, A2=19  (TB6612FNG)
 *   Servo SG90   : SERVO_PIN=13
 *   MPU-6050     : I2C (SDA/SCL por defecto)
 *   Serial Pi    : Serial2 RX=17, TX=16 @ 115200
 */

#include <Wire.h>
#include <MPU6050_tockn.h>

MPU6050 mpu(Wire);

// ── Pines ─────────────────────────────────────────────────────────────────────
#define TRIG_L      27
#define ECHO_L      32
#define TRIG_R      26
#define ECHO_R      35

#define PWMA        23
#define A1          18
#define A2          19
#define SERVO_PIN   13
#define TRIG_F      14   // HC-SR04 frontal (ronda de obstáculos: CRUCERO/MANIOBRA)
#define ECHO_F      33

// ── RONDA OBSTACULOS ──────────────────────────────────────────────────────────
const bool rondaObstaculos  = true;    // false = giro continuo de siempre (ronda abierta)


// ── PWM ───────────────────────────────────────────────────────────────────────
const int freqServo  = 50;
const int resServo   = 16;
const int freqMotor  = 1000;
const int resMotor   = 8;

// ── PID Paredes (ultrasónicos) ────────────────────────────────────────────────
float KpWall = 1.0;
float KiWall = 0.0;
float KdWall = 1.35;

// Ronda ABIERTA: en vez de centrar entre paredes (distL-distR -> 0), una vez que
// ya se sabe el sentido de giro de la pista, el wall PID mantiene esta distancia
// fija a la pared INTERIOR (la del lado hacia donde gira). Menos "hunting" y
// línea más corta en rectas anchas; además la pared interior no "desaparece" en
// las esquinas (esa es la exterior), así que el PID no se clava. 
const float WALL_HOLD_CM = 25.0;

// El error de UNA sola pared tiene ~la mitad de ganancia geométrica que
// distL-distR (al desplazarte lateralmente solo cambia un sensor, no dos). Sin
// re-escalar, el término de pared no le gana al gyro PID —que tras cada giro
// continuo defiende un heading viciado (~13°, el giro se pasa por inercia)— y el
// carro se ABRE hacia la pared de afuera en vez de pegarse a la de adentro.
const float WALL_HOLD_GAIN = 2;

// ── Seguir la pared EXTERIOR en la recta (ronda de obstáculos) ───────────────
// wallCorr solo centra con AMBAS paredes. En la recta la pared INTERIOR está
// ausente (dR/dL ~200) casi todo el tramo -> sin centrado lateral: tras esquivar
// un cono el carro holdea el heading pero DERIVA lateralmente hacia el interior
// (run 750 giro 10: dL 54->71 = se corre 17cm adentro con ang~+5) -> llega a la
// esquina pegado a la pared/cono interior -> lo roza / la cola lo barre al
// reversear. La pared EXTERIOR sí está SIEMPRE (dL 54-72). PID para mantener
// EXT_WALL_TARGET_CM de ella -> el carro va parejo y a posición lateral estable
// toda la recta, sin importar que falte la interior. Solo con dir de giro
// conocida y cuando wallCorr (ambas paredes) NO está actuando.
const float EXT_WALL_TARGET_CM = 50.0f; // distancia a mantener de la pared exterior
const float EXT_WALL_GAIN      = 0.5f;  // grados de servo por cm de error
const float EXT_WALL_MAX_DEG   = 15.0f; // tope de la corrección

float errorWall    = 0;
float prevErrorWall = 0;
float integralWall  = 0;

// ── PID Gyro ──────────────────────────────────────────────────────────────────
float KpGyro = 2.0;
float KiGyro = 0.0;
float KdGyro = 0.5;
float gyroScale = 1;

float errorGyro     = 0;
float prevErrorGyro = 0;
float integralGyro  = 0;

// ── Tiempo ────────────────────────────────────────────────────────────────────
unsigned long lastPIDTime  = 0;
unsigned long lastGyroTime = 0;

// ── Giroscopio ────────────────────────────────────────────────────────────────
float anguloGyro    = 0;
float anguloObjetivo = 0;
// Velocidad de giro filtrada (°/s, mismo signo que anguloGyro). La usa el gate
// del re-referenciado de CRUCERO para no adoptar como "recto" un chasis que
// TODAVÍA está rotando. Ver REREF_RATE_MAX_DEG_S.
float gyroRate      = 0;
unsigned long lastDodgeMs = 0;   // último loop con esquiva activa (prio/mem/RECUPERANDO)
unsigned int  rerefCount  = 0;   // diagnóstico: veces que el re-referenciado SÍ corrió

// ── Control ───────────────────────────────────────────────────────────────────
int velocidadMotor = 180;
int centroServo    = 80;

// ── Integración Pi → ESP32 ───────────────────────────────────────────────────
float obsBiasNorm  = 0.0;     // obs  [-1, 1] del mensaje V2
int   turnHint     = 0;       // turn {-1, 0, +1}
bool  piPriority   = false;   // prio=1: obstáculo activo en Pi
int   piMemoryFrames = 0;     // mem=N: frames de memoria restantes
bool  piPurePursuit = false;  // pp=1: Pi en modo Pure Pursuit
bool  piPasado     = false;   // pasado=1: la Pi confirma que un obstáculo quedó
                               // FÍSICAMENTE detrás del robot este frame (no que
                               // simplemente dejó de verlo) — dispara RECUPERANDO.
bool  piInteriorPass = false; // intr=1: el obstáculo actual se pasa por el mismo
                               // lado hacia el que va a girar la pista (la Pi ya
                               // sabe la dirección de giro, por visión, y el color
                               // del obstáculo) — el giro mismo resuelve el paso,
                               // no hace falta seguir bloqueando detectarEsquina().
bool  piInicioEstacionamiento = false; // inicio=1: al arrancar, la Pi vio rosa
                               // MAYORITARIO en el frame -> el carro está dentro
                               // del estacionamiento y toca la maniobra INICIO
                               // antes de SIGUIENDO. Debe venir ya en el 1er V2.
                               // "Sticky": una vez visto en 1 se queda en 1 (la
                               // Pi puede dejar de mandarlo tras el arranque); el
                               // one-shot de loop() lo consume una sola vez.
int   piPark       = 0;       // park=N: etapa del cajón vista por la Pi en la recta
                               // final (0 nada | 1 magenta ADELANTE | 2 magenta ya
                               // salió del campo de visión = lo estamos pasando /
                               // pasamos). Solo tiene sentido con parkBuscando.
int   piParkDistCm = 0;       // pd=N: distancia (cm, BEV) al bloque magenta más
                               // cercano que sigue adelante. 0 = desconocida.
bool  piReady      = false;

unsigned long lastPiMsgMs = 0;
const unsigned long piTimeoutMs = 800;  // ms sin mensaje → fallback

// Ganancia de visión V1 (modo fallback / obstáculo)
float visionSteerGain = 80.0;
float turnHintGain    = 7.0;

// Ganancia Pure Pursuit: obs = steer_deg / 60 → steerDeg = obs * 60 = steer_deg
const float ppSteerGain = 60.0;

// Cuánto se deflecta el servo por cada grado de PP.  steerDeg sale de la
// geometría (máx ±35°) y suele quedar corto para la mecánica del servo:
// súbelo si el carrito gira poco, bájalo si oscila/sobregira.
float ppServoGain = 1.0;
float PP_GYRO_BLEND = 0.12;  // 0 = solo vision, 1 = solo gyro. BAJO a propósito: en el modo
                            // por tramos el heading de referencia queda viciado tras cada
                            // MANIOBRA (no gira exacto 90°), y un blend alto "defiende" ese
                            // heading chueco -> el carro deriva lap a lap. Visión + wall PID
                            // re-referencian al carril/paredes reales y no tienen ese problema.
float PP_WALL_BLEND = 0.30;  // SUBIDO: el wall PID (distL-distR) centra en el carril y
                            // ayuda a des-enchuecar. Solo aplica en recta limpia
                            // (!piPriority && mem<=0), así que no choca con la esquina.

// ── Boot sincronización con Pi ────────────────────────────────────────────────
bool piReadyReceived = false;
// READY ya llegó PERO todavía no el primer V2. Entre esos dos, controlPID()
// caía en el fallback wall-PID y setMotor(velocidadMotor) -> el carro rodaba
// ~0.5 s hacia adelante sin ver el obstáculo. Con este gate el carro NO rueda
// hasta el primer V2 real de Pure Pursuit.
bool piFirstV2Received = false;
unsigned long readyMs = 0;                          // millis() cuando llegó READY
const unsigned long FIRST_V2_TIMEOUT_MS = 3000;     // si tras READY nunca llega V2, seguir igual
unsigned long bootStartMs = 0;
const unsigned long BOOT_WAIT_PI_MS = 1000;

// One-shot: la primera vez que el carro va a rodar de verdad (tras pasar el gate
// de WAIT_FIRST_V2) se re-anclan lastTurnTime y timeStart a ESE instante. Sin
// esto ambos quedaban fijados en el READY (varios segundos antes), así que el
// cooldownGiro y el gate de arranque ya estaban vencidos al primer frame de
// marcha y una lectura ancha de la zona de salida podía disparar el giro 1.
bool marchaIniciada = false;

// ── FSM estados ───────────────────────────────────────────────────────────────
// RECUPERANDO: la Pi confirma (piPasado=1) que el robot YA atravesó
// físicamente un obstáculo — no que la cámara simplemente dejó de verlo
// (perder de vista ≠ haber rebasado). En vez de que Pure Pursuit intente
// enderezar solo con lo que ve en ese instante incierto, aquí el wall PID +
// gyro PID (que YA se calculan siempre, ver controlPID()) toman el volante
// hasta que el robot vuelve a estar centrado/alineado.
// CRUCERO / MANIOBRA: solo ronda de obstáculos (rondaObstaculos=true). En lugar
// del giro continuo (GIRANDO), al llegar a una esquina el carro va DERECHO por
// ángulo (CRUCERO) hasta ~50cm de la pared y luego hace una maniobra por tramos
// (MANIOBRA): pivote hacia adelante o EN REVERSA según qué tan pegado va a la
// pared exterior del giro. Con rondaObstaculos=false nada de esto se usa.
// INICIO: solo ronda de obstáculos y solo si la Pi vio rosa mayoritario en el
// frame al arrancar (inicio=1) -> el carro parte dentro del estacionamiento y
// hace una "S" pre-programada para salir antes de entregar el volante a
// SIGUIENDO. Si inicio=0 nunca se entra a este estado (arranque normal).
// ESTACIONANDO: solo ronda de obstáculos, tras el giro 12 (PARK_ENABLED). Es el
// ESPEJO de INICIO: estacionamiento paralelo EN REVERSA dentro del cajón
// magenta de la recta de salida. La búsqueda del cajón NO es un estado aparte:
// es SIGUIENDO con la bandera parkBuscando (así conserva esquiva de conos y
// RECUPERANDO), solo que sin detectar esquinas y con la salida a ESTACIONANDO.
// ESTACIONANDO_PUNTA: alternativa a ESTACIONANDO (PARK_DE_PUNTA). Tras el giro 12
// sigue la pared exterior SIN visión y, al bajar el ultrasónico exterior por el
// 1er poste magenta, gira 90° y mete la trompa al lote (parcial). Ver su bloque.
enum Estado { SIGUIENDO, RECUPERANDO, GIRANDO, CRUCERO, MANIOBRA, TERMINANDO, INICIO, ESTACIONANDO, ESTACIONANDO_PUNTA };
Estado estado = SIGUIENDO;

// ── Giro por tramos (ronda de obstáculos) ────────────────────────────────────
const int  FRONT_TURN_FWD_CM = 75;     // CRUCERO -> MANIOBRA si la maniobra será FORWARD
                                       // (el arco necesita espacio adelante)
const int  FRONT_TURN_REV_CM = 25;     // ... si será REVERSE (hay que estar cerca de la pared
                                       // para que el pivote en reversa no sobrepase)
                                       // 2026-09-15: 20 -> 25. orillas946/947: pivote a dF~15 salía a
                                       // 36-39 cm de la exterior (salida ≈ dF + ~23) y el rojo de la boca
                                       // de la recta 1 quedaba encima -> esquiva a -66° -> pared interior.
const int  FRONT_CRUCERO_CM = 90;      // SIGUIENDO -> CRUCERO (recta ya limpia, esquina cerca)
const int  CRUCERO_GYRO_CM  = 90;      // dentro de CRUCERO: > esto -> visión (centerline recto);
                                       // <= esto -> pura gyro + wall PID (el centerline ya
                                       // curva para "esquivar" la pared del fondo y enchueca)
const int  CRUCERO_STRAIGHTEN_DEG = 15; // en CRUCERO, si el chasis entró/quedó chueco > esto,
                                        // fuerza gyro-hold para enderezar aunque dF > CRUCERO_GYRO_CM
                                        // (un obstáculo que se quedó `mia` hasta la esquina deja
                                        // el chasis ladeado y la MANIOBRA entra tardísima; orillas693 g5)
const int  MANIOBRA_OVERSHOOT_DEG = 14; // sale del pivote a (AngGiro - esto): el carro sigue
                                        // rotando por inercia y sin esto la recta nueva
                                        // arrancaba ~10-15° chueca (orillas460)
const int  HUG_CM           = 20;      // pared exterior <= esto -> FORWARD (no cabe reversear)
const int  MANIOBRA_VEL_REV  = 100;     // PWM objetivo del motor en la reversa-pivote
const int  MANIOBRA_VEL_MIN  = 80;     // PWM de arranque de la rampa (evita el golpe de corriente)

const float CRUCERO_WALL_BLEND = 0.8f; // en CRUCERO: cuánto del wall PID se mezcla para CENTRAR
                                       // en el carril (0 = solo heading, 1 = wall PID completo).
                                       // Solo aplica mientras ambas paredes existen.
const unsigned long MANIOBRA_FRENO_MS      = 300;   // coast (A1=A2=LOW) antes/después de invertir dirección
                                                    // — SIN esto el puente H se fríe por "plugging" (invertir
                                                    // con el motor girando). Reventó un TB6612 así (2026-09-01).
const unsigned long MANIOBRA_RAMP_MS       = 60;    // subir el PWM de reversa de a poco. Corto a
                                                    // propósito: el motor viene PARADO (coast de
                                                    // fase 0), así que arrancar a MANIOBRA_VEL_REV
                                                    // es un inrush normal, no "plugging". Puedes
                                                    // bajarlo más o dejarlo en 0.
const unsigned long MANIOBRA_REV_TIMEOUT_MS = 6000; // reversa no llegó a 88° -> frena y termina de frente
const unsigned long CRUCERO_TIMEOUT_MS      = 7000; // en CRUCERO tanto sin llegar a la pared -> MANIOBRA igual (red de seguridad anti-atasco)
const int CRUCERO_FRONT_DEBOUNCE = 5;  // lecturas consecutivas de dF<=30/70 (solo el frontal,
                                       // no los laterales) antes de disparar MANIOBRA
const int CRUCERO_PARED_DEBOUNCE = 3;  // ídem, cuando SÍ hay lateral abierta confirmando

// ── Anti-MANIOBRA-fantasma (run 2026-09-07) ──────────────────────────────────
// En 716 la maniobra salía con el carro rotando 40-60° sin control; la
// recuperación no alcanzaba a frenarlo y ~5 s después CRUCERO leía la pose
// chueca/descentrada (un lateral "abierto" por el yaw + dF~25 que era un CONO
// del tramo, no una pared) como esquina nueva -> disparaba otra MANIOBRA falsa
// -> oscilación divergente hasta chocar el verde. Estas dos constantes cortan
// el re-disparo; NO afectan a cruceroLargo (la red anti-atasco).
const unsigned long MANIOBRA_MIN_GAP_MS   = 3000; // el trigger por frontal exige al menos
                                                  // esto desde el último giro. Dos esquinas
                                                  // reales nunca caen tan seguidas (la
                                                  // maniobra + aproximación ya tarda varios s).
const unsigned long CRUCERO_YIELD_LATA_MS = 1500; // ventana tras ENTRAR a CRUCERO en la que
                                                  // una lata `mia` con dF corto se trata como
                                                  // lata (no esquina) y devuelve a SIGUIENDO
                                                  // para esquivarla. Fuera de esta ventana
                                                  // manda el "commit" de orillas696.
const int CRUCERO_YIELD_ANG_DEG = 30;  // |anguloGyro| máx. para esa salida (antes usaba
                                       // CRUCERO_STRAIGHTEN_DEG=15). orillas824 g5: el
                                       // centrado del propio CRUCERO (WALL_BLEND, despegándose
                                       // de la pared interior) llevó el chasis a -15.01° justo
                                       // cuando llegó el rojo `mia` (0.45 s tras entrar) ->
                                       // no soltó y lo ignoró ~1 s. En 78 CRUCEROs de
                                       // orillas821-824 el chasis pasó de 15° en la ventana
                                       // solo 4 veces; con 30 solo cambian 2 casos, ambos el
                                       // rojo de la misma recta. 15 sigue para enderezar.

// ── SETTLE de fin de MANIOBRA (runs 716/718/719) ─────────────────────────────
// El pivote (sobre todo REVERSA + backoff) entrega con velocidad angular: al
// llamar finalizarManiobra() con el carro TODAVÍA girando, se zeraba anguloGyro
// desde una mentira y RECUPERANDO/SIGUIENDO sobre-corregían 40-100° -> el carro
// terminaba encajado contra una pared un par de vueltas después. Antes de cerrar
// se hace coast + servo centro y se espera a que |ΔanguloGyro/Δt| baje del
// umbral por N samples (o venza el timeout).
const unsigned long MANIOBRA_SETTLE_SAMPLE_MS  = 40;    // cada cuánto se mide la velocidad angular
const float         MANIOBRA_SETTLE_RATE_DPS   = 30.0f; // deg/s por debajo de esto = "ya no rota"
const int           MANIOBRA_SETTLE_QUIETO_N   = 3;     // samples lentos SEGUIDOS para cerrar (~120 ms)
const unsigned long MANIOBRA_SETTLE_TIMEOUT_MS = 600;   // tope duro: cierra igual aunque no se aquiete

// Fase 4 (retroceso-post): la reversa a ciegas (servo centrado) giraba el carro
// ~3-5° hacia adentro CADA maniobra (run 745: fase4->fase6 -7±2° sistemático,
// asimetría mecánica + yaw residual). 2026-09-08 (tarde): la fase 4 corre un
// lazo cerrado de heading (aplicarReversaHold / KpRev·KiRev·KdRev) que mantiene
// el rumbo de entrada, así el carro reversea RECTO y puede hacerlo por más
// tiempo. Se quitó el corte por yaw (>4° -> abortaba el retroceso): con el PID
// manteniendo el rumbo ya no hace falta y cortaba la reversa antes de tiempo.

// ── Residual de giro: hace a MANIOBRA_OVERSHOOT_DEG NO crítico ───────────────
// finalizarManiobra() zeraba el heading a ciegas -> si el pivote sub/sobre-giró
// (varía con batería/piso/calibración del gyro), el carro arrancaba la recta
// nueva chueco y "se iba abriendo". Ahora se pasa el RESIDUAL real vs la recta
// nueva (que es 90° - inclinación de entrada) a la recuperación: over o under,
// SIGUIENDO/RECUPERANDO terminan de cuadrar el giro. OVERSHOOT_DEG solo decide
// cuándo el pivote suelta hacia el settle, ya no la precisión del heading final.
const float MANIOBRA_RESIDUAL_MAX_DEG = 45.0f;  // tope del residual que se pasa a la recuperación
                                               // (run 724: subido 30->45 -- el clamp escondía un
                                               //  error REAL grande; RECUPERANDO clampea a 60 igual)

// ── Sesgo hacia AFUERA al cerrar la MANIOBRA (2026-09-07) ────────────────────
// finalizarManiobra() ponía anguloObjetivo=0 = "apunta derecho por el carril".
// El carro venía terminando chueco hacia ADENTRO de la curva -> si RECUPERANDO/
// CRUCERO no cuadraban perfecto, se metía hacia el centro/un obstáculo. En vez
// de 0, el objetivo de la recta nueva se pone unos grados hacia la pared
// EXTERIOR del giro (signo = lado de afuera): el PID de giro lo MANTIENE, así
// que aunque la recuperación salga corta el carro deriva hacia la pared, no
// hacia una lata. NO es un error a corregir: es el objetivo. Costo: recorre la
// recta un poco cangrejo y el wall PID pelea contra este offset fijo.
// TAMBIÉN compensa la DERIVA mecánica del chasis (tira hacia adentro): sin este
// +5 el carro se va abriendo hacia adentro toda la recta y termina chocando
// obstáculos de la recta siguiente. NO bajarlo a 0.
//   giro DER -> exterior = izquierda -> anguloGyro positivo -> +BIAS
//   giro IZQ -> exterior = derecha   -> -BIAS
// Mantener < 12 (fuga de CRUCERO) y < CRUCERO_STRAIGHTEN_DEG(15) (adopt) para
// que esos dos no se lo coman.
const float MANIOBRA_BIAS_AFUERA_DEG = 5.0f;
// Tope duro de anguloObjetivo en la ronda de obstáculos. finalizarManiobra() lo
// pone en ±BIAS_AFUERA (±5); la fuga de CRUCERO y el adopt de esquina lo
// arrastran hacia el chasis chueco -> sin tope llegó a +14.8° en la vuelta 6 y
// cada maniobra entraba más torcida hasta morir (run 747). Con Opción A +
// residual handoff, ±5 YA es la referencia real; el tope deja ~3° de fuga para
// correcciones chicas y CORTA el runaway.
const float MANIOBRA_AO_CLAMP_DEG = 8.0f;
// ── Gate del re-referenciado de CRUCERO (2026-09-12) ─────────────────────────
// Medido sobre orillas853-856 (4 runs CCW, 37 rectas): el re-referenciado de
// abajo movía `anguloObjetivo` de −7.4 a +0.7 MIENTRAS `anguloGyro` subía de
// −1.9 a +15 (≈21 °/s) — o sea, adoptaba como "derecho" el latigazo de la
// esquiva y el carro entraba a la esquina apuntando +11..+15° hacia ADENTRO.
// Δ(heading) medio por recta: recta 0 −1.6°, recta 1 −2.3°, recta 2 +7.3°,
// recta 3 +19.2° — siempre hacia adentro en las rectas con esquivas.
// Ahora, además de chasis casi recto, exige yaw QUIETO y distancia temporal a la
// última esquiva. Si el gate bloquea siempre, `anguloObjetivo` se queda en el ±5
// que dejó finalizarManiobra(), que apunta hacia AFUERA = el lado seguro (el
// error de salida de maniobra medido es de solo −2.5° ± 0.9, el PID lo cubre).
const float         REREF_RATE_MAX_DEG_S = 6.0f;   // |gyroRate| máx. para ratificar
const unsigned long REREF_QUIET_MS       = 600;    // ms desde la última esquiva
// Tope de maniobraInclinacionEntrada: el wall-panic al llegar a la esquina
// spikea anguloGyro +6-10° en 2-3 frames y fase-0 lo captura como inclinación
// real -> envenena maniobraIdealRot/residual -> acumulación -> muere ~vuelta 8
// (run 748). El chasis real no entra a la esquina a más de ~±8.
const float MANIOBRA_INCL_ENTRADA_MAX_DEG = 10.0f;

const unsigned long MANIOBRA_BACKOFF_MS     = 700;   // retrocede esto tras el pivote (REV con holgura)
const unsigned long MANIOBRA_BACKOFF_FWD_MS = 700;   // retrocede esto tras el pivote (FWD)
const int           MANIOBRA_BACKOFF_VEL    = 100;  // PWM del retroceso
const int           MANIOBRA_BACKOFF_MIN_CM = 40;   // SOLO retrocede si la pared exterior del giro
                                                    // (la que sigues) está a MÁS de esto. Si vas
                                                    // pegado a ella, retroceder recto no ayuda.
// Tier "lejos de la pared exterior": si terminó la maniobra con MUCHA holgura
// (distExt > FAR_CM) retrocede más tiempo, para separarse bien de la recta nueva.
const int           MANIOBRA_BACKOFF_FAR_CM = 70;
const unsigned long MANIOBRA_BACKOFF_FAR_MS = 850;
// MANIOBRA 13 (solo con PARK_PUNTA_VUELTA_13): tras el pivote la pared que queda
// ATRÁS es la pared del lote. Aquí el retroceso ya no es por tiers: siempre corre
// (también en 20-40 cm, donde normalmente no hay) y dura lo suficiente para TOCAR
// esa pared en el peor caso (distExt > FAR_CM). En los casos cercanos llega antes
// y se queda empujando -> la media vuelta sale siempre desde la pared, y
// PARK_RETORNO_AVANCE_MS fija la distancia final al lote. Calibrar con el caso lejano.
const unsigned long MANIOBRA_BACKOFF_13_MS  = 1350;  // = FAR_MS + 500
const int           MANIOBRA_BACKOFF_13_VEL = 100;   // PWM (bájalo si empuja muy fuerte contra la pared)
// Tras los BACKOFF_13_MS sigue empujando en reversa esto más con el servo CENTRADO (sin
// heading-hold): la cola se asienta plana contra la pared del lote y el chasis queda
// perpendicular a ella de verdad. Ahí finalizarManiobra() pone anguloGyro = 0 (referencia
// real) en vez de heredar el residual de toda la carrera. orillas949-951: el 0 del gyro al
// estacionar estaba -30..+6° fuera de paralelo y el seguidor se iba contra la pared.
// 0 = desactivado (comportamiento anterior).
const unsigned long MANIOBRA_13_CUADRAR_MS  = 300;   // el retroceso normalmente ya llega pegado a la pared

// Grace post-esquiva: SIGUIENDO NO entra a CRUCERO por este tiempo tras el
// último frame CON obstáculo activo. La Pi manda `pasado=1` unos frames DESPUÉS
// de bajar prio/mem; sin el grace, SIGUIENDO ya saltó a CRUCERO y `case CRUCERO`
// se traga el pulso -> RECUPERANDO nunca corre -> el chasis queda chueco
// entrando a la esquina siguiente (run 724).
const unsigned long POST_DODGE_CRUCERO_GRACE_MS = 400;
unsigned long ultimoObstaculoMs = 0;   // millis del último V2 con piPriority || piMemoryFrames>0

unsigned long cruceroEntryMs = 0;
bool cruceroCerca      = false;        // en CRUCERO: true = cerca de la pared -> pura gyro+wall (sin visión)
int  contadorFront     = 0;            // debounce del sensor frontal
bool maniobraDecidida  = false;
bool maniobraGirarDer  = false;
bool maniobraReversa   = false;
bool maniobraRetroceso = false;        // true = hay espacio (pared exterior > MANIOBRA_BACKOFF_MIN_CM) -> retrocede un poco DESPUÉS de la maniobra
long maniobraDistExt   = 0;
int  maniobraFase      = -1;           // -1=sin init  0=frenar-antes  1=pivote  2=frenar-después  3=frenar-y-reintentar-fwd  4=retroceso-post  5=frenar-tras-retroceso  6=settle (esperar que deje de rotar)
unsigned long maniobraFaseMs   = 0;    // inicio de la fase actual (para las pausas de freno)
unsigned long maniobraPivoteMs = 0;    // inicio del pivote (para la rampa y el timeout de reversa)
float maniobraInclinacionEntrada = 0;  // anguloGyro (recta actual) justo al entrar a la maniobra
// Fase 6 (SETTLE): esperar a que la velocidad angular baje antes de cerrar.
unsigned long maniobraSettleMs      = 0;   // inicio de la fase 6
unsigned long maniobraSettleSampMs  = 0;   // millis del último sample de rate
float         maniobraSettleAngPrev = 0.0f;// anguloGyro en el último sample
int           maniobraSettleQuieto  = 0;   // samples consecutivos con rate por debajo del umbral
float         maniobraIdealRot      = 0.0f;// rotación (con signo) que deja el chasis cuadrado con la
                                           // recta nueva; finalizarManiobra() pasa (anguloGyro - esto)
                                           // como residual a la recuperación (ver MANIOBRA_RESIDUAL_MAX_DEG)
float         maniobraFase4AngIni   = 0.0f;// anguloGyro al entrar a fase 4 = setpoint del heading-hold de reversa

// Rectas con el cajón de estacionamiento: el borde del cajón tapa a ratos el
// lateral que debería "abrirse" en la esquina, así que justo en el frame en
// que dF llega a su umbral de giro, paredAbierta puede leer false -> enLaPared
// nunca dispara y la maniobra termina saliendo por cruceroLargo (el timeout,
// ya muy metida en la esquina). En vez de exigir paredAbierta EN ESE frame, se
// cuenta cuántas veces el lateral hacia el que se gira pasa de "pared" a
// "abierta" desde que dF entra a LATERAL_WATCH_CM: una esquina limpia cae 0-1
// veces; el cajón la hace oscilar y suma varias -> arma giroSucioArmado (ver
// case CRUCERO). El giro NUNCA se dispara SOLO por esto: sigue exigiendo
// distF <= umbral, porque el frontal solo puede leer "sin pared" por eco
// perdido con el lateral perfectamente bien — Y a su vez giroSucioArmado
// tampoco se arma con una sola lectura: distL/distR NO tienen mediana (a
// diferencia de distF/medianaFront), solo EMA con alpha=0.85, que con un solo
// eco malo (raw=200) ya salta el filtrado muy por encima de umbralPared en 1
// frame. LATERAL_OPEN_DEBOUNCE exige que el lateral se sostenga "abierto"
// LATERAL_OPEN_DEBOUNCE frames SEGUIDOS antes de contar la caída — un pico de
// ruido de 1 frame no confirma nada, hace falta apertura real (aunque breve)
// para sumar.
const int  LATERAL_WATCH_CM      = 70;   // dF por debajo de esto -> arranca la vigilancia
const int  LATERAL_DROP_MIN      = 3;    // caídas confirmadas del lateral para armar el trigger alterno
const int  LATERAL_OPEN_DEBOUNCE = 2;    // frames seguidos "abierto" para confirmar UNA caída
bool lateralWatchActivo = false;
int  lateralOpenStreak  = 0;   // frames consecutivos actuales con el lateral vigilado "abierto"
int  lateralDropCount   = 0;
bool giroSucioArmado    = false;

// ── Latch de dirección de APROXIMACIÓN ────────────────────────────────────────
// En la esquina con un obstáculo cercano al costado, el cono de sonido del
// lateral se lockea en ESE obstáculo (~75cm) en cuanto dF baja de ~50 ->
// paredAbierta se apaga justo cuando haría falta y la maniobra sale por el
// timeout (cruceroLargo), ya metida en la pared (esquinas C: turnos 3/7/11).
// PERO antes de eso (dF ~70->50) el lateral SÍ da la apertura real, sólida,
// 5-7 frames. Aquí se latchea el primer lado que abre de forma SOSTENIDA
// durante lateralWatchActivo. Sirve para: (a) habilitar el disparo de enLaPared
// por el frontal aunque el lateral ya no lea abierto, y (b) darle la dirección
// correcta a decidirManiobra en la 1ª esquina (primerGiro aún no latcheado).
// 0 = nada aún | 1 = abrió IZQUIERDA | 2 = abrió DERECHA.
int  direccionAproxLatch     = 0;
int  aproxOpenStreakIzq      = 0;
int  aproxOpenStreakDer      = 0;
const int APROX_DIR_LATCH_FRAMES = 3;  // frames seguidos de un lado abierto para latchear

const float wallSettleCm    = 8.0;   // |distL-distR| por debajo de esto = "centrado"
const float headingSettleDeg = 6.0;  // |errorGyro| por debajo de esto = "alineado"

// Red de seguridad: si el robot entra a RECUPERANDO cerca de una esquina real
// (donde un ultrasónico lee "sin pared" legítimamente, no por desalineación),
// wallOk puede no cumplirse NUNCA y el estado se quedaría atorado para siempre.
// Este timeout fuerza la salida aunque wallOk/headingOk no se hayan cumplido.
unsigned long recuperandoEntryMs = 0;
const unsigned long recuperandoTimeoutMs = 1500;
// Dwell de RECUPERANDO: NO sale en el mismísimo frame en que roza headingOk
// (ahí las lecturas están más sucias por el latiguazo). Tiene que llevar
// recuperandoMinMs SEGUIDOS con el heading YA alineado -> recién ahí sale, ya
// recto y con el frontal/laterales calmados. (2026-09-07: antes contaba desde
// que ENTRABA a RECUPERANDO, no desde que llegaba al heading -> mal aplicado.)
const unsigned long recuperandoMinMs = 75;
unsigned long headingOkSinceMs = 0;   // millis del 1er frame con headingOk (0 = aún no / se perdió)

// ── Giros ─────────────────────────────────────────────────────────────────────
bool direccionIzquierda = true;
bool primerGiro         = false;

// Ángulo de giro objetivo — DISTINTO por tipo de ronda:
//   ronda de obstáculos (rondaObstaculos=true) : ~90° reales (pivote/maniobra)
//   ronda cerrada       (rondaObstaculos=false): 76° (el giro continuo se pasa
//                                                por inercia, así que sale antes)
const int ANG_GIRO_OBSTACULOS = 90;   // <- bájalo a 88 si se pasa en la de obstáculos
const int ANG_GIRO_CERRADA    = 76;
const int AngGiro = rondaObstaculos ? ANG_GIRO_OBSTACULOS : ANG_GIRO_CERRADA;
// MANIOBRA de FRENTE (arco hacia adelante, no reversa): objetivo un poco menor
// que AngGiro — el arco fwd llega con más inercia y se pasa. La reversa usa AngGiro.
const int ANG_GIRO_MANIOBRA_FWD = 84;
unsigned long lastTurnTime = 0;
int timeStart = 0;
const int COOLDOWN_GIRO_OBSTACULOS = 3000;
const int COOLDOWN_GIRO_ABIERTA    = 1000;
const int cooldownGiro = rondaObstaculos ? COOLDOWN_GIRO_OBSTACULOS : COOLDOWN_GIRO_ABIERTA;

// ── INICIO — maniobra de salida del estacionamiento (ronda de obstáculos) ─────
// Solo corre si la Pi manda inicio=1 (rosa mayoritario al arrancar). El carro
// hace una "S" pre-programada: saca la nariz hacia el interior hasta ~60°,
// avanza un tramo recto, contravuelve hasta volver a ~0° (alineado con la
// recta), y retrocede un colchón por si hay un obstáculo pegado a la salida del
// cajón. Al terminar entra a SIGUIENDO. Ver `case INICIO`.
// El estacionamiento SIEMPRE está sobre la pared exterior, así que el lateral
// más corto (la pared) fija a la vez el lado de salida Y la dirección de giro de
// toda la pista (se latchea como hace GIRANDO en la 1ª esquina).
// TODOS estos números son de ARRANQUE — hay que tunearlos en el tapete.
const int  INICIO_PWM                  = 95;   // PWM de avance durante la maniobra
const int  INICIO_PWM_MIN              = 80;   // arranque de la rampa (evita stall en seco)
const unsigned long INICIO_RAMP_MS     = 120;  // sube de INICIO_PWM_MIN a INICIO_PWM
const int  INICIO_ANG_OUT_DEG          = 60;   // fase 1: ángulo de salida (nariz al interior)
const int  INICIO_OVERSHOOT_DEG        = 8;    // corta el servo antes; la inercia completa (0 = sin corte)
const unsigned long INICIO_SWING_TIMEOUT_MS   = 3000; // red de seguridad de la fase 1 (si no llega al
                                                     //  ángulo — patina / topa la pared del cajón —
                                                     //  pasa a la fase 2 igual; la contravuelta y la
                                                     //  reversa terminan de cuadrar lo que haya)
const unsigned long INICIO_MID_MS      = 1;  // fase 2: tramo recto entre los dos giros
const int  INICIO_ENDEREZA_MARGEN_DEG  = 8;    // fase 3: sale de la contravuelta con este margen a 0
const unsigned long INICIO_CONTRA_TIMEOUT_MS = 4000; // red de seguridad de la fase 3
const int  INICIO_REV_PWM             = 100;  // fase 5: PWM de la reversa
const unsigned long INICIO_REV_MS      = 2250; // fase 5: duración de la reversa (colchón de seguridad)
const int  INICIO_DIR_MIN_GAP_CM       = 25;   // |dL-dR| mínimo para latchear la dirección de PISTA
                                              // (si el cajón deja lectura ambigua, no se arriesga el
                                              //  latch global: la 1ª esquina real decide como siempre)

// Estado interno de INICIO
bool inicioEvaluado = false;   // one-shot: ¿ya se decidió si entrar a INICIO?
int  inicioFase     = -1;      // -1 init | 1 swing | 2 recto | 3 contra | 4 coast | 5 reversa | 6 settle
bool inicioGirarDer = false;   // lado de salida del cajón (servo full hacia ahí en la fase 1)
unsigned long inicioFaseMs = 0;              // inicio de la fase actual (timers/rampa)
float         inicioSettleAngPrev = 0.0f;   // anguloGyro en el último sample de la fase 6
unsigned long inicioSettleSampMs  = 0;
int           inicioSettleQuieto  = 0;

// ═══════════════════════════════════════════════════════════════════════════════
// ESTACIONANDO — Estacionamiento en paralelo en reversa (Obstacle Challenge)
// ═══════════════════════════════════════════════════════════════════════════════
// Tras el giro 12 el auto entra a la recta inicial en busca del cajón delimitado
// por los dos postes magenta (200x20x100 mm). La maniobra es el ESPEJO de INICIO:
//   Fase 0: FRENO_PREV  (motorCoast durante MANIOBRA_FRENO_MS antes de reversa)
//   Fase 1: REV_SWING   (reversa con servo a la pared exterior, cola al cajón)
//   Fase 2: REV_CONTRA  (reversa con contravuelta para re-alinear paralelo a 0°)
//   Fase 3: FRENO_MID   (motorCoast durante MANIOBRA_FRENO_MS antes de adelante)
//   Fase 4: CENTRADO    (avance corto recto con heading-hold entre postes)
//   Fase 5: FIN         (freno total, servo al centro, carrera terminada)

const bool          PARK_ENABLED             = true;  // true = busca y estaciona tras la vuelta 12
const bool          PARK_TEST_DIRECTO        = false; // true = inicia INMEDIATO en reversa (pon el carro AL LADO del cajón)
const bool          PARK_TEST_RECTA_COMPLETA = false; // true = inicia en recta final (pon el carro AL INICIO de la recta)
const bool          PARK_TEST_PARED_IZQ      = true;  // en tests: true = cajón en pared IZQ (como en tus fotos), false = DER
// Entrada a ESTACIONANDO desde la recta final (parkBuscando).
// false (2026-09-15) = entra a la fase 0 EN CUANTO empieza la recta: el escaneo por
//   sonar encuentra los postes solo (PARK_REQUIERE_PI=false) y el seguidor mantiene
//   los PARK_PUNTA_PARED_CM, así que tiene toda la recta para buscar.
// true = comportamiento viejo: espera a que la Pi mande park=2 (rosa pegado a la
//   defensa, y_max>=420) o al timeout de abajo. orillas974 entró a los 6.7 s porque
//   el rosa alcanzó 375 px justo antes de perderse; en 975/976 se quedó en 334/335,
//   la Pi reseteó la búsqueda y el disparo llegó con la trompa ya en la pared del
//   fondo (dF=2) -> "se va de paso". OJO: con false, en esa recta el carro sigue la
//   pared por sonar y NO esquiva con cámara (en la sección del cajón los conos van
//   en la fila interior, a ~60 cm de la pared exterior).
const bool          PARK_ESPERA_PI           = false;
const unsigned long PARK_BUSCANDO_TIMEOUT_MS = 4000;  // solo con PARK_ESPERA_PI=true: red de
                                                      // seguridad si la Pi nunca ve el cajón
                                                      // (10 s a ~30 cm/s eran 3 m, más que la recta)
const int           PARK_APPROACH_PWM        = 100;   // PWM en recta de aproximación al cajón
const int           PARK_ANG_IN_DEG          = 60;    // tope del ángulo de entrada (ver PARK_RADIO_CM)
const int           PARK_ANG_MIN_IN_DEG      = 35;    // piso: por debajo la maniobra ya no mete el carro
// 2026-09-15, runs 1008-1010: con 3 el ángulo total salía 59-66° en vez de 57.
// La inercia de la reversa a tope vale ~9°, no 3. Con 3 la maniobra entregaba
// 24-29 cm de corrimiento cuando sólo cabían 18-19 -> el carro se encajaba en la
// pared exterior, la contravuelta se quedaba sin poder girar y timeouteaba, y el
// carro terminaba 24-42° chueco (antes terminaba a 9-16°).
const int           PARK_OVERSHOOT_DEG       = 9;     // corte anticipado para absorber inercia (deg)
const unsigned long PARK_REV1_TIMEOUT_MS     = 3000;  // timeout fase 1 metida (ms)
const int           PARK_REV_PWM             = 85;    // PWM de marcha atrás — suave para control preciso
const int           PARK_ENDEREZA_MARGEN_DEG = 3;     // margen para considerar alineado (|ang| <= 3 deg)
const unsigned long PARK_REV2_TIMEOUT_MS     = 3500;  // timeout fase 2 contravuelta (ms)
const unsigned long PARK_CENTER_MS           = 400;   // avance suave para centrado entre postes (ms)
const int           PARK_CENTER_PWM          = 75;    // PWM suave de centrado adelante

// ── Límites ultrasónicos de estacionamiento ────────────────────────────────────
// Carrito = 22 cm, cajón = 33 cm (1.5*L). Margen libre total = 11 cm.
// El sensor frontal (distF) apunta al poste magenta delantero:
// - Si distF >= 8 cm frente al poste delantero, ya retrocedimos 8 cm y quedan solo ~3 cm atrás:
//   "¡YA ESTAMOS MUY ATRÁS!" -> Corte inmediato de reversa para no chocar el poste trasero.
const int           PARK_REV_MAX_DF_CM        = 8;    // cm: distancia máxima permitida a la barrera frontal en reversa
const int           PARK_REV_VALID_DF_MAX_CM  = 18;   // cm: rango para validar que distF ve la barrera y no pista vacía
const int           PARK_WALL_TARGET_CM       = 6;    // cm: distancia lateral a la pared para considerarse pegado
const int           PARK_CENTER_TARGET_DF_CM  = 5;    // cm: objetivo de centrado adelante (5 cm al frente, ~6 cm atrás)


// Paralelo 10 pts — scan poste1/hueco/poste2 + maniobra en reversa (wiggle)
const float         PARK_PARED_CM             = 28.0f;
const float         PARK_KPOS                 = 0.5f;
const float         PARK_ANG_MAX_DEG          = 7.0f;
const float         PARK_KP_ANG               = 2.2f;
const float         PARK_KD_RATE              = 0.30f;
const int           PARK_SERVO_MAX            = 35;
const int           PARK_PWM                  = 95;
const float         PARK_BASE_ALPHA           = 0.2f;
const int           PARK_BASE_N               = 8;
const int           PARK_CAIDA_CM             = 10;
// (PARK_CAIDA_MIN_CM=8 quitado 2026-09-15: descartaba el poste cuando el carro pasa
//  pegado, orillas959 dio 7,5. La lectura se clasifica como ESTACIONANDO_PUNTA y usa
//  PARK_PUNTA_SALTO_MAX_CM / _ALTO_REBASE_N / _CAIDA_HUECOS_N.)
const int           PARK_CAIDA_N              = 2;
const int           PARK_GAP_N                = 2;
const int           PARK_REBASE_N             = 20;
const int           PARK_PARED_MAX_CM         = 55;
const unsigned long PARK_ARMADO_MS            = 600;
const float         PARK_ARMADO_ANG_DEG       = 15.0f;
const bool          PARK_REQUIERE_PI          = false;
const unsigned long PARK_TIMEOUT_MS           = 10000;
const int           PARK_FRENTE_CM            = 18;
const unsigned long PARK_HUECO_MIN_MS         = 0;      // 2026-09-15: 500 -> 0. El poste 2 se acepta en
                                                        // cuanto vuelve a haber bajada tras el hueco confirmado
                                                        // (la máquina de estados ya ordena poste1/hueco/poste2)
const unsigned long PARK_HUECO_TIMEOUT_MS     = 2500;
const unsigned long PARK_PASS_EXTRA_MS        = 600;
const unsigned long PARK_POSTE2_WAIT_MS       = 650;
const float         PARK_ALIGN_CM             = 25.0f;
// 2026-09-15, medido en los 4 intentos de la noche (runs 1004-1007): con 700 ms
// la fase 3 SIEMPRE salía por timeout, nunca por ángulo — llegaba a 19-29° en vez
// de los 57 que pide PARK_ANG_IN_DEG. El giro lo terminaba de dar la fase 4, y
// como esa es de tiempo fijo, el ángulo TOTAL caía donde cayera: 42, 49, 53, 62°.
// Ese es el verdadero origen de la inconsistencia (no la altura lateral):
//     total 62° -> quedó a  7 cm      total 49° -> quedó a 15 cm
//     total 53° -> quedó a 13 cm      total 42° -> quedó a 14 cm
// 2000 ms es red de seguridad pura: a ~50 deg/s los 57° se cierran en ~1.3 s.
const unsigned long PARK_SWING_TIMEOUT_MS     = 2000;
// Y por eso mismo este se va a 0: con la fase 3 cerrando de verdad en 57°, los
// 550 ms extra a tope sumaban ~27° más (total ~84°) = casi 45 cm de corrimiento
// lateral, más del doble de lo que cabe. El reparto correcto es TODO el ángulo
// en la fase 3 y el ajuste fino en el tramo recto de la fase 14.
// El tiempo total de reversa casi no cambia (antes 700+550, ahora ~1300 solo que
// cortando por ángulo en vez de por reloj).
const unsigned long PARK_REV_EXT_HOLD_MS      = 0;
// ── Compensación de altura lateral de llegada (fase 14, 2026-09-15) ─────────
// PROBLEMA: arco 60° + PARK_REV_EXT_HOLD_MS a tope + contravuelta son los tres
// fijos, así que la maniobra entrega SIEMPRE el mismo corrimiento lateral, sin
// importar con qué distancia a la pared exterior llegó el carro. La distancia
// final al cajón sigue 1:1 a la de llegada: cada cm más abierto = un cm que se
// queda fuera del cajón.
// FIX: un tramo RECTO nuevo (fase 14) justo al cerrar los 60°, ANTES del tramo
// a tope ya calibrado. Con el carro a 60° cada cm que retrocede vale sin(60°) =
// 0.866 cm de corrimiento lateral, así que el tramo se come el exceso y a la
// maniobra vieja le entrega siempre la misma altura efectiva:
//     tau_ms = (parkBase - PARK_RECTO_D0_CM) / (0.866 * v) * 1000
// En D0 el tramo dura 0 ms => la maniobra queda EXACTAMENTE como está hoy.
// PEAJE: también retrocede cos(60°)/sin(60°) = 0.577 cm por cm corregido; el
// tope PARK_RECTO_MAX_MS es lo que lo separa del poste trasero.
// Si llega MÁS pegado que D0 el tramo se queda en 0 (no se puede restar): el
// carro acaba más metido hacia la pared, que es el lado seguro — y la guarda
// `pegado` (PARK_PEGADO_CM) de la fase 8 corta la contravuelta si se arrima.
const bool          PARK_RECTO_ENABLED        = true;
// OJO: D0 es la lectura del SONAR (parkBase), no una medida de flexómetro. La
// primera corrida imprime "d=" en la entrada a la fase 14 y manda pnb=/pnt= por
// telemetría: si el carro que sí entra bien reporta otra cosa, ajusta D0 a ESE
// número. Un error de 4 cm aquí son ~185 ms de sesgo en toda la tabla.
// 2026-09-15 (runs 1008-1010): D0 YA NO SE FIJA A MANO. Con el ángulo cerrando
// de verdad se vio que llegar a 32 era la anomalía — la fase 1 persigue
// PARK_ALIGN_CM=25, así que lo normal es llegar a 25-26. Y a 25 sólo caben 18 cm
// de corrimiento, mientras que un arco de 60° entrega 25 => el carro se encajaba
// en la pared. Ahora D0 sale de la geometría: es la altura donde el arco de 60°
// cae clavado, D0 = PARK_FINAL_CM + PARK_RADIO_CM (con 7+25 da los mismos 32,
// que es justo la corrida que sí funcionó).
//   delta = parkBase - PARK_FINAL_CM     (corrimiento lateral que hace falta)
//   delta >= R  -> ángulo 60° + tramo recto de (delta-R)/(0.866*v)   [caso lejos]
//   delta <  R  -> ángulo acos(1 - delta/2R), sin tramo recto        [caso cerca]
// Las dos ramas son la MISMA curva; la de abajo es la que faltaba y la que está
// mordiendo hoy.
// R medido ajustando final = pnb - 2R(1-cos a) a 4 corridas: 23.6/24.9/27.0/25.0.
const float         PARK_RADIO_CM             = 25.0f;
// Distancia final a la pared exterior que se busca (misma lectura que parkBase).
// 7 cm es lo que midió la corrida buena (21:38).
const float         PARK_FINAL_CM             = 7.0f;
const float         PARK_RECTO_D0_CM          = PARK_FINAL_CM + PARK_RADIO_CM;
// 2026-09-15: ya NO hace falta medirlo a mano. Ajustando final = pnb - 2R(1-cos a)
// a los 4 intentos de la noche sale R = 23.6 / 24.9 / 27.0 / 25.0 cm (media 25.1),
// y con la fase 4 girando a ~49 deg/s => v = R*w = 25.1 * 0.86 = 21.6 cm/s.
// Con 22 cm/s la corrección sale a 52 ms por cm. Si aun así queda corto/largo de
// forma consistente, esta es la constante que se mueve.
const float         PARK_RECTO_VEL_CMS        = 22.0f;
// Tope de seguridad. 500 ms a 25 cm/s = 12.5 cm de reversa = 10.8 cm laterales
// de rango (hasta parkBase ~43) y 6.2 cm extra hacia el fondo del cajón.
const unsigned long PARK_RECTO_MAX_MS         = 500;
// Meneo de ruedas entre la reversa de entrada y la contravuelta (fases 5 y 6).
// 2026-09-15: AMBOS EN 0 = se saltan esas fases (de la 4 se va directo a la 8).
// Eran ciegos, ~8 cm de reversa en total, y el usuario no los quiere.
const unsigned long PARK_WIGGLE_INT_MS        = 0;      // reversa con volante al lado contrario
const unsigned long PARK_WIGGLE_EXT_MS        = 0;      // reversa con volante hacia la pared
const unsigned long PARK_CONTRA_TIMEOUT_MS    = 2500;
const float         PARK_ENDEREZA_TOL_DEG     = 15.0f;
const int           PARK_PEGADO_CM            = 2;
const unsigned long PARK_FWD_MS               = 650;
const float         PARK_FINAL_TOL_DEG        = 3.0f;
// Reversa final de enderezado DESPUÉS del acomodo hacia adelante (fases 12 y 13).
// 2026-09-15: false = al terminar la fase 11 se acaba la maniobra ahí mismo. Con
// true vuelve a reversear hasta quedar a PARK_FINAL_TOL_DEG, que es lo que estaba
// empujando el carro contra la pared de atrás.
const bool          PARK_REV_FINAL            = false;
const unsigned long PARK_REV0_TIMEOUT_MS      = 2500;
const int           PARK_CENTER_HI_CM         = 4;
const int           PARK_CENTER_PWM_PAR       = 75;

// Estado interno de ESTACIONANDO
bool          parkBuscando          = false;
unsigned long parkBuscandoEntryMs   = 0;
int           parkFase              = -1;
unsigned long parkFaseMs            = 0;
bool          parkParedEsIzquierda  = false;

unsigned long parkEntryMs           = 0;
int           parkScanSubFase       = 0;
float         parkBase              = 0.0f;
int           parkBaseN             = 0;
int           parkCaidaCnt          = 0;
int           parkCaidaHuecos       = 0;   // "sin eco" tolerados a media bajada
int           parkAltasCnt          = 0;   // lecturas altas seguidas (eco perdido/rebote)
int           parkGapCnt            = 0;
int           parkInvalidasCnt      = 0;
int           parkFrenteCnt         = 0;
int           parkDfCnt             = 0;
bool          parkRosaVisto         = false;
float         parkErrPared          = 0.0f;
float         parkRumboRef          = 0.0f;
float         parkRumboArco0        = 0.0f;
long          parkExtRaw            = 0;
long          parkCaidaLectura      = 0;
unsigned long parkHuecoEntryMs      = 0;
unsigned long parkPassExtraMs       = 0;
int           parkCenterMode        = 0;
int           parkCenterOkCnt       = 0;
int           parkRetryN            = 0;
unsigned long parkWiggleMs          = 0;
float         parkBaseLlegada       = 0.0f; // parkBase latcheada al parar junto al cajón (fase 1 -> 2)
unsigned long parkRectoMs           = 0;    // duración calculada del tramo recto de la fase 14
float         parkAngObjetivo       = (float)PARK_ANG_IN_DEG; // ángulo de entrada calculado para esta llegada
float         parkRumboGiro0        = 0.0f;
int           parkSettleQuieto      = 0;
unsigned long parkSettleSampMs      = 0;
unsigned long parkLogMs             = 0;
bool          cajonParedEsIzquierda = false; // detectada y latcheada en INICIO al arrancar la carrera
bool          cajonParedDetectada    = false;

// ═══════════════════════════════════════════════════════════════════════════════
// ESTACIONANDO_PUNTA — Estacionamiento DE PUNTA (parcial) tras el giro 12
// ═══════════════════════════════════════════════════════════════════════════════
// Primer escalón antes del paralelo: basta con que PARTE del carro quede dentro
// del lote (regla 1.8.3 "partly or not parallel" = 7 pts; tocar un poste = 0).
// Tras el giro 12 el carro NO maneja con visión: sigue la pared EXTERIOR (donde
// siempre está el lote). Los postes magenta salen 20 cm de la pared, así que al
// llegar al primero el ultrasónico exterior BAJA ~20 cm de golpe vaya a la
// distancia que vaya (30->10, 45->25). Por eso la bajada se mide RELATIVA a una
// línea base (EMA de las lecturas de pared), nunca contra un valor absoluto.
//
// Wall follower en CASCADA: error de pared -> rumbo objetivo (capado) -> gyro PD.
// Un sesgo del gyro de X° solo corre el carril X/KPOS cm; no hace que derive,
// porque la pared siempre corrige la posición. La Pi solo sirve de filtro: la
// bajada se acepta si ya vio rosa (park>=1).
//   Fase 0 SEGUIR : wall follower + vigila la bajada
//   Fase 1 FRENO  : coast + servo recto (el carro se detiene)
//   Fase 2 AJUSTE : opcional, recto adelante (+ms) o en reversa (-ms): mueve el
//                   punto donde cae la trompa según tu radio de giro
//   Fase 3 PREP   : coast + servo a tope hacia la pared (espera a que llegue)
//   Fase 4 ARCO   : avanza con servo a tope hasta PARK_PUNTA_ARCO_DEG, o dF tope
//   Fase 5 ENTRA  : recto (gyro hold) hacia la pared hasta dF tope o timeout
//   Fase 6 FIN    : motor apagado, carrera terminada
//   Fase 10 MANO  : (solo PARK_PUNTA_TEST_MANO) servo a tope, motor apagado
//
// MEDIA VUELTA (PARK_PUNTA_VUELTA_13=true): el carro NO estaciona tras el giro 12.
// Sigue la carrera normal por la recta de salida (esquivando con colores: el cono
// del borde de la esquina 12 todavía es de la vuelta 3), hace la MANIOBRA 13 como
// siempre y, en vez de volver a SIGUIENDO, da 90° MÁS hacia adelante en el mismo
// sentido de la carrera. Queda de regreso hacia la recta de salida con la pared del
// lote del OTRO lado (vigila el sensor contrario) y ahí hace lo mismo de siempre.
//   Fase 20 GIRO     : avanza recto PARK_RETORNO_AVANCE_MS (aleja el final de la
//                      pared del lote) y luego servo a tope hacia la pared del
//                      regreso hasta ~90°
//   Fase 21 SETTLE   : coast hasta que deja de rotar; 0° = recta de regreso
//   Fase 22 REVERSA  : opcional, recto hacia atrás POR TIEMPO con heading-hold de GYRO
//                      (aplicarReversaHold, NO la pared) para ganar carrera (ver REV_*)
//   Fase 23 FRENO    : coast antes de volver a adelante -> fase 0
// Reglamento 9.23: en sentido contrario solo se anda en la sección donde cambió de
// sentido (esquina 13) y la vecina (recta de salida). Si no ve la bajada, TIENE que
// parar antes de salir de la recta de salida (PARK_PUNTA_TIMEOUT_MS).
//
// Prueba a mano (PARK_PUNTA_TEST_MANO=true): arranca directo aquí con el motor
// SIEMPRE apagado. Empuja el carro por el carril hasta que imprima "BAJADA";
// el servo se va a tope hacia la pared y ahí empujas el carro por el arco para
// ver dónde cae la trompa. Si cae antes del lote -> PARK_PUNTA_AJUSTE_MS > 0;
// si cae sobre el poste lejano -> PARK_PUNTA_AJUSTE_MS < 0.
const bool          PARK_DE_PUNTA              = true;   // punta 7 pts si PARK_MODO_PARALELO=false
const bool          PARK_MODO_PARALELO        = true;   // true = paralelo 10 pts (scan + reversa wiggle)
const bool          PARK_TEST_MANO            = false;
const bool          PARK_PUNTA_TEST_MANO       = false;  // true = prueba A MANO (motor apagado). Usa PARK_TEST_PARED_IZQ
                                                         // (con PARK_TEST_RECTA_COMPLETA=true arranca aquí CON motor)
const bool          PARK_PUNTA_VUELTA_13       = true;   // true = sigue hasta la MANIOBRA 13, media vuelta y estaciona REGRESANDO
                                                         // false = estaciona de frente justo tras el giro 12
const bool          PARK_TEST_RETORNO          = false;  // true = arranca directo en la media vuelta (pon el carro como si acabara
                                                         // la MANIOBRA 13). Gira hacia PARK_TEST_PARED_IZQ (= pared del regreso)
// Media vuelta (fases 20-23)
const int           PARK_RETORNO_PWM           = 95;
const int           PARK_RETORNO_PWM_MIN       = 80;     // arranque de la rampa
const unsigned long PARK_RETORNO_RAMP_MS       = 120;
const unsigned long PARK_RETORNO_AVANCE_MS     = 300;    // avance recto ANTES de la media vuelta: más = termina más lejos
                                                         // de la pared del lote (0 = gira desde parado como antes)
const int           PARK_RETORNO_OVERSHOOT_DEG = 10;     // corta antes de 90; la inercia completa
const unsigned long PARK_RETORNO_TIMEOUT_MS    = 4000;
const unsigned long PARK_RETORNO_SETTLE_MAX_MS = 670;    // tope de la espera a que deje de rotar (>= MANIOBRA_FRENO_MS)
// Reversa recta tras la media vuelta, para tener carrera antes del 1er poste. En CCW
// el lote está pegado a la esquina 13 (Fig. 4/8d del reglamento): sin esto la media
// vuelta deja el carro casi encima del 1er poste y la detección (BASE_N + ARMADO_MS)
// no alcanza a armarse. En CW el lote queda al fondo del regreso. Solo gyro, sin pared.
const unsigned long PARK_RETORNO_REV_CCW_MS    = 1200;   // regreso con pared a la IZQ (carrera CCW)
// 2026-09-15: estaba en 0, y en 0 la fase 22 NO EXISTE — el `if (revMs > 0)` de
// la fase 21 se la brinca y cae directo al scan. Por eso la reversa "a veces sí y
// a veces no": no era aleatorio, dependía del sentido de la carrera. En los logs
// de la noche, los dos regresos con pared IZQ hicieron la fase 22 y los dos con
// pared DER no la hicieron, y esos son justo los que acabaron pegados a la pared
// de enfrente ignorando el primer poste. Se iguala al valor de CCW, que ya está
// probado. Si en pista resulta que el CW necesita otro tiempo, se separa aquí.
const unsigned long PARK_RETORNO_REV_CW_MS     = 1200;   // regreso con pared a la DER (carrera CW)
const int           PARK_RETORNO_REV_PWM       = 90;
// Wall follower
const int           PARK_PUNTA_PWM             = 95;     // PWM de la recta final
const float         PARK_PUNTA_PARED_CM        = 31.0f;  // distancia a mantener de la pared exterior (lectura del sonar)
                                                         // 2026-09-15: 30 -> 35 (orillas949 se enchuecaba hacia la pared) -> 33
                                                         // (orillas950 con 35 se saltó el 1er poste: eco débil más lejos)
const float         PARK_PUNTA_KPOS            = 1.0f;   // grados de rumbo objetivo por cm de error de pared
                                                         // (2026-09-15: 0.8 -> 1.2 -> 1.0: con 1.2 + KP_ANG 3.0 el servo
                                                         // se iba a tope en ambos sentidos, orillas953)
// Integral del error de pared: compensa que el 0 del gyro no quede paralelo a la pared
// (orillas952: tras cuadrar contra la pared quedaron ~4-5° de abertura; con solo P se
// estabilizó en 43 cm pidiendo 33). Grados de rumbo por cm·s de error acumulado.
// Simulado con la respuesta medida en 952: a los 2.5 s 37 cm (antes 42); con abertura 0
// baja a ~30 a los 4 s; hacia la pared (-5°) mínimo ~27 (la zona CERCA sigue mandando).
const float         PARK_PUNTA_KI_POS          = 0.4f;
const float         PARK_PUNTA_I_MAX_DEG       = 6.0f;   // tope de lo que aporta el integral (±grados)
const float         PARK_PUNTA_ANG_MAX_DEG     = 8.0f;   // tope del rumbo objetivo (15 -> 8: orillas953 llegó al 1er
                                                         // poste a +10.8° y el arco relativo entró chueco)
const float         PARK_PUNTA_KP_ANG          = 2.0f;   // servo por grado de error de rumbo
const float         PARK_PUNTA_KP_ANG_SEGUIR   = 2.0f;   // idem SOLO en la fase 0 (seguir pared). 3.0 saturó el
                                                         // servo en orillas953 -> de vuelta a 2.0
const float         PARK_PUNTA_KD_RATE         = 0.3f;   // servo por deg/s de gyroRate (amortigua)
const int           PARK_PUNTA_SERVO_MAX       = 30;     // tope |servo - centro| con servo "recto"
const float         PARK_PUNTA_CERCA_CM        = 26.0f;  // más pegado que esto a la pared -> se aleja más fuerte
const float         PARK_PUNTA_KPOS_CERCA      = 1.5f;   // grados EXTRA por cm por debajo de CERCA_CM (19 cm -> ~-19°)
const float         PARK_PUNTA_ANG_MAX_CERCA_DEG = 12.0f; // tope del rumbo alejándose cuando va pegado (20 -> 12)
// Detección de la bajada (poste magenta)
const int           PARK_PUNTA_CAIDA_CM        = 10;     // bajada vs la base que cuenta como poste (el poste da ~20)
const int           PARK_PUNTA_CAIDA_N         = 2;      // lecturas bajas para confirmar
const int           PARK_PUNTA_CAIDA_HUECOS_N  = 2;      // "sin eco" seguidos tolerados a media bajada sin reiniciarla
const int           PARK_PUNTA_REBASE_N        = 25;     // bajada que dura esto NO es un poste: el carro se acercó -> nueva base
const int           PARK_PUNTA_BASE_N          = 8;      // lecturas de pared antes de armar la detección
const float         PARK_PUNTA_BASE_ALPHA      = 0.2f;   // EMA de la base (sigue derivas lentas, no la bajada)
const int           PARK_PUNTA_PARED_MAX_CM    = 80;     // lecturas > esto = sin pared (no entran a la base)
const int           PARK_PUNTA_SALTO_MAX_CM    = 8;      // lectura > base + esto = eco perdido/rebote, NO la pared: no toca
                                                         // base ni error. orillas942: antes del 1er poste dR dio 200/96/92/94
                                                         // y ~60-70; esas <=80 subieron la base 28->41 y metieron el carro a
                                                         // 15° hacia la pared -> la pared real a 23 pareció "bajada" y giró
const int           PARK_PUNTA_ALTO_REBASE_N   = 15;     // tantas lecturas altas (en rango) sin ninguna normal = el carro SÍ se
                                                         // alejó -> nueva base (los "sin eco" no cuentan ni reinician)
const unsigned long PARK_PUNTA_ARMADO_MS       = 500;    // no detecta antes de esto (deja asentar el giro 12)
const float         PARK_PUNTA_ARMADO_ANG_DEG  = 28.0f;  // ni con el chasis más chueco que esto (> ANG_MAX_CERCA:
                                                         // alejándose de la pared tiene que seguir armada)
const bool          PARK_PUNTA_REQUIERE_PI     = true;   // solo acepta la bajada si la Pi ya vio rosa (park>=1)
// Red de seguridad si nunca hay bajada
const unsigned long PARK_PUNTA_TIMEOUT_MS      = 6000;   // sin bajada en este tiempo -> se detiene donde está.
                                                         // AJÚSTALO: tiene que parar ANTES de salir de la recta de
                                                         // salida (en el regreso, cruzar a la esquina 12 detiene la ronda)
const int           PARK_PUNTA_FRENTE_CM       = 20;     // algo de frente en la fase 0 -> se detiene (inactivo con SOLO_EXTERIOR)
const bool          PARK_PUNTA_SOLO_EXTERIOR   = true;   // buscando el cajón (fase 0): solo el sonar exterior (interior y
                                                         // frontal apagados, sin crosstalk). Al detectar la bajada vuelven
                                                         // los tres como antes. OJO: en la fase 0 ya no frena por
                                                         // FRENTE_CM (usa el frontal), solo por TIMEOUT.
const unsigned long PARK_PUNTA_PING_MIN_MS     = 30;     // fase 0 con un solo sonar: espacio mínimo entre disparos (un HC-SR04
                                                         // disparado muy seguido oye el eco tardío de su pulso anterior)
// Maniobra
const unsigned long PARK_PUNTA_FRENO_MS        = 350;    // coast tras la bajada (el carro se detiene)
const long          PARK_PUNTA_AJUSTE_MS       = 0;      // + avanza recto / - retrocede recto antes del arco (0 = nada)
const int           PARK_PUNTA_AJUSTE_PWM      = 90;
const unsigned long PARK_PUNTA_SERVO_MS        = 200;    // espera a que el servo llegue a tope antes de avanzar
const int           PARK_PUNTA_ARCO_PWM        = 95;
const int           PARK_PUNTA_ARCO_PWM_MIN    = 80;     // arranque de la rampa (evita stall)
const unsigned long PARK_PUNTA_RAMP_MS         = 120;
const int           PARK_PUNTA_ARCO_DEG        = 90;     // giro total buscado
const int           PARK_PUNTA_OVERSHOOT_DEG   = 8;      // corta antes; la inercia completa
// Estacionando tras la media vuelta (retorno): el arco se mide desde el 0 del gyro (paralelo a
// la pared, referenciado al cuadrar en la MANIOBRA 13) y no desde el rumbo que traiga al detectar
// el poste. orillas953: detectó a +10.8°, el arco relativo giró 60° y frenó por dF a -49°.
const bool          PARK_PUNTA_ARCO_ABSOLUTO   = true;
const unsigned long PARK_PUNTA_ARCO_TIMEOUT_MS = 4000;
const unsigned long PARK_PUNTA_ENTRA_MS        = 800;    // tras el arco, recto hacia la pared máx. esto (0 = no entra más)
const int           PARK_PUNTA_ENTRA_PWM       = 85;
const int           PARK_PUNTA_DF_STOP_CM      = 8;      // frontal <= esto -> para (arco o entrada)
const int           PARK_PUNTA_DF_N            = 2;      // lecturas seguidas

// Estado interno de ESTACIONANDO_PUNTA
int           puntaFase         = -1;
unsigned long puntaFaseMs       = 0;
unsigned long puntaEntryMs      = 0;
bool          puntaParedIzq     = false;
float         puntaBase         = 0.0f;  // línea base del sonar exterior (cm)
int           puntaBaseN        = 0;
int           puntaCaidaCnt     = 0;
int           puntaCaidaHuecos  = 0;     // "sin eco" seguidos dentro de la bajada actual
int           puntaInvalidasCnt = 0;
int           puntaAltasCnt     = 0;     // lecturas altas (> base + SALTO_MAX) seguidas sin una normal
long          puntaAltasTot     = 0;     // total de lecturas altas descartadas en la fase 0 (ACK pna=)
int           puntaFrenteCnt    = 0;
int           puntaDfCnt        = 0;
bool          puntaRosaVisto    = false;
float         puntaErrPared     = 0.0f;  // último error de pared válido (congelado durante la bajada)
float         puntaIntPared     = 0.0f;  // integral del error de pared (cm·s), ver PARK_PUNTA_KI_POS
unsigned long puntaIntMs        = 0;
float         puntaRumboRef     = 0.0f;  // heading de la recta al detectar (fase 2)
float         puntaRumboArco0   = 0.0f;  // heading al empezar el arco
bool          puntaRetorno      = false; // estacionando tras la media vuelta (MANIOBRA 13)
float         puntaRumboGiro0   = 0.0f;  // heading al empezar la media vuelta
unsigned long puntaSettleSampMs = 0;
int           puntaSettleQuieto = 0;
long          puntaExtRaw       = 0;     // diag: última lectura cruda del sonar exterior
long          puntaCaidaLectura = 0;     // diag: lectura que confirmó la bajada
unsigned long puntaLogMs        = 0;

// ── Detección de esquinas ─────────────────────────────────────────────────────
int contadorEsquina    = 0;
int contadorForzado    = 0;   // debounce de giroForzado (giro 1, ver FRONT_FORCE_GIRO_CM)
const int umbralPared  = 100;   // cm — pared "desaparece" → esquina
const int esquinaDebounce = 2;  // lecturas consecutivas antes de confiar
const int FRONT_ESQUINA_MAX = 100;  // lateral abierto solo cuenta como esquina si
                                    // además hay pared de frente < esto (anti
                                    // giro-falso por glitch de un lateral).

// Ronda cerrada: al acercarse a la pared de ENFRENTE se baja la velocidad para
// darle tiempo a detectarEsquina() de leer limpio qué lado se abre antes de
// llegar a la esquina (a full 180 el carro se pasaba antes de confirmar).
const int FRONT_SLOWDOWN_CM  = 90;    // pared de frente más cerca que esto -> frena un poco
const int VEL_APROX_CERRADA  = 120;   // velocidad reducida en la aproximación a la esquina

// Giro 1: detectarEsquina espera a ver la pared de enfrente (paredFrente), así
// que confirma tarde y el carro tiende a pasarse. Red de seguridad SOLO en el
// giro 1: a menos de FRONT_SLOWDOWN2_CM de la pared de enfrente, corta a
// VEL_APROX2 (más lento que VEL_INICIAL) para que la esquina tardía no raspe.
const int FRONT_SLOWDOWN2_CM = 60;
const int VEL_APROX2         = 90;

// Giro 1: si detectarEsquina NO disparó y ya estás muy cerca de la pared de
// enfrente -> girá igual, hacia el lado con más hueco (distL vs distR). Cubre
// el caso "vengo despegado de la pared interior y el lateral nunca la perdió
// limpio". El frontal baja gradual y fiable dentro de ~1 m, así que es un
// trigger más estable que el lateral en esa aproximación. Subilo si el arco de
// 90° raspa la pared de enfrente al arrancar desde acá.
const int FRONT_FORCE_GIRO_CM = 37;

// giroForzado sin debounce disparaba con UN solo glitch de crosstalk/multipath
// del frontal (distF < 45 por 1-2 frames aunque la pared real estuviera a más
// de 1 m) -> giro falso en plena recta. 3 frames SEGUIDOS (reset duro, sin
// histéresis) antes de forzar — es una red de seguridad, no la ruta rápida, así
// que puede permitirse ser estricta.
const int FORZADO_DEBOUNCE = 3;

// Wall panic: un lateral crítico (< WALL_PANIC_CM) -> empujón FIJO al centro que
// escala al acercarse, ENCIMA de todo el control (visión, wall PID, gyro-hold).
// Cubre "esquiva hacia una pared sin espacio": la lata está del lado interior, el
// carro se pega a la pared, visión/wall PID no alcanzan y sin esto se clava y
// muere (orillas690 g6). Prefiere rozar la lata a incrustarse en la pared. Solo
// ronda de obstáculos. WALL_PANIC_CM apretado: el carro no debería llegar ahí.
const int   WALL_PANIC_CM   = 18;
const float WALL_PANIC_GAIN = 2.4f;
const int   WALL_PANIC_DEB  = 2;
int contadorPanicL = 0, contadorPanicR = 0;

// Antes del PRIMER giro el carro va lento: la aproximación va más tranquila
// (menos yaw del PID de centrado), el lateral lee limpio y detectarEsquina
// confirma a tiempo. Después del giro 1 -> velocidad normal.
const int VEL_INICIAL = 110;

// ── Carrera ───────────────────────────────────────────────────────────────────
int  turnsCompleted      = 0;
bool raceFinished        = false;
const int TURNS_PER_RACE = 12;
// true  = el carro NUNCA se detiene: al llegar a TURNS_PER_RACE sigue dando vueltas
//         (ni maniobra 13, ni media vuelta, ni estacionamiento, ni TERMINANDO).
//         turnsCompleted sigue subiendo (13, 14, ...) y se ve en tc= del ACK.
// false = corrida normal: giro 13, regreso y estacionamiento como está hoy.
const bool MODO_CONTINUO = false;

// ── Terminando (regreso al área de salida) ───────────────────────────────────
// Al completar la última vuelta el carro NO frena de golpe: entra en TERMINANDO,
// que maneja igual que SIGUIENDO (visión + PID) pero SIN buscar esquinas y solo
// durante TERMINANDO_MS, para meterse en el área de salida y ahí sí frenar.
// Sube/baja este tiempo según la distancia que falte hasta la zona de salida.
const unsigned long TERMINANDO_MS = 1000;
unsigned long terminandoEntryMs   = 0;

// ── Filtro EMA para ultrasonidos ──────────────────────────────────────────────
float alpha         = 0.85;
float distL_filtrada = 0;
float distR_filtrada = 0;
float distF_filtrada = 0;   // sensor frontal (solo ronda de obstáculos)

// Mediana de 3 previa al EMA en los laterales — rechaza picos de 1 frame
// (rasante por yaw / crosstalk) que si no cruzaban umbralPared y disparaban
// un giro falso.
long bufL[3] = {200, 200, 200};
long bufR[3] = {200, 200, 200};
int  idxL = 0;
int  idxR = 0;


// ═══════════════════════════════════════════════════════════════════════════════
// Actuadores
// ═══════════════════════════════════════════════════════════════════════════════

int ultimoServo = 80;   // último ángulo escrito al servo — para debug en el ACK

void escribirServo(int angulo) {
  angulo = constrain(angulo, 0, 180);
  ultimoServo = angulo;
  int pulso = map(angulo, 0, 180, 500, 2500);
  int duty  = (pulso * ((1 << resServo) - 1)) / 20000;
  ledcWrite(SERVO_PIN, duty);
}

// Techo del PWM del motor — DISTINTO por tipo de ronda:
//   ronda de obstáculos (rondaObstaculos=true) : 110 (maniobras lentas y finas)
//   ronda cerrada       (rondaObstaculos=false): 180 (fiuuummmmm)
const int MOTOR_MAX = rondaObstaculos ? 120 : 180;

void setMotor(int velocidad) {
  velocidad = constrain(velocidad, 0, MOTOR_MAX);
  ledcWrite(PWMA, velocidad);
}

// Motor en coast (terminales flotando) — para frenar suave y dejar caer la
// back-EMF antes de invertir la dirección del puente H.
void motorCoast() {
  setMotor(0);
  digitalWrite(A1, LOW);
  digitalWrite(A2, LOW);
}

void motorAdelante() { digitalWrite(A1, HIGH); digitalWrite(A2, LOW); }
void motorReversa()  { digitalWrite(A1, LOW);  digitalWrite(A2, HIGH); }

// Mediana de las últimas 3 lecturas del sensor frontal. El HC-SR04 frontal
// tira picos (55 <-> 199) por multipath / eco perdido; la mediana los rechaza,
// el EMA no. Sin esto un pico espurio disparaba MANIOBRA antes de la esquina.
// 3 (no 5): con 5 el retraso de la mediana metía tarde el slowdown y el carro
// llegaba con poca pista a la esquina.
long medianaFront(long nueva) {
  static long buf[3] = {200, 200, 200};
  static int  idx = 0;
  buf[idx] = nueva;
  idx = (idx + 1) % 3;
  long s[3];
  for (int i = 0; i < 3; i++) s[i] = buf[i];
  for (int i = 0; i < 3; i++)
    for (int j = i + 1; j < 3; j++)
      if (s[j] < s[i]) { long t = s[i]; s[i] = s[j]; s[j] = t; }
  return s[1];
}

long mediana3(long nueva, long buf[3], int &idx) {
  buf[idx] = nueva;
  idx = (idx + 1) % 3;
  long a = buf[0], b = buf[1], c = buf[2];
  return max(min(a, b), min(max(a, b), c));
}

// La MANIOBRA en curso es la 13 (la de la media vuelta para estacionar). Durante
// ella turnsCompleted todavía vale 12: finalizarManiobra() lo incrementa al cerrar.
bool esManiobra13() {
  return !MODO_CONTINUO
         && rondaObstaculos && PARK_ENABLED && PARK_DE_PUNTA && PARK_PUNTA_VUELTA_13
         && turnsCompleted == TURNS_PER_RACE;
}

// Decide dirección de giro (lado con hueco > umbralPared) y FORWARD vs REVERSE
// (según la distancia a la pared EXTERIOR, la que SÍ existe — el "sin pared"
// nunca se usa como número). La llama CRUCERO en el frame del trigger (para
// elegir el umbral frontal) y latchea con maniobraDecidida=true.
void decidirManiobra(long distL, long distR) {
  bool derAbierta = (distR > umbralPared);
  bool izqAbierta = (distL > umbralPared);

  // ── DIRECCIÓN de giro ── se decide en la 1ª esquina por la pared que ABRE y
  //    se LATCHEA (primerGiro): todas las esquinas de la pista son del mismo
  //    sentido, así que no se re-evalúa. Como CRUCERO ya exige paredAbierta para
  //    disparar, aquí SIEMPRE llega con una abierta -> nunca se usa el fallback.
  if (primerGiro) {
    maniobraGirarDer = !direccionIzquierda;                 // ya latcheada
  } else {
    // 1ª esquina: la dirección sale de qué lado abrió. Prioridad al latch de la
    // APROXIMACIÓN (leído cuando el lateral aún daba el hueco real, antes de que
    // el cono de sonido se lockeara en un obstáculo cercano). Si no hay latch,
    // se usa la lectura viva; y por último el fallback histórico.
    if      (direccionAproxLatch == 1) maniobraGirarDer = false;  // abrió IZQUIERDA
    else if (direccionAproxLatch == 2) maniobraGirarDer = true;   // abrió DERECHA
    else if (derAbierta && !izqAbierta) maniobraGirarDer = true;
    else if (izqAbierta && !derAbierta) maniobraGirarDer = false;
    else                               maniobraGirarDer = (distR > distL);  // fallback (no debería pasar)
    primerGiro = true;                                      // latcheado para el resto de la carrera
  }
  direccionIzquierda = !maniobraGirarDer;   // para el dir= del ACK

  // ── FWD vs REVERSE ── SIEMPRE fresco, según la pared EXTERIOR del giro
  //    (giro der -> exterior = izq/distL; giro izq -> exterior = der/distR).
  long distExt = maniobraGirarDer ? distL : distR;
  maniobraDistExt  = distExt;
  maniobraReversa  = (maniobraDistExt >= HUG_CM);

  // ── ¿RETROCEDER un poco DESPUÉS de la maniobra? ── solo si la pared exterior
  //    (la que sigo) tiene holgura: > MANIOBRA_BACKOFF_MIN_CM. Si voy pegado a
  //    ella, retroceder recto no me separa de la recta nueva, solo raspa.
  maniobraRetroceso = (maniobraDistExt > MANIOBRA_BACKOFF_MIN_CM);

  maniobraDecidida = true;
}

void servoRumboPark(float rumboRef) {
  float out = PARK_KP_ANG * (rumboRef - anguloGyro) - PARK_KD_RATE * gyroRate;
  out = constrain(out, (float)-PARK_SERVO_MAX, (float)PARK_SERVO_MAX);
  escribirServo(constrain(centroServo + (int)out, 20, 150));
}

// ── Cierre de ESTACIONANDO (apaga motor, centra servo y finaliza carrera) ─────
void finalizarPark(const char *motivo) {
  motorCoast();
  setMotor(0);
  escribirServo(centroServo);
  parkFase     = 7;
  raceFinished = true;
  Serial.println("==================================================");
  Serial.print  ("  ESTACIONAMIENTO ");
  Serial.print  (PARK_MODO_PARALELO ? "PARALELO (10 pts)" : "DE PUNTA (7 pts)");
  Serial.print  (" TERMINADO: ");
  Serial.println(motivo);
  Serial.print  ("  Heading final vs recta: "); Serial.print(anguloGyro, 2); Serial.println(" deg");
  Serial.print  ("  Base pared="); Serial.print(parkBase, 1);
  Serial.print  ("  Caida lectura="); Serial.println(parkCaidaLectura);
  Serial.print  ("  dL="); Serial.print(distL_filtrada);
  Serial.print  (" dR="); Serial.print(distR_filtrada);
  Serial.print  (" dF="); Serial.println(distF_filtrada);
  Serial.println("  RACE FINISHED -> STOP");
  Serial.println("==================================================");
}

void latchParedCajon(bool retorno) {
  if (PARK_TEST_MANO || PARK_TEST_DIRECTO || PARK_TEST_RECTA_COMPLETA || PARK_TEST_RETORNO) {
    parkParedEsIzquierda = PARK_TEST_PARED_IZQ;
  } else if (cajonParedDetectada) {
    parkParedEsIzquierda = retorno ? !cajonParedEsIzquierda : cajonParedEsIzquierda;
  } else {
    bool ida = !direccionIzquierda;
    parkParedEsIzquierda = retorno ? !ida : ida;
  }
}

void resetParkScan() {
  parkFase             = 0;
  parkFaseMs           = millis();
  parkEntryMs          = millis();
  parkScanSubFase      = 0;
  parkBase             = 0.0f;
  parkBaseN            = 0;
  parkCaidaCnt         = 0;
  parkCaidaHuecos      = 0;
  parkAltasCnt         = 0;
  parkGapCnt           = 0;
  parkInvalidasCnt     = 0;
  parkFrenteCnt        = 0;
  parkDfCnt            = 0;
  parkRosaVisto        = false;
  parkErrPared         = 0.0f;
  parkRumboRef         = 0.0f;
  parkRumboArco0       = 0.0f;
  parkExtRaw           = 0;
  parkCaidaLectura     = 0;
  parkHuecoEntryMs     = 0;
  parkPassExtraMs      = 0;
  parkCenterMode       = 0;
  parkCenterOkCnt      = 0;
  parkBaseLlegada      = 0.0f;
  parkRectoMs          = 0;
  parkAngObjetivo      = (float)PARK_ANG_IN_DEG;
  parkRetryN           = 0;
  parkWiggleMs         = 0;
  puntaIntPared       = 0.0f;
  puntaIntMs          = millis();
  motorAdelante();
  escribirServo(centroServo);
}

// ── Entrada a ESTACIONANDO ───────────────────────────────────────────────────
void iniciarEstacionando() {
  estado       = ESTACIONANDO;
  parkBuscando = false;
  latchParedCajon(false);
  resetParkScan();

  if (PARK_TEST_DIRECTO) {
    motorCoast();
    escribirServo(centroServo);
    parkRumboRef = anguloGyro;
    parkFase     = PARK_MODO_PARALELO ? 2 : 1;
    parkFaseMs   = millis();
    Serial.println("-> TEST DIRECTO: Iniciando en posicion de lanzamiento para reversa");
  } else if (PARK_TEST_MANO) {
    motorCoast();
    setMotor(0);
    escribirServo(centroServo);
    Serial.println("-> TEST A MANO: Motor apagado, empuja el carro para ver deteccion y angulos");
  }

  Serial.println("==================================================");
  Serial.print("-> ESTACIONANDO ");
  Serial.print(PARK_MODO_PARALELO ? "EN PARALELO (10 pts)" : "DE PUNTA (7 pts)");
  Serial.print(": sigo pared ");
  Serial.print(parkParedEsIzquierda ? "IZQUIERDA" : "DERECHA");
  Serial.print(" a "); Serial.print(PARK_PARED_CM, 0); Serial.print(" cm");
  Serial.println(PARK_TEST_MANO ? " (PRUEBA A MANO, motor apagado)" : "");
  Serial.println("==================================================");
}

void iniciarEstacionandoRetorno() {
  estado       = ESTACIONANDO;
  parkBuscando = false;
  latchParedCajon(true);
  parkRumboGiro0   = anguloGyro;
  parkSettleQuieto = 0;
  parkSettleSampMs = millis();
  parkFase         = 20;
  parkFaseMs       = millis();
  motorCoast();
  escribirServo(PARK_RETORNO_AVANCE_MS > 0 ? centroServo : (parkParedEsIzquierda ? 150 : 20));
  Serial.println("==================================================");
  Serial.print("-> MEDIA VUELTA (giro 13), luego paralelo. Pared del regreso: ");
  Serial.println(parkParedEsIzquierda ? "IZQUIERDA" : "DERECHA");
  Serial.println("==================================================");
}

void iniciarParkBuscando() {
  // Sin espera a la Pi: a la fase 0 DIRECTO, sin pasar por SIGUIENDO. Así el
  // escaneo tiene toda la recta (y no se come el cooldownGiro de 3 s del gate de
  // SIGUIENDO, que a ~30 cm/s son ~90 cm). Ver PARK_ESPERA_PI.
  if (!PARK_ESPERA_PI) {
    Serial.println("-> Recta final: ESCANEO DE CAJON YA (sin esperar a la Pi)");
    iniciarEstacionando();
    return;
  }
  parkBuscando        = true;
  parkBuscandoEntryMs = millis();
  estado              = SIGUIENDO;
  Serial.println("-> Recta final: BUSCANDO ESTACIONAMIENTO MAGENTA");
}

// Fase 0 desde cero: base del sonar, timers de armado/timeout y contadores.
void arrancarSeguirPunta() {
  puntaFase         = 0;
  puntaFaseMs       = millis();
  puntaEntryMs      = millis();
  puntaBase         = 0.0f;
  puntaBaseN        = 0;
  puntaCaidaCnt     = 0;
  puntaCaidaHuecos  = 0;
  puntaInvalidasCnt = 0;
  puntaAltasCnt     = 0;
  puntaAltasTot     = 0;
  puntaFrenteCnt    = 0;
  puntaDfCnt        = 0;
  puntaErrPared     = 0.0f;
  puntaIntPared     = 0.0f;
  puntaIntMs        = millis();
  Serial.print("PUNTA fase 0: sigo pared ");
  Serial.print(puntaParedIzq ? "IZQUIERDA" : "DERECHA");
  Serial.print(" a "); Serial.print(PARK_PUNTA_PARED_CM, 0); Serial.println(" cm");
}

// retorno=false: estaciona de frente tras el giro 12 (fase 0 directo).
// retorno=true : tras la MANIOBRA 13, media vuelta (fase 20) y estaciona regresando.
void iniciarEstacionandoPunta(bool retorno) {
  estado         = ESTACIONANDO_PUNTA;
  parkBuscando   = false;
  puntaRosaVisto = false;
  puntaRetorno   = retorno;
  // El lote SIEMPRE está en la pared EXTERIOR de la recta de salida (misma lógica
  // que iniciarEstacionando). Yendo en el sentido de la carrera está del lado
  // contrario al giro; al regresar tras la media vuelta queda del OTRO lado, que
  // es justo hacia donde gira la media vuelta.
  if (PARK_PUNTA_TEST_MANO || PARK_TEST_RECTA_COMPLETA || PARK_TEST_RETORNO) {
    puntaParedIzq = PARK_TEST_PARED_IZQ;
  } else {
    bool paredIzqIda = cajonParedDetectada ? cajonParedEsIzquierda : !direccionIzquierda;
    puntaParedIzq    = retorno ? !paredIzqIda : paredIzqIda;
  }
  Serial.println("==================================================");
  Serial.print("-> ESTACIONANDO DE PUNTA");
  Serial.print(retorno ? " (media vuelta, pared del regreso " : " (de frente, pared ");
  Serial.print(puntaParedIzq ? "IZQUIERDA)" : "DERECHA)");
  Serial.println(PARK_PUNTA_TEST_MANO ? " PRUEBA A MANO, motor apagado" : "");
  Serial.println("==================================================");
  if (retorno) {
    motorCoast();
    // Con avance previo arranca recto; sin él, la fase 20 espera a que llegue a tope.
    escribirServo(PARK_RETORNO_AVANCE_MS > 0 ? centroServo : (puntaParedIzq ? 150 : 20));
    puntaRumboGiro0 = anguloGyro;
    puntaFase       = 20;
    puntaFaseMs     = millis();
  } else {
    motorAdelante();
    escribirServo(centroServo);
    arrancarSeguirPunta();
  }
}

// Servo "recto" hacia un heading (gyro PD) para ESTACIONANDO_PUNTA. Convención
// del proyecto: servo > centro = izquierda; anguloGyro/gyroRate > 0 = CCW (izq).
void servoRumboPunta(float rumboRef, float kpAng = PARK_PUNTA_KP_ANG) {
  float out = kpAng * (rumboRef - anguloGyro) - PARK_PUNTA_KD_RATE * gyroRate;
  out = constrain(out, (float)-PARK_PUNTA_SERVO_MAX, (float)PARK_PUNTA_SERVO_MAX);
  escribirServo(constrain(centroServo + (int)out, 20, 150));
}

void finalizarPunta(const char *motivo) {
  motorCoast();
  setMotor(0);
  escribirServo(centroServo);
  puntaFase    = 6;
  raceFinished = true;
  Serial.println("==================================================");
  Serial.print  ("  ESTACIONANDO DE PUNTA terminado: ");
  Serial.println(motivo);
  Serial.print  ("  ang="); Serial.print(anguloGyro, 1);
  Serial.print  (" girado="); Serial.print(anguloGyro - puntaRumboArco0, 1);
  Serial.print  (" base="); Serial.print(puntaBase, 1);
  Serial.print  (" caida="); Serial.println(puntaCaidaLectura);
  Serial.println("==================================================");
}

// Arranca el regreso al área de salida. Se llama al completar la última vuelta
// EN LUGAR de frenar en seco (raceFinished=true): el carro sigue manejando como
// en SIGUIENDO durante TERMINANDO_MS y después frena (ver terminando()).
void iniciarTerminando() {
  estado            = TERMINANDO;
  terminandoEntryMs = millis();
  Serial.print("-> TERMINANDO ");
  Serial.print(turnsCompleted);
  Serial.print("/");
  Serial.print(TURNS_PER_RACE);
  Serial.print(" (");
  Serial.print(TERMINANDO_MS);
  Serial.println(" ms hacia el area de salida)");
}

// Cierre de MANIOBRA: endereza, deja el puente en adelante, resetea ángulos
// (recta nueva desde 0) y vuelve a SIGUIENDO. Cuenta el giro.
void finalizarManiobra() {
  motorAdelante();
  escribirServo(centroServo);
  setMotor(0);
  velocidadMotor = 180;
  integralWall = 0; prevErrorWall = 0;
  integralGyro = 0; prevErrorGyro = 0;
  // Residual real vs la recta nueva (NO zerar a ciegas): si el pivote sub/sobre-
  // giró, la recuperación termina de cuadrarlo. anguloObjetivo=0 => errorGyro =
  // -residual. Clamp por si una lectura loca. (ver MANIOBRA_RESIDUAL_MAX_DEG)
  if (esManiobra13() && MANIOBRA_13_CUADRAR_MS > 0) {
    // MANIOBRA 13: la fase 4 terminó empujando con la cola plana contra la pared del
    // lote -> el chasis ES perpendicular a ella. Referencia real, no el residual.
    Serial.print("MANIOBRA 13: cuadrado contra la pared, residual descartado=");
    Serial.println(anguloGyro - maniobraIdealRot, 1);
    anguloGyro = 0.0f;
  } else {
    anguloGyro     = constrain(anguloGyro - maniobraIdealRot,
                               -MANIOBRA_RESIDUAL_MAX_DEG, MANIOBRA_RESIDUAL_MAX_DEG);
  }
  // Recta nueva: en vez de apuntar a 0, apunta unos grados hacia la pared
  // EXTERIOR del giro que se acaba de hacer (ver MANIOBRA_BIAS_AFUERA_DEG).
  anguloObjetivo   = maniobraGirarDer ? +MANIOBRA_BIAS_AFUERA_DEG
                                      : -MANIOBRA_BIAS_AFUERA_DEG;
  lastTurnTime     = millis();
  maniobraDecidida = false;
  maniobraFase     = -1;
  estado           = SIGUIENDO;
  turnsCompleted++;
  if (MODO_CONTINUO) {
    // Sin parada: ni estaciona ni termina, sigue corriendo vueltas.
  } else if (turnsCompleted >= TURNS_PER_RACE) {
    if (rondaObstaculos && PARK_ENABLED && PARK_DE_PUNTA) {
      if (!PARK_PUNTA_VUELTA_13) {
        iniciarEstacionandoPunta(false);         // giro 12: estaciona de frente
      } else if (turnsCompleted > TURNS_PER_RACE) {
        if (PARK_MODO_PARALELO) iniciarEstacionandoRetorno();
        else iniciarEstacionandoPunta(true);   // giro 13: media vuelta y estaciona regresando
      }
      // giro 12 con PARK_PUNTA_VUELTA_13: sigue la carrera normal (SIGUIENDO) hasta la esquina 13
    } else if (rondaObstaculos && PARK_ENABLED) {
      iniciarParkBuscando();
    } else {
      iniciarTerminando();
    }
  }
  Serial.print("MANIOBRA completada ");
  Serial.print(turnsCompleted);
  Serial.print("/");
  Serial.println(TURNS_PER_RACE);
}

// Entra a la fase 6 (SETTLE): coast + servo centro, y NO cierra la maniobra
// hasta que el carro deje de rotar (o venza MANIOBRA_SETTLE_TIMEOUT_MS). Se
// llama donde antes se llamaba finalizarManiobra() directo (fin de fase 2 REV
// pegada, y fin de fase 5 tras el retroceso).
void iniciarSettleManiobra() {
  motorCoast();
  escribirServo(centroServo);
  maniobraFase          = 6;
  maniobraSettleMs      = millis();
  maniobraSettleSampMs  = millis();
  maniobraSettleAngPrev = anguloGyro;
  maniobraSettleQuieto  = 0;
}

// Cierre de INICIO: endereza, deja el puente en adelante, resetea integrales y
// entrega el volante a SIGUIENDO. NO zera anguloGyro (la fase 6 ya esperó a que
// el carro dejara de rotar, la lectura es fiable): anguloGyro se puso en 0 al
// INIT con el carro paralelo a la pared exterior = alineado con la recta, así
// que apuntar a anguloObjetivo=0 es correcto y si la contravuelta quedó corta el
// gyro PID de SIGUIENDO termina de cuadrar. NO toca turnsCompleted: salir del
// cajón NO es una de las 12 vueltas.
void finalizarInicio() {
  motorAdelante();
  escribirServo(centroServo);
  setMotor(0);
  velocidadMotor = 180;
  integralWall = 0; prevErrorWall = 0;
  integralGyro = 0; prevErrorGyro = 0;
  anguloObjetivo = 0;
  lastTurnTime   = millis();   // cooldown: sin giro-falso en la zona de salida
  timeStart      = millis();   // el grace de arranque cuenta desde acá
  inicioFase     = -1;
  estado         = SIGUIENDO;
  Serial.print("INICIO completado -> SIGUIENDO  ang=");
  Serial.print(anguloGyro, 1);
  Serial.print(" dirPista=");
  Serial.println(!primerGiro ? "?" : (direccionIzquierda ? "IZQ" : "DER"));
}

// ── Heading-hold en REVERSA (fase 4 de la MANIOBRA) ──────────────────────────
// Retroceder con el servo fijo al centro NO sale recto: la asimetría mecánica
// + el yaw residual del pivote giran el chasis -7±2° "hacia adentro" cada
// maniobra (run 745). Antes se tapaba cortando el retroceso a los 4° (reversa
// cortísima); ahora se cierra el lazo sobre anguloGyro.
//
// OJO: en reversa el servo actúa INVERTIDO respecto a marcha adelante — con el
// servo a la IZQUIERDA (>centro) el chasis rota a la DERECHA. Lo confirman los
// pivotes de la fase 1 ("servo CONTRARIO al giro": maniobraGirarDer -> servo
// 150/izq para cerrar un giro a la derecha). Por eso el término de control va
// con signo NEGADO respecto al PID de gyro de controlPID().
//
// Gains no-const a propósito: se tunean en pista igual que KpGyro/KpWall.
float KpRev = 2.4f;
float KiRev = 0.6f;   // ataca el sesgo sistemático; se resetea al entrar a fase 4
float KdRev = 0.30f;
float integralRev  = 0;
float prevErrorRev = 0;
unsigned long lastRevHoldMs = 0;
const float REV_HOLD_I_CLAMP  = 20.0f;  // tope del integral (grados·s)
const float REV_HOLD_OUT_MAX  = 34.0f;  // tope de |servo - centro| durante la fase 4

// Mantiene anguloGyro en headingRef mientras el carro retrocede recto en fase 4.
// Escribe el servo directo (como escribirServo(centroServo) al que reemplaza).
void aplicarReversaHold(float headingRef) {
  unsigned long now = millis();
  float dt = (now - lastRevHoldMs) / 1000.0f;
  lastRevHoldMs = now;
  if (dt < 0.001f || dt > 0.2f) dt = 0.02f;   // arranque de fase / hueco de loop

  float err   = headingRef - anguloGyro;      // >0 => falta rotar a la IZQ (CCW)
  integralRev = constrain(integralRev + err * dt, -REV_HOLD_I_CLAMP, REV_HOLD_I_CLAMP);
  float deriv = (err - prevErrorRev) / dt;    // headingRef fijo -> sin patada de setpoint
  prevErrorRev = err;

  // Signo NEGADO vs el PID de adelante: en reversa, servo izq -> el chasis va der.
  float out = -(KpRev * err + KiRev * integralRev + KdRev * deriv);
  out = constrain(out, -REV_HOLD_OUT_MAX, REV_HOLD_OUT_MAX);
  escribirServo(constrain(centroServo + (int)out, 20, 150));
}


// ═══════════════════════════════════════════════════════════════════════════════
// Sensores
// ═══════════════════════════════════════════════════════════════════════════════

long leerDistancia(int trig, int echo) {
  digitalWrite(trig, LOW);
  delayMicroseconds(2);
  digitalWrite(trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig, LOW);

  // 6000 us: 100 cm ida+vuelta = 5882 us, cubre todo el rango útil (umbralPared
  // = 100 cm incluido). El caso "sin eco" es justo el de la esquina y quema el
  // timeout completo cada loop -> con 6000 en vez de 7000 el loop respira un
  // poco más rápido cuando más importa.
  long dur  = pulseIn(echo, HIGH, 7000);
  long dist = dur * 0.034 / 2;
  if (dist == 0 || dist > 200) dist = 200;
  return dist;
}

float filtroEMA(float nueva, float anterior) {
  return alpha * nueva + (1.0 - alpha) * anterior;
}

void actualizarGyro() {
  unsigned long now = millis();
  float dt = (now - lastGyroTime) / 1000.0;
  lastGyroTime = now;

  float gz = mpu.getGyroZ() / gyroScale;
  if (abs(gz) < 1.0) gz = 0;
  anguloGyro += gz * dt;
  gyroRate = 0.7f * gyroRate + 0.3f * gz;   // EMA ligera: quita ruido, conserva el latigazo
}

bool detectarEsquina(long distL, long distR, long distF) {
  bool paredFrente = (distF > 0 && distF < FRONT_ESQUINA_MAX);
  bool apertura = paredFrente && ((distL > umbralPared) || (distR > umbralPared));
  // Histéresis en vez de reset duro; satura en esquinaDebounce.
  if (apertura) contadorEsquina = min(contadorEsquina + 1, (int)esquinaDebounce);
  else          contadorEsquina = max(contadorEsquina - 1, 0);
  return contadorEsquina >= esquinaDebounce;
}


// ═══════════════════════════════════════════════════════════════════════════════
// Protocolo serial con Raspberry Pi
// ═══════════════════════════════════════════════════════════════════════════════

void parsePiMessage(String line) {
  line.trim();

  // ── Handshake ──────────────────────────────────────────────────────────────
  if (line.startsWith("READY")) {
    piReadyReceived = true;
    piReady         = true;
    lastPiMsgMs     = millis();
    if (readyMs == 0) readyMs = millis();
    Serial2.println("ACK:READY");
    return;
  }

  // ── Protocolo V2 (Pure Pursuit) y V1 (obstáculo) ─────────────────────────
  // Formato: V2,obs=+0.350,turn=0,state=pp_follow,prio=0,mem=0,pp=1
  //      o:  V1,obs=+0.123,turn=0,state=avoid_red,prio=1,mem=18,pp=0
  //          (ambos se parsean igual — solo difiere el campo pp=)
  if (line.startsWith("V1,") || line.startsWith("V2,")) {

    // obs
    int idx = line.indexOf("obs=");
    if (idx >= 0) {
      int end = line.indexOf(',', idx);
      String s = (end >= 0) ? line.substring(idx + 4, end) : line.substring(idx + 4);
      obsBiasNorm = constrain(s.toFloat(), -1.0, 1.0);
    }

    // turn
    idx = line.indexOf("turn=");
    if (idx >= 0) {
      int end = line.indexOf(',', idx);
      String s = (end >= 0) ? line.substring(idx + 5, end) : line.substring(idx + 5);
      int v = s.toInt();
      turnHint = (v > 0) ? 1 : (v < 0) ? -1 : 0;
    }

    // prio
    idx = line.indexOf("prio=");
    if (idx >= 0) {
      int end = line.indexOf(',', idx);
      String s = (end >= 0) ? line.substring(idx + 5, end) : line.substring(idx + 5);
      piPriority = (s.toInt() != 0);
    }

    // mem
    idx = line.indexOf("mem=");
    if (idx >= 0) {
      int end = line.indexOf(',', idx);
      String s = (end >= 0) ? line.substring(idx + 4, end) : line.substring(idx + 4);
      piMemoryFrames = max(0L, s.toInt());
    }

    // Marca del último frame CON obstáculo activo -> grace post-esquiva de
    // SIGUIENDO->CRUCERO (deja aterrizar el pulso `pasado`).
    if (piPriority || piMemoryFrames > 0) ultimoObstaculoMs = millis();

    // pp  — campo nuevo en V2; ausente en mensajes V1 → pp=false por defecto
    idx = line.indexOf(",pp=");
    if (idx >= 0) {
      String s = line.substring(idx + 4);
      // solo toma el dígito antes de cualquier coma extra
      int end = s.indexOf(',');
      if (end >= 0) s = s.substring(0, end);
      piPurePursuit = (s.toInt() != 0);
    } else {
      piPurePursuit = false;   // mensaje V1 sin campo pp → modo obstáculo
    }

    // pasado — evento de un solo frame: el robot ya atravesó físicamente el
    // obstáculo. Ausente en V1 → false por defecto.
    idx = line.indexOf("pasado=");
    if (idx >= 0) {
      int end = line.indexOf(',', idx);
      String s = (end >= 0) ? line.substring(idx + 7, end) : line.substring(idx + 7);
      piPasado = (s.toInt() != 0);
    } else {
      piPasado = false;
    }

    // intr — el obstáculo actual se pasa por el mismo lado hacia el que va
    // a girar la pista (ver corner_lines.py en la Pi). Ausente en V1 o si
    // la Pi aún no confirmó la dirección de giro → false por defecto
    // (mismo comportamiento de siempre: sigue bloqueando).
    idx = line.indexOf("intr=");
    if (idx >= 0) {
      int end = line.indexOf(',', idx);
      String s = (end >= 0) ? line.substring(idx + 5, end) : line.substring(idx + 5);
      piInteriorPass = (s.toInt() != 0);
    } else {
      piInteriorPass = false;
    }

    // inicio — la Pi vio rosa MAYORITARIO en el frame al arrancar (está en el
    // estacionamiento) -> el ESP32 hace la maniobra de salida (case INICIO)
    // antes de SIGUIENDO. Debe venir ya en el PRIMER V2. Ausente en V1 -> se
    // ignora. SOLO se setea a true (sticky): la Pi solo tiene que afirmarlo una
    // vez; el one-shot de loop() lo consume una sola vez.
    idx = line.indexOf("inicio=");
    if (idx >= 0) {
      int end = line.indexOf(',', idx);
      String s = (end >= 0) ? line.substring(idx + 7, end) : line.substring(idx + 7);
      if (s.toInt() != 0) piInicioEstacionamiento = true;
    }

    // park — etapa del cajón vista por la Pi en la recta final
    idx = line.indexOf("park=");
    if (idx >= 0) {
      int end = line.indexOf(',', idx);
      String s = (end >= 0) ? line.substring(idx + 5, end) : line.substring(idx + 5);
      piPark = s.toInt();
    }
    // pd — distancia estimada (cm, BEV) al bloque magenta más cercano
    idx = line.indexOf("pd=");
    if (idx >= 0) {
      int end = line.indexOf(',', idx);
      String s = (end >= 0) ? line.substring(idx + 3, end) : line.substring(idx + 3);
      piParkDistCm = s.toInt();
    }

    piReady           = true;
    piFirstV2Received  = true;   // desde aquí el carro ya puede rodar
    lastPiMsgMs = millis();
    // Regresamos el heading integrado del gyro para que la Pi pueda mantener
    // su mapa rodante de obstáculos alineado al doblar (obstacle_memory.py).
    Serial2.print("ACK:V2,ang=");
    Serial2.print(anguloGyro, 2);
    Serial2.print(",est=");
    // MANIOBRA -> "G" (la Pi hace su manejo de giro: borra memoria, resetea
    // line_tracker). CRUCERO -> "C" (la Pi lo trata igual que "S"; solo sirve
    // para verlo en el journalctl). RECUPERANDO -> "R". SIGUIENDO -> "S".
    // INICIO -> "I" (la Pi debe quedarse en stand-down).
    // ESTACIONANDO -> "E" (la Pi queda en stand-down mientras el ESP ejecuta la reversa).
    // TERMINANDO -> "T".
    Serial2.print(estado == INICIO ? "I"
                  : (estado == GIRANDO || estado == MANIOBRA) ? "G"
                  : (estado == RECUPERANDO ? "R"
                     : (estado == CRUCERO ? "C"
                        : ((estado == ESTACIONANDO || estado == ESTACIONANDO_PUNTA) ? "E"
                           : (estado == TERMINANDO ? "T" : "S")))));
    // Dirección de giro de la pista: '?' hasta el 1er GIRANDO, luego L/R
    // (direccionIzquierda se fija ahí con distL>distR). La Pi la usa para el
    // manejo de conos exteriores de esquina — fiable de la esquina 2 en
    // adelante (la 1 la sigue estimando por visión). primerGiro arranca en
    // false cada corrida porque el ESP se resetea entre runs.
    Serial2.print(",dir=");
    Serial2.print(!primerGiro ? "?" : (direccionIzquierda ? "L" : "R"));
    // ── DEBUG: estado interno del ESP para verlo en el journalctl de la Pi ──
    // (la Pi loguea el ACK crudo; sus parsers ignoran campos que no conocen).
    //   fase  : maniobraFase  (-1 sin init, 0 frenar-antes, 1 pivote, 2 frenar-desp, 3 frenar-y-fwd)
    //   rev   : maniobraReversa (1 = pivote en reversa)
    //   gd    : maniobraGirarDer (1 = giro a la derecha)
    //   dL/dR : ultrasónicos laterales filtrados (cm)
    //   dF    : ultrasónico frontal filtrado (cm)  — 0 si rondaObstaculos=false
    //   drop  : lateralDropCount (CRUCERO: caídas del lateral vigilado, ver LATERAL_WATCH_CM)
    //   sucio : giroSucioArmado (1 = esquina "sucia" armada, el giro va a disparar sin paredAbierta)
    //   alat  : direccionAproxLatch (0 nada, 1 abrió IZQ, 2 abrió DER — hueco leído en la aproximación)
    Serial2.print(",fase="); Serial2.print(maniobraFase);
    Serial2.print(",rev=");  Serial2.print(maniobraReversa ? 1 : 0);
    Serial2.print(",gd=");   Serial2.print(maniobraGirarDer ? 1 : 0);
    Serial2.print(",dL=");   Serial2.print((long)distL_filtrada);
    Serial2.print(",dR=");   Serial2.print((long)distR_filtrada);
    Serial2.print(",dF=");   Serial2.print((long)distF_filtrada);
    Serial2.print(",cerca="); Serial2.print(cruceroCerca ? 1 : 0);  // CRUCERO: 1 = gyro+wall (sin visión)
    Serial2.print(",drop=");  Serial2.print(lateralDropCount);
    Serial2.print(",sucio="); Serial2.print(giroSucioArmado ? 1 : 0);
    Serial2.print(",alat=");  Serial2.print(direccionAproxLatch);
    // ── DEBUG heading/control (2026-09-07: "heading es mi pata de palo") ──
    //   ao   : anguloObjetivo — el TARGET de heading que persigue el gyro PID
    //   eg   : errorGyro (anguloObjetivo - anguloGyro, capado ±20 en controlPID)
    //   srv  : último ángulo escrito al servo (80 = centro; <80 der, >80 izq)
    //   tc   : turnsCompleted — qué esquina física va (0..12)
    Serial2.print(",ao=");   Serial2.print(anguloObjetivo, 1);
    Serial2.print(",eg=");   Serial2.print(errorGyro, 1);
    Serial2.print(",srv=");  Serial2.print(ultimoServo);
    Serial2.print(",tc=");   Serial2.print(turnsCompleted);
    Serial2.print(",pb=");   Serial2.print(parkBuscando ? 1 : 0);
    //   pnf/pnx/pnb : ESTACIONANDO(_PUNTA) — fase, sonar exterior, línea base (pns = subfase del scan paralelo)
    if (estado == ESTACIONANDO) {
      Serial2.print(",pnf="); Serial2.print(parkFase);
      Serial2.print(",pns="); Serial2.print(parkScanSubFase);
      Serial2.print(",pnx="); Serial2.print(parkExtRaw);   // sonar exterior (mediana 3) que ve la detección
      Serial2.print(",pnb="); Serial2.print((int)parkBase);
      //   pnd/pnt : altura lateral latcheada al llegar al cajón y el tramo recto
      //             (fase 14) que se calculó para compensarla
      Serial2.print(",pnd="); Serial2.print((int)parkBaseLlegada);
      Serial2.print(",pnt="); Serial2.print(parkRectoMs);
      //   pnq : ángulo de entrada calculado para esta llegada (antes fijo en 60)
      Serial2.print(",pnq="); Serial2.print((int)parkAngObjetivo);
    }
    if (estado == ESTACIONANDO_PUNTA) {
      Serial2.print(",pnf="); Serial2.print(puntaFase);
      Serial2.print(",pnx="); Serial2.print(puntaExtRaw);
      Serial2.print(",pnb="); Serial2.print((long)puntaBase);
      Serial2.print(",pna="); Serial2.print(puntaAltasTot);   // lecturas altas descartadas (eco perdido/rebote)
    }
    //   rr   : rerefCount — cuántas veces corrió el re-referenciado de CRUCERO.
    //          Si deja de crecer en una recta, el gate nuevo está bloqueando.
    Serial2.print(",rr=");   Serial2.print(rerefCount);
    Serial2.println();
    return;
  }

  // ── Modo legado: solo píxel X (controlPI.py original) ─────────────────────
  bool numeric = true;
  for (unsigned int i = 0; i < line.length(); i++) {
    char c = line.charAt(i);
    if (!(c >= '0' && c <= '9')) { numeric = false; break; }
  }
  if (numeric && line.length() > 0) {
    int x = line.toInt();
    if (x >= 0 && x <= 640) {
      obsBiasNorm   = constrain((float(x) - 320.0) / 320.0, -1.0, 1.0);
      piPurePursuit = false;
      piReady       = true;
      lastPiMsgMs   = millis();
      Serial2.println("ACK:X");
    }
  }
}

void readPiSerial() {
  while (Serial2.available() > 0) {
    String line = Serial2.readStringUntil('\n');
    if (line.length() > 0) parsePiMessage(line);
  }
}


// ═══════════════════════════════════════════════════════════════════════════════
// PID + control de servo
// ═══════════════════════════════════════════════════════════════════════════════

void controlPID(long distL, long distR) {
  unsigned long now = millis();
  float dt = (now - lastPIDTime) / 1000.0;
  lastPIDTime = now;
  if (dt < 0.01) dt = 0.01;

  // Marca de esquiva activa: la usa el gate del re-referenciado de CRUCERO.
  if (piPriority || piMemoryFrames > 0 || estado == RECUPERANDO) lastDodgeMs = now;

  // ── Siempre calculamos wall y gyro (se usan en fallback y logs) ───────────
  bool wallHold = (!rondaObstaculos && primerGiro);

  // Ventana entre "se perdió la pared que venía siguiendo" y "entra GIRANDO"
  // (~esquinaDebounce frames): la distancia de ese lado ya vale 200 (saturada),
  // así que si dejamos el wall PID trabajando, errorWall se clava en el clamp
  // ±50 y suelta un steerazo (+ golpe de derivada KdWall) ANTES de girar ->
  // GIRANDO arranca desde una pose perturbada. En la ronda abierta congelamos
  // el término de pared en esa ventana; el gyro PID mantiene el rumbo hasta el
  // giro. (En la de obstáculos NO se toca: RECUPERANDO/CRUCERO usan errorWall.)
  bool esquinaInminente = !rondaObstaculos
                          && ((distL > umbralPared) || (distR > umbralPared));

  if (esquinaInminente) {
    errorWall     = 0;
    prevErrorWall = 0;   // sin patada de derivada al congelar
  } else if (wallHold) {
    // Ronda abierta con sentido de giro ya conocido: seguir la pared INTERIOR a
    // WALL_HOLD_CM en vez de centrar. Giro izq -> interior = distL; giro der ->
    // interior = distR. El signo se elige para que quede IGUAL que distL-distR:
    // errorWall > 0 -> el carro vira y distL baja / distR sube.
    //   giro izq (interior=distL): errorWall = distL - HOLD
    //       distL > HOLD (abierto) -> +  -> vira a la interior (izq), distL baja. OK
    //   giro der (interior=distR): errorWall = HOLD - distR
    //       distR > HOLD (abierto) -> -  -> vira a la interior (der), distR baja. OK
    long  distInt = direccionIzquierda ? distL : distR;
    float e       = direccionIzquierda ? (distInt - WALL_HOLD_CM)
                                       : (WALL_HOLD_CM - distInt);
    errorWall = e * WALL_HOLD_GAIN;   // compensa la mitad de ganancia de 1 pared
  } else {
    // Antes del 1er giro (dirección aún desconocida) o ronda de obstáculos:
    // comportamiento de siempre, centrar entre ambas paredes.
    errorWall = distL - distR;
  }
  errorWall = constrain(errorWall, -50, 50);

  // Ronda abierta en hold: el heading de referencia (anguloObjetivo) que dejó el
  // último giro continuo está viciado — el giro se pasa por inercia, así que el
  // gyro PID se pasa la recta "corrigiéndolo" y empuja al carro hacia la pared
  // de AFUERA. Mientras las DOS paredes existan, re-referencia anguloObjetivo
  // poco a poco al heading actual: el gyro pasa a ser solo amortiguador y la
  // pared interior manda. (Mismo truco que usa CRUCERO en la ronda de obstáculos.)
  if (wallHold && distL <= umbralPared && distR <= umbralPared) {
    anguloObjetivo += (anguloGyro - anguloObjetivo) * 0.05f;
  }
  integralWall += errorWall * dt;
  integralWall  = constrain(integralWall, -40, 40);
  float derivWall  = (errorWall - prevErrorWall) / dt;
  float outputWall = KpWall * errorWall + KiWall * integralWall + KdWall * derivWall;
  prevErrorWall = errorWall;

  errorGyro = anguloObjetivo - anguloGyro;
  errorGyro = constrain(errorGyro, -20, 20);
  integralGyro += errorGyro * dt;
  integralGyro  = constrain(integralGyro, -30, 30);
  float derivGyro  = (errorGyro - prevErrorGyro) / dt;
  float outputGyro = KpGyro * errorGyro + KiGyro * integralGyro + KdGyro * derivGyro;
  prevErrorGyro = errorGyro;

  // ── Wall panic (ver consts) — se suma al final en cualquier branch de servo.
  // Convención "centroServo + X": X positivo = izquierda.
  float wallPanic = 0.0;
  if (rondaObstaculos) {
    contadorPanicL = (distL > 0 && distL < WALL_PANIC_CM) ? min(contadorPanicL + 1, WALL_PANIC_DEB) : 0;
    contadorPanicR = (distR > 0 && distR < WALL_PANIC_CM) ? min(contadorPanicR + 1, WALL_PANIC_DEB) : 0;
    // No dejes que wallPanic pelee contra un esquive ACTIVO de la Pi: el lateral
    // crítico del lado hacia el que la Pi vira suele ser el POSTE que rodea, no
    // una pared -> empujarlo al centro tira el carro contra/al lado equivocado
    // del poste (verde de recta 3, run 2026-09-07). El lado opuesto (overshoot
    // contra la pared de enfrente) sí queda protegido.
    bool piEsquivando = piPriority || piMemoryFrames > 0;
    bool dodgeIzq = piEsquivando && obsBiasNorm < -0.15f;   // Pi vira izquierda (p.ej. verde)
    bool dodgeDer = piEsquivando && obsBiasNorm >  0.15f;   // Pi vira derecha  (p.ej. rojo)
    if (contadorPanicR >= WALL_PANIC_DEB && !dodgeDer) wallPanic += (WALL_PANIC_CM - distR) * WALL_PANIC_GAIN;  // cerca DER -> izq
    if (contadorPanicL >= WALL_PANIC_DEB && !dodgeIzq) wallPanic -= (WALL_PANIC_CM - distL) * WALL_PANIC_GAIN;  // cerca IZQ -> der
    wallPanic = constrain(wallPanic, -30.0f, 30.0f);
  }

  // ── Decisión según modo ───────────────────────────────────────────────────
  bool piAlive = (millis() - lastPiMsgMs) <= piTimeoutMs;
  float outputVision = 0.0;
  float outputFinal  = 0.0;

  if (!piAlive) {
    // Pi desconectada → fallback autónomo: paredes + gyro
    piPriority    = false;
    piMemoryFrames = 0;
    piPurePursuit = false;
    turnHint      = 0;
    obsBiasNorm   = 0.0;
    outputFinal   = outputWall + outputGyro;

  } else if (!rondaObstaculos) {
    // ── Ronda ABIERTA (sin obstáculos) ─────────────────────────────────────
    // Se ignora por completo la visión / Pure Pursuit de la Pi: el carro se
    // maneja con wall PID (centra entre paredes) + gyro PID (mantiene el
    // heading hacia anguloObjetivo). Los giros los dispara el propio ESP32 con
    // detectarEsquina(); la Pi solo se usa para el ACK del heading.
    piPurePursuit  = false;
    piPriority     = false;
    piMemoryFrames = 0;
    turnHint       = 0;
    obsBiasNorm    = 0.0;
    // ANTES del 1er giro -> SOLO gyro (heading recto hacia anguloObjetivo=0),
    // SIN wall PID: así el carro no "cazapared" ni se ladea en la aproximación
    // a la esquina 1. Es un test para aislar si la inclinación del centrado es
    // lo que traba/retrasa el giro 1. DESPUÉS del 1er giro -> wall + gyro normal.
    outputFinal    = primerGiro ? (outputWall + outputGyro) : outputGyro;

  } else if (estado == RECUPERANDO
             || (estado == CRUCERO && cruceroCerca)
             || (estado == SIGUIENDO && !piPriority && piMemoryFrames <= 0)) {
    // RECUPERANDO, CRUCERO cerca de la pared, y SIGUIENDO en recta LIMPIA (sin
    // obstáculo mío): el RUMBO lo maneja el gyro hacia anguloObjetivo (NO la
    // visión — el centerline curva hacia la esquina y enchueca el carro). La
    // visión solo maneja el rumbo para esquivar (piPriority/memoria) -> branch
    // piPurePursuit de abajo. En SIGUIENDO se suma un centrado lateral por visión
    // capado (visCorr, más abajo) que NO toca el rumbo.
    // Recalcular error SIN el cap de ±20 usado en controlPID general.
    float errorGyroRecup = anguloObjetivo - anguloGyro;
    errorGyroRecup = constrain(errorGyroRecup, -60, 60);   // más margen real

    float outputRecup = KpGyro * errorGyroRecup + KdGyro * ((errorGyroRecup - prevErrorGyro) / dt);
    prevErrorGyro = errorGyroRecup;
    outputRecup = constrain(outputRecup, -60, 60);   // más rango de servo

    // CRUCERO y SIGUIENDO: además CENTRA en el carril con el wall PID (KiWall=0, así
    // que outputWall es solo P+D, sin integral rancio). PERO solo mientras AMBAS
    // paredes existen; en cuanto una se abre (esquina) errorWall = distL - distR
    // se dispararía y clavaría el servo -> ahí, pura gyro.
    float wallCorr = 0.0;
    if (estado != RECUPERANDO && distL <= umbralPared && distR <= umbralPared) {
      wallCorr = constrain(outputWall * CRUCERO_WALL_BLEND, -25.0f, 25.0f);
      // El heading de referencia post-MANIOBRA está viciado (no gira exacto 90°).
      // Mientras las paredes centran, re-referencia anguloObjetivo hacia el heading
      // ACTUAL poco a poco. SOLO en CRUCERO, SOLO si el chasis ya está casi recto
      // (|anguloGyro| < 12) y con clamp ±12: el offset de maniobra es de pocos
      // grados. Sin ese gate, un CRUCERO largo tras una esquiva dejaba que
      // anguloObjetivo persiguiera un yaw que se iba de mano -> el carro terminaba
      // a −30° y clasificaba el rojo del final de la recta como "beyond" -> lo
      // ignoraba (orillas684). En SIGUIENDO anguloObjetivo ES la recta y queda fijo.
      // 2026-09-12: NO ratificar un chasis que todavía está girando (ver
      // REREF_RATE_MAX_DEG_S). El gate viejo era solo |anguloGyro| < 12, y el
      // latigazo de la esquiva pasa por esa ventana rotando a ~20 °/s.
      bool _yawQuieto      = fabs(gyroRate) < REREF_RATE_MAX_DEG_S;
      bool _lejosDeEsquiva = (now - lastDodgeMs) >= REREF_QUIET_MS;
      if (estado == CRUCERO && fabs(anguloGyro) < 12.0f
          && _yawQuieto && _lejosDeEsquiva) {
        anguloObjetivo += (anguloGyro - anguloObjetivo) * 0.05f;
        anguloObjetivo  = constrain(anguloObjetivo, -MANIOBRA_AO_CLAMP_DEG, MANIOBRA_AO_CLAMP_DEG);
        rerefCount++;
      }
    }

    // Centrado lateral por VISIÓN, SOLO en SIGUIENDO (recta). El centerline centra
    // en el carril y NO depende de los ultrasónicos -> cubre el caso de un lateral
    // muerto (lee 200) que deja al wall PID sin centrar y el carro se va contra la
    // pared tras una esquiva (orillas690, giro 6). En CRUCERO NO: ahí el centerline
    // ya curva hacia la esquina. Capado (±15) para que centre pero no maneje el rumbo.
    float visCorr = 0.0;
    if (estado == SIGUIENDO && piAlive && piPurePursuit) {
      visCorr = constrain(-(obsBiasNorm * ppSteerGain) * 0.5f, -15.0f, 15.0f);
    }

    // Seguir la pared EXTERIOR (ver EXT_WALL_* arriba). Solo cuando wallCorr
    // (ambas paredes) NO actúa: interior ausente. Convención "centroServo + X":
    // X>0 = izquierda. Exterior a la IZQ (giro der): si dL > target (lejos),
    // steer izq (+) para acercarse.
    float extWallCorr = 0.0f;
    if (rondaObstaculos && primerGiro && wallCorr == 0.0f) {
      long distExt = direccionIzquierda ? distR : distL;
      long distIntW = direccionIzquierda ? distL : distR;
      bool intAbierta = (distIntW <= 0 || distIntW > umbralPared);
      if (distExt > 0 && distExt <= umbralPared && intAbierta) {
        float err  = (float)distExt - EXT_WALL_TARGET_CM;   // >0 = lejos del exterior
        float corr = constrain(err * EXT_WALL_GAIN, -EXT_WALL_MAX_DEG, EXT_WALL_MAX_DEG);
        extWallCorr = direccionIzquierda ? -corr : +corr;
      }
    }

    int servoRecup = constrain(centroServo + (int)(outputRecup + wallCorr + visCorr + wallPanic + extWallCorr), 20, 150);
    escribirServo(servoRecup);
    setMotor(velocidadMotor);

    Serial.print(estado == CRUCERO ? " | Mode:CRUCERO"
                 : (estado == SIGUIENDO ? " | Mode:SIG-GYRO" : " | Mode:RECUPERANDO"));
    Serial.print(" | ErrGyro:"); Serial.print(errorGyroRecup);
    Serial.print(" | Wall:");    Serial.print(wallCorr);
    Serial.print(" | Vis:");     Serial.print(visCorr);
    Serial.print(" | Servo:");   Serial.print(servoRecup);
    return;

  } else if (piPurePursuit) {
    // ── Modo Pure Pursuit ────────────────────────────────────────────────────
    // La Pi ya calculó el ángulo de dirección óptimo siguiendo la centerline.
    // obs = steer_deg / ppSteerGain  →  steerDeg = obs * ppSteerGain = steer_deg
    //   steerDeg > 0 = derecha,  steerDeg < 0 = izquierda  (convención de la Pi).
    float steerDeg = obsBiasNorm * ppSteerGain;
    steerDeg = constrain(steerDeg, -ppSteerGain, ppSteerGain);

    // Corrección de heading SOLO en recta limpia (sin obstáculo activo ni en
    // memoria) — cancela la deriva del centerline sin pelear contra PP
    // cuando sí está esquivando algo.
    float headingCorr = 0.0;
    float wallCorr     = 0.0;

    if (!piPriority && piMemoryFrames <= 0) {
        headingCorr = outputGyro * PP_GYRO_BLEND;   // peso bajo, no domina
        wallCorr    = outputWall * PP_WALL_BLEND;
    }

    int servoAngle = centroServo - (int)((steerDeg * ppServoGain) - headingCorr - wallCorr);
    servoAngle = constrain(servoAngle + (int)wallPanic, 20, 150);
    escribirServo(servoAngle);
    setMotor(velocidadMotor);

    // ── Debug UART ────────────────────────────────────────────────────────────
    Serial.print(" | Mode:PP");
    Serial.print(" | Steer:");  Serial.print(steerDeg);
    Serial.print(" | Servo:");  Serial.print(servoAngle);
    return;

  } else {
    // ── Modo V1: obstáculo / fallback PID ────────────────────────────────────
    // Comportamiento idéntico a Controller_PI.ino
    float localGain = visionSteerGain;
    if (piPriority) localGain *= 1.20;
    outputVision = (obsBiasNorm * localGain) + (float(turnHint) * turnHintGain);
    outputFinal  = outputWall + outputGyro + outputVision;
  }

  outputFinal = constrain(outputFinal, -25, 25);
  escribirServo(constrain(centroServo + (int)outputFinal + (int)wallPanic, 20, 150));
  setMotor(velocidadMotor);

  // ── Debug UART ────────────────────────────────────────────────────────────
  Serial.print(" | Mode:");
  Serial.print(!rondaObstaculos ? "ABIERTA"
               : (piPurePursuit ? "PP" : (piAlive ? "V1" : "FALLBACK")));
  Serial.print(" | Wall:");   Serial.print(outputWall);
  Serial.print(" | eWall:");  Serial.print(errorWall);
  Serial.print(esquinaInminente ? "(esq)" : (wallHold ? "(hold)" : "(center)"));
  Serial.print(" | Gyro:");   Serial.print(outputGyro);
  Serial.print(" | Vis:");    Serial.print(outputVision);
  Serial.print(" | Servo:");  Serial.print(centroServo + (int)outputFinal);
}


// ═══════════════════════════════════════════════════════════════════════════════
// Terminando — regreso al área de salida tras la última vuelta
// ═══════════════════════════════════════════════════════════════════════════════
// Mismo control que SIGUIENDO (controlPID: PP/visión + wall/gyro PID) pero SIN
// detectarEsquina() y acotado a TERMINANDO_MS. Al vencer el tiempo frena y marca
// la carrera como terminada (raceFinished) — el loop() ya deja el carro parado.
void terminando(long distL, long distR) {
  velocidadMotor = 180;
  controlPID(distL, distR);

  if (millis() - terminandoEntryMs >= TERMINANDO_MS) {
    raceFinished = true;
    setMotor(0);
    escribirServo(centroServo);
    Serial.println("TERMINANDO completado -> STOP");
  }
}


// ═══════════════════════════════════════════════════════════════════════════════
// Setup
// ═══════════════════════════════════════════════════════════════════════════════

void setup() {
  Serial.begin(115200);
  Serial2.begin(115200, SERIAL_8N1, 17, 16);   // RX=17, TX=16 → Raspberry Pi

  Wire.begin();
  mpu.begin();
  mpu.calcGyroOffsets(true);

  float oz = mpu.getGyroZoffset();
  Serial.print("Offset Z: ");
  Serial.println(oz);
  gyroScale = (abs(oz) > 7.0) ? 2.0 : 1.0;
  Serial.println(gyroScale == 2.0 ? "Offset sucio → /2" : "Offset limpio");

  pinMode(TRIG_L, OUTPUT); pinMode(ECHO_L, INPUT);
  pinMode(TRIG_R, OUTPUT); pinMode(ECHO_R, INPUT);
  pinMode(TRIG_F, OUTPUT); pinMode(ECHO_F, INPUT);

  pinMode(A1, OUTPUT); pinMode(A2, OUTPUT);
  digitalWrite(A1, HIGH); digitalWrite(A2, LOW);

  ledcAttach(PWMA,      freqMotor, resMotor);
  ledcAttach(SERVO_PIN, freqServo, resServo);
  escribirServo(centroServo);
  delay(200);

  distL_filtrada = leerDistancia(TRIG_L, ECHO_L);
  distR_filtrada = leerDistancia(TRIG_R, ECHO_R);
  distF_filtrada = leerDistancia(TRIG_F, ECHO_F);

  lastPIDTime  = millis();
  lastGyroTime = millis();
  lastPiMsgMs  = 0;
  bootStartMs  = millis();
  anguloObjetivo = 0;

  // Esperar READY de la Pi (igual que Controller_PI.ino)
  Serial.println("Esperando READY desde Pi...");
  piReadyReceived = false;
  unsigned long waitStart = millis();
  while (!piReadyReceived) { // && (millis() - waitStart < 30000)) {
    if (Serial2.available()) {
      String line = Serial2.readStringUntil('\n');
      line.trim();
      if (line.indexOf("READY") >= 0) {
        piReadyReceived = true;
        piReady         = true;
        timeStart = millis();
        readyMs   = millis();
        Serial.println("Recibido READY. Esperando primer V2...");
      }
    }
    delay(50);
  }
  if (!piReadyReceived) {
    Serial.println("Timeout Pi — continuando sin señal Pi.");
  }
  Serial.println("Sistema listo (PurePursuit)");
}


// ═══════════════════════════════════════════════════════════════════════════════
// Loop
// ═══════════════════════════════════════════════════════════════════════════════

void loop() {
  readPiSerial();

  // Espera boot si la Pi todavía no respondió
  if (!piReadyReceived) {
    if ((millis() - bootStartMs) < BOOT_WAIT_PI_MS) {
      escribirServo(centroServo);
      setMotor(0);
      Serial.println(" | Estado:WAIT_PI");
      delay(20);
      return;
    }
    // Pasó el timeout de arranque → continuar de todas formas
  }

  // READY llegó PERO todavía no el primer V2 -> NO rodar. Si no, controlPID()
  // cae en el fallback wall-PID y el carro avanza ~0.5 s sin ver el obstáculo.
  // Con la Pi mandando el primer V2 justo tras READY, esto son unos ms; el
  // timeout evita quedar atorado para siempre si la Pi muere tras el READY.
  if (piReadyReceived && !piFirstV2Received
      && (millis() - readyMs) < FIRST_V2_TIMEOUT_MS) {
    escribirServo(centroServo);
    setMotor(0);
    Serial.println(" | Estado:WAIT_FIRST_V2");
    delay(10);
    return;
  }

  if (raceFinished) {
    setMotor(0);
    escribirServo(centroServo);
    Serial.println(" | TERMINADO giros=12/12");
    delay(20);
    return;
  }

  // Arranque REAL del carro (ya pasó WAIT_PI y WAIT_FIRST_V2): re-ancla timeStart
  // aquí para que (millis()-timeStart) mida desde que empieza a rodar, no desde
  // el READY -> el grace de arranque (millis()-timeStart > 500) y el cooldownGiro
  // cuentan desde la marcha real. Contra el giro-falso en la zona de salida queda
  // ese grace + el gate de pared de frente de detectarEsquina() (paredFrente).
  if (!marchaIniciada) {
    marchaIniciada = true;
    timeStart      = millis();
  }

  // One-shot (ya pasados los gates de boot): si es ronda de obstáculos y la Pi
  // vio rosa mayoritario al arrancar (inicio=1 en el V2), el carro NO arranca en
  // SIGUIENDO sino que hace la maniobra de salida del estacionamiento. Se evalúa
  // una sola vez; inicio=0 / ronda abierta -> no cambia nada.
  if (!inicioEvaluado) {
    inicioEvaluado = true;
    // MODO_CONTINUO ignora los arranques de PRUEBA de estacionamiento, pero NO la
    // maniobra de salida del cajón (INICIO): esa es del arranque, no del final.
    if (MODO_CONTINUO) {
      Serial.println("-> MODO CONTINUO: sin estacionamiento ni parada");
    }
    if (!MODO_CONTINUO && PARK_DE_PUNTA && PARK_TEST_RETORNO) {
      turnsCompleted = TURNS_PER_RACE + 1;   // tc=13 -> la Pi busca el rosa
      iniciarEstacionandoPunta(true);
    } else if (!MODO_CONTINUO && PARK_DE_PUNTA && (PARK_PUNTA_TEST_MANO || PARK_TEST_RECTA_COMPLETA)) {
      turnsCompleted = TURNS_PER_RACE;   // tc=12 -> la Pi busca el rosa
      iniciarEstacionandoPunta(false);
    } else if (!MODO_CONTINUO && PARK_TEST_DIRECTO) {
      iniciarEstacionando();
    } else if (!MODO_CONTINUO && PARK_TEST_RECTA_COMPLETA) {
      turnsCompleted = TURNS_PER_RACE;
      iniciarParkBuscando();
    } else if (rondaObstaculos && piInicioEstacionamiento) {
      estado     = INICIO;
      inicioFase = -1;
      Serial.println("-> INICIO (rosa mayoritario: salir del estacionamiento)");
    }
  }

  // PARK_PUNTA_SOLO_EXTERIOR: en la búsqueda del cajón solo el sonar exterior (sin
  // crosstalk del interior ni del frontal). Aplica a ESTACIONANDO_PUNTA fase 0 y al
  // scan paralelo (ESTACIONANDO fase 0). Al pasar a fase >= 1 vuelven los tres.
  bool parkSoloExterior = PARK_PUNTA_SOLO_EXTERIOR && !PARK_PUNTA_TEST_MANO && !PARK_TEST_MANO
                          && ((estado == ESTACIONANDO_PUNTA && puntaFase == 0)
                           || (estado == ESTACIONANDO && PARK_MODO_PARALELO && parkFase == 0));
  bool paredExtIzq = (estado == ESTACIONANDO_PUNTA) ? puntaParedIzq : parkParedEsIzquierda;
  bool leerL = !parkSoloExterior || paredExtIzq;
  bool leerR = !parkSoloExterior || !paredExtIzq;
  bool leerF = !parkSoloExterior;
  if (parkSoloExterior) {
    static unsigned long ultPingPuntaMs = 0;
    unsigned long desdePing = millis() - ultPingPuntaMs;
    if (desdePing < PARK_PUNTA_PING_MIN_MS) delay(PARK_PUNTA_PING_MIN_MS - desdePing);
    ultPingPuntaMs = millis();
  }

  mpu.update();
  actualizarGyro();

  long distL_raw = leerL ? leerDistancia(TRIG_L, ECHO_L) : (long)distL_filtrada;
  long distR_raw = leerR ? leerDistancia(TRIG_R, ECHO_R) : (long)distR_filtrada;
  distL_filtrada = filtroEMA(distL_raw, distL_filtrada);
  distR_filtrada = filtroEMA(distR_raw, distR_filtrada);

  long distL = (long)distL_filtrada;
  long distR = (long)distR_filtrada;

  // Sensor frontal: en ronda de obstáculos alimenta CRUCERO/MANIOBRA; en ronda
  // cerrada sirve para frenar un poco al acercarse a la pared de enfrente
  // (FRONT_SLOWDOWN_CM). Mediana de 5 (rechaza picos) + un EMA suave encima.
  // Apagado = 200 ("nada enfrente") para la lógica; el filtrado conserva su valor.
  long distF_med = leerF ? medianaFront(leerDistancia(TRIG_F, ECHO_F)) : 200;
  if (leerF) distF_filtrada = filtroEMA(distF_med, distF_filtrada);
  long distF = (long)distF_filtrada;

  switch (estado) {

    // ═══════════════════════════════════════════════════════════════════════════
    // INICIO — solo ronda de obstáculos, solo si la Pi vio rosa mayoritario al
    // arrancar (piInicioEstacionamiento). Maniobra pre-programada en "S" para
    // salir del estacionamiento y entregar el carro a SIGUIENDO alineado con la
    // recta (heading ~0) y con la dirección de giro de la pista ya latcheada.
    //
    // Fases (mismos coast obligatorios que MANIOBRA — invertir el puente H con el
    // motor girando frió un TB6612, 2026-09-01):
    //   -1 INIT   : lee dL/dR (mediana de 3, frescas) y latchea el lado de
    //               salida; si la lectura es decisiva, latchea también la
    //               dirección de giro de TODA la pista (el cajón siempre está en
    //               la pared exterior). anguloGyro := 0 = referencia de la recta.
    //    1 SWING  : servo full hacia el lado de salida, avanza (rampa de PWM)
    //               hasta |anguloGyro| >= INICIO_ANG_OUT_DEG - INICIO_OVERSHOOT_DEG.
    //    2 RECTO  : servo centro, avanza de frente INICIO_MID_MS.
    //    3 CONTRA : servo full al lado CONTRARIO, avanza hasta |anguloGyro| <=
    //               INICIO_ENDEREZA_MARGEN_DEG (la inercia lo lleva a ~0).
    //    4 COAST  : motorCoast + servo centro, MANIOBRA_FRENO_MS (antes de reversa).
    //    5 REVERSA: motorReversa + aplicarReversaHold(0) (recto, lazo cerrado)
    //               durante INICIO_REV_MS — colchón por si hay un obstáculo
    //               pegado a la salida del cajón.
    //    6 SETTLE : motorCoast, espera a que deje de rotar, luego finalizarInicio().
    // ═══════════════════════════════════════════════════════════════════════════
    case INICIO: {
      // ── Fase -1: INIT (una vez) ─────────────────────────────────────────────
      if (inicioFase < 0) {
        // Lecturas frescas (mediana de 3): un solo eco perdido devuelve 200 y
        // podría voltear la dirección latcheada para toda la carrera.
        long l1 = leerDistancia(TRIG_L, ECHO_L);
        long l2 = leerDistancia(TRIG_L, ECHO_L);
        long l3 = leerDistancia(TRIG_L, ECHO_L);
        long r1 = leerDistancia(TRIG_R, ECHO_R);
        long r2 = leerDistancia(TRIG_R, ECHO_R);
        long r3 = leerDistancia(TRIG_R, ECHO_R);
        long dL0 = max(min(l1, l2), min(max(l1, l2), l3));   // mediana de 3
        long dR0 = max(min(r1, r2), min(max(r1, r2), r3));

        // El cajón SIEMPRE está sobre la pared exterior. El lateral más corto es
        // esa pared: si es dL, la pared está a la izquierda -> el carro sale
        // girando a la DERECHA (y la pista entera gira a la derecha).
        long dc = dL0 - dR0;                 // <0 => pared a la izq => salir/girar DERECHA
        inicioGirarDer = (dc < 0);

        // Latch de dirección de PISTA para toda la carrera (igual que GIRANDO en
        // la 1ª esquina) — SOLO si la lectura del cajón fue decisiva.
        if (abs(dc) >= INICIO_DIR_MIN_GAP_CM) {
          cajonParedEsIzquierda = (dL0 < dR0);
          cajonParedDetectada   = true;
          direccionIzquierda    = (dL0 > dR0);
          primerGiro            = true;
        }

        anguloGyro     = 0;                  // referencia de la recta / de la maniobra
        anguloObjetivo = 0;
        integralGyro   = 0; prevErrorGyro = 0;
        integralWall   = 0; prevErrorWall = 0;
        inicioFaseMs   = millis();
        inicioFase     = 1;
        motorAdelante();
        Serial.print("INICIO fase 1 SWING salida=");
        Serial.print(inicioGirarDer ? "DER" : "IZQ");
        Serial.print(" dL="); Serial.print(dL0);
        Serial.print(" dR="); Serial.print(dR0);
        Serial.print(" cajonPared=");
        Serial.print(cajonParedEsIzquierda ? "IZQ" : "DER");
        Serial.print(" dirPista=");
        Serial.println(!primerGiro ? "?(ambiguo)" : (direccionIzquierda ? "IZQ" : "DER"));
        break;
      }

      float deltaIni = fabs(anguloGyro);

      // ── Fase 1: SWING — saca la nariz hacia el interior ────────────────────
      if (inicioFase == 1) {
        unsigned long tS = millis() - inicioFaseMs;
        int vel = (tS < INICIO_RAMP_MS)
                  ? (int)map((long)tS, 0, (long)INICIO_RAMP_MS, INICIO_PWM_MIN, INICIO_PWM)
                  : INICIO_PWM;
        motorAdelante();
        escribirServo(inicioGirarDer ? 20 : 150);   // full hacia el lado de salida
        delay(20);
        setMotor(vel);
        bool swingListo   = (deltaIni >= (float)(INICIO_ANG_OUT_DEG - INICIO_OVERSHOOT_DEG));
        bool swingTimeout = (millis() - inicioFaseMs >= INICIO_SWING_TIMEOUT_MS);
        if (swingListo || swingTimeout) {
          escribirServo(centroServo);
          inicioFase   = 2;
          inicioFaseMs = millis();
          if (swingTimeout) Serial.println("INICIO fase 1: timeout de swing");
        }
        break;
      }

      // ── Fase 2: RECTO — avanza de frente un tramo corto ────────────────────
      if (inicioFase == 2) {
        motorAdelante();
        escribirServo(centroServo);
        setMotor(INICIO_PWM);
        if (millis() - inicioFaseMs >= INICIO_MID_MS) {
          inicioFase   = 3;
          inicioFaseMs = millis();
        }
        break;
      }

      // ── Fase 3: CONTRA — contravuelta para re-alinear con la recta ─────────
      if (inicioFase == 3) {
        motorAdelante();
        escribirServo(inicioGirarDer ? 150 : 20);   // full al lado CONTRARIO
        setMotor(INICIO_PWM);
        bool alineado = (deltaIni <= (float)INICIO_ENDEREZA_MARGEN_DEG);
        bool timeout  = (millis() - inicioFaseMs >= INICIO_CONTRA_TIMEOUT_MS);
        if (alineado || timeout) {
          escribirServo(centroServo);
          motorCoast();
          inicioFase   = 4;
          inicioFaseMs = millis();
          if (timeout) Serial.println("INICIO fase 3: timeout de contravuelta");
        }
        break;
      }

      // ── Fase 4: COAST antes de invertir a reversa (protege el TB6612) ──────
      if (inicioFase == 4) {
        motorCoast();
        escribirServo(centroServo);
        if (millis() - inicioFaseMs >= MANIOBRA_FRENO_MS) {
          motorReversa();
          integralRev   = 0; prevErrorRev = 0;
          lastRevHoldMs = millis();
          inicioFase    = 5;
          inicioFaseMs  = millis();
        }
        break;
      }

      // ── Fase 5: REVERSA con heading-hold (retrocede RECTO) ─────────────────
      if (inicioFase == 5) {
        motorReversa();
        aplicarReversaHold(0.0f);   // mantiene anguloGyro en 0 mientras retrocede
        setMotor(INICIO_REV_PWM);
        if (millis() - inicioFaseMs >= INICIO_REV_MS) {
          motorCoast();
          escribirServo(centroServo);
          inicioFase          = 6;
          inicioFaseMs        = millis();
          inicioSettleAngPrev = anguloGyro;
          inicioSettleSampMs  = millis();
          inicioSettleQuieto  = 0;
        }
        break;
      }

      // ── Fase 6: SETTLE — espera a que deje de rotar y cierra ───────────────
      if (inicioFase == 6) {
        motorCoast();
        escribirServo(centroServo);
        unsigned long nowMs = millis();
        if (nowMs - inicioSettleSampMs >= MANIOBRA_SETTLE_SAMPLE_MS) {
          float dtS  = (nowMs - inicioSettleSampMs) / 1000.0f;
          float rate = fabs(anguloGyro - inicioSettleAngPrev) / (dtS > 0.001f ? dtS : 0.001f);
          inicioSettleAngPrev = anguloGyro;
          inicioSettleSampMs  = nowMs;
          if (rate < MANIOBRA_SETTLE_RATE_DPS) inicioSettleQuieto++;
          else                                 inicioSettleQuieto = 0;
        }
        if (inicioSettleQuieto >= MANIOBRA_SETTLE_QUIETO_N
            || nowMs - inicioFaseMs >= MANIOBRA_SETTLE_TIMEOUT_MS) {
          finalizarInicio();
        }
        break;
      }

      break;
    }

    case SIGUIENDO: {
      velocidadMotor = (turnsCompleted == 0) ? VEL_INICIAL : 180;
      if (parkBuscando) velocidadMotor = PARK_APPROACH_PWM;

      // Ronda cerrada: si la pared de ENFRENTE ya está cerca, baja la velocidad
      // en la aproximación para que detectarEsquina() alcance a confirmar qué
      // lado se abre antes de que el carro se pase la esquina.
      if (!rondaObstaculos && distF > 0 && distF < FRONT_SLOWDOWN_CM) {
        velocidadMotor = min(velocidadMotor, VEL_APROX_CERRADA);
      }

      // Giro 1 nada más: ya muy cerca de la pared de enfrente -> crawl, para
      // absorber la detección tardía de detectarEsquina (paredFrente) sin raspar.
      if (!rondaObstaculos && turnsCompleted == 0
          && distF > 0 && distF < FRONT_SLOWDOWN2_CM) {
        velocidadMotor = min(velocidadMotor, VEL_APROX2);
      }

      // La Pi confirma que el robot ya atravesó físicamente el obstáculo
      // (evento de un solo frame) -> entrar a RECUPERANDO. Ya no depende de
      // que la cámara simplemente haya dejado de verlo.
      // En ronda ABIERTA (!rondaObstaculos) no hay obstáculos: se ignora el
      // pulso y el carro sigue en puro wall+gyro PID.
      if (piPasado && rondaObstaculos) {
        estado = RECUPERANDO;
        recuperandoEntryMs = millis();
        headingOkSinceMs   = 0;   // el dwell arranca recién cuando se alcance el heading
        integralWall  = 0; prevErrorWall  = 0;
        integralGyro  = 0; prevErrorGyro  = 0;

        // Consumir el pulso.  piPasado es un evento de UN frame en la Pi, pero
        // en el ESP32 se queda en 1 hasta que llega el siguiente mensaje V2
        // (~70-150 ms) y el loop() corre cientos de veces en ese lapso.  Sin
        // esto, si RECUPERANDO sale rápido (headingOk ya se cumple porque el
        // rebase fue casi recto), SIGUIENDO vuelve a entrar a RECUPERANDO en la
        // iteración siguiente con el MISMO pulso viejo -> rebote de estado y
        // reseteo repetido de integrales / patada en la derivada del servo.
        // Un rebase nuevo real llega en otro mensaje y vuelve a poner piPasado=1.
        piPasado = false;

        controlPID(distL, distR);   // ya toma el branch RECUPERANDO (estado ya cambió)
        break;
      }

      controlPID(distL, distR);

      // No girar si hay obstáculo activo en Pi.  (El gate de heading ya no
      // hace falta aquí — mientras el chasis sigue desalineado, ese trabajo
      // lo hace el estado RECUPERANDO, que ni siquiera llega a evaluar
      // detectarEsquina() porque vive en otro case del switch.)
      // Excepción: si el obstáculo se pasa por el mismo lado hacia el que
      // ya se sabe que va a girar la pista (piInteriorPass), el giro mismo
      // resuelve el paso — no tiene caso seguir bloqueando. piInteriorPass
      // es false por defecto (sin dirección confirmada aún, o exterior),
      // así que sin eso el comportamiento es idéntico al de siempre.
      bool bloqueadoPorObstaculo = (piPriority || (piMemoryFrames > 0)) && !piInteriorPass;

      // 2026-08-28: gate de alineación. En un latiguazo de esquiva el chasis
      // queda ladeado (visto en pista a +37deg) y un ultrasónico lateral lee
      // "sin pared" -> detectarEsquina() disparaba una falsa esquina.
      // 25° (no 15): con 15 el carro no lograba mantenerse recto y NO detectaba
      // la esquina real -> se iba de frente. 25 bloquea el latiguazo (37°) pero
      // permite esquinas reales con algo de error de heading. (Con
      // INTERIOR_PASS_ENABLED=False el bloqueo por obstáculo ya tapa el caso
      // original; esto es red de seguridad.)
      bool chasisAlineado = fabs(anguloGyro) < 25.0f;

      if ((millis() - lastTurnTime > cooldownGiro)
          && millis() - timeStart > 500)
      {
        if (rondaObstaculos) {
          if (parkBuscando) {
            // Buscando el cajón de estacionamiento tras el giro 12:
            // NO se entra a CRUCERO ni a MANIOBRA de esquina.
            // Se activa cuando la Pi confirma que el cajón quedó a nuestro lado (piPark == 2)
            // o por timeout de seguridad si la Pi no vio el magenta.
            bool timeoutPark = (millis() - parkBuscandoEntryMs >= PARK_BUSCANDO_TIMEOUT_MS);
            bool piParkReady = (piPark == 2);
            if (piParkReady || timeoutPark) {
              if (timeoutPark && !piParkReady) {
                Serial.println("-> PARK BUSCANDO: Disparo por TIMEOUT (Pi no vio cajon)");
              } else {
                Serial.println("-> PARK BUSCANDO: Disparo por Pi (cajon alineado)");
              }
              iniciarEstacionando();
            }
          } else if (!piPriority && piMemoryFrames <= 0
              && (millis() - ultimoObstaculoMs > POST_DODGE_CRUCERO_GRACE_MS)
              && distF > 0 && distF < FRONT_CRUCERO_CM) {
            contadorFront++;
            if (contadorFront >= esquinaDebounce) {
              contadorFront   = 0;
              cruceroEntryMs  = millis();
              cruceroCerca    = false;   // fuerza el edge-detect de la 1ª frame de CRUCERO
              lateralWatchActivo = false;
              lateralOpenStreak  = 0;
              lateralDropCount   = 0;
              giroSucioArmado    = false;
              direccionAproxLatch = 0;
              aproxOpenStreakIzq  = 0;
              aproxOpenStreakDer  = 0;
              estado          = CRUCERO;
              Serial.println("-> CRUCERO");
            }
          } else {
            contadorFront = 0;
          }
        } else {
          // Ronda cerrada: giro continuo de siempre.
          bool giroNormal  = !bloqueadoPorObstaculo && detectarEsquina(distL, distR, distF);
          // Red de seguridad SOLO giro 1: detectarEsquina no disparó y ya
          // estás pegado a la pared de enfrente -> girá igual hacia el hueco.
          // Debounce duro: un glitch de 1-2 frames no alcanza a forzar el giro.
          bool forzadoCond = (turnsCompleted == 0 && !primerGiro
                              && distF > 0 && distF < FRONT_FORCE_GIRO_CM);
          contadorForzado  = forzadoCond ? min(contadorForzado + 1, FORZADO_DEBOUNCE) : 0;
          bool giroForzado = (contadorForzado >= FORZADO_DEBOUNCE);

          if (giroNormal || giroForzado) {
            estado     = GIRANDO;
            anguloGyro = 0;
            if (!primerGiro) {
              direccionIzquierda = (distL > distR);
              primerGiro         = true;
            }
            piPurePursuit = false;   // suspender PP durante el giro
            if (giroForzado && !giroNormal)
              Serial.println("Giro 1 FORZADO (pared de enfrente)");
            Serial.println(direccionIzquierda ? "Giro izquierda" : "Giro derecha");
          }
        }
      }
      break;
    }

    case RECUPERANDO: {
      velocidadMotor = 180;
      controlPID(distL, distR);   // toma el branch RECUPERANDO de controlPID()

      // Prioridad absoluta durante la recta final: si la Pi ya confirmó que el
      // cajón quedó a nuestro lado (piPark==2), estacionar de inmediato aunque
      // estemos a media esquiva. Nunca dejar que una esquiva desemboque en otra
      // vuelta cuando el objetivo es estacionar.
      if (parkBuscando && piPark == 2) {
        iniciarEstacionando();
        break;
      }

      bool wallOk    = abs(errorWall) < wallSettleCm;
      bool headingOk = abs(errorGyro) < headingSettleDeg;
      // El dwell arranca cuando el heading YA se alcanzó (no desde que entró a
      // RECUPERANDO). Si se pierde, se re-arma.
      if (headingOk) { if (headingOkSinceMs == 0) headingOkSinceMs = millis(); }
      else           { headingOkSinceMs = 0; }
      bool timedOut  = (millis() - recuperandoEntryMs) > recuperandoTimeoutMs;
      bool dwellOk   = (headingOkSinceMs != 0)
                       && (millis() - headingOkSinceMs > recuperandoMinMs);

      // NO se aborta por piPriority: primero recuperar la recta (heading hacia
      // anguloObjetivo). Un obstáculo visto con el chasis todavía chueco suele ser
      // el que ya se pasó, o uno de la recta siguiente por encima de la esquina —
      // abortar aquí mandaba el carro hacia él. Con el chasis derecho, SIGUIENDO/
      // visión lo maneja bien. timedOut acota por si headingOk no llega (esquina
      // real: un lado lee "sin pared" y errorGyro nunca baja del umbral).
      // Sale cuando llevás recuperandoMinMs SEGUIDOS ya alineado (dwellOk implica
      // headingOk), o cuando vence el timeout (esquina real: un lado lee "sin
      // pared" y errorGyro nunca baja del umbral).
      if (dwellOk || timedOut) {
        // A CRUCERO SOLO si hay pared adelante (distF < FRONT_CRUCERO_CM); sin
        // eso -> SIGUIENDO. Antes bastaba piMemoryFrames<=0: cuando `pasado=1`
        // disparaba a media recta (memoria se limpia, sin pared enfrente) el
        // carro commiteaba a CRUCERO->MANIOBRA y giraba contra la pared
        // (recta 3, run 2026-09-07).
        bool paredAdelante = (distF > 0 && distF < FRONT_CRUCERO_CM);
        // parkBuscando: en la recta final NUNCA se entra a CRUCERO (que
        // compromete otra esquina/vuelta). Se vuelve a SIGUIENDO, donde el
        // chequeo de piPark==2 dispara el estacionamiento.
        if (rondaObstaculos && !piPriority && paredAdelante && !parkBuscando) {
          cruceroEntryMs = millis();
          cruceroCerca   = false;   // fuerza el edge-detect de la 1ª frame de CRUCERO
          lateralWatchActivo = false;
          lateralOpenStreak  = 0;
          lateralDropCount   = 0;
          giroSucioArmado    = false;
          direccionAproxLatch = 0;
          aproxOpenStreakIzq  = 0;
          aproxOpenStreakDer  = 0;
          estado         = CRUCERO;
        } else {
          estado = SIGUIENDO;
        }
      }
      break;
    }

    case GIRANDO: {
      float delta = abs(anguloGyro);

      if (turnsCompleted == 0) {
        velocidadMotor = VEL_INICIAL;   // primera curva: lento todo el arco, sin salto
      } else if (delta < 45) velocidadMotor = 165;
      else if (delta < 70)   velocidadMotor = 145;
      else                   velocidadMotor = 120;

      setMotor(velocidadMotor);
      escribirServo(direccionIzquierda ? 150 : 20);

      if (delta >= AngGiro) {
        escribirServo(centroServo);
        velocidadMotor = 180;

        // Resetear integrales
        integralWall = 0; prevErrorWall = 0;
        integralGyro = 0; prevErrorGyro = 0;

        anguloObjetivo = anguloGyro;
        lastTurnTime   = millis();
        estado         = SIGUIENDO;
        turnsCompleted++;

        if (!MODO_CONTINUO && turnsCompleted >= TURNS_PER_RACE) {
          iniciarTerminando();
        }

        Serial.print("Giro completado ");
        Serial.print(turnsCompleted);
        Serial.print("/");
        Serial.println(TURNS_PER_RACE);
      }
      break;
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // CRUCERO — solo ronda de obstáculos. Recta ya limpia + esquina cerca:
    // va DERECHO por ángulo (mismo control que RECUPERANDO en controlPID) hasta
    // ~FRONT_TURN_CM de la pared, luego MANIOBRA. Si aparece un obstáculo mío,
    // vuelve a SIGUIENDO para esquivarlo.
    // ═══════════════════════════════════════════════════════════════════════════
    case CRUCERO: {
      velocidadMotor = 180;
      if (piPasado) piPasado = false;   // pulso viejo/rezagado: CRUCERO ya mantiene heading
      // Cerca de la pared -> pura gyro+wall (controlPID lo enruta con cruceroCerca).
      // Lejos -> visión (el centerline todavía va recto).
      // TAMBIÉN gyro-hold si el chasis está chueco (> CRUCERO_STRAIGHTEN_DEG): hay
      // que enderezar antes de la esquina sin importar dF.
      bool _cercaAntes    = cruceroCerca;
      bool _cercaPorFrente = (distF > 0 && distF <= CRUCERO_GYRO_CM);
      bool _cercaPorChueco = (fabs(anguloGyro) > CRUCERO_STRAIGHTEN_DEG);
      cruceroCerca = _cercaPorFrente || _cercaPorChueco;
      if (cruceroCerca && !_cercaAntes && _cercaPorFrente
          && fabs(anguloGyro) < CRUCERO_STRAIGHTEN_DEG) {
        // Cortamos visión por el frontal CON el chasis ~recto: adopta el heading
        // actual como referencia (lo que visión dejó, recto) y no vuelvas al
        // anguloObjetivo viejo. Si entramos a gyro-hold por estar CHUECO, NO
        // hacer esto -> ese heading es justo el que hay que corregir.
        // Clamp: sin él este adopt llegó a +14.8° (run 747) -> acumulación.
        anguloObjetivo = constrain(anguloGyro, -MANIOBRA_AO_CLAMP_DEG, MANIOBRA_AO_CLAMP_DEG);
        integralGyro   = 0;
        prevErrorGyro  = 0;
      }
      controlPID(distL, distR);

      // Sale a SIGUIENDO por un obstáculo SOLO si NO hay pared adelante
      // (distF >= FRONT_CRUCERO_CM: la entrada a CRUCERO fue un eco falso, ya
      // pasó). Con pared adelante estás comprometido con la esquina: la lata es
      // de la recta siguiente mal clasificada o ruido, y esquivarla en la boca
      // mata la maniobra (orillas696 g5).
      bool _paredAdelanteCru = (distF > 0 && distF < FRONT_CRUCERO_CM);
      bool _hayLataMia = (piPriority || piMemoryFrames > 0);
      // Salida normal (orillas696 intacto): lata mía y NO hay pared adelante ->
      // la entrada a CRUCERO fue un eco ya pasado, a esquivar.
      bool _saleClaro = _hayLataMia && !_paredAdelanteCru;
      // Salida run-2026-09-07: lata mía CON "pared adelante", pero recién entré a
      // CRUCERO y el chasis está recto -> NO es una esquina en la que esté
      // comprometido (esas se aproximan con tiempo y enderezando); ese dF corto
      // es la lata. Devolver a SIGUIENDO para esquivarla. La ventana + el gate de
      // ángulo dejan el commit de orillas696 (carro ya aproximando, chueco) intacto.
      bool _saleLata  = _hayLataMia && _paredAdelanteCru
                        && (millis() - cruceroEntryMs) < CRUCERO_YIELD_LATA_MS
                        && fabs(anguloGyro) < CRUCERO_YIELD_ANG_DEG;
      if (_saleClaro || _saleLata) {
        estado = SIGUIENDO;             // apareció obstáculo mío -> a esquivarlo
        contadorFront      = 0;
        lateralWatchActivo = false;
        lateralOpenStreak  = 0;
        lateralDropCount   = 0;
        giroSucioArmado    = false;
        direccionAproxLatch = 0;
        aproxOpenStreakIzq  = 0;
        aproxOpenStreakDer  = 0;
        break;
      }

      bool paredAbierta = (distL > umbralPared || distR > umbralPared);

      // ── Vigilancia de "caídas" del lateral (ver comentario junto a
      // LATERAL_WATCH_CM más arriba) ── arranca al entrar a la ventana y ya no
      // se desactiva por jitter de dF; solo se resetea al entrar a CRUCERO o
      // al disparar la maniobra. Vigila el lateral hacia el que YA se sabe que
      // gira la pista (primerGiro latcheado desde la curva 1); si aún no se
      // sabe (curva 1), usa paredAbierta (cualquier lado) como fallback.
      if (!lateralWatchActivo && distF > 0 && distF <= LATERAL_WATCH_CM) {
        lateralWatchActivo = true;
        lateralOpenStreak  = 0;
      }
      if (lateralWatchActivo) {
        // ── Latch de dirección de aproximación ── el PRIMER lado que sostenga
        // apertura APROX_DIR_LATCH_FRAMES frames seguidos. Se lee aquí (dF ~70->50)
        // donde el lateral aún da el hueco real, antes de que el cono de sonido
        // se lockee en un obstáculo cercano de la esquina. No se re-evalúa.
        if (direccionAproxLatch == 0) {
          aproxOpenStreakIzq = (distL > umbralPared) ? aproxOpenStreakIzq + 1 : 0;
          aproxOpenStreakDer = (distR > umbralPared) ? aproxOpenStreakDer + 1 : 0;
          if (primerGiro) {
            // Dirección ya conocida (misma en toda la pista): basta con confirmar
            // que ESTA es una esquina (cualquier lado sostuvo apertura).
            if (aproxOpenStreakIzq >= APROX_DIR_LATCH_FRAMES
                || aproxOpenStreakDer >= APROX_DIR_LATCH_FRAMES)
              direccionAproxLatch = direccionIzquierda ? 1 : 2;
          } else {
            // 1ª esquina: la dirección SÍ sale de qué lado abrió.
            if      (aproxOpenStreakIzq >= APROX_DIR_LATCH_FRAMES) direccionAproxLatch = 1;
            else if (aproxOpenStreakDer >= APROX_DIR_LATCH_FRAMES) direccionAproxLatch = 2;
          }
        }

        bool ladoVigilado = primerGiro
            ? (direccionIzquierda ? (distL > umbralPared) : (distR > umbralPared))
            : paredAbierta;
        if (ladoVigilado) {
          lateralOpenStreak++;
          // Cuenta la caída UNA vez, al confirmarse (no en cada frame que sigue
          // abierto) — LATERAL_OPEN_DEBOUNCE frames seguidos, no 1 solo, porque
          // distL/distR solo tienen EMA (sin mediana como distF) y un eco malo
          // aislado puede cruzar umbralPared en un único frame.
          if (lateralOpenStreak == LATERAL_OPEN_DEBOUNCE) {
            lateralDropCount++;
            if (lateralDropCount >= LATERAL_DROP_MIN) giroSucioArmado = true;
          }
        } else {
          lateralOpenStreak = 0;
        }
      }

      // Preview de la decisión para elegir el UMBRAL frontal: REVERSE dispara ya
      // pegado a la pared (FRONT_TURN_REV_CM), FORWARD necesita espacio para el
      // arco (FRONT_TURN_FWD_CM, ancho). `_de` = distancia a la pared EXTERIOR
      // del giro (la que SÍ existe), igual que la calcula decidirManiobra().
      bool _revPrev;
      {
        long _de;
        if (primerGiro) {
          // Dirección ya latcheada -> la pared exterior es la del lado CONTRARIO
          // al giro (giro izq -> exterior = derecha/distR; giro der -> distL).
          // Su lectura DIRECTA, nunca un min(dL,dR): si el carro llega aplastado
          // contra la pared INTERIOR tras esquivar un cono (dL/dR interior corto)
          // el min tomaba esa interior -> preview FORWARD -> ventana frontal
          // ancha -> MANIOBRA ~35 cm antes de la pared (run 2026-09-09, vuelta 5).
          _de = direccionIzquierda ? distR : distL;
        } else if (direccionAproxLatch == 1) {
          _de = distR;   // abrió IZQ -> giro a la izquierda -> exterior = derecha
        } else if (direccionAproxLatch == 2) {
          _de = distL;   // abrió DER -> giro a la derecha -> exterior = izquierda
        } else {
          // 1ª esquina, dirección aún desconocida -> heurística por qué lado abre.
          bool _da = (distR > umbralPared), _ia = (distL > umbralPared);
          if      (_da && !_ia) _de = distL;
          else if (_ia && !_da) _de = distR;
          else                  _de = ((distR > distL) ? distL : distR);
        }
        _revPrev = (_de >= HUG_CM);
      }
      int _umbralFront = _revPrev ? FRONT_TURN_REV_CM : FRONT_TURN_FWD_CM;

      // Esquina del cajón de estacionamiento (turno 4/8/12): el obstáculo chico
      // + el borde del cajón generan eco errático del lateral entre 60-100cm
      // (a veces el sonido pasa por el hueco, a veces rebota en el borde) --
      // por debajo de 40cm el frontal salió limpio y monotónico en los logs, así
      // que ahí se ignora la lateral por completo y se depende solo del
      // frontal con el umbral REV fijo. Resto de los giros: la maniobra sigue
      // exigiendo pared lateral ABIERTA, o giroSucioArmado como red de
      // seguridad si el cajón tapara el lateral en otra esquina no prevista.
      bool esquinaConCajon = ((turnsCompleted % 4) == 3);
      bool enLaPared;
      int  debounceNecesario;
      if (esquinaConCajon) {
        // Umbral según el preview FWD/REV, igual que las demás esquinas. Antes era
        // FRONT_TURN_REV_CM fijo: orillas948 giro 4 llegó pegado a la exterior
        // (dL=14 -> FORWARD), esperó a dF=17 y el arco de frente se incrustó en la
        // pared (fase 1 sin salida). En la ventana ancha de FORWARD se exige
        // !_hayLataMia (un cono a 50-75 cm no es la pared); REVERSE queda como estaba.
        enLaPared         = (distF > 0 && distF <= _umbralFront
                             && (_revPrev || !_hayLataMia));
        debounceNecesario = CRUCERO_FRONT_DEBOUNCE;
      } else {
        // !_hayLataMia: si la cámara ve una lata `mia`, ese dF corto ES la lata
        // (cono del tramo), no la pared de la esquina -> no dispares el giro por
        // el frontal (run 2026-09-07: 6+ MANIOBRA falsas encadenadas, cada una a
        // dF~25 con un rojo/verde enfrente). El _saleLata de arriba ya la mandó
        // a SIGUIENDO a esquivar; cruceroLargo sigue como red anti-atasco.
        // _laAprox: en la aproximación (dF ~70->50) SÍ vimos el hueco de la
        // esquina abrirse de forma sostenida (APROX_DIR_LATCH_FRAMES) -> es una
        // esquina real aunque AHORA el lateral esté tapado por un obstáculo
        // cercano. Habilita el disparo por el frontal sin exigir paredAbierta en
        // este frame. Sigue exigiendo distF <= _umbralFront y !_hayLataMia.
        bool _laAprox = (direccionAproxLatch != 0);
        // PEGADO A LA PARED DE FRENTE: gira aunque el lateral NUNCA confirme la
        // apertura de la esquina. Pasa cuando el carro llega aplastado contra la
        // pared INTERIOR tras esquivar (el lateral interior no se despega ->
        // paredAbierta/_laAprox/giroSucioArmado nunca se arman) -> antes el carro
        // se metía de frente hasta la pared y solo lo sacaba cruceroLargo 7 s
        // después, ya incrustado (run 2026-09-09 giro 2, dF=2). Mismo criterio
        // que la rama del cajón: frontal <= REV_CM, debounce CRUCERO_FRONT.
        // !_hayLataMia sigue: si es una lata, _saleLata ya la mandó a esquivar.
        bool _muyCerca = (distF > 0 && distF <= FRONT_TURN_REV_CM);
        enLaPared         = (distF > 0 && distF <= _umbralFront
                             && (paredAbierta || giroSucioArmado || _laAprox || _muyCerca)
                             && !_hayLataMia);
        debounceNecesario = (paredAbierta || _laAprox) ? CRUCERO_PARED_DEBOUNCE
                                                       : CRUCERO_FRONT_DEBOUNCE;
      }
      bool cruceroLargo = (millis() - cruceroEntryMs) > CRUCERO_TIMEOUT_MS;  // red de seguridad
      // gapOk: dos MANIOBRA reales nunca caen < MANIOBRA_MIN_GAP_MS (la maniobra
      // + aproximación ya tarda varios s). Si el trigger por frontal quiere
      // disparar antes, es fantasma (run 2026-09-07). NO gatea cruceroLargo.
      bool gapOk = (millis() - lastTurnTime) > MANIOBRA_MIN_GAP_MS;

      if (enLaPared) contadorFront++;
      else           contadorFront = 0;

      if ((contadorFront >= debounceNecesario && gapOk) || cruceroLargo) {
        bool _fueSucio = giroSucioArmado;
        contadorFront      = 0;
        lateralWatchActivo = false;
        lateralOpenStreak  = 0;
        lateralDropCount   = 0;
        giroSucioArmado    = false;
        decidirManiobra(distL, distR);   // decisión DEFINITIVA, latcheada (usa direccionAproxLatch)
        direccionAproxLatch = 0;         // consumido; la próxima esquina lo re-latchea
        aproxOpenStreakIzq  = 0;
        aproxOpenStreakDer  = 0;
        maniobraFase  = -1;              // MANIOBRA hará el phase-init
        piPurePursuit = false;
        estado        = MANIOBRA;
        Serial.print("-> MANIOBRA dir="); Serial.print(maniobraGirarDer ? "DER" : "IZQ");
        Serial.print(maniobraReversa ? " REVERSA" : " FORWARD");
        Serial.print(" distExt="); Serial.print(maniobraDistExt);
        Serial.print(" distF=");   Serial.print(distF);
        Serial.print(_fueSucio ? " SUCIO" : "");
        Serial.println(cruceroLargo ? " TIMEOUT" : "");
      }
      break;
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // MANIOBRA — solo ronda de obstáculos. Reemplaza al giro continuo.
    //   decide (1 vez): dirección = lado con hueco (>umbralPared). FORWARD vs
    //   REVERSE según la distancia a la pared EXTERIOR (la que SÍ existe).
    //   Máquina de fases (los coast evitan freír el puente H por "plugging",
    //   reventó un TB6612 así 2026-09-01):
    //     0 COAST         : (solo REV) frena — venía de frente de CRUCERO —
    //                       MANIOBRA_FRENO_MS, luego arranca reversa -> fase 1
    //     1 PIVOTE        : servo (contrario si REV) + motor, con rampa, hasta EXIT_DEG
    //                       (REV atascada -> fase 3: frena y termina de frente)
    //     2 COAST-FIN     : frena tras el pivote -> fase 4.
    //     4 RETROCESO-POST: motor en reversa — MANIOBRA_BACKOFF_MS si venía REV
    //                       con holgura (maniobraRetroceso), MANIOBRA_BACKOFF_FWD_MS
    //                       si FWD. -> fase 5
    //     5 COAST         : frena tras el retroceso, luego cierra
    //     3 FRENAR-Y-FWD  : coast, luego re-arranca el pivote de frente
    //   Fin: endereza, resetea (recta nueva desde 0), turnsCompleted++, SIGUIENDO.
    // ═══════════════════════════════════════════════════════════════════════════
    case MANIOBRA: {
      if (!maniobraDecidida) decidirManiobra(distL, distR);   // safety (normalmente CRUCERO ya decidió)

      if (maniobraFase < 0) {   // phase-init (una vez por maniobra)
        // Clamp: en la aproximación a la esquina el wall-panic (dR~2-6, servo a
        // tope) mete un pico de +6-10° en anguloGyro en 2-3 frames, y fase-0 lo
        // captura como si fuera la inclinación real (run 748: chasis venía a ~+8,
        // capturó +18.7). Eso envenena maniobraIdealRot -> el residual sale ~10°
        // mal -> finalizarManiobra deja el heading mal -> la recta siguiente
        // arranca chueca -> pico más grande la próxima -> muere en ~8 vueltas.
        // El chasis real entrando a la esquina no pasa de ~±8 (mantuvo eso toda
        // la recta). Un pico transitorio se recorta; una entrada REAL chueca se
        // recorta también pero ahí el residual + RECUPERANDO terminan de cuadrar.
        maniobraInclinacionEntrada = constrain(anguloGyro,
                                     -MANIOBRA_INCL_ENTRADA_MAX_DEG,
                                      MANIOBRA_INCL_ENTRADA_MAX_DEG);
        // Rotación ideal (con signo) para cuadrar con la recta nueva = 90° menos
        // lo que ya venías inclinado hacia el giro. Se usa AngGiro (90), NO el
        // ANG_GIRO_MANIOBRA_FWD (84): ese 84 es un ajuste de inercia para el
        // TIMING del pivote fwd, no la geometría real de la recta.
        {
          float _sg = maniobraGirarDer ? -1.0f : 1.0f;
          maniobraIdealRot = _sg * constrain(AngGiro - maniobraInclinacionEntrada * _sg, 70.0f, 110.0f);
        }
        // REV -> coast (0) -> pivote en reversa (1)
        // FWD -> pivote de frente (1) directo (mismo sentido que CRUCERO, sin coast)
        if (maniobraReversa) {
          maniobraFase   = 0;
          maniobraFaseMs = millis();
          motorCoast();
        } else {
          motorAdelante();
          anguloGyro       = 0;
          maniobraPivoteMs = millis();
          maniobraFase     = 1;
        }
      }

      float delta = abs(anguloGyro);
      // Ángulo objetivo ajustado por la inclinación de entrada: derecha=negativo,
      // izquierda=positivo (empírico). Ya inclinado hacia el giro -> gira menos.
      float signoGiro = maniobraGirarDer ? -1.0f : 1.0f;
      int   angBaseManiobra = maniobraReversa ? AngGiro : ANG_GIRO_MANIOBRA_FWD;   // fwd gira menos (más inercia)
      float anguloObjetivoManiobra = constrain(angBaseManiobra - maniobraInclinacionEntrada * signoGiro, 70.0f, 110.0f);
      const float EXIT_DEG = anguloObjetivoManiobra - MANIOBRA_OVERSHOOT_DEG;   // sale antes: la inercia completa

      // ── Fase 0: FRENAR (el motor viene de frente de CRUCERO) antes de invertir ─
      if (maniobraFase == 0) {
        motorCoast();
        escribirServo(centroServo);
        if (millis() - maniobraFaseMs >= MANIOBRA_FRENO_MS) {
          motorReversa();                 // motor parado -> arranca en reversa
          anguloGyro       = 0;
          maniobraPivoteMs = millis();
          maniobraFase     = 1;           // pivote en reversa
        }
        break;
      }

      // ── Fase 1: PIVOTE ───────────────────────────────────────────────────
      if (maniobraFase == 1) {
        // reversa atascada -> frenar y terminar de frente
        if (maniobraReversa && delta < EXIT_DEG
            && (millis() - maniobraPivoteMs) > MANIOBRA_REV_TIMEOUT_MS) {
          maniobraReversa = false;
          maniobraFase    = 3;
          maniobraFaseMs  = millis();
          motorCoast();
          Serial.println("MANIOBRA: reversa timeout -> freno -> forward");
          break;
        }

        if (maniobraReversa) {
          unsigned long tR = millis() - maniobraPivoteMs;
          int vel = (tR < MANIOBRA_RAMP_MS)
                    ? (int)map((long)tR, 0, (long)MANIOBRA_RAMP_MS,
                               MANIOBRA_VEL_MIN, MANIOBRA_VEL_REV)
                    : MANIOBRA_VEL_REV;
          if (delta > EXIT_DEG - 20) vel = min(vel, MANIOBRA_VEL_MIN);   // frena el último tramo
          motorReversa();
          escribirServo(maniobraGirarDer ? 150 : 20);   // servo CONTRARIO al giro
          setMotor(vel);
        } else {
          if      (delta < 45)            velocidadMotor = 165;
          else if (delta < EXIT_DEG - 20) velocidadMotor = 145;
          else                            velocidadMotor = 100;   // último tramo: crawl
          motorAdelante();
          escribirServo(maniobraGirarDer ? 20 : 150);    // servo hacia el giro
          setMotor(velocidadMotor);
        }

        if (delta >= EXIT_DEG) {
          maniobraFase   = 2;
          maniobraFaseMs = millis();
          motorCoast();
          escribirServo(centroServo);
        }
        break;
      }

      // ── Fase 2: FRENAR tras el pivote ───────────────────────────────────
      //   REV pegado (sin holgura) -> cierra. Resto (holgura o FWD) -> fase 4.
      //   MANIOBRA 13 -> fase 4 siempre (retrocede hasta la pared del lote).
      if (maniobraFase == 2) {
        motorCoast();
        escribirServo(centroServo);
        if (millis() - maniobraFaseMs >= MANIOBRA_FRENO_MS) {
          if (maniobraRetroceso || !maniobraReversa || esManiobra13()) {
            motorReversa();               // motor parado -> arranca en reversa
            maniobraFaseMs      = millis();
            maniobraFase4AngIni = anguloGyro;   // referencia (setpoint) del heading-hold de reversa
            integralRev   = 0;                  // PID de reversa limpio para esta fase 4
            prevErrorRev  = 0;
            lastRevHoldMs = millis();
            maniobraFase        = 4;
            if (esManiobra13()) {
              Serial.print("MANIOBRA 13: retroceso a la pared del lote distExt=");
              Serial.println(maniobraDistExt);
            }
          } else {
            iniciarSettleManiobra();      // esperar a que deje de rotar -> cierra
          }
        }
        break;
      }

      // ── Fase 4: RETROCESO-POST — toma distancia de la recta nueva ─────────
      //   Retrocede recto con heading-hold de lazo cerrado (aplicarReversaHold):
      //   el servo se corrige para mantener maniobraFase4AngIni en vez de quedar
      //   fijo al centro -> ya NO acumula el giro "hacia adentro". Sale solo por
      //   tiempo (backoffMs); el PID mantiene el rumbo, así que puede reversear
      //   largo y estable sin cortarse por yaw.
      if (maniobraFase == 4) {
        motorReversa();
        bool m13 = esManiobra13();
        unsigned long tF4 = millis() - maniobraFaseMs;
        if (m13 && MANIOBRA_13_CUADRAR_MS > 0 && tF4 >= MANIOBRA_BACKOFF_13_MS) {
          escribirServo(centroServo);              // cuadrar: empuja con la cola plana contra la pared del lote
        } else {
          aplicarReversaHold(maniobraFase4AngIni); // lazo cerrado: reversa RECTA (antes: servo al centro)
        }
        setMotor(m13 ? MANIOBRA_BACKOFF_13_VEL : MANIOBRA_BACKOFF_VEL);
        unsigned long backoffMs;
        if      (m13)                                          backoffMs = MANIOBRA_BACKOFF_13_MS + MANIOBRA_13_CUADRAR_MS;
        else if (!maniobraReversa)                             backoffMs = MANIOBRA_BACKOFF_FWD_MS;
        else if (maniobraDistExt > MANIOBRA_BACKOFF_FAR_CM)    backoffMs = MANIOBRA_BACKOFF_FAR_MS;
        else                                                  backoffMs = MANIOBRA_BACKOFF_MS;
        if (millis() - maniobraFaseMs >= backoffMs) {
          motorCoast();
          maniobraFaseMs = millis();
          maniobraFase   = 5;
        }
        break;
      }

      // ── Fase 5: FRENAR tras el retroceso, luego SETTLE ──────────────────
      if (maniobraFase == 5) {
        motorCoast();
        escribirServo(centroServo);
        if (millis() - maniobraFaseMs >= MANIOBRA_FRENO_MS) iniciarSettleManiobra();
        break;
      }

      // ── Fase 6: SETTLE — espera a que el carro DEJE de rotar antes de cerrar ─
      //   El pivote (sobre todo REV + backoff) entrega con velocidad angular:
      //   finalizarManiobra() zeraba anguloGyro con el carro girando -> la
      //   recuperación arrancaba desde una mentira y sobre-corregía 40-100°
      //   (runs 716/718/719, el carro terminaba encajado contra una pared).
      //   Coast + servo centro; se cierra recién cuando |ΔanguloGyro/Δt| baja de
      //   MANIOBRA_SETTLE_RATE_DPS por MANIOBRA_SETTLE_QUIETO_N samples, o al
      //   vencer MANIOBRA_SETTLE_TIMEOUT_MS.
      if (maniobraFase == 6) {
        motorCoast();
        escribirServo(centroServo);
        unsigned long nowMs = millis();
        if (nowMs - maniobraSettleSampMs >= MANIOBRA_SETTLE_SAMPLE_MS) {
          float dtS  = (nowMs - maniobraSettleSampMs) / 1000.0f;
          float rate = fabs(anguloGyro - maniobraSettleAngPrev) / (dtS > 0.001f ? dtS : 0.001f);
          maniobraSettleAngPrev = anguloGyro;
          maniobraSettleSampMs  = nowMs;
          if (rate < MANIOBRA_SETTLE_RATE_DPS) maniobraSettleQuieto++;
          else                                 maniobraSettleQuieto = 0;
        }
        if (maniobraSettleQuieto >= MANIOBRA_SETTLE_QUIETO_N
            || nowMs - maniobraSettleMs >= MANIOBRA_SETTLE_TIMEOUT_MS) {
          finalizarManiobra();
        }
        break;
      }

      // ── Fase 3: FRENAR tras timeout de reversa, luego pivote de frente ───
      if (maniobraFase == 3) {
        motorCoast();
        escribirServo(centroServo);
        if (millis() - maniobraFaseMs >= MANIOBRA_FRENO_MS) {
          motorAdelante();
          maniobraPivoteMs = millis();
          maniobraFase     = 1;          // vuelve al pivote, ahora de frente
        }
        break;
      }
      break;
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // TERMINANDO — tras la última vuelta: maneja como SIGUIENDO (sin esquinas)
    // durante TERMINANDO_MS para entrar al área de salida, luego frena.
    // ═══════════════════════════════════════════════════════════════════════════
    case TERMINANDO: {
      terminando(distL, distR);
      break;
    }

    // ═══════════════════════════════════════════════════════════════════════════════
    // ESTACIONANDO — Estacionamiento en paralelo en reversa (Obstacle Challenge)
    // ═══════════════════════════════════════════════════════════════════════════
    case ESTACIONANDO: {
      // Sónar lateral del cajón CRUDO, igual que ESTACIONANDO_PUNTA (2026-09-15:
      // se quitó la mediana de 3). Los picos de 55-106 cm por multipath ya no se
      // cuelan: la clasificación de la fase 0 los marca como "lectura alta"
      // (PARK_PUNTA_SALTO_MAX_CM) y no tocan base ni error, y tanto la bajada
      // (PARK_CAIDA_N) como el hueco (PARK_GAP_N) piden 2 lecturas seguidas.
      // La mediana además retrasaba un frame el flanco del poste.
      long  extRaw          = parkParedEsIzquierda ? distL_raw : distR_raw;
      float haciaPared      = parkParedEsIzquierda ? 1.0f : -1.0f;  // >0 hacia la pared (izq = +deg)
      int   servoHaciaPared = parkParedEsIzquierda ? 150 : 20;      // tope hacia la pared exterior
      int   servoDesdePared = parkParedEsIzquierda ? 20 : 150;      // contravuelta hacia el interior
      parkExtRaw = extRaw;
      if (piPark >= 1) parkRosaVisto = true;

      // ── Fase 20: MEDIA VUELTA — avance recto, luego 90° hacia la pared del regreso
      if (parkFase == 20) {
        unsigned long t = millis() - parkFaseMs;
        unsigned long tArranque = (PARK_RETORNO_AVANCE_MS > 0) ? 0UL : 200UL;
        unsigned long tMov = (t > tArranque) ? (t - tArranque) : 0UL;
        int vel = (tMov < PARK_RETORNO_RAMP_MS)
                  ? (int)map((long)tMov, 0, (long)PARK_RETORNO_RAMP_MS, PARK_RETORNO_PWM_MIN, PARK_RETORNO_PWM)
                  : PARK_RETORNO_PWM;
        if (t < PARK_RETORNO_AVANCE_MS) {
          motorAdelante();
          servoRumboPunta(parkRumboGiro0);
          setMotor(vel);
          break;
        }
        escribirServo(servoHaciaPared);
        if (t < tArranque) {
          motorCoast();
          break;
        }
        motorAdelante();
        setMotor(vel);
        float girado = (anguloGyro - parkRumboGiro0) * haciaPared;
        bool  listo  = girado >= (float)(90 - PARK_RETORNO_OVERSHOOT_DEG);
        bool  tout   = tMov >= PARK_RETORNO_AVANCE_MS + PARK_RETORNO_TIMEOUT_MS;
        if (listo || tout) {
          motorCoast();
          escribirServo(centroServo);
          parkFase         = 21;
          parkFaseMs       = millis();
          parkSettleSampMs = millis();
          parkSettleQuieto = 0;
          Serial.print("PARK fase 21: media vuelta girado="); Serial.print(girado, 1);
          Serial.println(tout ? " TIMEOUT" : "");
        }
        break;
      }

      if (parkFase == 21) {
        motorCoast();
        escribirServo(centroServo);
        unsigned long nowMs = millis();
        if (nowMs - parkSettleSampMs >= MANIOBRA_SETTLE_SAMPLE_MS) {
          parkSettleSampMs = nowMs;
          parkSettleQuieto = (fabs(gyroRate) < MANIOBRA_SETTLE_RATE_DPS) ? parkSettleQuieto + 1 : 0;
        }
        unsigned long tS = nowMs - parkFaseMs;
        bool quieto = (parkSettleQuieto >= MANIOBRA_SETTLE_QUIETO_N) && (tS >= MANIOBRA_FRENO_MS);
        if (quieto || tS >= PARK_RETORNO_SETTLE_MAX_MS) {
          anguloGyro -= haciaPared * 90.0f;
          Serial.print("PARK: rumbo de regreso ang="); Serial.println(anguloGyro, 1);
          unsigned long revMs = parkParedEsIzquierda ? PARK_RETORNO_REV_CCW_MS : PARK_RETORNO_REV_CW_MS;
          if (revMs > 0) {
            motorReversa();
            integralRev   = 0; prevErrorRev = 0;
            lastRevHoldMs = millis();
            parkFase      = 22;
            parkFaseMs    = millis();
          } else {
            Serial.println("PARK: busco cajon en el regreso (paralelo)");
            resetParkScan();
          }
        }
        break;
      }

      if (parkFase == 22) {
        unsigned long revMs = parkParedEsIzquierda ? PARK_RETORNO_REV_CCW_MS : PARK_RETORNO_REV_CW_MS;
        motorReversa();
        aplicarReversaHold(0.0f);
        setMotor(PARK_RETORNO_REV_PWM);
        if (millis() - parkFaseMs >= revMs) {
          motorCoast();
          escribirServo(centroServo);
          parkFase   = 23;
          parkFaseMs = millis();
        }
        break;
      }

      if (parkFase == 23) {
        motorCoast();
        escribirServo(centroServo);
        if (millis() - parkFaseMs >= MANIOBRA_FRENO_MS) {
          Serial.println("PARK: busco cajon en el regreso (paralelo)");
          resetParkScan();
        }
        break;
      }

      // ── Fase 0: SEGUIR pared exterior + ESCANEAR cajón ─────────────────────
      if (parkFase == 0) {
        unsigned long tEn = millis() - parkEntryMs;
        bool lecturaEnRango = (extRaw > 2 && extRaw <= PARK_PARED_MAX_CM);
        bool baseLista      = (parkBaseN >= PARK_BASE_N);

        // Clasificación de la lectura IGUAL que ESTACIONANDO_PUNTA fase 0 (la de
        // ParkinHalf que agarra el 1er poste). orillas959: el carro iba a ~26 cm de
        // la pared, el 1er poste dio 7 y 5 (< el viejo PARK_CAIDA_MIN_CM=8) -> no
        // contó como bajada y además esas lecturas entraron a la base (26 -> 14);
        // el 2o poste (12, 11) se tomó como POSTE 1 y buscó un poste 2 que ya había
        // pasado hasta el TIMEOUT. extRaw ya es mediana de 3: un pico suelto no llega.
        //
        // Lectura muy por ENCIMA de la base: eco perdido o rebote, no la pared -> no
        // mueve base ni error (PARK_PUNTA_SALTO_MAX_CM, orillas942).
        bool lecturaAlta = baseLista && lecturaEnRango
                           && extRaw > (long)(parkBase + PARK_PUNTA_SALTO_MAX_CM);
        if (lecturaAlta) {
          if (++parkAltasCnt >= PARK_PUNTA_ALTO_REBASE_N) {
            // Muchas seguidas y ninguna normal: el carro de verdad se alejó.
            Serial.print("PARK: re-base alto "); Serial.print(parkBase, 1);
            Serial.print(" -> "); Serial.println(extRaw);
            parkBase     = (float)extRaw;
            parkAltasCnt = 0;
            lecturaAlta  = false;
          }
        } else if (lecturaEnRango) {
          parkAltasCnt = 0;
        }
        bool lecturaValida = lecturaEnRango && !lecturaAlta;   // pared: base y error

        // Bajada = muy por debajo de la base, INCLUYENDO 1-7 cm: pasando pegado al
        // poste el HC-SR04 da su mínimo (orillas898: 2,3; orillas959: 7,5).
        bool bajada    = (baseLista && extRaw >= 1
                          && extRaw <= (long)(parkBase - PARK_CAIDA_CM));
        bool sinEco    = (extRaw > PARK_PARED_MAX_CM) || lecturaAlta;   // 200 = sin eco
        bool despejado = (lecturaValida && baseLista && extRaw >= (long)(parkBase - 6));

        // Mientras hay bajada NO se toca la base (el poste no es la pared). Un "sin
        // eco" a media bajada no la reinicia (hasta PARK_PUNTA_CAIDA_HUECOS_N).
        if (bajada) {
          parkCaidaCnt++;
          parkCaidaHuecos = 0;
          if (parkCaidaCnt > PARK_REBASE_N) {
            // Duró demasiado para un poste de 2 cm: el carro se acercó a la pared
            Serial.print("PARK: re-base "); Serial.print(parkBase, 1);
            Serial.print(" -> "); Serial.println(extRaw);
            parkBase     = (float)max(extRaw, 3L);
            parkCaidaCnt = 0;
          }
        } else if (sinEco && parkCaidaCnt > 0 && parkCaidaHuecos < PARK_PUNTA_CAIDA_HUECOS_N) {
          parkCaidaHuecos++;
        } else {
          parkCaidaCnt    = 0;
          parkCaidaHuecos = 0;
          if (lecturaValida) {
            parkBase = (parkBaseN == 0)
                       ? (float)extRaw
                       : parkBase + PARK_BASE_ALPHA * ((float)extRaw - parkBase);
            if (parkBaseN < 1000) parkBaseN++;
          }
        }
        // Poste confirmado: PARK_CAIDA_N lecturas bajas (mismo criterio para los dos postes).
        bool posteConfirmado = (parkCaidaCnt >= PARK_CAIDA_N);

        {
          unsigned long nowI = millis();
          float dtI = (nowI - puntaIntMs) / 1000.0f;
          puntaIntMs = nowI;
          if (dtI > 0.2f) dtI = 0.2f;
          if (lecturaValida && parkCaidaCnt == 0 && PARK_PUNTA_KI_POS > 0.0f) {
            float iMax = PARK_PUNTA_I_MAX_DEG / PARK_PUNTA_KI_POS;
            puntaIntPared = constrain(puntaIntPared + ((float)extRaw - PARK_PUNTA_PARED_CM) * dtI,
                                      -iMax, iMax);
            if (extRaw < PARK_PUNTA_CERCA_CM && puntaIntPared > 0.0f) puntaIntPared = 0.0f;
          }
        }
        if (lecturaValida && parkCaidaCnt == 0) {
          parkErrPared     = (float)extRaw - PARK_PUNTA_PARED_CM;
          parkInvalidasCnt = 0;
        } else if (!lecturaValida && ++parkInvalidasCnt > 10) {
          parkErrPared = 0.0f;  // sin pared válida: solo sostiene el rumbo
        }

        bool armada = baseLista
                      && (tEn >= PARK_ARMADO_MS)
                      && (fabs(anguloGyro) < PARK_ARMADO_ANG_DEG)
                      && (parkRosaVisto || !PARK_REQUIERE_PI || PARK_TEST_MANO);

        // ── Subfase 0: Buscando Poste 1 ──
        if (parkScanSubFase == 0) {
          if (bajada && !armada && parkCaidaCnt == PARK_CAIDA_N) {
            Serial.print("PARK: bajada IGNORADA (no armada: base=");
            Serial.print(baseLista ? 1 : 0);
            Serial.print(" rosa="); Serial.print(parkRosaVisto ? 1 : 0);
            Serial.print(" ang="); Serial.print(anguloGyro, 1);
            Serial.println(")");
          }

          if (bajada && armada && posteConfirmado) {
            parkCaidaLectura = extRaw;
            Serial.print("PARK: POSTE 1 DETECTADO! base="); Serial.print(parkBase, 1);
            Serial.print(" ext="); Serial.print(extRaw);
            Serial.print(" ang="); Serial.print(anguloGyro, 1);
            Serial.print(" rosa="); Serial.println(parkRosaVisto ? 1 : 0);

            if (!PARK_MODO_PARALELO) {
              // Fallback De Punta: salta directo a preparar el arco
              parkRumboRef = anguloGyro;
              if (PARK_TEST_MANO) {
                parkRumboArco0 = anguloGyro;
                parkFase       = 10;
                Serial.println("PUNTA MANO: servo a tope -> empuja el carro por el arco");
              } else {
                motorCoast();
                escribirServo(centroServo);
                parkFase   = 1;
                parkFaseMs = millis();
              }
              break;
            }

            // Modo Paralelo: avanza a subfase 1 (espera hueco tras poste 1)
            parkScanSubFase = 1;
            parkGapCnt      = 0;
            parkCaidaCnt    = 0;  // no arrastrar la caída del poste 1
          }
        }
        // ── Subfase 1: Sobre Poste 1, esperando hueco ──
        else if (parkScanSubFase == 1) {
          if (despejado) {
            parkGapCnt++;
            if (parkGapCnt >= PARK_GAP_N) {
              parkScanSubFase  = 2;
              parkCaidaCnt     = 0;
              parkHuecoEntryMs = millis();
              Serial.print("PARK: HUECO DE CAJON (33cm) alcanzado en t=");
              Serial.println(millis() - parkEntryMs);
            }
          } else {
            parkGapCnt = 0;
          }
        }
        // ── Subfase 2: En Hueco de 33 cm, buscando Poste 2 ──
        else if (parkScanSubFase == 2) {
          unsigned long tHueco = millis() - parkHuecoEntryMs;
          // El orden de la máquina de estados ya es el candado: bajada confirmada
          // (poste 1) -> hueco confirmado (PARK_GAP_N lecturas de pared) -> bajada
          // confirmada otra vez (poste 2). El trailing edge del poste 1 no puede
          // colarse porque para llegar aquí ya hubo pared limpia. PARK_HUECO_MIN_MS
          // queda en 0; súbelo solo si un rebote del poste 1 dispara el poste 2.
          bool huecoMaduro   = (tHueco >= PARK_HUECO_MIN_MS);
          // Mismo conteo que el poste 1 (arriba). Antes aquí se volvía a sumar
          // parkCaidaCnt -> cada lectura baja contaba doble y el "sin eco" a media
          // bajada la reiniciaba.
          bool poste2Detect  = (bajada && huecoMaduro && posteConfirmado);

          // SOLO caída de sónar. La Pi manda park=2 al ver magenta lejos
          // (recta final, ~1 s tras entrar a ESTACIONANDO) y el timeout de
          // hueco disparaba reversa en el trailing del POSTE 1.
          if (poste2Detect) {
            parkScanSubFase = 3;
            parkGapCnt      = 0;
            parkCaidaCnt    = 0;
            Serial.print("PARK: 2a PARED / POSTE 2 (sonar caida tHueco=");
            Serial.print(tHueco);
            Serial.print("ms ext="); Serial.print(extRaw);
            Serial.print(" base="); Serial.print(parkBase, 1);
            Serial.println(")");

            if (PARK_MODO_PARALELO) {
              parkRumboRef = anguloGyro;
              if (PARK_TEST_MANO) {
                parkFase = 10;
                Serial.println("PARK MANO: 2a pared -> servo externo, empuja el carro");
              } else {
                parkFase   = 1;
                parkFaseMs = millis();
                Serial.println("PARK fase 1: espera 500ms a 25cm de pared EXTERIOR");
              }
              break;
            }
          } else if (tHueco >= PARK_HUECO_TIMEOUT_MS && (tHueco - PARK_HUECO_TIMEOUT_MS) < 40) {
            Serial.print("PARK: hueco largo sin 2a pared (t=");
            Serial.print(tHueco); Serial.println("ms) — sigo buscando, no reverseo");
          }
        }
        // ── Subfase 3: Sobre Poste 2, esperando despeje ──
        else if (parkScanSubFase == 3) {
          if (despejado) {
            parkGapCnt++;
            if (parkGapCnt >= PARK_GAP_N) {
              parkScanSubFase = 4;
              parkPassExtraMs = millis();
              Serial.println("PARK: POSTE 2 REBASADO -> Avance extra para colocar eje trasero");
            }
          } else {
            parkGapCnt = 0;
          }
        }
        // ── Subfase 4: Avance extra para colocar eje trasero delante de Poste 2 ──
        else if (parkScanSubFase == 4) {
          if (millis() - parkPassExtraMs >= PARK_PASS_EXTRA_MS) {
            Serial.print("PARK: POSICION DE LANZAMIENTO OK! t=");
            Serial.print(millis() - parkEntryMs);
            Serial.print("ms ang="); Serial.println(anguloGyro, 1);

            if (PARK_TEST_MANO) {
              parkRumboRef = anguloGyro;
              parkFase     = 10;
              Serial.println("PARK MANO: Listo para simular reversa");
            } else {
              motorCoast();
              escribirServo(centroServo);
              parkFase   = 1;
              parkFaseMs = millis();
            }
            break;
          }
        }

        // Control dinámico durante la Fase 0
        if (PARK_TEST_MANO) {
          motorCoast();
          setMotor(0);
          escribirServo(centroServo);
        } else {
          // Red de seguridad: timeout. FRENTE solo si SOLO_EXTERIOR está apagado (con
          // SOLO_EXTERIOR el frontal no se lee en fase 0, igual que ParkinHalf punta).
          if (!PARK_PUNTA_SOLO_EXTERIOR) {
            parkFrenteCnt = (distF_filtrada > 0 && distF_filtrada <= PARK_FRENTE_CM) ? parkFrenteCnt + 1 : 0;
            if (parkFrenteCnt >= 3)      { finalizarPark("FRENTE cerca sin cajon"); break; }
          } else {
            parkFrenteCnt = 0;
          }
          if (tEn >= PARK_TIMEOUT_MS)    { finalizarPark("TIMEOUT sin cajon");      break; }

          // Wall follower ParkinHalf (mismo que ESTACIONANDO_PUNTA fase 0)
          if (parkScanSubFase < 4) {
            float distPared = PARK_PUNTA_PARED_CM + parkErrPared;
            float rumboRaw  = PARK_PUNTA_KPOS * parkErrPared
                              + PARK_PUNTA_KI_POS * puntaIntPared
                              + PARK_PUNTA_KPOS_CERCA * min(0.0f, distPared - PARK_PUNTA_CERCA_CM);
            float limAlejar = (distPared < PARK_PUNTA_CERCA_CM) ? PARK_PUNTA_ANG_MAX_CERCA_DEG
                                                                : PARK_PUNTA_ANG_MAX_DEG;
            float rumboObj  = haciaPared * constrain(rumboRaw, -limAlejar, PARK_PUNTA_ANG_MAX_DEG);
            motorAdelante();
            servoRumboPunta(rumboObj, PARK_PUNTA_KP_ANG_SEGUIR);
            setMotor(PARK_PUNTA_PWM);
          } else {
            motorAdelante();
            servoRumboPunta(anguloGyro);
            setMotor(PARK_PUNTA_PWM);
          }
        }

        Serial.print(" | PARK f=0 sub="); Serial.print(parkScanSubFase);
        Serial.print(" ext="); Serial.print(extRaw);
        Serial.print(" base="); Serial.print(parkBase, 1);
        Serial.print(" caida="); Serial.print(parkCaidaCnt);
        Serial.print(" arm="); Serial.print(armada ? 1 : 0);
        break;
      }

      // ── Fase 1: ESPERA 500 ms a 25 cm (Paralelo) / FRENO PREVIO (Punta) ──
      if (parkFase == 1) {
        if (PARK_MODO_PARALELO) {
          if (PARK_TEST_MANO) {
            motorCoast();
            setMotor(0);
            escribirServo(centroServo);
            break;
          }
          bool lecturaValida = (extRaw > 2 && extRaw <= PARK_PARED_MAX_CM);
          if (lecturaValida) {
            parkErrPared     = (float)extRaw - PARK_ALIGN_CM;
            parkInvalidasCnt = 0;
          } else if (++parkInvalidasCnt > 10) {
            parkErrPared = 0.0f;
          }
          float rumboObj = haciaPared * constrain(PARK_KPOS * parkErrPared,
                                                  -PARK_ANG_MAX_DEG, PARK_ANG_MAX_DEG);
          motorAdelante();
          servoRumboPark(rumboObj);
          setMotor(PARK_PWM);
          if (millis() - parkFaseMs >= PARK_POSTE2_WAIT_MS) {
            motorCoast();
            escribirServo(servoHaciaPared);  // pre-coloca FULL EXTERNO
            // Altura lateral con la que el carro llegó al cajón: es lo que la
            // fase 14 tiene que compensar. parkBase (EMA de la pared, no el
            // instantáneo) porque el sonar ya está viendo el poste 2.
            parkBaseLlegada = parkBase;
            // Ángulo de entrada para ESTA llegada. delta = corrimiento lateral
            // que hace falta; un arco de ida+vuelta de ángulo a entrega
            // 2R(1-cos a), así que se invierte. Si delta >= R el arco se queda
            // en el tope de 60° y lo que sobra lo pone el recto de la fase 14.
            {
              float delta = parkBaseLlegada - PARK_FINAL_CM;
              float dosR  = 2.0f * PARK_RADIO_CM;
              if (!(parkBaseLlegada > 5.0f) || delta >= PARK_RADIO_CM) {
                parkAngObjetivo = (float)PARK_ANG_IN_DEG;
              } else if (delta <= 0.0f) {
                parkAngObjetivo = (float)PARK_ANG_MIN_IN_DEG;
              } else {
                parkAngObjetivo = degrees(acos(1.0f - delta / dosR));
                parkAngObjetivo = constrain(parkAngObjetivo,
                                            (float)PARK_ANG_MIN_IN_DEG,
                                            (float)PARK_ANG_IN_DEG);
              }
            }
            parkFase   = 2;
            parkFaseMs = millis();
            Serial.print("PARK fase 2: COAST (ext=");
            Serial.print(extRaw);
            Serial.print(" base="); Serial.print(parkBaseLlegada, 1);
            Serial.print(" ang="); Serial.print(anguloGyro, 1);
            Serial.println(") -> reversa FULL EXTERNO");
          }
        } else {
          motorCoast();
          escribirServo(centroServo);
          if (millis() - parkFaseMs >= PARK_PUNTA_FRENO_MS) {
            parkFaseMs   = millis();
            parkRumboRef = anguloGyro;
            parkFase     = 3;
          }
        }
        break;
      }

      // ── Fase 2: COAST + servo FULL EXTERNO, luego REV SWING (Paralelo) ──
      if (parkFase == 2 && PARK_MODO_PARALELO) {
        motorCoast();
        escribirServo(servoHaciaPared);
        if (millis() - parkFaseMs >= MANIOBRA_FRENO_MS) {
          motorReversa();
          parkFase   = 3;
          parkFaseMs = millis();
          Serial.print("PARK fase 3: REV SWING FULL EXTERNO hasta 60deg / 1s (ref=");
          Serial.print(parkRumboRef, 1); Serial.println(")");
        }
        break;
      }

      // ── Fase 3: REV SWING FULL EXTERNO (Paralelo) / PREP (Punta) ────────
      if (parkFase == 3) {
        if (PARK_MODO_PARALELO) {
          motorReversa();
          escribirServo(servoHaciaPared);
          setMotor(PARK_REV_PWM);
          float swingMag    = fabs(anguloGyro - parkRumboRef);
          bool  swingListo  = (swingMag >= (parkAngObjetivo - (float)PARK_OVERSHOOT_DEG));
          bool  swingTimeout = (millis() - parkFaseMs >= PARK_SWING_TIMEOUT_MS);
          if (swingListo || swingTimeout) {
            // Tramo recto que se come el exceso de altura lateral (ver
            // PARK_RECTO_*). Sale 0 si viene igual o más pegado que D0, si la
            // base no es creíble, o si la compensación está apagada.
            float exceso = parkBaseLlegada - PARK_RECTO_D0_CM;
            bool  baseOk = (parkBaseLlegada > 5.0f && parkBaseLlegada <= (float)PARK_PARED_MAX_CM);
            if (PARK_RECTO_ENABLED && baseOk && exceso > 0.0f && PARK_RECTO_VEL_CMS > 1.0f) {
              float ms = (exceso / (0.866f * PARK_RECTO_VEL_CMS)) * 1000.0f;
              parkRectoMs = (unsigned long)min(ms, (float)PARK_RECTO_MAX_MS);
            } else {
              parkRectoMs = 0;
            }
            parkFase   = (parkRectoMs > 0) ? 14 : 4;
            parkFaseMs = millis();
            Serial.print("PARK fase "); Serial.print(parkFase);
            Serial.print(parkRectoMs > 0 ? ": RECTO a 60deg " : ": REV FULL EXTERNO (recto 0) ");
            Serial.print("(swing="); Serial.print(swingMag, 1);
            Serial.print(swingTimeout ? " TIMEOUT" : "");
            Serial.print(" d="); Serial.print(parkBaseLlegada, 1);
            Serial.print(" d0="); Serial.print(PARK_RECTO_D0_CM, 1);
            Serial.print(" tau="); Serial.print(parkRectoMs);
            Serial.println("ms)");
          }
        } else {
          motorCoast();
          escribirServo(servoHaciaPared);
          if (millis() - parkFaseMs >= PARK_PUNTA_SERVO_MS) {
            motorAdelante();
            parkRumboArco0 = anguloGyro;
            parkDfCnt      = 0;
            parkFase       = 4;
            parkFaseMs     = millis();
            Serial.print("PUNTA fase 4: ARCO desde ang="); Serial.println(anguloGyro, 1);
          }
        }
        break;
      }

      // ── Fase 14: RECTO a 60° — compensa la altura lateral de llegada ────
      // Servo al CENTRO: el carro ya viene apuntando 60° a la pared, así que
      // retroceder recto es puro corrimiento lateral (0.866 cm por cm). Al
      // terminar, la maniobra de siempre (fase 4 en adelante) arranca como si
      // el carro hubiera llegado a PARK_RECTO_D0_CM.
      if (parkFase == 14 && PARK_MODO_PARALELO) {
        motorReversa();
        escribirServo(centroServo);
        setMotor(PARK_REV_PWM);
        if (millis() - parkFaseMs >= parkRectoMs) {
          escribirServo(servoHaciaPared);
          parkFase   = 4;
          parkFaseMs = millis();
          Serial.println("PARK fase 4: REV FULL EXTERNO (maniobra base)");
        }
        break;
      }

      // ── Fase 4: REV 300 ms FULL EXTERNO (Paralelo) / ARCO (Punta) ───────
      if (parkFase == 4) {
        if (PARK_MODO_PARALELO) {
          motorReversa();
          escribirServo(servoHaciaPared);
          setMotor(PARK_REV_PWM);
          if (millis() - parkFaseMs >= PARK_REV_EXT_HOLD_MS) {
            // Sin meneo (PARK_WIGGLE_*_MS = 0) se salta directo a la contravuelta.
            parkFase   = (PARK_WIGGLE_INT_MS > 0) ? 5
                       : (PARK_WIGGLE_EXT_MS > 0) ? 6 : 8;
            parkFaseMs = millis();
            Serial.print("PARK fase "); Serial.print(parkFase);
            Serial.println(parkFase == 8 ? ": FULL INTERNO hasta heading recta ~ 0 (sin meneo)"
                                         : ": meneo");
          }
        } else {
          unsigned long tA = millis() - parkFaseMs;
          int vel = (tA < PARK_PUNTA_RAMP_MS)
                    ? (int)map((long)tA, 0, (long)PARK_PUNTA_RAMP_MS, PARK_PUNTA_ARCO_PWM_MIN, PARK_PUNTA_ARCO_PWM)
                    : PARK_PUNTA_ARCO_PWM;
          motorAdelante();
          escribirServo(servoHaciaPared);
          setMotor(vel);

          float girado = (anguloGyro - parkRumboArco0) * haciaPared;
          parkDfCnt = (distF_filtrada > 0 && distF_filtrada <= PARK_PUNTA_DF_STOP_CM) ? parkDfCnt + 1 : 0;

          if (parkDfCnt >= PARK_PUNTA_DF_N)      { finalizarPark("PUNTA ARCO: dF tope"); break; }
          if (tA >= PARK_PUNTA_ARCO_TIMEOUT_MS)  { finalizarPark("PUNTA ARCO: timeout"); break; }
          if (girado >= (float)(PARK_PUNTA_ARCO_DEG - PARK_PUNTA_OVERSHOOT_DEG)) {
            if (PARK_PUNTA_ENTRA_MS == 0) { finalizarPark("PUNTA ARCO: angulo"); break; }
            parkFase   = 5;
            parkFaseMs = millis();
            Serial.print("PUNTA fase 5: ENTRA girado="); Serial.println(girado, 1);
            break;
          }
        }
        break;
      }

      // ── Fase 5: FULL INTERNO 200 ms (Paralelo) / ENTRA (Punta) ──────────
      if (parkFase == 5) {
        if (PARK_MODO_PARALELO) {
          motorReversa();
          escribirServo(servoDesdePared);
          setMotor(PARK_REV_PWM);
          if (millis() - parkFaseMs >= PARK_WIGGLE_INT_MS) {
            parkFase   = (PARK_WIGGLE_EXT_MS > 0) ? 6 : 8;
            parkFaseMs = millis();
            Serial.print("PARK fase "); Serial.println(parkFase);
          }
        } else {
          motorAdelante();
          servoRumboPark(parkRumboArco0 + haciaPared * (float)PARK_PUNTA_ARCO_DEG);
          setMotor(PARK_PUNTA_ENTRA_PWM);
          parkDfCnt = (distF_filtrada > 0 && distF_filtrada <= PARK_PUNTA_DF_STOP_CM) ? parkDfCnt + 1 : 0;
          if (parkDfCnt >= PARK_PUNTA_DF_N)                 { finalizarPark("PUNTA ENTRA: dF tope"); break; }
          if (millis() - parkFaseMs >= PARK_PUNTA_ENTRA_MS) { finalizarPark("PUNTA ENTRA: tiempo");  break; }
        }
        break;
      }

      // ── Fase 6: FULL EXTERNO 100 ms (Paralelo) ─────────────────────────
      if (parkFase == 6 && PARK_MODO_PARALELO) {
        motorReversa();
        escribirServo(servoHaciaPared);
        setMotor(PARK_REV_PWM);
        if (millis() - parkFaseMs >= PARK_WIGGLE_EXT_MS) {
          parkFase   = 8;
          parkFaseMs = millis();
          Serial.println("PARK fase 8: FULL INTERNO hasta heading recta ~ 0");
        }
        break;
      }

      // ── Fase 8: FULL INTERNO; luego SIEMPRE acomodo ENFRENTE ────────────
      if (parkFase == 8 && PARK_MODO_PARALELO) {
        motorReversa();
        escribirServo(servoDesdePared);
        setMotor(PARK_REV_PWM);
        float difHeading = fabs(anguloGyro - parkRumboRef);
        bool  alineado   = (difHeading <= PARK_ENDEREZA_TOL_DEG);
        bool  pegado     = (extRaw > 0 && extRaw <= PARK_PEGADO_CM);
        bool  timeout    = (millis() - parkFaseMs >= PARK_CONTRA_TIMEOUT_MS);
        if (alineado || pegado || timeout) {
          motorCoast();
          escribirServo(centroServo);
          parkFase   = 9;
          parkFaseMs = millis();
          Serial.print("PARK fase 9: COAST -> ENFRENTE (dif=");
          Serial.print(difHeading, 1);
          Serial.print(" ext="); Serial.print(extRaw);
          if (alineado) Serial.print(" ALINEADO");
          if (pegado)   Serial.print(" PEGADO");
          Serial.println(timeout ? " TIMEOUT)" : ")");
        }
        break;
      }

      // ── Fase 9: COAST antes de ir adelante ─────────────────────────────
      if (parkFase == 9 && PARK_MODO_PARALELO) {
        motorCoast();
        escribirServo(centroServo);
        if (millis() - parkFaseMs >= MANIOBRA_FRENO_MS) {
          motorAdelante();
          parkFase   = 11;
          parkFaseMs = millis();
          Serial.println("PARK fase 11: ACOMODO ENFRENTE");
        }
        break;
      }

      // ── Fase 11: ACOMODO ENFRENTE ──────────────────────────────────────
      if (parkFase == 11 && PARK_MODO_PARALELO) {
        motorAdelante();
        servoRumboPark(parkRumboRef);
        setMotor(PARK_CENTER_PWM_PAR);
        bool dfTope  = (distF_filtrada > 0 && distF_filtrada <= PARK_CENTER_HI_CM);
        bool timeout = (millis() - parkFaseMs >= PARK_FWD_MS);
        if (dfTope || timeout) {
          motorCoast();
          escribirServo(centroServo);
          float dif = fabs(anguloGyro - parkRumboRef);
          if (!PARK_REV_FINAL || dif <= PARK_FINAL_TOL_DEG) {
            // Sin reversa final (PARK_REV_FINAL=false): aquí se acaba. La reversa
            // de las fases 12/13 metía el carro contra la pared de atrás.
            finalizarPark(dfTope ? "ENFRENTE dF ya a 0" : "ENFRENTE tiempo ya a 0");
          } else {
            parkFase   = 12;
            parkFaseMs = millis();
            Serial.print("PARK fase 12: COAST -> REV a 0 (dif=");
            Serial.print(dif, 1); Serial.println(")");
          }
        }
        break;
      }

      // ── Fase 12: COAST antes de reversear a 0 ──────────────────────────
      if (parkFase == 12 && PARK_MODO_PARALELO) {
        motorCoast();
        escribirServo(centroServo);
        if (millis() - parkFaseMs >= MANIOBRA_FRENO_MS) {
          integralRev   = 0;
          prevErrorRev  = 0;
          lastRevHoldMs = millis();
          motorReversa();
          parkFase   = 13;
          parkFaseMs = millis();
          Serial.println("PARK fase 13: REV hasta heading 0");
        }
        break;
      }

      // ── Fase 13: reversa hasta ángulo de la recta = 0 ──────────────────
      if (parkFase == 13 && PARK_MODO_PARALELO) {
        motorReversa();
        aplicarReversaHold(parkRumboRef);
        setMotor(PARK_REV_PWM);
        float dif      = fabs(anguloGyro - parkRumboRef);
        bool  alineado = (dif <= PARK_FINAL_TOL_DEG);
        bool  pegado   = (extRaw > 0 && extRaw <= PARK_PEGADO_CM);
        bool  timeout  = (millis() - parkFaseMs >= PARK_REV0_TIMEOUT_MS);
        if (alineado || timeout) {
          finalizarPark(alineado ? "REV a 0 OK" : "REV a 0 TIMEOUT");
        } else if (pegado) {
          // pegado de lado: no stalls; corta igual (ya acomodó enfrente)
          finalizarPark("REV a 0 PEGADO");
        }
        break;
      }

      // ── Fase 6 legacy: sawtooth (solo de punta no llega aquí; idle) ────
      if (parkFase == 6 && !PARK_MODO_PARALELO) {
        finalizarPark("PUNTA: fase 6 inesperada");
        break;
      }

      // ── Fase 7: FIN — robot completamente estacionado y apagado ───────────
      if (parkFase == 7) {
        motorCoast();
        setMotor(0);
        escribirServo(centroServo);
        raceFinished = true;
        break;
      }

      // ── Fase 10: PRUEBA A MANO — servo activo hacia pared, motor apagado ───
      if (parkFase == 10) {
        motorCoast();
        setMotor(0);
        escribirServo(servoHaciaPared);
        if (millis() - parkLogMs >= 200) {
          parkLogMs = millis();
          Serial.print("PARK MANO ang=");
          Serial.print(anguloGyro, 1);
          Serial.print(" dF="); Serial.print(distF_filtrada);
          Serial.print(" ext="); Serial.print(extRaw);
          Serial.print(" base="); Serial.println(parkBase, 1);
        }
        break;
      }

      break;
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // ESTACIONANDO_PUNTA — ver el bloque de constantes PARK_PUNTA_*
    // ═══════════════════════════════════════════════════════════════════════════
    case ESTACIONANDO_PUNTA: {
      long  extRaw     = puntaParedIzq ? distL_raw : distR_raw;
      float haciaPared = puntaParedIzq ? 1.0f : -1.0f;   // signo de "rotar hacia la pared"
      int   servoTope  = puntaParedIzq ? 150 : 20;       // servo a tope hacia la pared
      puntaExtRaw = extRaw;
      if (piPark >= 1) puntaRosaVisto = true;

      // ── Fase 0: SEGUIR pared exterior + vigilar la bajada ──────────────────
      if (puntaFase == 0) {
        unsigned long tEn = millis() - puntaEntryMs;
        bool lecturaEnRango = (extRaw > 2 && extRaw <= PARK_PUNTA_PARED_MAX_CM);
        bool baseLista      = (puntaBaseN >= PARK_PUNTA_BASE_N);
        // Lectura muy por ENCIMA de la base: eco perdido o rebote, no la pared -> se
        // trata como "sin eco" (no mueve base ni error). Ver PARK_PUNTA_SALTO_MAX_CM.
        bool lecturaAlta    = baseLista && lecturaEnRango
                              && extRaw > (long)(puntaBase + PARK_PUNTA_SALTO_MAX_CM);
        if (lecturaAlta) {
          puntaAltasTot++;
          if (++puntaAltasCnt >= PARK_PUNTA_ALTO_REBASE_N) {
            // Muchas seguidas y ninguna normal: el carro de verdad se alejó.
            Serial.print("PUNTA: re-base alto "); Serial.print(puntaBase, 1);
            Serial.print(" -> "); Serial.println(extRaw);
            puntaBase     = (float)extRaw;
            puntaAltasCnt = 0;
            lecturaAlta   = false;
          }
        } else if (lecturaEnRango) {
          puntaAltasCnt = 0;
        }
        bool lecturaValida  = lecturaEnRango && !lecturaAlta;   // pared: base y error
        // Bajada = lectura muy por debajo de la base. Incluye 1-2 cm: pasando pegado
        // al poste el HC-SR04 está en su mínimo y da 2-3 (orillas898: el 1er poste
        // dio 2,3 y el "2" contaba como inválido y reiniciaba la cuenta).
        bool lecturaBaja   = baseLista && extRaw >= 1
                             && extRaw <= (long)(puntaBase - PARK_PUNTA_CAIDA_CM);
        bool sinEco        = (extRaw > PARK_PUNTA_PARED_MAX_CM) || lecturaAlta;   // 200 = sin eco

        // Mientras hay bajada NO actualiza la base ni el error de pared (el poste no
        // es la pared). Un "sin eco" a media bajada no la reinicia (hasta HUECOS_N).
        if (lecturaBaja) {
          puntaCaidaCnt++;
          puntaCaidaHuecos = 0;
          if (puntaCaidaCnt > PARK_PUNTA_REBASE_N) {
            // Duró demasiado para un poste de 2 cm: el carro se acercó de verdad.
            Serial.print("PUNTA: re-base "); Serial.print(puntaBase, 1);
            Serial.print(" -> "); Serial.println(extRaw);
            puntaBase     = (float)max(extRaw, 3L);
            puntaCaidaCnt = 0;
          }
        } else if (sinEco && puntaCaidaCnt > 0 && puntaCaidaHuecos < PARK_PUNTA_CAIDA_HUECOS_N) {
          puntaCaidaHuecos++;
        } else {
          puntaCaidaCnt    = 0;
          puntaCaidaHuecos = 0;
          if (lecturaValida) {
            puntaBase = (puntaBaseN == 0)
                        ? (float)extRaw
                        : puntaBase + PARK_PUNTA_BASE_ALPHA * ((float)extRaw - puntaBase);
            if (puntaBaseN < 1000) puntaBaseN++;
          }
        }

        {
          unsigned long nowI = millis();
          float dtI = (nowI - puntaIntMs) / 1000.0f;
          puntaIntMs = nowI;
          if (dtI > 0.2f) dtI = 0.2f;
          if (lecturaValida && puntaCaidaCnt == 0 && PARK_PUNTA_KI_POS > 0.0f) {
            float iMax = PARK_PUNTA_I_MAX_DEG / PARK_PUNTA_KI_POS;
            puntaIntPared = constrain(puntaIntPared + ((float)extRaw - PARK_PUNTA_PARED_CM) * dtI,
                                      -iMax, iMax);
            // Pegado a la pared no se permite que el integral siga empujando hacia ella.
            if (extRaw < PARK_PUNTA_CERCA_CM && puntaIntPared > 0.0f) puntaIntPared = 0.0f;
          }
        }
        if (lecturaValida && puntaCaidaCnt == 0) {
          puntaErrPared     = (float)extRaw - PARK_PUNTA_PARED_CM;   // >0 = lejos de la pared
          puntaInvalidasCnt = 0;
        } else if (!lecturaValida && ++puntaInvalidasCnt > 10) {
          puntaErrPared = 0.0f;   // sin pared un rato: solo sostiene el rumbo
        }

        bool armada = baseLista
                      && (tEn >= PARK_PUNTA_ARMADO_MS)
                      && (fabs(anguloGyro) < PARK_PUNTA_ARMADO_ANG_DEG)
                      && (puntaRosaVisto || !PARK_PUNTA_REQUIERE_PI || PARK_PUNTA_TEST_MANO);

        if (lecturaBaja && puntaCaidaCnt == PARK_PUNTA_CAIDA_N && !armada) {
          Serial.print("PUNTA: bajada IGNORADA (no armada: base=");
          Serial.print(baseLista ? 1 : 0);
          Serial.print(" rosa="); Serial.print(puntaRosaVisto ? 1 : 0);
          Serial.print(" ang="); Serial.print(anguloGyro, 1);
          Serial.println(")");
        }

        if (lecturaBaja && puntaCaidaCnt >= PARK_PUNTA_CAIDA_N && armada) {
          puntaCaidaLectura = extRaw;
          puntaRumboRef     = anguloGyro;
          Serial.print("PUNTA: BAJADA base="); Serial.print(puntaBase, 1);
          Serial.print(" lectura="); Serial.print(extRaw);
          Serial.print(" ang="); Serial.print(anguloGyro, 1);
          Serial.print(" rosa="); Serial.println(puntaRosaVisto ? 1 : 0);
          if (PARK_PUNTA_TEST_MANO) {
            puntaRumboArco0 = anguloGyro;
            puntaFase       = 10;
            Serial.println("PUNTA MANO: servo a tope -> empuja el carro por el arco");
          } else {
            motorCoast();
            escribirServo(centroServo);
            puntaFase = 1;
          }
          puntaFaseMs = millis();
          break;
        }

        if (PARK_PUNTA_TEST_MANO) {
          motorCoast();
          setMotor(0);
          escribirServo(centroServo);
        } else {
          // Red de seguridad: algo muy cerca de frente o se acabó el tiempo.
          puntaFrenteCnt = (distF_med > 0 && distF_med <= PARK_PUNTA_FRENTE_CM) ? puntaFrenteCnt + 1 : 0;
          if (puntaFrenteCnt >= 3)            { finalizarPunta("FRENTE cerca sin bajada"); break; }
          if (tEn >= PARK_PUNTA_TIMEOUT_MS)   { finalizarPunta("TIMEOUT sin bajada");      break; }

          // Cascada: error de pared (congelado en la bajada) -> rumbo objetivo -> gyro PD.
          // Pegado a la pared (< CERCA_CM) se aleja con más ganancia y más ángulo:
          // las puntas de los postes están a 20 cm y el 1er poste puede estar a
          // pocos cm de donde deja la media vuelta (orillas898: pasó a 2-3 cm).
          float distPared = PARK_PUNTA_PARED_CM + puntaErrPared;
          float rumboRaw  = PARK_PUNTA_KPOS * puntaErrPared
                            + PARK_PUNTA_KI_POS * puntaIntPared
                            + PARK_PUNTA_KPOS_CERCA * min(0.0f, distPared - PARK_PUNTA_CERCA_CM);
          float limAlejar = (distPared < PARK_PUNTA_CERCA_CM) ? PARK_PUNTA_ANG_MAX_CERCA_DEG
                                                              : PARK_PUNTA_ANG_MAX_DEG;
          float rumboObj  = haciaPared * constrain(rumboRaw, -limAlejar, PARK_PUNTA_ANG_MAX_DEG);
          motorAdelante();
          servoRumboPunta(rumboObj, PARK_PUNTA_KP_ANG_SEGUIR);
          setMotor(PARK_PUNTA_PWM);
        }

        Serial.print(" | PUNTA f=0 ext="); Serial.print(extRaw);
        Serial.print(" base=");  Serial.print(puntaBase, 1);
        Serial.print(" caida="); Serial.print(puntaCaidaCnt);
        Serial.print(" int=");   Serial.print(PARK_PUNTA_KI_POS * puntaIntPared, 1);
        Serial.print(" alta=");  Serial.print(lecturaAlta ? 1 : 0);
        Serial.print(" rosa=");  Serial.print(puntaRosaVisto ? 1 : 0);
        Serial.print(" arm=");   Serial.print(armada ? 1 : 0);
        break;
      }

      // ── Fase 1: FRENO — coast, el carro se detiene ─────────────────────────
      if (puntaFase == 1) {
        motorCoast();
        escribirServo(centroServo);
        if (millis() - puntaFaseMs >= PARK_PUNTA_FRENO_MS) {
          puntaFaseMs = millis();
          if (PARK_PUNTA_AJUSTE_MS > 0) {
            motorAdelante();
            puntaFase = 2;
          } else if (PARK_PUNTA_AJUSTE_MS < 0) {
            motorReversa();
            integralRev   = 0; prevErrorRev = 0;
            lastRevHoldMs = millis();
            puntaFase     = 2;
          } else {
            puntaFase = 3;
          }
        }
        break;
      }

      // ── Fase 2: AJUSTE — recto adelante (+) o en reversa (-) ────────────────
      if (puntaFase == 2) {
        if (PARK_PUNTA_AJUSTE_MS > 0) {
          motorAdelante();
          servoRumboPunta(puntaRumboRef);
        } else {
          motorReversa();
          aplicarReversaHold(puntaRumboRef);
        }
        setMotor(PARK_PUNTA_AJUSTE_PWM);
        if (millis() - puntaFaseMs >= (unsigned long)labs(PARK_PUNTA_AJUSTE_MS)) {
          motorCoast();
          puntaFase   = 3;
          puntaFaseMs = millis();
        }
        break;
      }

      // ── Fase 3: PREP — coast + servo a tope hacia la pared ─────────────────
      if (puntaFase == 3) {
        motorCoast();
        escribirServo(servoTope);
        // Tras la reversa también cuenta como coast antes de volver a adelante.
        unsigned long espera = (PARK_PUNTA_AJUSTE_MS < 0)
                               ? max(PARK_PUNTA_SERVO_MS, MANIOBRA_FRENO_MS)
                               : PARK_PUNTA_SERVO_MS;
        if (millis() - puntaFaseMs >= espera) {
          motorAdelante();
          puntaRumboArco0 = (puntaRetorno && PARK_PUNTA_ARCO_ABSOLUTO && MANIOBRA_13_CUADRAR_MS > 0)
                            ? 0.0f : anguloGyro;
          puntaDfCnt      = 0;
          puntaFase       = 4;
          puntaFaseMs     = millis();
          Serial.print("PUNTA fase 4: ARCO desde ang="); Serial.println(anguloGyro, 1);
        }
        break;
      }

      // ── Fase 4: ARCO — servo a tope hasta el ángulo o dF tope ───────────────
      if (puntaFase == 4) {
        unsigned long tA = millis() - puntaFaseMs;
        int vel = (tA < PARK_PUNTA_RAMP_MS)
                  ? (int)map((long)tA, 0, (long)PARK_PUNTA_RAMP_MS, PARK_PUNTA_ARCO_PWM_MIN, PARK_PUNTA_ARCO_PWM)
                  : PARK_PUNTA_ARCO_PWM;
        motorAdelante();
        escribirServo(servoTope);
        setMotor(vel);

        float girado = (anguloGyro - puntaRumboArco0) * haciaPared;   // >0 = hacia la pared
        puntaDfCnt = (distF_med > 0 && distF_med <= PARK_PUNTA_DF_STOP_CM) ? puntaDfCnt + 1 : 0;

        if (puntaDfCnt >= PARK_PUNTA_DF_N)      { finalizarPunta("ARCO: dF tope");  break; }
        if (tA >= PARK_PUNTA_ARCO_TIMEOUT_MS)   { finalizarPunta("ARCO: timeout");  break; }
        if (girado >= (float)(PARK_PUNTA_ARCO_DEG - PARK_PUNTA_OVERSHOOT_DEG)) {
          if (PARK_PUNTA_ENTRA_MS == 0) { finalizarPunta("ARCO: angulo"); break; }
          puntaFase   = 5;
          puntaFaseMs = millis();
          Serial.print("PUNTA fase 5: ENTRA girado="); Serial.println(girado, 1);
          break;
        }
        Serial.print(" | PUNTA f=4 girado="); Serial.print(girado, 1);
        Serial.print(" dF="); Serial.print(distF_med);
        break;
      }

      // ── Fase 5: ENTRA — recto hacia la pared hasta dF tope o timeout ───────
      if (puntaFase == 5) {
        motorAdelante();
        servoRumboPunta(puntaRumboArco0 + haciaPared * (float)PARK_PUNTA_ARCO_DEG);
        setMotor(PARK_PUNTA_ENTRA_PWM);
        puntaDfCnt = (distF_med > 0 && distF_med <= PARK_PUNTA_DF_STOP_CM) ? puntaDfCnt + 1 : 0;
        if (puntaDfCnt >= PARK_PUNTA_DF_N)                 { finalizarPunta("ENTRA: dF tope"); break; }
        if (millis() - puntaFaseMs >= PARK_PUNTA_ENTRA_MS) { finalizarPunta("ENTRA: tiempo");  break; }
        Serial.print(" | PUNTA f=5 dF="); Serial.print(distF_med);
        break;
      }

      // ── Fase 10: PRUEBA A MANO — servo a tope, motor apagado ────────────────
      if (puntaFase == 10) {
        motorCoast();
        setMotor(0);
        escribirServo(servoTope);
        if (millis() - puntaLogMs >= 200) {
          puntaLogMs = millis();
          Serial.print("PUNTA MANO girado=");
          Serial.print((anguloGyro - puntaRumboArco0) * haciaPared, 1);
          Serial.print(" dF="); Serial.print(distF_med);
          Serial.print(" ext="); Serial.println(extRaw);
        }
        break;
      }

      // ── Fase 20: MEDIA VUELTA — 90° extra hacia adelante, hacia la pared ────
      if (puntaFase == 20) {
        unsigned long t = millis() - puntaFaseMs;
        // Sin avance previo el carro arranca parado -> espera a que el servo llegue a
        // tope. Con avance previo ya va rodando y el servo llega en movimiento.
        unsigned long tArranque = (PARK_RETORNO_AVANCE_MS > 0) ? 0UL : PARK_PUNTA_SERVO_MS;
        unsigned long tMov = (t > tArranque) ? (t - tArranque) : 0UL;
        int vel = (tMov < PARK_RETORNO_RAMP_MS)
                  ? (int)map((long)tMov, 0, (long)PARK_RETORNO_RAMP_MS, PARK_RETORNO_PWM_MIN, PARK_RETORNO_PWM)
                  : PARK_RETORNO_PWM;

        // 1) Avance RECTO antes de girar: la media vuelta termina más lejos de la
        //    pared del lote (orillas898 quedó a 17-19 cm y el seguidor no alcanzó a
        //    salir a 30 antes del 1er poste).
        if (t < PARK_RETORNO_AVANCE_MS) {
          motorAdelante();
          servoRumboPunta(puntaRumboGiro0);   // recto con gyro sobre el rumbo actual
          setMotor(vel);
          Serial.print(" | PUNTA f=20 avance t="); Serial.print(t);
          break;
        }

        // 2) Giro: servo a tope hacia la pared del regreso.
        escribirServo(servoTope);
        if (t < tArranque) {                     // (solo sin avance) espera al servo parado
          motorCoast();
          break;
        }
        motorAdelante();
        setMotor(vel);

        float girado = (anguloGyro - puntaRumboGiro0) * haciaPared;   // >0 = hacia la pared del regreso
        bool  listo  = girado >= (float)(90 - PARK_RETORNO_OVERSHOOT_DEG);
        bool  tout   = tMov >= PARK_RETORNO_AVANCE_MS + PARK_RETORNO_TIMEOUT_MS;
        if (listo || tout) {
          motorCoast();
          escribirServo(centroServo);
          puntaFase         = 21;
          puntaFaseMs       = millis();
          puntaSettleSampMs = millis();
          puntaSettleQuieto = 0;
          Serial.print("PUNTA fase 21: media vuelta girado="); Serial.print(girado, 1);
          Serial.println(tout ? " (TIMEOUT)" : "");
          break;
        }
        Serial.print(" | PUNTA f=20 girado="); Serial.print(girado, 1);
        break;
      }

      // ── Fase 21: SETTLE — coast hasta que deje de rotar ────────────────────
      if (puntaFase == 21) {
        motorCoast();
        escribirServo(centroServo);
        unsigned long nowMs = millis();
        if (nowMs - puntaSettleSampMs >= MANIOBRA_SETTLE_SAMPLE_MS) {
          puntaSettleSampMs = nowMs;
          puntaSettleQuieto = (fabs(gyroRate) < MANIOBRA_SETTLE_RATE_DPS) ? puntaSettleQuieto + 1 : 0;
        }
        unsigned long tS = nowMs - puntaFaseMs;
        bool quieto = (puntaSettleQuieto >= MANIOBRA_SETTLE_QUIETO_N) && (tS >= MANIOBRA_FRENO_MS);
        if (quieto || tS >= PARK_RETORNO_SETTLE_MAX_MS) {
          // 0° = recta de REGRESO ideal. Antes de la media vuelta 0° era la recta
          // ideal que dejó finalizarManiobra(); la media vuelta suma 90° hacia la pared.
          anguloGyro -= haciaPared * 90.0f;
          Serial.print("PUNTA: rumbo de regreso ang="); Serial.println(anguloGyro, 1);
          unsigned long revMs = puntaParedIzq ? PARK_RETORNO_REV_CCW_MS : PARK_RETORNO_REV_CW_MS;
          if (revMs > 0) {
            motorReversa();
            integralRev   = 0; prevErrorRev = 0;
            lastRevHoldMs = millis();
            puntaFase     = 22;
            puntaFaseMs   = millis();
          } else {
            motorAdelante();
            arrancarSeguirPunta();
          }
        }
        break;
      }

      // ── Fase 22: REVERSA — recto hacia atrás por tiempo, solo gyro ──────────
      if (puntaFase == 22) {
        unsigned long revMs = puntaParedIzq ? PARK_RETORNO_REV_CCW_MS : PARK_RETORNO_REV_CW_MS;
        motorReversa();
        aplicarReversaHold(0.0f);   // PID de gyro hacia la recta de regreso (0°)
        setMotor(PARK_RETORNO_REV_PWM);
        if (millis() - puntaFaseMs >= revMs) {
          motorCoast();
          escribirServo(centroServo);
          puntaFase   = 23;
          puntaFaseMs = millis();
        }
        break;
      }

      // ── Fase 23: FRENO — coast antes de volver a adelante ──────────────────
      if (puntaFase == 23) {
        motorCoast();
        escribirServo(centroServo);
        if (millis() - puntaFaseMs >= MANIOBRA_FRENO_MS) {
          motorAdelante();
          arrancarSeguirPunta();
        }
        break;
      }

      break;
    }
  }

  // ── Log periódico ─────────────────────────────────────────────────────────
  Serial.print(" | Estado:");
  if      (estado == GIRANDO)      Serial.print("GIRANDO");
  else if (estado == RECUPERANDO)  Serial.print("RECUPERANDO");
  else if (estado == CRUCERO)      Serial.print("CRUCERO");
  else if (estado == MANIOBRA)     Serial.print("MANIOBRA");
  else if (estado == TERMINANDO)   Serial.print("TERMINANDO");
  else if (estado == ESTACIONANDO) Serial.print("ESTACIONANDO");
  else if (estado == ESTACIONANDO_PUNTA) Serial.print("ESTACIONANDO_PUNTA");
  else if (estado == INICIO)       Serial.print("INICIO");
  else                              Serial.print("SIGUIENDO");
  Serial.print(" | PP:");       Serial.print(piPurePursuit ? 1 : 0);
  Serial.print(" | L:");        Serial.print(distL);
  Serial.print(" | R:");        Serial.print(distR);
  Serial.print(" | F:"); Serial.print(distF);
  Serial.print(" | Ang:");      Serial.print(anguloGyro);
  Serial.print(" | Obj:");      Serial.print(anguloObjetivo);
  Serial.print(" | obs:");      Serial.print(obsBiasNorm, 3);
  Serial.print(" | turn:");     Serial.print(turnHint);
  Serial.print(" | prio:");     Serial.print(piPriority ? 1 : 0);
  Serial.print(" | mem:");      Serial.print(piMemoryFrames);
  Serial.print(" | intr:");     Serial.print(piInteriorPass ? 1 : 0);
  Serial.print(" | giros:");    Serial.print(turnsCompleted);
  Serial.print("/");            Serial.println(TURNS_PER_RACE);
}
