#define lidarDebugSerial Serial
#include "thijs_rplidar.h"
#include <stdio.h>
#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include <esp_system.h>
#if !defined(ARDUINO_USB_CDC_ON_BOOT) || !ARDUINO_USB_CDC_ON_BOOT
#error "Activa USB CDC On Boot: Enabled para separar USB de UART0."
#endif
struct DistanceReading {
  uint16_t mm;
  uint32_t lastValidMs;
};
struct OpeningReading {
  uint16_t mm;
  uint32_t receivedMs;
  bool received;
};
enum TrackDirection { UNKNOWN, CCW, CW };
enum FirstWall { FIRST_NONE, FIRST_LEFT, FIRST_RIGHT };
enum TurnState { 
  NO_TURN, 
  TURNING_LEFT, TURNING_RIGHT 
};
enum RunState { RUNNING_LAPS, FINAL_APPROACH, FINISHED };

// Tramas @tipo,sesion,solicitud,arranque,dato*CRC16\n.
// CRC detecta corrupcion accidental; no es autenticacion criptografica.
#include <stdint.h>
#include <stdio.h>
#include <string.h>
struct CamFrame { char type; uint32_t session, id, boot; char data; };
inline uint16_t camCRC(const char *s, size_t n) {
  uint16_t crc=0xFFFF;
  for(size_t i=0;i<n;i++) {
    crc ^= (uint16_t)(uint8_t)s[i]<<8;
    for(int b=0;b<8;b++) crc=(crc&0x8000)?(uint16_t)((crc<<1)^0x1021):(uint16_t)(crc<<1);
  }
  return crc;
}
inline bool camHex(const char *s,int n,uint32_t &v) {
  v=0;
  for(int i=0;i<n;i++) {
    int d=s[i]>='0'&&s[i]<='9'?s[i]-'0':s[i]>='A'&&s[i]<='F'?s[i]-'A'+10:-1;
    if(d<0)return false;
    v=(v<<4)|(unsigned)d;
  }
  return true;
}
inline bool camParse(const char *s,CamFrame &f) {
  if(strlen(s)!=36 || s[0]!='@' || s[2]!=',' || s[11]!=',' || s[20]!=',' || s[29]!=',' || s[31]!='*')return false;
  uint32_t crc;
  if(!camHex(s+3,8,f.session)||!camHex(s+12,8,f.id)||!camHex(s+21,8,f.boot)||!camHex(s+32,4,crc))return false;
  if(camCRC(s+1,30)!=crc)return false;
  f.type=s[1];f.data=s[30];return true;
}
template<class Port> inline void camSend(Port &p,char type,uint32_t session,uint32_t id,uint32_t boot,char data) {
  char body[40],line[48];
  snprintf(body,sizeof(body),"%c,%08lX,%08lX,%08lX,%c",type,(unsigned long)session,(unsigned long)id,(unsigned long)boot,data);
  snprintf(line,sizeof(line),"@%s*%04X\n",body,(unsigned)camCRC(body,strlen(body)));
  p.print(line);
}
// Un @ nuevo resincroniza incluso si se perdio un salto de linea.
struct CamReader {
  char line[48]={}; size_t used=0; bool active=false; uint32_t bad=0;
  bool push(char c,CamFrame &f) {
    if(c=='@'){used=0;active=true;line[used++]=c;return false;}
    if(!active)return false;
    if(c=='\r')return false;
    if(c=='\n'){
      line[used]=0;active=false;
      if(camParse(line,f))return true;
      ++bad;return false;
    }
    if(used>=sizeof(line)-1){active=false;++bad;return false;}
    line[used++]=c;return false;
  }
};



// Tipos antes de cualquier funcion: compatibles con los prototipos de Arduino.








// UART0 libre porque Serial usa USB; Serial1 sigue siendo exclusivamente LiDAR.
static const int START_RX_PIN=17, START_TX_PIN=21;
static bool robotStarted=false;
static bool motionEnabled=false;
static uint32_t startAcceptedMs=0;
static const uint32_t START_DELAY_MS=500;
static uint32_t startSession=0, startCameraBoot=0, lastStartHello=0;
static CamReader startReader;
void setupStartLink() {
  startSession=esp_random(); if(!startSession)startSession=1;
  camSend(Serial0,'T',startSession,1,0,'?');
  lastStartHello=millis();
  Serial.println("[ARRANQUE] Sensores preparados. Esperando enlace con la camara...");
}
void pollStartLink() {
  CamFrame f; int budget=128;
  while(Serial0.available() && budget-->0) {
    if(!startReader.push((char)Serial0.read(),f) || f.session!=startSession)continue;
    if(f.type=='U' && f.id==1 && f.data=='R' && f.boot) {
      if(!robotStarted && startCameraBoot!=f.boot)
        Serial.println("[ARRANQUE] LISTO: pulsa y suelta el boton de la camara.");
      startCameraBoot=f.boot;
    } else if(f.type=='D' && !robotStarted && f.boot==startCameraBoot && startCameraBoot &&
              (f.data=='0' || f.data=='1')) {
      Serial.print("[BOTON CAM IO15] ");
      Serial.print(f.data=='0' ? "LOW/presionado" : "HIGH/suelto");
      Serial.print(" | estable="); Serial.print((f.id&1)?"HIGH":"LOW");
      Serial.print(" | habilitado="); Serial.print((f.id&2)?"SI":"NO");
      Serial.print(" | orden pendiente="); Serial.println((f.id&4)?"SI":"NO");
    } else if(f.type=='S' && (f.data=='0' || f.data=='1') && f.id && f.boot==startCameraBoot && startCameraBoot) {
      camSend(Serial0,'K',f.session,f.id,f.boot,'K');
      if(!robotStarted) {
        robotStarted=true;
        startAcceptedMs=millis();
        Serial.println("[ARRANQUE] Boton recibido. Esperando 500 ms.");
      }
    }
  }
  if(!robotStarted && millis()-lastStartHello>=500) {
    lastStartHello=millis();camSend(Serial0,'T',startSession,1,0,'?');
  }
}

// XIAO ESP32-C6
static const int SDA_PIN = 18; // D10
static const int SCL_PIN = 1;  // D1

// Cambia a -1.0f si deseas invertir el sentido del yaw.
// Convencion del control: derecha/CW positivo. Verificar con el sensor montado.
static const float YAW_SIGN = -1.0f;

// Giroscopio configurado a +/-500 grados/segundo.
static const float GYRO_SCALE = 65.6f;
// 500 muestras a 100 Hz: unos 5 s de calibracion si esta quieto.
// Tras preparar los sensores, esperar una pulsacion nueva del boton.
static const int IMU_CALIBRATION_SAMPLES = 500;

uint8_t bmiAddress = 0;
float biasZ = 0.0f;
float yaw = 0.0f;
float velocidadZ = 0.0f;

uint32_t ultimaMuestraUs = 0;
uint32_t ultimaImpresionMs = 0;
bool yawValido = true;
const char *imuInvalidReason = "NINGUNO";

bool leerRegistros(uint8_t reg, uint8_t *datos, size_t n) {
  Wire.beginTransmission(bmiAddress);
  Wire.write(reg);

  if (Wire.endTransmission(false) != 0) {
    return false;
  }

  size_t recibidos = Wire.requestFrom(bmiAddress, n);

  if (recibidos != n) {
    while (Wire.available()) Wire.read();
    return false;
  }

  for (size_t i = 0; i < n; i++) {
    datos[i] = Wire.read();
  }

  return true;
}

bool escribirRegistro(uint8_t reg, uint8_t valor) {
  Wire.beginTransmission(bmiAddress);
  Wire.write(reg);
  Wire.write(valor);
  return Wire.endTransmission() == 0;
}

void detenerPorError(const char *mensaje) {
  Serial.print("ERROR: ");
  Serial.println(mensaje);

  while (true) {
    delay(1000);
  }
}

bool buscarBMI160() {
  const uint8_t direcciones[] = {0x68, 0x69};

  for (uint8_t direccion : direcciones) {
    bmiAddress = direccion;

    uint8_t chipID = 0;

    if (leerRegistros(0x00, &chipID, 1) && chipID == 0xD1) {
      return true;
    }
  }

  return false;
}

// Devuelve:
//  1: muestra nueva
//  0: todavia no hay muestra nueva
// -1: error I2C
int leerGiroscopio(float &gx, float &gy, float &gz) {
  uint8_t estado;

  if (!leerRegistros(0x1B, &estado, 1)) {
    return -1;
  }

  // STATUS bit 6: datos nuevos del giroscopio.
  if (!(estado & 0x40)) {
    return 0;
  }

  uint8_t datos[6];

  if (!leerRegistros(0x0C, datos, sizeof(datos))) {
    return -1;
  }

  int16_t rawX = (int16_t)(
    (uint16_t)datos[0] | ((uint16_t)datos[1] << 8)
  );

  int16_t rawY = (int16_t)(
    (uint16_t)datos[2] | ((uint16_t)datos[3] << 8)
  );

  int16_t rawZ = (int16_t)(
    (uint16_t)datos[4] | ((uint16_t)datos[5] << 8)
  );

  gx = rawX / GYRO_SCALE;
  gy = rawY / GYRO_SCALE;
  gz = rawZ / GYRO_SCALE;

  return 1;
}

void calibrar() {
  Serial.println("Deja el sensor horizontal y completamente quieto.");
  Serial.println("Preparando calibracion...");
  // La inicializacion del sensor ya dio tiempo de asentamiento.

  // Descartar una muestra anterior a la calibracion.
  float gx, gy, gz;
  leerGiroscopio(gx, gy, gz);

  const int TOTAL = IMU_CALIBRATION_SAMPLES;
  int muestras = 0;

  double sumaZ = 0;
  float minimoZ = 10000;
  float maximoZ = -10000;

  uint32_t inicio = millis();
  bool avisoMovimiento = false;

  while (muestras < TOTAL) {
    int estado = leerGiroscopio(gx, gy, gz);

    if (estado < 0) {
      detenerPorError("Fallo I2C durante la calibracion.");
    }

    if (estado == 1) {
      // Rechazar movimientos evidentes.
      if (fabsf(gx) > 5.0f ||
          fabsf(gy) > 5.0f ||
          fabsf(gz) > 5.0f) {
        muestras = 0; sumaZ = 0; minimoZ = 10000; maximoZ = -10000;
        if (!avisoMovimiento) Serial.println("Movimiento: repitiendo calibracion automaticamente.");
        avisoMovimiento = true;
      } else {
        float nuevoMin = gz < minimoZ ? gz : minimoZ;
        float nuevoMax = gz > maximoZ ? gz : maximoZ;
        if (nuevoMax - nuevoMin > 2.0f) {
          // Empezar otra ventana, sin exigir reiniciar la placa.
          muestras = 1; sumaZ = gz; minimoZ = maximoZ = gz;
        } else {
          sumaZ += gz; minimoZ = nuevoMin; maximoZ = nuevoMax; muestras++;
        }
      }
    }

    if (millis() - inicio > 8000) {
      yawValido = false;
      imuInvalidReason = "CALIBRACION";
      ultimaMuestraUs = 0;
      Serial.println("Sin calibracion estable: arrancando con control LiDAR, sin IMU hasta reiniciar.");
      return;
    }

    delay(1);
  }

  biasZ = sumaZ / TOTAL;
  yaw = 0;
  ultimaMuestraUs = 0;
  yawValido = true;
  imuInvalidReason = "NINGUNO";

  Serial.print("Calibrado. Offset Z: ");
  Serial.print(biasZ, 4);
  Serial.println(" grados/s");
  Serial.println("IMU lista. Referencia inicial establecida.");
}

void imuSetup() {
  Serial.begin(115200);

  // El arranque de pista no depende de que una computadora abra el USB.

  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(100000);
  Wire.setTimeOut(5);

  delay(100);

  if (!buscarBMI160()) {
    detenerPorError(
      "BMI160 no encontrado en 0x68/0x69. Revisa conexiones."
    );
  }

  Serial.print("BMI160 encontrado en 0x");
  Serial.println(bmiAddress, HEX);

  // Reset del sensor.
  if (!escribirRegistro(0x7E, 0xB6)) {
    detenerPorError("No se pudo reiniciar el BMI160.");
  }

  delay(100);

  // Giroscopio en modo normal.
  if (!escribirRegistro(0x7E, 0x15)) {
    detenerPorError("No se pudo activar el giroscopio.");
  }

  delay(100);

  // 100 Hz, filtrado normal.
  if (!escribirRegistro(0x42, 0x28) ||
      !escribirRegistro(0x43, 0x02)) {
    detenerPorError("No se pudo configurar el giroscopio.");
  }

  delay(100);

  uint8_t configuracion, rango, modo;

  if (!leerRegistros(0x42, &configuracion, 1) ||
      !leerRegistros(0x43, &rango, 1) ||
      !leerRegistros(0x03, &modo, 1) ||
      configuracion != 0x28 ||
      (rango & 0x07) != 0x02 ||
      (modo & 0x0C) != 0x04) {
    detenerPorError("La configuracion del BMI160 no se confirmo.");
  }

  calibrar();
}

void imuUpdate() {
  static uint32_t lastPollUs = 0;
  uint32_t pollUs = micros();
  if (pollUs - lastPollUs < 5000) return;
  lastPollUs = pollUs;
  float gx, gy, gz;
  int estado = leerGiroscopio(gx, gy, gz);
  uint32_t ahoraUs = micros();

  if (estado < 0) {
    if (yawValido) {
      Serial.println("ERROR I2C: control LiDAR de respaldo hasta reiniciar.");
    }

    yawValido = false;
    imuInvalidReason = "I2C";
    ultimaMuestraUs = 0;
  }

  if (estado == 1) {
    velocidadZ = (gz - biasZ) * YAW_SIGN;

    if (ultimaMuestraUs != 0) {
      uint32_t intervaloUs = ahoraUs - ultimaMuestraUs;

      // No fingir continuidad si hubo un hueco grande de lecturas.
      if (intervaloUs > 50000) {
        yawValido = false;
        imuInvalidReason = "HUECO_MUESTRAS_50MS";
      } else if (yawValido) {
        float dt = intervaloUs * 0.000001f;
        yaw += velocidadZ * dt;

        // Salida entre -180 y +180 grados.
        while (yaw >= 180.0f) yaw -= 360.0f;
        while (yaw < -180.0f) yaw += 360.0f;
      }
    }

    ultimaMuestraUs = ahoraUs;
  }

  if (ultimaMuestraUs != 0 &&
      (uint32_t)(ahoraUs - ultimaMuestraUs) > 100000) {
    yawValido = false;
    imuInvalidReason = "SIN_MUESTRAS_100MS";
  }

}


// Solo observacion: no cambia las decisiones de navegacion.
static const bool DEBUG_CURVAS_CW = true;
static const uint32_t DEBUG_CURVAS_INTERVAL_MS = 500;


// Ajustes iniciales para pista. Yaw y objetivos: CW positivo, CCW negativo.
static const float KP_DISTANCE_YAW = 0.025f; // grados por mm de error de rayo
static const float MAX_DISTANCE_YAW = 8.0f;
static const float KP_YAW = 1.5f;            // grados de servo / grado de yaw
static const float KD_YAW_RATE = 0.12f;      // amortiguacion por grados/s
static const float DISTANCE_SLEW_DPS = 12.0f;
static const float TURN_YAW_TOL_DEG = 4.0f;
static const float TURN_RATE_TOL_DPS = 35.0f;
static const uint32_t TURN_SETTLE_MS = 60;
static const float DRIFT_MAX_DPS = 0.5f;
static const float DRIFT_TIME_CONSTANT_S = 8.0f;
static const uint32_t WALL_PAIR_MAX_AGE_MS = 250;
static const uint32_t WALL_PAIR_MAX_SKEW_MS = 80;
float corridorYaw = 0, yawOffset = 0, distanceYaw = 0;
float commandedYaw = 0;
uint32_t turnSettledSince = 0;
bool imuTurnActive = false;
bool driftReferenceOK = false;
float wrapYaw(float x) {
  while (x >= 180) x -= 360;
  while (x < -180) x += 360;
  return x;
}
float clampValue(float x,float lo,float hi) { return x < lo ? lo : x > hi ? hi : x; }
float correctedYaw() { return wrapYaw(yaw + yawOffset); }

// --- Pines ESP32-C6 ---
#define LIDAR_RX_PIN 19  
#define LIDAR_TX_PIN 16  
#define MOTOR_PIN    20  
#define SERVO_PIN     0  

// --- TB6612FNG: motor de avance ---
#define PWMA 22
#define AIN1 23
#define AIN2 2

// ==========================================
// --- CONSTANTES DE DETECCIÓN DE CURVA ---
// ==========================================
#define FRONT_TURN_DISTANCE_MM 1200 
#define LOOK_OPEN_DISTANCE_MM 1200
#define LOOK_OPEN_CONFIRM_MS 100
#define SIDE_WALL_CLOSED_MM 900   
#define LOOK_WALL_CLOSED_MM 1200  
#define BACK_CLEAR_DISTANCE_MM 1500

#define PARALLEL_TOLERANCE_MM 60 
#define MAX_TURN_TIMEOUT_MS 1400 // Ampliado ligeramente para evitar salidas forzadas prematuras en CW
#define NEW_HALLWAY_CLEAR_MM 1500

// Setpoint Acotado (Clamping)
#define SETPOINT_MIN_MM 325
#define SETPOINT_MAX_MM 375
// ==========================================

// --- Otras Configuraciones ---
#define TOLERANCE_DEG 2 // Estrechado a 2° para eliminar el sesgo de barrido del LiDAR
#define SIDE_PID_TOLERANCE_DEG 10
#define MAX_DISTANCE_MM 6000
#define HOLD_LAST_VALID_MS 300
#define PRINT_INTERVAL_MS 50
#define PID_INTERVAL_MS 20 
#define PID_MAX_VALID_DISTANCE_MM 1000
#define PID_STALE_MS 250

// Velocidades dinámicas
#define DRIVE_PWM_NORMAL 255
#define DRIVE_PWM_SLOW   255


// --- FIN DE RECORRIDO: circuito de 4 curvas por vuelta ---
#define LAPS_TO_COMPLETE 3
#define CORNERS_PER_LAP 4
// Ajustar en pista: tiempo desde la salida de curva 12 hasta la zona inicial.
// 1000 ms es SOLO un valor inicial de calibracion; no localiza el inicio.
#define FINAL_APPROACH_MS 800UL
#define FINAL_APPROACH_PWM DRIVE_PWM_NORMAL

// --- CONSTANTES PID HÍBRIDO ---
#define KP_POS       0.05f 
// Ganancias de paralelismo editables segun el sentido detectado.
static const float KP_PARALLEL_FIRST = 2.5f; // Primer carril, sentido UNKNOWN
static const float KP_PARALLEL_CCW   = 2.5f; // Antihorario
static const float KP_PARALLEL_CW    = 2.5f; // Horario
#define KI           0.0f
#define KD           0.001f

#define SERVO_LEFT 60
#define SERVO_CENTER 90
#define SERVO_RIGHT 120

RPlidar lidar(Serial1);
bool lidarStarted = false;



// --- Cono frontal ---
DistanceReading frontReading = {0, 0};       
DistanceReading frontLeftReading = {0, 0};   
DistanceReading frontRightReading = {0, 0};  

// --- Cono trasero ---
DistanceReading backReading = {0, 0};        
DistanceReading back170Reading = {0, 0};     
DistanceReading back190Reading = {0, 0};     

// --- Laterales y Lookaheads ---
DistanceReading leftReading = {0, 0};        
DistanceReading leftLookReading = {0, 0};    // 300°
DistanceReading rightReading = {0, 0};       
DistanceReading rightLookReading = {0, 0};   // 60°   

// --- Lookaheads Traseros (Paralelismo simétrico exacto) ---
DistanceReading leftBackLookReading = {0, 0};  // 240° (Espejo perfecto de 120°)
DistanceReading rightBackLookReading = {0, 0}; // 120°

DistanceReading pidLeftReading = {0, 0};
DistanceReading pidRightReading = {0, 0};

float pidIntegral = 0.0f;
float pidLastError = 0.0f;
float smoothedError = 0.0f; // Global para evitar arrastrar residuos de la curva
uint32_t pidLastMs = 0;

int servoPosition = SERVO_CENTER;

// --- SETPOINT DINÁMICO ACOTADO ---
float targetWallDistance = 375.0f; 

// --- ESTADOS DE PISTA ---

TrackDirection trackDir = UNKNOWN;

// Solo primer carril: referencia independiente del setpoint de las curvas.

FirstWall firstWall = FIRST_NONE;
float firstWallTarget = 0.0f;
DistanceReading firstLeftFront = {0, 0};   // 300 grados
DistanceReading firstLeftBack = {0, 0};    // 240 grados
DistanceReading firstRightFront = {0, 0};  // 60 grados
DistanceReading firstRightBack = {0, 0};   // 120 grados


// --- MÁQUINA DE ESTADOS ---

TurnState turnState = NO_TURN;

uint32_t turnStartTime = 0;


RunState runState = RUNNING_LAPS;
uint16_t completedCorners = 0;
uint32_t finalApproachStartMs = 0;
// Requerir que la apertura anterior se cierre antes de armar otra curva.
bool cornerRearmPending = false;

bool leftLookSawWall = false;
bool rightLookSawWall = false;
uint32_t leftOpenSinceMs = 0;
uint32_t rightOpenSinceMs = 0;

bool angleMatches(float angle, float target) {
  float difference = angle - target;
  if (difference > 180.0f) difference -= 360.0f;
  if (difference < -180.0f) difference += 360.0f;
  return (difference >= -TOLERANCE_DEG && difference <= TOLERANCE_DEG);
}

bool angleMatchesSidePID(float angle, float target) {
  float difference = angle - target;
  if (difference > 180.0f) difference -= 360.0f;
  if (difference < -180.0f) difference += 360.0f;
  return (difference >= -SIDE_PID_TOLERANCE_DEG &&
          difference <= SIDE_PID_TOLERANCE_DEG);
}

void saveReading(DistanceReading &reading, uint16_t distance) {
  reading.mm = distance;
  reading.lastValidMs = millis();
}


// Prototipos explicitos: Arduino no debe generarlos antes de este struct.
bool openingFresh(const OpeningReading &r);
bool openingDetected(const OpeningReading &r);
bool openingWall(const OpeningReading &r);

OpeningReading leftOpening = {0, 0, false};
OpeningReading rightOpening = {0, 0, false};

bool openingFresh(const OpeningReading &r) {
  return r.received && millis() - r.receivedMs <= HOLD_LAST_VALID_MS;
}
bool openingDetected(const OpeningReading &r) {
  return openingFresh(r) && (r.mm == 0 || r.mm >= LOOK_OPEN_DISTANCE_MM);
}
bool openingWall(const OpeningReading &r) {
  return openingFresh(r) && r.mm > 0 && r.mm < LOOK_OPEN_DISTANCE_MM;
}

uint32_t lidarPointCount=0, lidarLastPointMs=0;
void lidarDataHandler(RPlidar* lidarPtr, uint16_t distance, uint16_t angle_q6,
                      uint8_t newRotation, int8_t quality) {
  // Atender IMU tambien mientras la libreria vacia una tanda de puntos.
  ++lidarPointCount; lidarLastPointMs=millis();
  imuUpdate();
  float receivedAngle = angle_q6 * 0.015625f;
  if (angleMatches(receivedAngle, 300)) leftOpening = {distance, millis(), true};
  if (angleMatches(receivedAngle, 60)) rightOpening = {distance, millis(), true};
  // Antes la libreria filtraba los ceros. Mantener ese comportamiento
  // para primer carril, PID, pared opuesta y finalizacion del giro.
  if (distance == 0) return;
  // Las distancias fuera de rango invalidan la referencia del primer carril.
  // Las lecturas originales del resto de la navegacion no cambian.
  if (trackDir == UNKNOWN) {
    float firstAngle = angle_q6 * 0.015625f;
    uint16_t valid = distance > 0 && distance <= MAX_DISTANCE_MM ? distance : 0;
    if (angleMatches(firstAngle, 300)) saveReading(firstLeftFront, valid);
    if (angleMatches(firstAngle, 240)) saveReading(firstLeftBack, valid);
    if (angleMatches(firstAngle, 60)) saveReading(firstRightFront, valid);
    if (angleMatches(firstAngle, 120)) saveReading(firstRightBack, valid);
  }
  if (distance == 0 || distance > MAX_DISTANCE_MM) return;

  float angle = angle_q6 * 0.015625f;

  if (angleMatches(angle, 0))   saveReading(frontReading, distance);
  if (angleMatches(angle, 340)) saveReading(frontLeftReading, distance);
  if (angleMatches(angle, 20))  saveReading(frontRightReading, distance);
  
  if (angleMatches(angle, 180)) saveReading(backReading, distance);
  if (angleMatches(angle, 170)) saveReading(back170Reading, distance);
  if (angleMatches(angle, 190)) saveReading(back190Reading, distance);

  if (angleMatchesSidePID(angle, 270)) {
    saveReading(leftReading, distance);
    if (distance <= PID_MAX_VALID_DISTANCE_MM) saveReading(pidLeftReading, distance);
  }
  if (angleMatchesSidePID(angle, 90)) {
    saveReading(rightReading, distance);
    if (distance <= PID_MAX_VALID_DISTANCE_MM) saveReading(pidRightReading, distance);
  }

  if (angleMatches(angle, 300)) saveReading(leftLookReading, distance);
  if (angleMatches(angle, 60))  saveReading(rightLookReading, distance);
  
  if (angleMatches(angle, 240)) saveReading(leftBackLookReading, distance);
  if (angleMatches(angle, 120)) saveReading(rightBackLookReading, distance);
}

uint16_t getDistance(DistanceReading &reading) {
  if (millis() - reading.lastValidMs > HOLD_LAST_VALID_MS) return 0;
  return reading.mm;
}

uint16_t getPidDistance(DistanceReading &reading) {
  if (millis() - reading.lastValidMs > PID_STALE_MS) return 0;
  return reading.mm;
}

// Promedio de los rayos validos. Cero significa ausencia de medida.
float firstSideDistance(uint16_t front, uint16_t back) {
  if (front && back) return ((float)front + back) * 0.5f;
  return front ? (float)front : (float)back;
}

bool selectFirstWall() {
  if (firstWall != FIRST_NONE) return true;
  float left = firstSideDistance(getDistance(firstLeftFront), getDistance(firstLeftBack));
  float right = firstSideDistance(getDistance(firstRightFront), getDistance(firstRightBack));
  if (left == 0 && right == 0) return false;
  // En empate exacto se elige derecha. La eleccion queda fija hasta la primera curva.
  if (right > 0 && right >= left) {
    firstWall = FIRST_RIGHT;
    firstWallTarget = right;
  } else {
    firstWall = FIRST_LEFT;
    firstWallTarget = left;
  }
  Serial.print("Primer carril: seguir pared ");
  Serial.print(firstWall == FIRST_RIGHT ? "DERECHA" : "IZQUIERDA");
  Serial.print(" | referencia de rayos: ");
  Serial.print(firstWallTarget, 1);
  Serial.println(" mm");
  return true;
}

bool firstLaneError(float &error) {
  if (!selectFirstWall()) return false;
  uint16_t front = getDistance(firstWall == FIRST_RIGHT ? firstRightFront : firstLeftFront);
  uint16_t back = getDistance(firstWall == FIRST_RIGHT ? firstRightBack : firstLeftBack);
  if (!front && !back) return false;
  float distance = firstSideDistance(front, back);
  float parallel = front && back ? (float)front - back : 0.0f;
  float sign = firstWall == FIRST_RIGHT ? 1.0f : -1.0f;
  error = sign * (distance - firstWallTarget + KP_PARALLEL_FIRST * parallel);
  return true;
}

void setSteeringServo(int angle) {
  if (angle < SERVO_LEFT) angle = SERVO_LEFT;
  if (angle > SERVO_RIGHT) angle = SERVO_RIGHT;

  uint32_t pulseUs = map(angle, 0, 180, 500, 2400);
  uint32_t duty = (pulseUs * 65535UL) / 20000UL;
  ledcWrite(SERVO_PIN, duty);
  servoPosition = angle;
}

void driveForward(int speed) {
  if (!motionEnabled) return;
  if (runState == FINISHED) return;
  // No arrancar a ciegas mientras falta la referencia del primer carril.
  if (trackDir == UNKNOWN && turnState == NO_TURN && !selectFirstWall()) {
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, LOW);
    analogWrite(PWMA, 0);
    return;
  }
  if (runState == FINAL_APPROACH) speed = FINAL_APPROACH_PWM;
  digitalWrite(AIN1, HIGH);
  digitalWrite(AIN2, LOW);
  analogWrite(PWMA, speed);
}

void stopDriveMotor() {
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, LOW);
  analogWrite(PWMA, 0);
}

bool lookIsOpen(uint16_t distance) {
  return (distance == 0 || distance >= LOOK_OPEN_DISTANCE_MM);
}

float sideControlDistance(const DistanceReading &front, const DistanceReading &back);
float sideControlDistance(const DistanceReading &front, const DistanceReading &back) {
  uint32_t now = millis();
  uint32_t af = now - front.lastValidMs, ab = now - back.lastValidMs;
  uint32_t skew = af > ab ? af - ab : ab - af;
  // Promediar solo una pareja reciente; conservar el rayo frontal como respaldo.
  if (front.mm && back.mm && af <= WALL_PAIR_MAX_AGE_MS &&
      ab <= WALL_PAIR_MAX_AGE_MS && skew <= WALL_PAIR_MAX_SKEW_MS)
    return ((float)front.mm + back.mm) * 0.5f;
  return af <= HOLD_LAST_VALID_MS ? front.mm : 0;
}

// Instantanea de observacion; nunca se usa para dirigir el robot.
const char *lastTurnReason = "NINGUNO";
float lastTurnYawError = 0;
uint32_t lastTurnDiagnosticMs = 0;

void finishTurnAndCaptureSetpoint(TrackDirection dir, bool timedOut = false) {
  if (runState != RUNNING_LAPS || turnState == NO_TURN) return;
  lastTurnReason = timedOut ? "TIMEOUT" : imuTurnActive && yawValido ? "YAW" : "LIDAR";
  lastTurnYawError = yawValido ? wrapYaw(corridorYaw - correctedYaw()) : NAN;
  lastTurnDiagnosticMs = millis();
  imuTurnActive = false;
  distanceYaw = 0;
  trackDir = dir;
  completedCorners++;
  Serial.print("Curvas completadas: ");
  Serial.print(completedCorners);
  Serial.print(" / ");
  Serial.print(LAPS_TO_COMPLETE * CORNERS_PER_LAP);
  Serial.println(lastTurnReason);
  leftLookSawWall = false;
  rightLookSawWall = false;
  leftOpenSinceMs = 0;
  rightOpenSinceMs = 0;
  cornerRearmPending = true;
  
  float rawCaptured = 0;
  if (dir == CCW) {
    rawCaptured = sideControlDistance(rightLookReading, rightBackLookReading);
  } else {
    rawCaptured = sideControlDistance(leftLookReading, leftBackLookReading);
  }

  if (rawCaptured > 0 && rawCaptured < 2000) {
    // Mantener la distancia de entrada a este carril, como en el primero.
    // No forzar 325-375 mm: eso solicitaba una diagonal hacia la pared.
    targetWallDistance = rawCaptured;
  }

  turnState = NO_TURN;
  setSteeringServo(SERVO_CENTER);
  pidIntegral = 0.0f;
  pidLastError = 0.0f;
  smoothedError = 0.0f; // Reseteamos el filtro para evitar impulsos residuales al salir de curva
  pidLastMs = millis();
  if (completedCorners >= LAPS_TO_COMPLETE * CORNERS_PER_LAP) {
    runState = FINAL_APPROACH;
    finalApproachStartMs = millis();
    driveForward(FINAL_APPROACH_PWM);
    Serial.println("Aproximacion final: tiempo calibrable, no posicion medida.");
  }
}

void updateRunCompletion() {
  if (runState == FINAL_APPROACH &&
      millis() - finalApproachStartMs >= FINAL_APPROACH_MS) {
    runState = FINISHED;
    stopDriveMotor();
    setSteeringServo(SERVO_CENTER);
    Serial.println("Recorrido terminado. Reiniciar para volver a arrancar.");
  }
}

void checkForTurn() {
  if (runState != RUNNING_LAPS) return;
  if (trackDir == UNKNOWN && turnState == NO_TURN && firstWall == FIRST_NONE) return;
  uint32_t now = millis();
  
  uint16_t front = getDistance(frontReading);
  uint16_t frontLeft = getDistance(frontLeftReading);
  uint16_t frontRight = getDistance(frontRightReading);
  
  uint16_t back = getDistance(backReading);
  uint16_t back170 = getDistance(back170Reading);
  uint16_t back190 = getDistance(back190Reading);
  
  uint16_t leftLook = getDistance(leftLookReading);
  uint16_t rightLook = getDistance(rightLookReading);

  uint16_t leftWall = getDistance(leftReading);
  uint16_t rightWall = getDistance(rightReading);
  
  uint16_t leftBackLook = getDistance(leftBackLookReading);
  uint16_t rightBackLook = getDistance(rightBackLookReading);

  bool frontIsClear = (front == 0 || front > NEW_HALLWAY_CLEAR_MM) &&
                      (frontLeft == 0 || frontLeft > NEW_HALLWAY_CLEAR_MM) &&
                      (frontRight == 0 || frontRight > NEW_HALLWAY_CLEAR_MM);

  if (imuTurnActive && yawValido &&
      (turnState == TURNING_LEFT || turnState == TURNING_RIGHT)) {
    float error = wrapYaw(corridorYaw - correctedYaw());
    if (fabsf(error) <= TURN_YAW_TOL_DEG && fabsf(velocidadZ) <= TURN_RATE_TOL_DPS) {
      if (!turnSettledSince) turnSettledSince = now;
      if (now - turnSettledSince >= TURN_SETTLE_MS) {
        finishTurnAndCaptureSetpoint(turnState == TURNING_LEFT ? CCW : CW);
      }
    } else turnSettledSince = 0;
    // Mantener el timeout original como respaldo; se informa en el monitor.
    if (turnState != NO_TURN && now - turnStartTime > MAX_TURN_TIMEOUT_MS)
      finishTurnAndCaptureSetpoint(turnState == TURNING_LEFT ? CCW : CW, true);
    return;
  }

  if (turnState == TURNING_LEFT) {
    if (millis() - turnStartTime > MAX_TURN_TIMEOUT_MS) {
      finishTurnAndCaptureSetpoint(CCW, true);
      return;
    }
    if (frontIsClear) {
      if (rightLook > 0 && rightBackLook > 0) {
        if (abs((int)rightLook - (int)rightBackLook) <= PARALLEL_TOLERANCE_MM) {
          finishTurnAndCaptureSetpoint(CCW);
        }
      }
    }
    return;
  }
  else if (turnState == TURNING_RIGHT) {
    if (millis() - turnStartTime > MAX_TURN_TIMEOUT_MS) {
      finishTurnAndCaptureSetpoint(CW, true);
      return;
    }
    if (frontIsClear) {
      // Misma lógica simétrica exacta usando los rayos espejo 300° y 240°
      if (leftLook > 0 && leftBackLook > 0) {
        if (abs((int)leftLook - (int)leftBackLook) <= PARALLEL_TOLERANCE_MM) {
          finishTurnAndCaptureSetpoint(CW);
        }
      }
    }
    return;
  }

  bool leftOpen = openingDetected(leftOpening);
  bool rightOpen = openingDetected(rightOpening);
  // Una interrupcion de datos no confirma apertura ni rearma una curva.
  if (!openingFresh(leftOpening)) leftOpenSinceMs = 0;
  if (!openingFresh(rightOpening)) rightOpenSinceMs = 0;

  if (cornerRearmPending) {
    bool previousOpeningClosed = (trackDir == CCW) ? openingWall(leftOpening) : openingWall(rightOpening);
    if (!previousOpeningClosed) return;
    cornerRearmPending = false;
  }

  if (openingWall(leftOpening)) leftLookSawWall = true;
  if (openingWall(rightOpening)) rightLookSawWall = true;

  if (leftLookSawWall && leftOpen) {
    if (leftOpenSinceMs == 0) leftOpenSinceMs = now;
  } else {
    leftOpenSinceMs = 0;
  }

  if (rightLookSawWall && rightOpen) {
    if (rightOpenSinceMs == 0) rightOpenSinceMs = now;
  } else {
    rightOpenSinceMs = 0;
  }

  bool leftConfirmed = leftOpenSinceMs != 0 && (now - leftOpenSinceMs >= LOOK_OPEN_CONFIRM_MS);
  bool rightConfirmed = rightOpenSinceMs != 0 && (now - rightOpenSinceMs >= LOOK_OPEN_CONFIRM_MS);

  bool isFrontBlocked = (front != 0 && front < FRONT_TURN_DISTANCE_MM);
  bool isFrontLeftBlocked = (frontLeft != 0 && frontLeft < FRONT_TURN_DISTANCE_MM);
  bool isFrontRightBlocked = (frontRight != 0 && frontRight < FRONT_TURN_DISTANCE_MM);
  bool frontBlocked = isFrontBlocked || isFrontLeftBlocked || isFrontRightBlocked;

  bool isBackClear = (back == 0 || back > BACK_CLEAR_DISTANCE_MM);
  bool isBack170Clear = (back170 == 0 || back170 > BACK_CLEAR_DISTANCE_MM);
  bool isBack190Clear = (back190 == 0 || back190 > BACK_CLEAR_DISTANCE_MM);
  bool backClear = isBackClear && isBack170Clear && isBack190Clear;

  bool rightWallClosed = (rightWall != 0 && rightWall < SIDE_WALL_CLOSED_MM) && 
                         (rightLook != 0 && rightLook < LOOK_WALL_CLOSED_MM);

  bool leftWallClosed = (leftWall != 0 && leftWall < SIDE_WALL_CLOSED_MM) && 
                        (leftLook != 0 && leftLook < LOOK_WALL_CLOSED_MM);

  if (!frontBlocked || !backClear) return;

  if (leftConfirmed && rightWallClosed) {
    corridorYaw = wrapYaw(corridorYaw - 90.0f);
    imuTurnActive = true; turnSettledSince = 0; distanceYaw = 0;
    turnState = TURNING_LEFT;
    turnStartTime = millis();
    setSteeringServo(SERVO_LEFT);
    driveForward(DRIVE_PWM_SLOW);
  } 
  else if (rightConfirmed && leftWallClosed) {
    corridorYaw = wrapYaw(corridorYaw + 90.0f);
    imuTurnActive = true; turnSettledSince = 0; distanceYaw = 0;
    turnState = TURNING_RIGHT;
    turnStartTime = millis();
    setSteeringServo(SERVO_RIGHT);
    driveForward(DRIVE_PWM_SLOW);
  }
}

// Se ejecuta DESPUES de checkForTurn(). No modifica estados, lecturas ni motores.
void diagnosticarCurvasCW() {
  static char line[1024];
  static size_t pending = 0, sent = 0;
  if (DEBUG_CURVAS_CW && Serial && sent < pending) {
    int room = Serial.availableForWrite();
    if (room > 0) {
      size_t count = pending - sent;
      if (count > (size_t)room) count = (size_t)room;
      if (count > 32) count = 32;
      sent += Serial.write((const uint8_t*)line + sent, count);
    }
    return;
  }
  if (!DEBUG_CURVAS_CW || !Serial || runState != RUNNING_LAPS) return;
  uint32_t now = millis();
  uint16_t f = getDistance(frontReading);
  uint16_t fl = getDistance(frontLeftReading);
  uint16_t fr = getDistance(frontRightReading);
  uint16_t b = getDistance(backReading);
  uint16_t b170 = getDistance(back170Reading);
  uint16_t b190 = getDistance(back190Reading);
  uint16_t r60 = getDistance(rightLookReading);
  uint16_t l270 = getDistance(leftReading);
  uint16_t l300 = getDistance(leftLookReading);
  uint16_t l240 = getDistance(leftBackLookReading);
  bool frontOK = (f && f < FRONT_TURN_DISTANCE_MM) ||
                 (fl && fl < FRONT_TURN_DISTANCE_MM) ||
                 (fr && fr < FRONT_TURN_DISTANCE_MM);
  bool backOK = (!b || b > BACK_CLEAR_DISTANCE_MM) &&
                (!b170 || b170 > BACK_CLEAR_DISTANCE_MM) &&
                (!b190 || b190 > BACK_CLEAR_DISTANCE_MM);
  bool oppositeOK = l270 && l270 < SIDE_WALL_CLOSED_MM &&
                    l300 && l300 < LOOK_WALL_CLOSED_MM;
  bool open = openingDetected(rightOpening);
  uint32_t openMs = rightOpenSinceMs ? now - rightOpenSinceMs : 0;
  bool openingOK = rightLookSawWall && open && rightOpenSinceMs &&
                   openMs >= LOOK_OPEN_CONFIRM_MS;

  static uint32_t lastPrint = 0;
  static bool printed = false;
  static TurnState lastPrintedState = NO_TURN;
  bool entering = turnState == TURNING_RIGHT && lastPrintedState != TURNING_RIGHT;
  if (printed && !entering && now - lastPrint < DEBUG_CURVAS_INTERVAL_MS) return;

  // Encolar una muestra; enviarla en bloques pequenos sin esperar espacio USB.
  int n = snprintf(line, sizeof(line),
    "[CW-DIAG] t=%lu dir=%s estado=%s F=%u/%u/%u B=%u/%u/%u "
    "R60=%u L270/300/240=%u/%u/%u OK(frente/atras/opuesta/apertura/rearme)=%u%u%u%u%u "
    "vioPared=%u abreMs=%lu edadR60=%lu edadL300=%lu RX60=%s/%u Y=%.1f OBJ=%.1f OFF=%.2f IMU=%u DRIFT=%u\n",
    (unsigned long)now, trackDir == CW ? "CW" : trackDir == CCW ? "CCW" : "UNKNOWN",
    turnState == TURNING_RIGHT ? "GIRANDO" : turnState == TURNING_LEFT ? "IZQUIERDA" : "RECTO",
    (unsigned)f,(unsigned)fl,(unsigned)fr,(unsigned)b,(unsigned)b170,(unsigned)b190,
    (unsigned)r60,(unsigned)l270,(unsigned)l300,(unsigned)l240,
    (unsigned)frontOK,(unsigned)backOK,(unsigned)oppositeOK,(unsigned)openingOK,(unsigned)!cornerRearmPending,
    (unsigned)rightLookSawWall,(unsigned long)openMs,
    (unsigned long)(now-rightLookReading.lastValidMs),
    (unsigned long)(now-leftLookReading.lastValidMs),
    !openingFresh(rightOpening) ? "SIN_DATOS" : rightOpening.mm == 0 ? "CERO" : "MEDIDA",
    (unsigned)rightOpening.mm, (double)correctedYaw(), (double)commandedYaw,
    (double)yawOffset, (unsigned)yawValido, (unsigned)driftReferenceOK);
  if (n <= 0 || n >= (int)sizeof(line)) return;
  bool right = trackDir == CCW || (trackDir == UNKNOWN && firstWall == FIRST_RIGHT);
  bool selected = trackDir != UNKNOWN || firstWall != FIRST_NONE;
  const DistanceReading &wf = right ? rightLookReading : leftLookReading;
  const DistanceReading &wb = right ? rightBackLookReading : leftBackLookReading;
  uint32_t ageF = now - wf.lastValidMs, ageB = now - wb.lastValidMs;
  uint32_t skew = ageF > ageB ? ageF - ageB : ageB - ageF;
  bool wallOK = selected && wf.mm > 100 && wb.mm > 100 && wf.mm < 1000 && wb.mm < 1000 &&
    ageF <= WALL_PAIR_MAX_AGE_MS && ageB <= WALL_PAIR_MAX_AGE_MS &&
    skew <= WALL_PAIR_MAX_SKEW_MS && turnState == NO_TURN &&
    openingWall(right ? rightOpening : leftOpening);
  float wallAngle = wallOK ? atan2f(1.7320508f * (right ? (float)wb.mm-wf.mm : (float)wf.mm-wb.mm),
    (float)wf.mm+wb.mm)*57.2957795f : NAN;
  float relativeYaw = yawValido ? wrapYaw(correctedYaw()-corridorYaw) : NAN;
  float difference = wallOK && yawValido ? wrapYaw(wallAngle-relativeYaw) : NAN;
  float distance = selected ? sideControlDistance(wf,wb) : NAN;
  float reference = trackDir == UNKNOWN ? firstWallTarget : targetWallDistance;
  int extra = snprintf(line+n, sizeof(line)-(size_t)n,
    "[RUMBO] t=%lu curva=%u lado=%s YREL=%.2f PARED=%.2f DIF=%.2f OFF=%.2f "
    "PARED_OK=%u DIST=%.0f REF=%.0f AJUSTE=%.2f EDAD=%lu/%lu "
    "FIN=%s ERR_FIN=%.2f tFIN=%lu\n",
    (unsigned long)now,(unsigned)completedCorners,selected ? (right ? "DER" : "IZQ") : "NINGUNO",
    (double)relativeYaw,(double)wallAngle,(double)difference,(double)yawOffset,
    (unsigned)wallOK,(double)distance,(double)reference,(double)distanceYaw,
    (unsigned long)ageF,(unsigned long)ageB,lastTurnReason,(double)lastTurnYawError,
    (unsigned long)lastTurnDiagnosticMs);
  if (extra > 0 && extra < (int)(sizeof(line)-(size_t)n)) n += extra;
  pending = (size_t)n;
  sent = 0;
  lastPrint=now;printed=true;lastPrintedState=turnState;
}

void updateSteeringLegacy() {
  if (runState == FINISHED) return;
  if (turnState == TURNING_LEFT || turnState == TURNING_RIGHT) {
    return; 
  }

  uint32_t now = millis();
  if (now - pidLastMs < PID_INTERVAL_MS) return;

  float dt = (now - pidLastMs) / 1000.0f;
  pidLastMs = now;

  float rawError = 0.0f;

  if (trackDir == UNKNOWN) {
    if (!firstLaneError(rawError)) {
      // Sin datos del lado elegido: no cambiar de pared ni inventar distancia.
      stopDriveMotor();
      setSteeringServo(SERVO_CENTER);
      pidIntegral = 0.0f;
      pidLastError = 0.0f;
      smoothedError = 0.0f;
      return;
    }
  } 
  else if (trackDir == CCW) {
    uint16_t rLook = getDistance(rightLookReading);       // 60°
    uint16_t rBackLook = getDistance(rightBackLookReading); // 120°
    
    if (rLook != 0 && rBackLook != 0) {
      float positionError = (float)rLook - targetWallDistance;
      float parallelError = (float)rLook - (float)rBackLook;
      rawError = positionError + (KP_PARALLEL_CCW * parallelError);
    }
  } 
  else if (trackDir == CW) {
    uint16_t lLook = getDistance(leftLookReading);         // 300° (Espejo exacto de 60°)
    uint16_t lBackLook = getDistance(leftBackLookReading); // 240° (Espejo exacto de 120°)
    
    if (lLook != 0 && lBackLook != 0) {
      // Estructura matemática perfectamente simétrica a CCW con inversión de signo de espejo
      float positionError = targetWallDistance - (float)lLook;
      float parallelError = (float)lBackLook - (float)lLook;
      rawError = positionError + (KP_PARALLEL_CW * parallelError);
    }
  }

  smoothedError = (0.6f * smoothedError) + (0.4f * rawError);

  pidIntegral += smoothedError * dt;
  float derivative = (smoothedError - pidLastError) / dt;
  pidLastError = smoothedError;

  float output = (KP_POS * smoothedError) + (KI * pidIntegral) + (KD * derivative);
  int targetAngle = SERVO_CENTER + (int)output;
  setSteeringServo(targetAngle);
  
  if (turnState == NO_TURN) {
    driveForward(DRIVE_PWM_NORMAL); 
  } else {
    driveForward(DRIVE_PWM_SLOW);   
  }
}


// Correccion de deriva: solo parejas nuevas, estables y pared sin apertura.
// Sus tiempos son de recepcion del callback, no tiempos internos del LiDAR.
void correctDrift(bool right, const DistanceReading &f, const DistanceReading &b);
void correctDrift(bool right, const DistanceReading &f, const DistanceReading &b) {
  static uint32_t prevF=0, prevB=0, firstGood=0, lastGood=0;
  static float filteredResidual=0;
  static bool previousSide=false, initialized=false;
  static uint16_t previousCorner=0;
  static unsigned goodPairs=0;
  uint32_t now=millis(), af=now-f.lastValidMs, ab=now-b.lastValidMs;
  uint32_t skew=af>ab ? af-ab : ab-af;
  driftReferenceOK=false;
  // Resetear por cambio real de contexto, no por esperar el otro rayo.
  if (!yawValido || turnState!=NO_TURN || cornerRearmPending ||
      previousCorner!=completedCorners || (initialized && previousSide!=right) ||
      fabsf(velocidadZ)>=8 || !openingWall(right ? rightOpening : leftOpening) ||
      f.mm<=100 || b.mm<=100 || f.mm>=1000 || b.mm>=1000 ||
      af>WALL_PAIR_MAX_AGE_MS || ab>WALL_PAIR_MAX_AGE_MS) {
    initialized=false; goodPairs=0; previousCorner=completedCorners;
    return;
  }
  // El desfase normal de un barrido solo pausa la correccion. No suma muestras.
  if (skew>WALL_PAIR_MAX_SKEW_MS ||
      f.lastValidMs==prevF || b.lastValidMs==prevB) return;
  prevF=f.lastValidMs; prevB=b.lastValidMs;
  float angle=atan2f(1.7320508f*(right ? (float)b.mm-f.mm : (float)f.mm-b.mm),
                    (float)f.mm+b.mm)*57.2957795f;
  float residual=wrapYaw(corridorYaw+angle-correctedYaw());
  if (fabsf(angle)>18 || fabsf(residual)>10) {
    initialized=false; goodPairs=0; return;
  }
  // Consistencia de la diferencia IMU/pared, no del angulo absoluto:
  // el robot puede estar corrigiendo suavemente su distancia.
  if (!initialized || now-lastGood>350 || fabsf(wrapYaw(residual-filteredResidual))>3) {
    initialized=true; previousSide=right; previousCorner=completedCorners;
    filteredResidual=residual; firstGood=lastGood=now; goodPairs=1;
    return;
  }
  float dt=clampValue((now-lastGood)*0.001f,0,0.25f);
  lastGood=now;
  if (goodPairs<1000) ++goodPairs;
  filteredResidual+=0.25f*wrapYaw(residual-filteredResidual);
  if (goodPairs<3 || now-firstGood<350) return;
  float correction=clampValue(filteredResidual*dt/DRIFT_TIME_CONSTANT_S,
                              -DRIFT_MAX_DPS*dt,DRIFT_MAX_DPS*dt);
  yawOffset=wrapYaw(yawOffset+correction);
  filteredResidual=wrapYaw(filteredResidual-correction);
  driftReferenceOK=true;
}

void updateSteeringPID() {
  if (!yawValido) { updateSteeringLegacy(); return; }
  if (runState==FINISHED) return;
  uint32_t now=millis();
  if (now-pidLastMs<PID_INTERVAL_MS) return;
  float dt=clampValue((now-pidLastMs)*0.001f,0.001f,0.1f);
  pidLastMs=now;
  bool turning=turnState!=NO_TURN;
  if (!turning) {
    if (trackDir==UNKNOWN && !selectFirstWall()) {
      stopDriveMotor(); setSteeringServo(SERVO_CENTER); return;
    }
    bool right=trackDir==CCW || (trackDir==UNKNOWN && firstWall==FIRST_RIGHT);
    const DistanceReading &f=right ? rightLookReading : leftLookReading;
    const DistanceReading &b=right ? rightBackLookReading : leftBackLookReading;
    float desired=0;
    if (trackDir==UNKNOWN) {
      uint16_t ff=getDistance(right ? firstRightFront : firstLeftFront);
      uint16_t bb=getDistance(right ? firstRightBack : firstLeftBack);
      if (!ff && !bb) { stopDriveMotor(); setSteeringServo(SERVO_CENTER); return; }
      desired=(firstSideDistance(ff,bb)-firstWallTarget)*KP_DISTANCE_YAW;
    } else {
      float distance=sideControlDistance(f,b);
      if (distance) desired=(distance-targetWallDistance)*KP_DISTANCE_YAW;
    }
    if (!right) desired=-desired;
    desired=clampValue(desired,-MAX_DISTANCE_YAW,MAX_DISTANCE_YAW);
    distanceYaw+=clampValue(desired-distanceYaw,-DISTANCE_SLEW_DPS*dt,DISTANCE_SLEW_DPS*dt);
    correctDrift(right,f,b);
  } else { distanceYaw=0; driftReferenceOK=false; }
  commandedYaw=wrapYaw(corridorYaw+distanceYaw);
  float error=wrapYaw(commandedYaw-correctedYaw());
  float output=KP_YAW*error-KD_YAW_RATE*velocidadZ;
  setSteeringServo(SERVO_CENTER+(int)clampValue(output,-30,30));
  driveForward(turning ? DRIVE_PWM_SLOW : DRIVE_PWM_NORMAL);
}

void startLidar() {
  lidar.stopScan();
  delay(50);
  while (Serial1.available()) Serial1.read();

  lidarStarted = lidar.startStandardScan();
}

void setup() {
  Serial.begin(115200);

  pinMode(PWMA, OUTPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  stopDriveMotor();

  // Abrir UART0 ANTES de asignar los pines de UART1 y de calibrar la IMU.
  Serial0.setRxBufferSize(256);
  Serial0.begin(38400,SERIAL_8N1,START_RX_PIN,START_TX_PIN);

  ledcAttach(MOTOR_PIN, 500, 8);
  ledcWrite(MOTOR_PIN, 200);

  ledcAttach(SERVO_PIN, 50, 16);
  setSteeringServo(SERVO_CENTER);
  // El motor del LiDAR acelera y el servo se centra durante la calibracion.
  imuSetup();

  lidar.init(LIDAR_RX_PIN, LIDAR_TX_PIN);
  lidar.postParseCallback = lidarDataHandler;
  startLidar();
  ultimaMuestraUs = 0; // Todos los delays anteriores ocurrieron con el robot quieto.

  if (lidarStarted) {
    uint32_t warmUpStart = millis();
    // El rotor acelero durante la calibracion. Recoger lecturas nuevas.
    while (millis() - warmUpStart < 350) {
      imuUpdate();
      lidar.handleData(true, false);
    }
    pidLastMs = millis();
    driveForward(DRIVE_PWM_NORMAL);
  }
  setupStartLink();
}

void loop() {
  imuUpdate();
  static uint32_t sensorReportMs=0;
  if(firstWall==FIRST_NONE && millis()-sensorReportMs>=1000) {
    sensorReportMs=millis();
    Serial.print("[SENSORES] puntosLidar=");Serial.print(lidarPointCount);
    Serial.print(" | edadUltimoPunto=");Serial.print(millis()-lidarLastPointMs);
    Serial.print(" | IMU=");Serial.print(yawValido?"OK":"INVALIDA");
    Serial.print(" | motivo=");Serial.println(imuInvalidReason);
  }
  pollStartLink();
  if(!robotStarted || millis()-startAcceptedMs<START_DELAY_MS) {
    if(lidarStarted) lidar.handleData(true,false);
    else {
      static uint32_t retry=0;
      if(millis()-retry>=1000){retry=millis();startLidar();}
    }
    return;
  }
  if(!motionEnabled) {
    motionEnabled=true;
    Serial.println("[ARRANQUE] Espera terminada. Iniciando recorrido.");
    // Inicio desde la orientacion en que el usuario coloco el robot.
    // No convertir una IMU invalida en valida por recibir el boton.
    yaw=0; yawOffset=0; corridorYaw=0; commandedYaw=0; distanceYaw=0;
    ultimaMuestraUs=0; pidLastMs=millis();
    firstWall=FIRST_NONE; firstWallTarget=0;
    driveForward(DRIVE_PWM_NORMAL);
  }
  updateRunCompletion();
  if (runState == FINISHED) return;
  if (lidarStarted) {
    lidar.handleData(true, false);
  } else {
    static uint32_t lastRetryMs = 0;
    if (millis() - lastRetryMs >= 1000) {
      lastRetryMs = millis();
      startLidar();
      if (lidarStarted) {
         if (turnState == NO_TURN) driveForward(DRIVE_PWM_NORMAL);
         else driveForward(DRIVE_PWM_SLOW); 
      }
    }
  }

  updateRunCompletion();
  if (runState == FINISHED) return;
  updateSteeringPID();
  checkForTurn();
  diagnosticarCurvasCW();
}
