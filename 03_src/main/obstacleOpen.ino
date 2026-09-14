/* Obstacle Challenge - CW-CARRIL2
   Esquina referida a pared, reversa con IMU, parada por hueco de carroceria.
   Arranque con tres fotos vacias validas y paredes; primera evasion gradual.
   Conserva PWM 150, direccion +/-40, minimo 60 mm y admision 350-1000 mm.
   Paredes: dos medidas o una medida + ancho reciente; sin eco, breve rumbo recto.
   Datos perdidos: ESPERA_DATOS sin traccion y recuperacion verificada.
   No recupera yaw invalido ni ignora peligros confirmados. Centrado sin limite de 12 s.
   MOVING anticipa fotos al enderezar; LIBRE exige que la parte trasera haya pasado.
*/
#include <Arduino.h>
#include <esp_system.h>
#include <Wire.h>
#include <math.h>
#define lidarDebugSerial Serial
#include "thijs_rplidar.h"
#if !defined(ARDUINO_USB_CDC_ON_BOOT) || !ARDUINO_USB_CDC_ON_BOOT
#error "Activa USB CDC On Boot: Enabled en la C6."
#endif
// Tramas @tipo,sesion,solicitud,arranque,dato*CRC16\n.
// CRC detecta corrupcion accidental; no es autenticacion criptografica.
#include <stdint.h>
#include <stdio.h>
#include <string.h>

// Definir el tipo antes de cualquier funcion: Arduino puede insertar aqui
// prototipos automaticos de funciones declaradas mucho mas abajo.
struct ApproachWall {
  bool valid=false;
  float gap=0, normalAngle=0, yawAtFit=0, span=0;
  bool held=false;
  uint32_t stamp=0, age=0;
  unsigned count=0;
};
ApproachWall measureApproachWall();
ApproachWall fitCornerWall(bool rear,uint32_t minimumAge=0);
ApproachWall cornerWallReading(bool rear);
void clearCornerTracker();
float estimatedWallGap(const ApproachWall &wall);
bool finishKnownPassedPillar();

// --- ESTADOS ACTUALIZADOS ---
enum MotionState { WAIT_PILLAR, MOVING, PARALLEL, DECIDE_NEXT, CENTERING, CORNER_HOLD, RECOVERING, DONE, FAULT, APPROACHING_FRONT, REVERSING_TURN, REVERSING_STRAIGHT, FINISHED_SEQ, WAIT_BEFORE_REVERSE, PREPARE_SECOND_LANE };
enum RecoveryCause { RX_MISSING, PILLAR_MISSING, VISION_MISSING, WALLS_MISSING };
void pauseMotion(RecoveryCause cause,const char *reason);
bool centerReference(float &error,bool &oneWall);
void enterCentering();
void serviceRecovery();
struct PillarReading {
  bool valid=false; float x=0,y=0,distance=0,bearing=0;
  float minX=0,maxX=0,minY=0,maxY=0;
  uint32_t stamp=0; unsigned count=0;
};
// Prototipos explicitos: evitar que Arduino los coloque antes del tipo.
PillarReading findPillar(float center,float halfWindow,bool tracking,const PillarReading *reference=nullptr);
bool distinctFromCurrent(const PillarReading &c);
bool nextReady();
void beginManeuver();
bool viewAdjustmentAllowed();
struct TrackDiagnostic {
  unsigned returns=0,groups=0,shape=0,time=0,association=0,accepted=0;
  unsigned edge=0,narrow=0,wide=0,sparse=0;
  float halfWindow=0;
};
struct FrontDiagnostic {
  unsigned count=0;float distance=0,angle=0,x=0,y=0;uint32_t age=0;
};
// --- Deteccion de fin de carril / esquina CW ---------------------------
struct CornerEvidence {
  bool frontWall=false, rightOpen=false, leftWall=false, cameraAgrees=false;
  float frontDistanceMM=0, frontSpanMM=0, leftWallMM=0, rightWallMM=0;
  unsigned frontCount=0; uint32_t frontAge=0;
};
bool frontWallEvidence(CornerEvidence &ev);
bool sampleCornerEvidence(CornerEvidence &ev);
bool checkCornerCW();
void printCornerDiagnostic();
void holdCorner();
void handleCornerHold();

struct LidarStreamReader {
  uint8_t bytes[15]={};unsigned used=0;
  uint32_t discarded=0,packets=0,headerRejects=0,angleRejects=0;
  static uint16_t angle(const uint8_t *p){return ((uint16_t)p[2]<<7)|(p[1]>>1);}
  static bool valid(const uint8_t *p){
    unsigned flags=p[0]&3;
    return (flags==1 || flags==2) && (p[1]&1) && angle(p)<360*64;
  }
  static bool follows(const uint8_t *a,const uint8_t *b){
    int delta=(int)angle(b)-(int)angle(a);
    if(delta< -180*64)delta+=360*64;
    if(delta>180*64)delta-=360*64;
    return delta>=-2*64 && delta<=20*64;
  }
  bool push(uint8_t b,uint16_t &distance,uint16_t &angleQ6,uint8_t &start,int8_t &quality){
    bytes[used++]=b;
    if(used<sizeof(bytes))return false;
    bool headers=valid(bytes) && valid(bytes+5) && valid(bytes+10);
    bool angles=headers && follows(bytes,bytes+5) && follows(bytes+5,bytes+10);
    bool good=headers && angles;
    if(good){
      distance=(((uint16_t)bytes[4]<<8)|bytes[3])>>2;
      angleQ6=angle(bytes);start=(bytes[0]&3)==1;quality=bytes[0]>>2;
      memmove(bytes,bytes+5,10);used=10;++packets;return true;
    }
    if(!headers)++headerRejects;else ++angleRejects;
    memmove(bytes,bytes+1,14);used=14;++discarded;return false;
  }
  void reset(){used=0;}
};
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

static const int SDA_PIN = 18; // D10
static const int SCL_PIN = 1;  // D1
static const float YAW_SIGN = -1.0f;
static const float GYRO_SCALE = 65.6f;
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
    Serial.print("[ARRANQUE ERROR IMU] "); Serial.println(mensaje);
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

int leerGiroscopio(float &gx, float &gy, float &gz) {
  uint8_t estado;
  if (!leerRegistros(0x1B, &estado, 1)) return -1;
  if (!(estado & 0x40)) return 0;
  uint8_t datos[6];
  if (!leerRegistros(0x0C, datos, sizeof(datos))) return -1;
  int16_t rawX = (int16_t)((uint16_t)datos[0] | ((uint16_t)datos[1] << 8));
  int16_t rawY = (int16_t)((uint16_t)datos[2] | ((uint16_t)datos[3] << 8));
  int16_t rawZ = (int16_t)((uint16_t)datos[4] | ((uint16_t)datos[5] << 8));
  gx = rawX / GYRO_SCALE;
  gy = rawY / GYRO_SCALE;
  gz = rawZ / GYRO_SCALE;
  return 1;
}

void calibrar() {
  Serial.println("Deja el sensor horizontal y completamente quieto.");
  Serial.println("Preparando calibracion...");
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
    if (estado < 0) detenerPorError("Fallo I2C durante la calibracion.");
    if (estado == 1) {
      if (fabsf(gx) > 5.0f || fabsf(gy) > 5.0f || fabsf(gz) > 5.0f) {
        muestras = 0; sumaZ = 0; minimoZ = 10000; maximoZ = -10000;
        if (!avisoMovimiento) Serial.println("Movimiento: repitiendo calibracion automaticamente.");
        avisoMovimiento = true;
      } else {
        float nuevoMin = gz < minimoZ ? gz : minimoZ;
        float nuevoMax = gz > maximoZ ? gz : maximoZ;
        if (nuevoMax - nuevoMin > 2.0f) {
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
      Serial.println("Sin calibracion estable: prueba bloqueada. Reinicia con el robot quieto.");
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
}

void imuSetup() {
  Serial.begin(115200);
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(100000);
  Wire.setTimeOut(5);
  delay(100);
  if (!buscarBMI160()) detenerPorError("BMI160 no encontrado en 0x68/0x69. Revisa conexiones.");
  Serial.print("BMI160 encontrado en 0x");
  Serial.println(bmiAddress, HEX);
  if (!escribirRegistro(0x7E, 0xB6)) detenerPorError("No se pudo reiniciar el BMI160.");
  delay(100);
  if (!escribirRegistro(0x7E, 0x15)) detenerPorError("No se pudo activar el giroscopio.");
  delay(100);
  if (!escribirRegistro(0x42, 0x28) || !escribirRegistro(0x43, 0x02)) detenerPorError("No se pudo configurar el giroscopio.");
  delay(100);
  uint8_t configuracion, rango, modo;
  if (!leerRegistros(0x42, &configuracion, 1) || !leerRegistros(0x43, &rango, 1) || !leerRegistros(0x03, &modo, 1) || configuracion != 0x28 || (rango & 0x07) != 0x02 || (modo & 0x0C) != 0x04) {
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
    if (yawValido) Serial.println("ERROR I2C: detener prueba de obstaculo.");
    yawValido = false;
    imuInvalidReason = "I2C";
    ultimaMuestraUs = 0;
  }
  if (estado == 1) {
    velocidadZ = (gz - biasZ) * YAW_SIGN;
    if (ultimaMuestraUs != 0) {
      uint32_t intervaloUs = ahoraUs - ultimaMuestraUs;
      if (intervaloUs > 50000) {
        yawValido = false;
        imuInvalidReason = "HUECO_MUESTRAS_50MS";
      } else if (yawValido) {
        float dt = intervaloUs * 0.000001f;
        yaw += velocidadZ * dt;
        while (yaw >= 180.0f) yaw -= 360.0f;
        while (yaw < -180.0f) yaw += 360.0f;
      }
    }
    ultimaMuestraUs = ahoraUs;
  }
  if (ultimaMuestraUs != 0 && (uint32_t)(ahoraUs - ultimaMuestraUs) > 100000) {
    yawValido = false;
    imuInvalidReason = "SIN_MUESTRAS_100MS";
  }
}

static const int CAM_RX=17,CAM_TX=21,LIDAR_RX=19,LIDAR_TX=16;
static const int LIDAR_MOTOR=20,SERVO_PIN=0,PWMA=22,AIN1=23,AIN2=2;
static const float CAMERA_HFOV_DEG=60.0f, CAMERA_CENTER_OFFSET_DEG=0.0f, IMAGE_RIGHT_SIGN=1.0f;
static const uint32_t LIDAR_FRESH_MS=250, PHOTO_INTERVAL_MS=150, RETRY_MS=2000;
static const int TEST_DRIVE_PWM=150;
// Ajustes de esquina: distancias desde el LiDAR, NO desde la carroceria.
static const int CORNER_DRIVE_PWM=TEST_DRIVE_PWM;
static const float LIDAR_TO_FRONT_MM=100.0f;
static const float FRONT_CHASSIS_GAP_MM=50.0f;
// Margen inicial para el avance DESPUES de cortar PWM. Ajustar con [FRENTE_FINAL].
static const float FRONT_STOP_RESERVE_MM=50.0f;
static const float LIDAR_TO_REAR_MM=100.0f, REAR_CHASSIS_GAP_MM=50.0f;
static const float REAR_STOP_RESERVE_MM=50.0f;
static const float FIRST_EVASION_LOOKAHEAD_MM=180.0f, FORWARD_SERVO_SLEW_DEG_S=200.0f;
static const float FIRST_DIRECT_MM=700.0f, FIRST_SMOOTH_MM=900.0f;
static const float CORNER_TURN_LEAD_DEG=5.0f, REVERSE_HEADING_KP=1.5f, REVERSE_STEER_LIMIT=15.0f;
static const uint32_t WALL_TRACK_MAX_AGE_MS=250;
static const uint32_t FRONT_FIT_FRESH_MS=220, FRONT_FIT_SPAN_MS=60;
static const uint32_t CORNER_DIRECTION_PAUSE_MS=500;
static const uint32_t CORNER_STAGE_TIMEOUT_MS=12000;
static const float BODY_HALF_WIDTH_MM=70, PILLAR_HALF_MM=25;
static const float DESIRED_GAP_MM=120;
static const float MIN_GAP_MM=60;
static const float MIN_CENTER_OFFSET_MM=BODY_HALF_WIDTH_MM+PILLAR_HALF_MM+MIN_GAP_MM;
static const float PASS_OFFSET_MM=BODY_HALF_WIDTH_MM+PILLAR_HALF_MM+DESIRED_GAP_MM+15;
float plannedOffset=PASS_OFFSET_MM;
static const float MAX_HEADING_DEG=40.0f;
static const float MAX_CROSS_HEADING_DEG=65.0f;
static const float CROSS_MARGIN_EXTRA_MM=10.0f, CROSS_MARGIN_BY_Y_MM=120.0f;
static const float HEADING_KP=1.5f, HEADING_KD=0.12f;
static const float START_MIN_MM=350.0f, START_MAX_MM=1000.0f;
static const uint32_t TRACK_LOST_MS=500, MAX_MOTION_MS=12000;
static const uint32_t WALL_GRACE_MS=500, LANE_WIDTH_HOLD_MS=5000, VISION_FRESH_MS=2500;
static const float REACQUIRE_RADIUS_MM=180, CENTER_SLEW_DEG_S=80;
static const uint32_t VIEW_ADJUST_MS=700;
static const float REJOIN_MAX_DEG=20.0f, REJOIN_SLEW_DEG_S=80.0f;
static const float REJOIN_CENTER_KP=0.22f;
static const float REJOIN_LOOKAHEAD_MM=100.0f, REJOIN_EXTRA_GAP_MM=15.0f;
static const float CORNER_FRONT_MIN_MM=150.0f, CORNER_FRONT_MAX_MM=700.0f;
static const unsigned CORNER_FRONT_MIN_POINTS=2;
static const float CORNER_FRONT_MAX_DEPTH_MM=400.0f;
static const uint32_t CORNER_HOLD_MS=500;
static const uint32_t CORNER_HOLD_FAST_MS=150;
static const uint32_t SECOND_CORNER_CONFIRM_MS=150; // Solo parada en segunda esquina.
static const uint32_t CORNER_MIN_CENTERING_MS=300;
static const float DEG=0.01745329252f;
RPlidar lidar(Serial1);
// Temporal: false restaura el arranque por boton de la ESPcam.
static const bool AUTO_START_ENABLED=true;
static const uint32_t AUTO_START_DELAY_MS=2000;
void serviceAutoStart();
bool lidarStarted=false,enabled=false,pending=false,haveBox=false,haveResult=false;
uint16_t ranges[360]={};uint32_t rangeMs[360]={};float rangeYaw[360]={};
uint32_t points=0,lastPointMs=0,session=0,cameraBoot=0,requestId=0;
uint32_t sentMs=0,lastPhotoMs=0,lastHelloMs=0,packedBox=0,buttonMs=0;
unsigned sends=0,stableFrames=0;char result='E',boxQuality='U',previousColor='E';
uint32_t photoBoxes[2]={},photoStartedMs=0;
char photoColors[2]={'E','E'};
unsigned photoMask=0;
bool photoFinished=false;
bool photoMoving=false;
float photoYaw=0;
uint32_t nextTrackMs=0;
float rejoinRequested=0;
const char *rejoinReason="CENTRO";
float previousX=0;CamReader reader;
// Observacion solamente: no cambia condiciones, PWM ni tiempos de arranque.
uint8_t debugRawBytes[24]={};
unsigned debugRawCount=0;
uint32_t debugHelloSends=0;
uint32_t debugCamBytes=0,debugCamFrames=0,debugForeignFrames=0,debugButtons=0;
uint32_t debugPhotos=0,debugTimeouts=0,debugLidarBytes=0;
const char *debugVision="AUN SIN FOTO COMPLETA";
bool debugStarted=false;
void printStartupDiagnostic();
MotionState motion=WAIT_PILLAR;
MotionState resumeState=WAIT_PILLAR;
RecoveryCause recoveryCause=RX_MISSING;
uint32_t pauseStartedMs=0,recoveryCheckMs=0,recoveryLogMs=0;
bool reacquiring=false;
bool reacquiringSide=false;
// Busqueda local solo detenido, tras perder un pilar ya proximo al costado.
static const float SIDE_RECOVERY_MAX_SHIFT_MM=300;
bool sideRecoveryCandidate(const PillarReading &c,const PillarReading &old);

PillarReading recoveryCandidate;
unsigned recoveryHits=0;
uint32_t lastWallReferenceMs=0,laneWidthMs=0,centeringStartedMs=0;
float laneWidthMM=0;
float wallRightReport=0,wallLeftReport=0;
uint32_t lastLaneSampleMs=0;
uint32_t headingTimes[32]={};float headingHistory[32]={};unsigned headingWrite=0;
int lastServoAngle=90;
float crossingDemand=0;
PillarReading target,previousCandidate;

// Variable agregada para la maniobra
float initialYaw=0,headingTarget=0, turnStartYaw=0;

ApproachWall lastApproachWall;
float approachClosingSpeed=0, approachStopGap=0;
bool approachSpeedKnown=false;
uint32_t approachLogMs=0;
ApproachWall cornerWallCache[2];
float cornerLaneYaw=0, cornerWallHeading=0;
unsigned cornerHeadingHits=0;
uint32_t cornerHeadingStamp=0, finalStopMs=0;
bool rearFinalLogged=false;
unsigned emptyStartFrames=0;
bool laneReferenceLocked=false;
static const unsigned TARGET_LAPS=3;
static const unsigned CORNERS_PER_LAP=4;
// Tiempo de avance tras la esquina 12: ajustar en pista, no es posicion medida.
static const uint32_t FINAL_APPROACH_MS=800UL;
unsigned completedCorners=0;
// Intentar pasar pilares cercanos sin finalizar la prueba por margen insuficiente.
static const bool TRY_CLOSE_EVASION=true;
float closeSteering=0;
bool closeOverride=false;
uint32_t closeReportMs=0;
bool finalForwardCommand=false;
uint32_t finalAdvanceMs=0,finalClockMs=0;
void updateLapFinish();
unsigned currentLane=1;
bool secondLaneExit=false, secondLaneSawRight=false;
unsigned cornerRightHits=0;
uint32_t cornerRightCheckMs=0;
uint32_t lastRawEmptyMs=0,lastRawColorMs=0;
unsigned rawEmptyFrames=0;
const char *visionDecision="SIN FOTO";
const char *visionColorName(char c);
static const bool FILTER_OUTSIDE_LANE=true; // Solo seleccionar objetivos con limites laterales comprobables.

float secondLaneLeftReference=0;
uint32_t secondLaneStartedMs=0,secondLaneLogMs=0;
void prepareSecondLane();
void serviceSecondLaneStart();
void resetNextVision();
void printSecondLaneDiagnostic();
bool cornerWaiting=false;
uint32_t cornerWaitSince=0,cornerWaitTick=0,cornerWaitLog=0,lidarRestartMs=0;

int passSide=0; bool returningStraight=false;
uint32_t motionMs=0,lastTrackCheck=0;
LidarStreamReader lidarReader;
uint32_t lastLidarServiceMs=0,maxLidarGapMs=0;
unsigned maxLidarQueue=0;
unsigned sideConfirmations=0,passedPillars=0,nextConfirmations=0,noPillarFrames=0;
unsigned lanePillars=0;
bool laneGuaranteedEnd=false;
uint32_t sideStamp=0,nextVisionMs=0,centerStableMs=0;
char nextColor='E';float nextImageX=0;
PillarReading nextPillar;
TrackDiagnostic trackDiag;
FrontDiagnostic frontDiag;
uint32_t cornerHoldStartMs=0;
CornerEvidence lastCornerEvidence;
int bucketDegree=-1,bucketError=1000;
uint16_t bucketDistance=0;
uint32_t bucketMs=0;float bucketYaw=0;

float limitFloat(float x,float lo,float hi) {return x<lo?lo:(x>hi?hi:x);}
float angleDifference(float a,float b) {
  float d=a-b;while(d>180)d-=360;while(d< -180)d+=360;return d;
}
void setSteering(float delta) {
  int angle=(int)lroundf(90+limitFloat(delta,-40,40));
  lastServoAngle=angle;
  uint32_t us=map(angle,0,180,500,2400);
  ledcWrite(SERVO_PIN,us*65535UL/20000UL);
}
void stopMotor() {finalForwardCommand=false;analogWrite(PWMA,0);digitalWrite(AIN1,LOW);digitalWrite(AIN2,LOW);}
void stopTrial(const char *reason) {
  stopMotor();setSteering(0);motion=FAULT;
  Serial.print("[PARADA] ");Serial.println(reason);
  Serial.println("[PRUEBA] Detenida. Reinicia C6 para repetir; no reanuda sola.");
  if(currentLane>=2)printSecondLaneDiagnostic();
}
MotionState effectiveState(){return motion==RECOVERING?resumeState:motion;}
void pauseMotion(RecoveryCause cause,const char *reason) {
  if(motion==FAULT || motion==DONE || motion==RECOVERING || motion==CORNER_HOLD)return;
  if(cause==VISION_MISSING || cause==WALLS_MISSING){
    if(millis()-closeReportMs>=500){closeReportMs=millis();Serial.print("[CONTINUA] ");Serial.println(reason);}
    return;
  }
  stopMotor();setSteering(0);
  resumeState=motion;motion=RECOVERING;recoveryCause=cause;
  pauseStartedMs=millis();recoveryCheckMs=0;recoveryHits=0;
  recoveryCandidate=PillarReading();
  if(resumeState==CENTERING || resumeState==PARALLEL || resumeState==DECIDE_NEXT){
    pending=false;lastPhotoMs=0;nextVisionMs=0;nextConfirmations=noPillarFrames=0;
  }
  Serial.print("[ESPERA_DATOS] ");Serial.println(reason);
}
void lidarPoint(RPlidar*,uint16_t distance,uint16_t angleQ6,uint8_t,int8_t) {
  imuUpdate();
  int degree=((int)((angleQ6+32u)/64u))%360;
  if(degree!=bucketDegree) {
    if(bucketDegree>=0) {
      ranges[bucketDegree]=bucketDistance;
      rangeMs[bucketDegree]=bucketMs;rangeYaw[bucketDegree]=bucketYaw;
    }
    bucketDegree=degree;bucketError=1000;bucketDistance=0;
    bucketMs=millis();bucketYaw=yaw;
  }
  int angularError=abs((int)angleQ6-degree*64);
  if(angularError>180*64)angularError=360*64-angularError;
  if(distance && distance<=6000 && angularError<bucketError) {
    bucketDistance=distance;bucketError=angularError;
    bucketMs=millis();bucketYaw=yaw;
  }
  ++points;lastPointMs=millis();
}
void serviceLidar() {
  if(!lidarStarted)return;
  uint32_t now=millis();
  if(lastLidarServiceMs && now-lastLidarServiceMs>maxLidarGapMs)maxLidarGapMs=now-lastLidarServiceMs;
  lastLidarServiceMs=now;
  unsigned queued=Serial1.available();if(queued>maxLidarQueue)maxLidarQueue=queued;
  uint32_t began=micros();unsigned budget=256;
  uint16_t distance,angleQ6;uint8_t start;int8_t quality;
  while(budget-- && Serial1.available() && micros()-began<2000) {
    ++debugLidarBytes;
    if(lidarReader.push((uint8_t)Serial1.read(),distance,angleQ6,start,quality))
      lidarPoint(&lidar,distance,angleQ6,start,quality);
  }
}
float imageAngle(float x) {
  float focal=80.0f/tanf(CAMERA_HFOV_DEG*0.5f*DEG);
  return CAMERA_CENTER_OFFSET_DEG+IMAGE_RIGHT_SIGN*atanf((x-79.5f)/focal)/DEG;
}
PillarReading findPillar(float center,float halfWindow,bool tracking,const PillarReading *reference) {
  const PillarReading &tracked=reference?*reference:target;
  PillarReading chosen;
  TrackDiagnostic diag;
  diag.halfWindow=halfWindow;
  int lo=(int)floorf(center-halfWindow),hi=(int)ceilf(center+halfWindow);
  int count=0,first=0,last=0;float sumX=0,sumY=0,sum=0;
  float minX=0,maxX=0,minY=0,maxY=0;
  uint16_t previous=0;uint32_t youngest=0,oldest=0;
  unsigned candidates=0;
  for(int a=lo;a<=hi+1;a++) {
    int index=(a%360+360)%360;
    uint32_t age=millis()-rangeMs[index];
    uint16_t mm=a<=hi && age<=LIDAR_FRESH_MS ? ranges[index] : 0;
    if(mm)++diag.returns;
    if(!mm && count && a<hi) {
      int ni=((a+1)%360+360)%360;
      uint32_t nextAge=millis()-rangeMs[ni];
      if(ranges[ni] && nextAge<=LIDAR_FRESH_MS && abs((int)ranges[ni]-(int)previous)<=50 && abs((int)nextAge-(int)youngest)<=80)continue;
    }
    float pointGap=count && mm ? sqrtf((float)mm*mm+(float)previous*previous-2.0f*mm*previous*cosf((a-last)*DEG)) : 0;
    bool cut=!mm || (count && (pointGap>35+0.02f*previous || abs((int)age-(int)youngest)>80));
    if(cut && count) {
      ++diag.groups;
      float mean=sum/count;
      float width=2*mean*sinf((last-first+1)*0.5f*DEG);
      float minWidth=tracking?8.0f:20.0f;
      bool shapeOK=count>=2 && first>lo && last<hi && width>=minWidth && width<=100 && count*2>=last-first+1;
      if(first<=lo || last>=hi)++diag.edge;
      if(width<minWidth)++diag.narrow;
      if(width>100)++diag.wide;
      if(count<2 || count*2<last-first+1)++diag.sparse;
      if(!shapeOK)++diag.shape;
      else if(oldest-youngest>80)++diag.time;
      if(shapeOK && oldest-youngest<=80) {
        PillarReading c;
        c.distance=mean; c.x=sumX/count; c.y=sumY/count;
        c.minX=minX;c.maxX=maxX;c.minY=minY;c.maxY=maxY;
        float norm=hypotf(c.x,c.y);
        if(norm>0){c.x*=1+25/norm;c.y*=1+25/norm;}
        c.stamp=millis()-youngest;c.count=count;
        c.bearing=atan2f(c.x,c.y)/DEG;
        bool compatible=true;
        if(tracking) {
          uint32_t dt=c.stamp-tracked.stamp;
          float gate=45+0.5f*dt;
          bool timeOK=reacquiring ? ((int32_t)(c.stamp-pauseStartedMs)>=200 && youngest<=150) : (dt>=30 && dt<=TRACK_LOST_MS && oldest<=millis()-tracked.stamp);
          bool positionOK=hypotf(c.x-tracked.x,c.y-tracked.y)<=(reacquiring?REACQUIRE_RADIUS_MM:gate);
          if(reacquiringSide)positionOK=sideRecoveryCandidate(c,tracked);
          if(!timeOK)++diag.time;
          else if(!positionOK)++diag.association;
          compatible=timeOK && positionOK;
        }
        if(compatible){
          c.valid=true;
          if(candidates==0){chosen=c;candidates=1;}
          else {
            float lx=fminf(chosen.minX,c.minX),hx=fmaxf(chosen.maxX,c.maxX);
            float ly=fminf(chosen.minY,c.minY),hy=fmaxf(chosen.maxY,c.maxY);
            bool merge=tracking && candidates==1 && hypotf(hx-lx,hy-ly)<=90 && abs((int32_t)(c.stamp-chosen.stamp))<=80;
            if(merge){
              float n=chosen.count+c.count;
              chosen.x=(chosen.x*chosen.count+c.x*c.count)/n;
              chosen.y=(chosen.y*chosen.count+c.y*c.count)/n;
              chosen.distance=(chosen.distance*chosen.count+c.distance*c.count)/n;
              chosen.count+=c.count;chosen.bearing=atan2f(chosen.x,chosen.y)/DEG;
              if(millis()-c.stamp<millis()-chosen.stamp)chosen.stamp=c.stamp;
              chosen.minX=lx;chosen.maxX=hx;chosen.minY=ly;chosen.maxY=hy;
            } else ++candidates;
          }
        }
      }
      count=0;sumX=sumY=sum=0;
    }
    if(mm) {
      if(!count){first=a;youngest=oldest=age;}
      if(age<youngest)youngest=age;
      if(age>oldest)oldest=age;
      float heading=angleDifference(rangeYaw[index],initialYaw);
      float rad=(a+heading)*DEG;
      float px=mm*sinf(rad),py=mm*cosf(rad);
      if(!count){minX=maxX=px;minY=maxY=py;}
      minX=fminf(minX,px);maxX=fmaxf(maxX,px);
      minY=fminf(minY,py);maxY=fmaxf(maxY,py);
      sumX+=px;sumY+=py;sum+=mm;
      last=a;previous=mm;++count;
    }
  }
  diag.accepted=candidates;
  if(tracking && !reference)trackDiag=diag;
  if(candidates!=1)chosen.valid=false;
  return chosen;
}
float trackingHalfWindow() {
  float radius=hypotf(target.x,target.y);
  if(radius<1)return 100;
  float age=(float)(millis()-target.stamp);
  float displacement=45+0.5f*age;
  float angularTravel=asinf(limitFloat(displacement/radius,0,1))/DEG;
  return limitFloat(angularTravel+20,35,100); 
}
void printTrackDiagnostic() {
  Serial.print("[TRACK] retornos=");Serial.println(trackDiag.returns);
}
bool frontTooClose() {
  frontDiag=FrontDiagnostic();
  unsigned count=0;int lastAngle=-1000;
  float lastX=0,lastY=0;uint32_t youngest=0,oldest=0;
  FrontDiagnostic nearest;
  for(int a=-85;a<=85;a++) {
    int i=(a+360)%360;float d=ranges[i];
    uint32_t age=millis()-rangeMs[i];
    if(!d || age>LIDAR_FRESH_MS)continue;
    float rad=(a+angleDifference(rangeYaw[i],yaw))*DEG;
    float x=d*sinf(rad),y=d*cosf(rad);
    if(!(fabsf(x)<100 && y>0 && y<210))continue;
    uint32_t lo=age<youngest?age:youngest,hi=age>oldest?age:oldest;
    bool connected=count && a-lastAngle<=2 && hypotf(x-lastX,y-lastY)<=30 && hi-lo<=40;
    if(!connected){count=0;lo=hi=age;nearest=FrontDiagnostic();}
    ++count;youngest=lo;oldest=hi;lastAngle=a;lastX=x;lastY=y;
    if(nearest.distance==0 || d<nearest.distance){
      nearest.distance=d;nearest.angle=fmodf(a+360.0f,360.0f);
      nearest.x=x;nearest.y=y;nearest.age=age;
    }
    if(count>=3){frontDiag=nearest;frontDiag.count=count;return true;}
  }
  return false;
}
void printFrontDiagnostic() {
  Serial.print("[FRENTE] distancia_lidar_mm=");Serial.println(frontDiag.distance,0);
}
bool drivingState() {return motion==MOVING || motion==PARALLEL || motion==CENTERING;}
float laneWallDistance(bool right) {
  struct Band {float sum=0,minY=10000,maxY=-10000;unsigned count=0;};
  Band bands[160];
  float relativeYaw=angleDifference(yaw,initialYaw);
  int center=(int)lroundf((right?90:270)-relativeYaw);
  for(int a=center-60;a<=center+60;++a) {
    int index=(a%360+360)%360;
    if(!ranges[index] || millis()-rangeMs[index]>LIDAR_FRESH_MS)continue;
    float rad=(a+angleDifference(rangeYaw[index],initialYaw))*DEG;
    float x=ranges[index]*sinf(rad)*(right?1:-1),y=ranges[index]*cosf(rad);
    if(x<=0 || x>=6360 || fabsf(y)>400)continue;
    int bin=(int)(x/40);
    for(int b=bin-1;b<=bin;++b) {
      if(b<0 || b>=160)continue;
      Band &v=bands[b];v.sum+=x;++v.count;
      v.minY=fminf(v.minY,y);v.maxY=fmaxf(v.maxY,y);
    }
  }
  unsigned bestCount=0;float best=0;
  for(const Band &v:bands) {
    if(v.count<8 || v.maxY-v.minY<150)continue;
    float mean=v.sum/v.count;
    if(v.count>bestCount || (v.count==bestCount && mean<best)) {
      bestCount=v.count;best=mean;
    }
  }
  return best;
}
// Solo la esquina usa esta lectura: una pared exclusivamente atras no cierra la salida.
float cornerRightWallAhead() {
  const bool right=true;
  struct Band {float sum=0,minY=10000,maxY=-10000;unsigned count=0;};
  Band bands[160];
  float relativeYaw=angleDifference(yaw,initialYaw);
  int center=(int)lroundf((right?90:270)-relativeYaw);
  for(int a=center-60;a<=center+60;++a) {
    int index=(a%360+360)%360;
    if(!ranges[index] || millis()-rangeMs[index]>LIDAR_FRESH_MS)continue;
    float rad=(a+angleDifference(rangeYaw[index],initialYaw))*DEG;
    float x=ranges[index]*sinf(rad)*(right?1:-1),y=ranges[index]*cosf(rad);
    if(x<=0 || x>=6360 || (y<0 || y>400))continue;
    int bin=(int)(x/40);
    for(int b=bin-1;b<=bin;++b) {
      if(b<0 || b>=160)continue;
      Band &v=bands[b];v.sum+=x;++v.count;
      v.minY=fminf(v.minY,y);v.maxY=fmaxf(v.maxY,y);
    }
  }
  unsigned bestCount=0;float best=0;
  for(const Band &v:bands) {
    if(v.count<8 || v.maxY-v.minY<150 || v.minY>80 || v.maxY<200)continue;
    float mean=v.sum/v.count;
    if(v.count>bestCount || (v.count==bestCount && mean<best)) {
      bestCount=v.count;best=mean;
    }
  }
  return best;
}
bool frontWallEvidence(CornerEvidence &ev) {
  unsigned count=0;float minX=0,maxX=0,minY=100000,maxY=-100000;
  uint32_t newestAge=100000;
  for(int a=-40;a<=40;++a) {
    int i=(a+360)%360;uint16_t d=ranges[i];
    uint32_t age=millis()-rangeMs[i];
    if(!d || age>LIDAR_FRESH_MS)continue;
    float rad=(a+angleDifference(rangeYaw[i],yaw))*DEG;
    float x=d*sinf(rad),y=d*cosf(rad);
    if(!(y>=CORNER_FRONT_MIN_MM && y<=CORNER_FRONT_MAX_MM && fabsf(x)<=450))continue;
    if(!count){minX=maxX=x;}
    minX=fminf(minX,x);maxX=fmaxf(maxX,x);
    minY=fminf(minY,y);maxY=fmaxf(maxY,y);
    if(age<newestAge)newestAge=age;
    ++count;
  }
  bool ok=count>=6 && maxX-minX>=150 && (maxY-minY)<=CORNER_FRONT_MAX_DEPTH_MM;
  ev.frontWall=ok;
  ev.frontSpanMM=count?maxX-minX:0;
  ev.frontDistanceMM=count?(minY+maxY)*0.5f:0;
  ev.frontCount=count;ev.frontAge=count?newestAge:0;
  return ok;
}
bool sampleCornerEvidence(CornerEvidence &ev) {
  ev=CornerEvidence();
  frontWallEvidence(ev);
  ev.leftWallMM=laneWallDistance(false);
  ev.leftWall=ev.leftWallMM>0;
  ev.rightWallMM=cornerRightWallAhead();
  ev.rightOpen=ev.rightWallMM<=0;
  ev.cameraAgrees=rawEmptyFrames>=3 && lastRawEmptyMs && millis()-lastRawEmptyMs<=VISION_FRESH_MS;
  return ev.frontWall && ev.rightOpen && ev.leftWall && ev.cameraAgrees;
}
bool parallelCornerReady() {
  return motion==PARALLEL && target.valid && millis()-target.stamp<=LIDAR_FRESH_MS &&
    target.y<=0 && -passSide*target.x>=MIN_CENTER_OFFSET_MM &&
    fabsf(angleDifference(yaw,initialYaw))<=8;
}
bool checkCornerCW() {
  if(currentLane>=2 && !secondLaneSawRight){cornerHoldStartMs=0;return false;}
  bool visionWaitAtEnd=motion==RECOVERING && resumeState==CENTERING && recoveryCause==VISION_MISSING && laneGuaranteedEnd;
  bool fromParallel=parallelCornerReady();
  if((motion!=CENTERING && !visionWaitAtEnd && !fromParallel) ||
     (!fromParallel && millis()-centeringStartedMs<CORNER_MIN_CENTERING_MS)){
    cornerHoldStartMs=0;return false;
  }
  CornerEvidence ev;
  sampleCornerEvidence(ev);
  // La esquina exige UNA recta frontal extensa, alineada con el carril.
  // No basta con el rectangulo que contiene puntos de varias superficies.
  ApproachWall fitted=fitCornerWall(false);
  float wallLaneAngle=angleDifference(yaw+fitted.normalAngle,initialYaw);
  ev.frontWall=fitted.valid && !fitted.held && fitted.age<=150 && fitted.count>=8 &&
    fitted.span>=250 && fitted.gap>=20 && fitted.gap<=600 && fabsf(wallLaneAngle)<=15;
  if(ev.frontWall){ev.frontDistanceMM=fitted.gap+100;ev.frontCount=fitted.count;ev.frontAge=fitted.age;ev.frontSpanMM=fitted.span;}
  lastCornerEvidence=ev;
  bool geometryOK=ev.frontWall && ev.rightOpen && ev.leftWall;
  bool all=geometryOK && (laneGuaranteedEnd || (!nextReady() && ev.cameraAgrees));
  if(!all){cornerHoldStartMs=0;return false;}
  if(!cornerHoldStartMs)cornerHoldStartMs=millis();
  uint32_t needed=currentLane>=2?SECOND_CORNER_CONFIRM_MS:(laneGuaranteedEnd?CORNER_HOLD_FAST_MS:CORNER_HOLD_MS);
  return millis()-cornerHoldStartMs>=needed;
}
void printCornerDiagnostic() {
  const CornerEvidence &ev=lastCornerEvidence;
  Serial.print("[ESQUINA] pared_frontal=");Serial.print(ev.frontWall?"SI":"NO");
  Serial.println("");
}

// --- NUEVA HELPER PARA DISTANCIA DIRECTA ---
// Recta frontal en coordenadas del cuerpo. Solo agrupa puntos cercanos en tiempo.
// RANSAC pequeno excluye retornos aislados; la extension evita usar un pilar de 50mm.
ApproachWall fitCornerWall(bool rear,uint32_t minimumAge) {
  ApproachWall out;
  float xs[61],ys[61];uint32_t ages[61];unsigned n=0;
  uint32_t now=millis(),youngest=FRONT_FIT_FRESH_MS+1;
  for(int a=-60;a<=60;a+=2) {
    int i=(a+(rear?180:0)+360)%360;uint32_t age=now-rangeMs[i];
    if(ranges[i]>=100 && ranges[i]<=1800 && age>=minimumAge && age<=FRONT_FIT_FRESH_MS && age<youngest)youngest=age;
  }
  for(int a=-60;a<=60;a+=2) {
    int i=(a+(rear?180:0)+360)%360;uint32_t age=now-rangeMs[i];
    if(ranges[i]<100 || ranges[i]>1800 || age<minimumAge || age>FRONT_FIT_FRESH_MS || age>youngest+FRONT_FIT_SPAN_MS)continue;
    float rad=(a+angleDifference(rangeYaw[i],yaw))*DEG;
    xs[n]=ranges[i]*sinf(rad);ys[n]=ranges[i]*cosf(rad);ages[n]=age;++n;
  }
  if(n<6)return out;
  float bestNX=0,bestNY=0,bestD=0;unsigned best=0;
  for(unsigned i=0;i<n;i+=3)for(unsigned j=i+4;j<n;j+=3) {
    float dx=xs[j]-xs[i],dy=ys[j]-ys[i],len=hypotf(dx,dy);
    if(len<100)continue;
    float nx=-dy/len,ny=dx/len;
    if(ny<0){nx=-nx;ny=-ny;}
    if(ny<0.766f)continue; // Pared frontal dentro de +/-40 grados.
    float d=nx*xs[i]+ny*ys[i];if(d<80 || d>1200)continue;
    unsigned hits=0;float lo=10000,hi=-10000;
    for(unsigned k=0;k<n;++k)if(fabsf(nx*xs[k]+ny*ys[k]-d)<=12) {
      ++hits;float t=ny*xs[k]-nx*ys[k];lo=fminf(lo,t);hi=fmaxf(hi,t);
    }
    if(hits>=6 && hi-lo>=100 && hits>best){best=hits;bestNX=nx;bestNY=ny;bestD=d;}
  }
  if(best<6)return out;
  float sx=0,sy=0,sxx=0,sxy=0,fitLo=10000,fitHi=-10000;uint32_t sumAge=0;unsigned count=0;
  for(unsigned k=0;k<n;++k)if(fabsf(bestNX*xs[k]+bestNY*ys[k]-bestD)<=12) {
    float projection=bestNY*xs[k]-bestNX*ys[k];fitLo=fminf(fitLo,projection);fitHi=fmaxf(fitHi,projection);
    sx+=xs[k];sy+=ys[k];sxx+=xs[k]*xs[k];sxy+=xs[k]*ys[k];sumAge+=ages[k];++count;
  }
  float variance=sxx-sx*sx/count;if(variance<1)return out;
  float slope=(sxy-sx*sy/count)/variance,scale=sqrtf(1+slope*slope);
  float nx=-slope/scale,ny=1/scale;if(ny<0.766f)return out;
  float d=(sy-slope*sx)/count/scale;
  // Distancia perpendicular a la esquina delantera mas proxima del rectangulo.
  out.gap=d-((rear?LIDAR_TO_REAR_MM:LIDAR_TO_FRONT_MM)*ny+BODY_HALF_WIDTH_MM*fabsf(nx));
  out.span=fitHi-fitLo;
  out.normalAngle=atan2f(nx,ny)/DEG;
  out.age=sumAge/count;out.stamp=now-out.age;out.count=count;out.yawAtFit=yaw;out.valid=true;
  return out;
}

ApproachWall measureApproachWall(){return fitCornerWall(false);}
void clearCornerTracker() {
  lastApproachWall=ApproachWall();approachClosingSpeed=0;approachSpeedKnown=false;
  cornerWallCache[0]=cornerWallCache[1]=ApproachWall();
}
ApproachWall cornerWallReading(bool rear) {
  ApproachWall fresh=fitCornerWall(rear),&cache=cornerWallCache[rear?1:0];
  // Si el barrido trasero nuevo apenas empieza, intentar un grupo anterior
  // todavia fresco, sin mezclarlo con los pocos puntos del barrido nuevo.
  if(rear && !fresh.valid)for(uint32_t age=40;age<=160 && !fresh.valid;age+=40) {
    imuUpdate();serviceLidar();fresh=fitCornerWall(true,age);
  }
  if(fresh.valid && (!cache.valid || (int32_t)(fresh.stamp-cache.stamp)>0))cache=fresh;
  if(!cache.valid || millis()-cache.stamp>WALL_TRACK_MAX_AGE_MS || fabsf(angleDifference(yaw,cache.yawAtFit))>8)return ApproachWall();
  ApproachWall out=cache;out.age=millis()-out.stamp;
  out.held=!fresh.valid || fresh.stamp!=cache.stamp;
  float oldAngle=cache.normalAngle*DEG;
  out.normalAngle=angleDifference(cache.yawAtFit+cache.normalAngle,yaw);
  float newAngle=out.normalAngle*DEG,extent=rear?LIDAR_TO_REAR_MM:LIDAR_TO_FRONT_MM;
  out.gap+=extent*(cosf(oldAngle)-cosf(newAngle))+BODY_HALF_WIDTH_MM*(fabsf(sinf(oldAngle))-fabsf(sinf(newAngle)));
  return out;
}
float estimatedWallGap(const ApproachWall &wall) {
  if(lastApproachWall.valid) {
    int32_t dt=(int32_t)(wall.stamp-lastApproachWall.stamp);
    if(dt>=60 && dt<=400 && fabsf(angleDifference(wall.normalAngle,lastApproachWall.normalAngle))<5) {
      float rate=(lastApproachWall.gap-wall.gap)*1000.0f/dt;
      if(rate>=-100 && rate<=900) {
        rate=fmaxf(0,rate);
        approachClosingSpeed=approachSpeedKnown?0.5f*(approachClosingSpeed+rate):rate;
        approachSpeedKnown=true;
      } else approachSpeedKnown=false;
    } else if(dt>400)approachSpeedKnown=false;
  }
  if(!lastApproachWall.valid || (int32_t)(wall.stamp-lastApproachWall.stamp)>=60)lastApproachWall=wall;
  float speed=approachSpeedKnown?approachClosingSpeed:400.0f;
  return wall.gap-speed*(wall.age+20)*0.001f;
}
float getDirectionDistance(int targetAngle) {
  float minDist = 10000;
  for (int a = targetAngle - 3; a <= targetAngle + 3; ++a) {
    int i = (a + 360) % 360;
    if (ranges[i] > 0 && (millis() - rangeMs[i] <= LIDAR_FRESH_MS)) {
      if (ranges[i] < minDist) minDist = ranges[i];
    }
  }
  return minDist < 10000 ? minDist : 0; // Cero = sin medida valida.
}

// --- TRANSICION REEMPLAZADA ---
void holdCorner() {
  if(completedCorners>=TARGET_LAPS*CORNERS_PER_LAP) {
    stopMotor();setSteering(0);motion=DONE;pending=false;
    Serial.println("[FIN] Limite del carril final alcanzado. Revisar ajuste FINAL_APPROACH_MS.");return;
  }
  // En vez de detenernos permanentemente, iniciamos la maniobra a la pared frontal
  motion = APPROACHING_FRONT;
  motionMs = millis();
  clearCornerTracker();cornerHeadingHits=0;cornerHeadingStamp=0;
  approachLogMs=0;
  Serial.print("[FIN_CARRIL] Esquina CW. Separacion objetivo CHASIS-pared mm: ");
  Serial.println(FRONT_CHASSIS_GAP_MM);
  printCornerDiagnostic();
}

void handleCornerHold() {
  stopMotor();setSteering(0);
}
void prepareSecondLane() {
  stopMotor();setSteering(0);
  ++completedCorners;currentLane=completedCorners+1;
  Serial.print("[VUELTAS] Esquinas completadas: ");Serial.print(completedCorners);
  Serial.print("/12 | vueltas por esquinas: ");Serial.print(completedCorners/4);
  Serial.print(" | entrando al tramo: ");Serial.println(currentLane);
  if(completedCorners==TARGET_LAPS*CORNERS_PER_LAP) {
    finalAdvanceMs=0;finalClockMs=millis();
    Serial.println("[FINAL] Regreso al carril inicial. Tramo final temporizado activo.");
  }
  motion=PREPARE_SECOND_LANE;
  secondLaneExit=false;secondLaneSawRight=false;secondLaneLeftReference=0;
  cornerRightHits=0;cornerRightCheckMs=0;lastRawEmptyMs=lastRawColorMs=0;rawEmptyFrames=0;
  secondLaneStartedMs=millis();secondLaneLogMs=0;
  // Conserva la referencia del nuevo carril calculada con la pared; no recalibra IMU.
  initialYaw=angleDifference(cornerLaneYaw,0);laneReferenceLocked=true;
  headingTarget=0;passSide=0;returningStraight=false;passedPillars=lanePillars=0;laneGuaranteedEnd=false;
  target=PillarReading();previousCandidate=PillarReading();recoveryCandidate=PillarReading();
  resetNextVision();sideConfirmations=stableFrames=emptyStartFrames=recoveryHits=0;
  sideStamp=nextTrackMs=lastTrackCheck=0;
  laneWidthMM=0;laneWidthMs=lastWallReferenceMs=lastLaneSampleMs=0;
  wallRightReport=wallLeftReport=0;cornerHoldStartMs=0;
  clearCornerTracker();cornerWaiting=false;
  for(int i=0;i<360;++i){ranges[i]=0;rangeMs[i]=0;rangeYaw[i]=yaw;}
  bucketDegree=-1;lidarReader.reset();
  for(int i=0;i<32;++i){headingTimes[i]=0;headingHistory[i]=yaw;}headingWrite=0;
  // Invalida transacciones anteriores; la siguiente foto tendra otro identificador.
  pending=false;haveBox=haveResult=photoFinished=false;photoMask=0;
  packedBox=0;result='E';boxQuality='U';photoStartedMs=0;lastPhotoMs=0;
  reader.active=false;reader.used=0;
  Serial.println("[CARRIL] Preparando datos nuevos. Nuevo carril con o sin pilares; busqueda continua.");
}
void serviceSecondLaneStart() {
  stopMotor();setSteering(0);
  if(!yawValido){stopTrial("IMU invalida al entrar al carril 2");return;}
  float left=laneWallDistance(false),right=laneWallDistance(true);
  if(millis()-secondLaneLogMs>=1000){
    secondLaneLogMs=millis();Serial.print("[CARRIL] edad_lidar_ms=");Serial.print(millis()-lastPointMs);
    Serial.print(" fotos_vacias=");Serial.print(noPillarFrames);
    Serial.print(" edad_foto_ms=");Serial.print(nextVisionMs?millis()-nextVisionMs:0);
    Serial.print(" pared_I/D_mm=");Serial.print(left);Serial.print("/");Serial.println(right);
  }
  if(millis()-secondLaneStartedMs<500 || !ultimaMuestraUs || micros()-ultimaMuestraUs>50000 || !lidarStarted || millis()-lastPointMs>150)return;
  if(!nextReady() && (!nextVisionMs || millis()-nextVisionMs>VISION_FRESH_MS || noPillarFrames<3))return;
  if(left<=BODY_HALF_WIDTH_MM+MIN_GAP_MM || frontTooClose())return;
  secondLaneLeftReference=left;secondLaneExit=true;
  float error;bool oneWall;centerReference(error,oneWall);
  if(fabsf(angleDifference(yaw,initialYaw))>15){stopTrial("Rumbo inicial del carril 2 fuera de margen");return;}
  if(nextReady()) {
    target=nextPillar;result=nextColor;
    Serial.println("[CARRIL] Pilar asociado: iniciando evasion en el nuevo carril.");
    beginManeuver();return;
  }
  enterCentering();Serial.println("[CARRIL] Saliendo de esquina con IMU y pared izquierda; derecha abierta permitida.");
}
void printSecondLaneDiagnostic() {
  CornerEvidence ev;sampleCornerEvidence(ev);
  Serial.print("[C2-DIAG] estado=");Serial.print((int)motion);
  Serial.print(" edad_lidar_ms=");Serial.print(millis()-lastPointMs);
  Serial.print(" edad_foto_ms=");Serial.print(nextVisionMs?millis()-nextVisionMs:999999);
  Serial.print(" vacias=");Serial.print(noPillarFrames);
  Serial.print(" color1/2=");Serial.print(photoColors[0]);Serial.print("/");Serial.print(photoColors[1]);
  Serial.print(" pared_I/D_mm=");Serial.print(ev.leftWallMM);Serial.print("/");Serial.print(ev.rightWallMM);
  Serial.print(" frente_mm/ancho/puntos=");Serial.print(ev.frontDistanceMM);Serial.print("/");Serial.print(ev.frontSpanMM);Serial.print("/");Serial.print(ev.frontCount);
  Serial.print(" esquina_F/abreDdelante/I/cam=");Serial.print(ev.frontWall?1:0);Serial.print(ev.rightOpen?1:0);Serial.print(ev.leftWall?1:0);Serial.print(ev.cameraAgrees?1:0);
  Serial.print(" vioD=");Serial.print(secondLaneSawRight?1:0);
  Serial.print(" saliendo=");Serial.print(secondLaneExit?1:0);
  Serial.print(" rumbo/objetivo/servo=");Serial.print(angleDifference(yaw,initialYaw));Serial.print("/");Serial.print(headingTarget);Serial.print("/");Serial.println(lastServoAngle);
}
void resetNextVision() {
  nextConfirmations=noPillarFrames=0;nextColor='E';nextPillar=PillarReading();
  nextVisionMs=0;
}
bool adaptPassToWall(float clearance) {
  float wall=laneWallDistance(passSide>0);
  if(!wall)return true; 
  float gap=wall+clearance-PILLAR_HALF_MM;
  float maxOffset=wall+clearance-BODY_HALF_WIDTH_MM-MIN_GAP_MM;
  if(maxOffset<MIN_CENTER_OFFSET_MM){
    if(!TRY_CLOSE_EVASION){stopTrial("Hueco insuficiente");return false;}
    plannedOffset=fmaxf(BODY_HALF_WIDTH_MM+PILLAR_HALF_MM, maxOffset);
    return true;
  }
  if(maxOffset<PASS_OFFSET_MM) {
    float balanced=PILLAR_HALF_MM+gap*0.5f;
    float adjusted=limitFloat(balanced,MIN_CENTER_OFFSET_MM,maxOffset);
    if(adjusted<plannedOffset){ plannedOffset=adjusted; }
  }
  return true;
}
void driveHeading(float relativeYaw) {
  float right=laneWallDistance(true),left=laneWallDistance(false);
  float limit=BODY_HALF_WIDTH_MM+MIN_GAP_MM;
  if(((headingTarget>1 || relativeYaw>2) && right>0 && right<limit) || ((headingTarget< -1 || relativeYaw< -2) && left>0 && left<limit)) {
    Serial.print("[LATERAL] pared_D/I_mm=");Serial.print(right);Serial.print("/");Serial.print(left);
    Serial.print(" rumbo/objetivo=");Serial.print(relativeYaw);Serial.print("/");Serial.println(headingTarget);
    closeOverride=true;
    closeSteering=(right>0 && right<limit && (left<=0 || right<left))?-40:40;
    headingTarget=0;
  }
  float steering=limitFloat(HEADING_KP*angleDifference(headingTarget,relativeYaw)-HEADING_KD*velocidadZ,-40,40);
  if(closeOverride)steering=closeSteering;
  bool gentle=!closeOverride && motion==MOVING && passedPillars==0 && target.y>FIRST_DIRECT_MM;
  if(gentle) {
    float step=FORWARD_SERVO_SLEW_DEG_S*0.020f,current=lastServoAngle-90;
    setSteering(current+limitFloat(steering-current,-step,step));
  } else setSteering(steering); // Respuesta original inmediata, incluidos cruces y centrado.
  finalForwardCommand=true;
  digitalWrite(AIN1,HIGH);digitalWrite(AIN2,LOW);analogWrite(PWMA,TEST_DRIVE_PWM);
}
bool recentHeadingBound(uint32_t since,float &bound) {
  bound=fabsf(angleDifference(yaw,initialYaw));
  bool bracket=false,recent=false;
  for(unsigned i=0;i<32;++i) {
    if(!headingTimes[i])continue;
    int32_t dt=(int32_t)(headingTimes[i]-since);
    if(dt>=-40 && (int32_t)(millis()-headingTimes[i])>=0) {
      bound=fmaxf(bound,fabsf(angleDifference(headingHistory[i],initialYaw)));
      if(dt<=0)bracket=true;
      if(millis()-headingTimes[i]<=40)recent=true;
    }
  }
  bound+=2; 
  return bracket && recent && bound<75;
}
bool rejoinHasRoom(float desired,float relativeYaw) {
  if(!target.valid || millis()-target.stamp>LIDAR_FRESH_MS)return false;
  float required=PILLAR_HALF_MM*1.414214f+MIN_GAP_MM+REJOIN_EXTRA_GAP_MM;
  uint32_t age=millis()-target.stamp;
  float bound=0,uncertainX=0,uncertainY=0;
  if(recentHeadingBound(target.stamp-80,bound)) {
    uncertainY=0.5f*(age+80); 
    uncertainX=uncertainY*sinf(bound*DEG);
  } else required+=0.5f*age; 
  for(int path=0;path<3;++path) {
    float px=0,py=0;
    for(int step=0;step<=5;++step) {
      float h=path==0?relativeYaw:path==1?desired:relativeYaw+(desired-relativeYaw)*(step/5.0f);
      float cs=cosf(h*DEG),sn=sinf(h*DEG);
      if(step){px+=REJOIN_LOOKAHEAD_MM/5*sn;py+=REJOIN_LOOKAHEAD_MM/5*cs;}
      float x=target.x-px,y=target.y-py-uncertainY*0.5f;
      float bx=x*cs-y*sn,by=x*sn+y*cs;
      float dx=fmaxf(0,fabsf(bx)-BODY_HALF_WIDTH_MM-uncertainX*fabsf(cs)-uncertainY*0.5f*fabsf(sn));
      float dy=fmaxf(0,fabsf(by)-100.0f-uncertainX*fabsf(sn)-uncertainY*0.5f*fabsf(cs));
      if(hypotf(dx,dy)<required)return false;
    }
  }
  return true;
}
void updateRejoin(float relativeYaw) {
  float desired=0;
  if(nextReady()) {
    int nextSide=nextColor=='R'?1:-1;
    float nextClearance=-nextSide*nextPillar.x;
    desired=nextSide*limitFloat((PASS_OFFSET_MM-nextClearance)*0.24f,0,REJOIN_MAX_DEG);
    rejoinReason="SIGUIENTE";
  } else {
    float error;bool oneWall;
    if(!centerReference(error,oneWall)) {
      headingTarget=0;rejoinRequested=0;rejoinReason="SIN_PARED";return;
    }
    desired=limitFloat(error*REJOIN_CENTER_KP,-REJOIN_MAX_DEG,REJOIN_MAX_DEG);
    rejoinReason="CENTRO";
  }
  rejoinRequested=desired;
  float step=REJOIN_SLEW_DEG_S*0.020f;
  float candidate=headingTarget+limitFloat(desired-headingTarget,-step,step);
  for(int i=0;i<6;++i) {
    if(rejoinHasRoom(candidate,relativeYaw)){
      headingTarget=candidate;
      if(i)rejoinReason="MARGEN_PILAR";
      return;
    }
    candidate*=0.5f;
  }
  headingTarget=0;rejoinReason="MARGEN_PILAR";
}
void beginManeuver() {
  if(!yawValido || !ultimaMuestraUs || micros()-ultimaMuestraUs>50000) {
    stopTrial("IMU no disponible");return;
  }
  bool establish=passedPillars==0 && !laneReferenceLocked;
  float rot=establish?angleDifference(yaw,initialYaw)*DEG:0;
  float x=target.x*cosf(rot)-target.y*sinf(rot);
  float y=target.x*sinf(rot)+target.y*cosf(rot);
  target.x=x;target.y=y;if(establish)initialYaw=yaw;
  laneReferenceLocked=true;
  float wallError;bool oneWall;centerReference(wallError,oneWall);
  passSide=result=='R'?1:-1;
  plannedOffset=PASS_OFFSET_MM;
  returningStraight=-passSide*target.x>=MIN_CENTER_OFFSET_MM;
  sideConfirmations=0;sideStamp=0;resetNextVision();
  motion=MOVING;motionMs=millis();lastTrackCheck=0;
}
bool nextReady() {
  return nextConfirmations>=1 && nextPillar.valid && (nextColor=='R' || nextColor=='V') && nextVisionMs && millis()-nextVisionMs<=VISION_FRESH_MS && millis()-nextPillar.stamp<=LIDAR_FRESH_MS;
}
bool distinctFromCurrent(const PillarReading &c) {
  MotionState phase=effectiveState();
  if(phase!=MOVING && phase!=PARALLEL)return true;
  return target.valid && millis()-target.stamp<=TRACK_LOST_MS && c.y>target.y+140 && hypotf(c.x-target.x,c.y-target.y)>160;
}
void trackNextPillar() {
  if(!nextConfirmations || !nextPillar.valid)return;
  if(!nextVisionMs || millis()-nextVisionMs>VISION_FRESH_MS || millis()-nextPillar.stamp>TRACK_LOST_MS) {
    resetNextVision();return;
  }
  if(millis()-nextTrackMs<60)return;
  nextTrackMs=millis();
  float radius=hypotf(nextPillar.x,nextPillar.y);
  float gate=45+0.5f*(millis()-nextPillar.stamp);
  float half=limitFloat(asinf(limitFloat(gate/fmaxf(radius,1),0,1))/DEG+12,20,65);
  float bearing=atan2f(nextPillar.x,nextPillar.y)/DEG-angleDifference(yaw,initialYaw);
  PillarReading c=findPillar(bearing,half,true,&nextPillar);
  if(c.valid && distinctFromCurrent(c))nextPillar=c;
}
// Conservative lateral gate in lane coordinates (same frame as PillarReading).
// A missing wall alone is not evidence that a pillar belongs to another lane.
static const uint32_t LANE_FILTER_WIDTH_AGE_MS=1000;
static const float LANE_FILTER_MARGIN_MM=80;
uint32_t laneRejectLogMs=0;
bool pillarOutsideCurrentLane(const PillarReading &c) {
  if(!FILTER_OUTSIDE_LANE || !c.valid)return false;
  float right=laneWallDistance(true),left=laneWallDistance(false);
  bool recentWidth=laneWidthMM>0 && laneWidthMs && millis()-laneWidthMs<=LANE_FILTER_WIDTH_AGE_MS;
  if(right<=0 && left>0 && recentWidth && left<laneWidthMM)right=laneWidthMM-left;
  if(left<=0 && right>0 && recentWidth && right<laneWidthMM)left=laneWidthMM-right;
  if(right<=0 || left<=0){visionDecision="LIMITES DE CARRIL INCIERTOS: SEGUIR BUSCANDO";return true;}
  bool outside=(right>0 && c.x>right+LANE_FILTER_MARGIN_MM) ||
               (left>0 && c.x< -left-LANE_FILTER_MARGIN_MM);
  if(outside && millis()-laneRejectLogMs>=500) {
    laneRejectLogMs=millis();
    Serial.print("[VISION CARRIL] Fuera del limite lateral: X=");Serial.print(c.x);
    Serial.print(" paredes I/D=");Serial.print(left);Serial.print("/");Serial.println(right);
  }
  return outside;
}
void reportNextPhoto() {
  visionDecision="ANALIZANDO";
  if(laneGuaranteedEnd && effectiveState()==CENTERING) {
    // Los dos pilares de este carril ya se completaron. No asociar un tercero
    // hasta preparar el nuevo carril; la proteccion frontal sigue activa.
    visionDecision="DOS PILARES COMPLETADOS: BUSCANDO ESQUINA";return;
  }
  if(nextVisionMs && millis()-nextVisionMs>VISION_FRESH_MS)resetNextVision();
  if(photoMoving && photoStartedMs && (millis()-photoStartedMs>900 || fabsf(angleDifference(yaw,photoYaw))>10)) {visionDecision="FOTO VIEJA O GIRO DESDE CAPTURA";return;}
  PillarReading best;
  char bestColor='E';float bestX=0;
  bool empty=true, uncertainPhoto=false, sawCurrent=false, colorConflict=false;
  unsigned outsideCount=0;
  unsigned total=photoMask==3?2:1;
  for(unsigned i=0;i<total;++i) {
    char color=photoMask==3?photoColors[i]:result;
    uint32_t box=photoMask==3?photoBoxes[i]:packedBox;
    if(color=='N')continue;
    empty=false;
    unsigned x=box>>24,y=(box>>16)&255,w=(box>>8)&255,h=box&255;
    bool clipped=color=='r'||color=='v';
    if(clipped && x>0 && x+w<160)color=color=='r'?'R':'V';
    if((color!='R' && color!='V') || !w || !h || x+w>160 || y+h>120 || (photoMask!=3 && boxQuality!='V')) {
      visionDecision="CUADRO INVALIDO O RECORTADO";uncertainPhoto=true;continue;
    }
    float cx=x+(w-1)*0.5f;
    float span=fabsf(imageAngle(x+w-1)-imageAngle(x))*0.5f+6;
    float turn=photoStartedMs?angleDifference(yaw,photoYaw):0;
    PillarReading c=findPillar(imageAngle(cx)-turn*0.5f,span,false);
    if(!c.valid || c.distance>1500 || millis()-c.stamp>LIDAR_FRESH_MS) { visionDecision="SIN ASOCIACION LIDAR RECIENTE";uncertainPhoto=true;continue; }
    if(pillarOutsideCurrentLane(c)){++outsideCount;continue;}
    if(!distinctFromCurrent(c)){sawCurrent=true;continue;}
    if(c.y<=180){uncertainPhoto=true;continue;}
    if(best.valid && bestColor!=color && hypotf(c.x-best.x,c.y-best.y)<100)colorConflict=true;
    if(!best.valid || c.y<best.y){best=c;bestColor=color;bestX=cx;}
  }
  if(outsideCount && !best.valid){visionDecision="OBJETIVO FUERA DEL CARRIL O LIMITES INCIERTOS";}
  if(colorConflict){best.valid=false;uncertainPhoto=true;}
  if(best.valid) {
    visionDecision="PILAR ASOCIADO";
    Serial.print("[OBJETIVO] ");Serial.print(visionColorName(bestColor));Serial.print(" X/Y_mm=");Serial.print(best.x);Serial.print("/");Serial.println(best.y);
    bool same=nextPillar.valid && nextColor==bestColor && hypotf(best.x-nextPillar.x,best.y-nextPillar.y)<120;
    nextConfirmations=same?nextConfirmations+1:1;
    if(nextConfirmations>3)nextConfirmations=3;
    nextPillar=best;nextColor=bestColor;nextImageX=bestX;
    nextVisionMs=millis();noPillarFrames=0;
    if(motion==CENTERING){stopMotor();setSteering(0);motion=DECIDE_NEXT;motionMs=millis();}
    return;
  }
  if(nextReady())return;
  if(empty) {
    visionDecision="SIN PILARES VALIDOS EN ESTA FOTO";
    unsigned emptyCount=noPillarFrames;
    resetNextVision();nextColor='N';nextVisionMs=millis();
    noPillarFrames=emptyCount<3?emptyCount+1:3;
  } else {
    // Una foto incierta no cancela inmediatamente una trayectoria con vision reciente.
    if(motion==CENTERING && (!nextVisionMs || millis()-nextVisionMs>VISION_FRESH_MS) && !viewAdjustmentAllowed())pauseMotion(VISION_MISSING,"Esperando asociacion");
  }
}
const char *visionColorName(char c) {
  switch(c){case 'R':return "ROJO";case 'V':return "VERDE";
    case 'r':return "ROJO RECORTADO";case 'v':return "VERDE RECORTADO";
    case 'N':return "NINGUNO";default:return "INVALIDO";}
}
void reportPhoto() {
  visionDecision="FOTO RECIBIDA";
  if(photoMask==3) {
    bool rawEmpty=photoColors[0]=='N' && photoColors[1]=='N';
    if(rawEmpty){if(rawEmptyFrames<3)++rawEmptyFrames;lastRawEmptyMs=millis();}
    else {rawEmptyFrames=0;lastRawEmptyMs=0;lastRawColorMs=millis();}
    Serial.print("[CAMARA FOTO] id=");Serial.print(requestId);
    Serial.print(" ve=");Serial.print(visionColorName(photoColors[0]));Serial.print(" / ");Serial.print(visionColorName(photoColors[1]));
    Serial.print(" edad_ms=");Serial.println(millis()-photoStartedMs);
  }
  MotionState phase=effectiveState();
  if(phase==PREPARE_SECOND_LANE) {
    // Usa ambas detecciones y la misma asociacion LiDAR que el primer carril.
    // Una foto vacia permite avanzar tras confirmar; no declara terminado el carril.
    if(photoMask==3)reportNextPhoto();
    return;
  }
  bool lookingNext=phase==MOVING || phase==PARALLEL || phase==DECIDE_NEXT || phase==CENTERING;
  if(lookingNext){reportNextPhoto();return;}
  if(motion!=WAIT_PILLAR)return;
  bool empty=photoMask==3 && photoColors[0]=='N' && photoColors[1]=='N';
  emptyStartFrames=empty?emptyStartFrames+1:0;
  if(empty) {
    debugVision="FOTOS VACIAS: faltan 3 fotos, datos frescos, referencia de pared o frente libre";
    stableFrames=0;previousColor='E';
    if(emptyStartFrames>=3 && yawValido && ultimaMuestraUs && micros()-ultimaMuestraUs<=50000 && lidarStarted && millis()-lastPointMs<=150) {
      float error;bool oneWall;
      bool referenceOK=centerReference(error,oneWall);
      bool frontBlocked=referenceOK && frontTooClose();
      debugVision=!referenceOK?"FOTOS VACIAS OK: FALTA REFERENCIA DE PARED":frontBlocked?"FOTOS VACIAS OK: RETORNO CERCANO AL FRENTE":"FOTOS VACIAS OK";
      if(referenceOK && !frontBlocked) {
        laneReferenceLocked=true;nextColor='N';nextVisionMs=millis();noPillarFrames=3;
        enterCentering();Serial.println("[CARRIL] Sin pilares en tres fotos validas. Avanzando y buscando.");
      }
    }
    return;
  }
  unsigned x=packedBox>>24,y=(packedBox>>16)&255,w=(packedBox>>8)&255,h=packedBox&255;
  if((result!='R' && result!='V') || boxQuality!='V' || !w || !h || x+w>160 || y+h>120) {
    debugVision="FOTO INVALIDA, COLOR NO FIABLE O CUADRO INVALIDO";
    stableFrames=0;previousColor='E'; return;
  }
  float cx=x+(w-1)*0.5f,bearing=imageAngle(cx);
  float span=fabsf(imageAngle(x+w-1)-imageAngle(x))*0.5f+6;
  PillarReading c=findPillar(bearing,span,false);
  bool outside= c.valid && pillarOutsideCurrentLane(c);
  bool usable=!outside && c.valid && c.distance>=START_MIN_MM && c.distance<=START_MAX_MM && fabsf(c.x)<=100 && millis()-c.stamp<=150;
  debugVision=outside?"PILAR FUERA DEL CARRIL":!c.valid?"PILAR SIN ASOCIACION LIDAR":
    c.distance<START_MIN_MM?"PILAR DEMASIADO CERCA PARA ARRANQUE":
    c.distance>START_MAX_MM?"PILAR DEMASIADO LEJOS PARA ARRANQUE":
    fabsf(c.x)>100?"PILAR FUERA DEL CENTRO (+/-100 mm)":
    millis()-c.stamp>150?"LECTURA DEL PILAR VIEJA (>150 ms)":"CONFIRMANDO PILAR: requiere 3 detecciones estables";
  if(usable) {
    bool same=result==previousColor && previousCandidate.valid && fabsf(cx-previousX)<=12 && hypotf(c.x-previousCandidate.x,c.y-previousCandidate.y)<60;
    stableFrames=same?stableFrames+1:1;
  } else stableFrames=0;
  previousColor=result;previousX=cx;previousCandidate=c;
  if(stableFrames>=3){target=c;beginManeuver();}
}
void decideNext() {
  if(!nextVisionMs || millis()-nextVisionMs>2500){enterCentering();return;}
  if(nextReady()) {
    float expected=atan2f(nextPillar.x,nextPillar.y)/DEG-angleDifference(yaw,initialYaw);
    PillarReading c=findPillar(expected,20,false);
    if(!c.valid || c.y<=0 || hypotf(c.x-nextPillar.x,c.y-nextPillar.y)>80){enterCentering();return;}
    int side=nextColor=='R'?1:-1;
    float shift=fmaxf(0,PASS_OFFSET_MM+side*c.x);
    target=c;result=nextColor;beginManeuver();return;
  }
  if(noPillarFrames>=3) {
    enterCentering();
  }
}
float maneuverHeading(float clearance) {
  crossingDemand=0;
  if(returningStraight)return 0;
  float deficitLateral=fmaxf(0,plannedOffset-clearance);
  float desired=deficitLateral*0.24f;
  if(!passedPillars && target.y>FIRST_DIRECT_MM) {
    float gentle=atan2f(deficitLateral,fmaxf(100,target.y-FIRST_EVASION_LOOKAHEAD_MM))/DEG;
    float blend=limitFloat((target.y-FIRST_DIRECT_MM)/(FIRST_SMOOTH_MM-FIRST_DIRECT_MM),0,1);
    desired=desired*(1-blend)+gentle*blend;
  }
  if(passedPillars && target.y>0) {
    float deficit=MIN_CENTER_OFFSET_MM+CROSS_MARGIN_EXTRA_MM-clearance;
    if(deficit>0) {
      float forward=fmaxf(30,target.y-CROSS_MARGIN_BY_Y_MM);
      crossingDemand=atan2f(deficit,forward)/DEG;
      desired=fmaxf(desired,crossingDemand);
    }
  }
  return passSide*limitFloat(desired,0,passedPillars?MAX_CROSS_HEADING_DEG:MAX_HEADING_DEG);
}

// --- ACTUALIZACION DE SECUENCIA PRINCIPAL ---
void updateMotion() {
  closeOverride=false;
  headingTimes[headingWrite]=millis();headingHistory[headingWrite]=yaw;
  headingWrite=(headingWrite+1)%32;
  if(motion==PREPARE_SECOND_LANE){serviceSecondLaneStart();return;}
  if(motion==DONE){stopMotor();return;}
  if(enabled && lidarStarted && millis()-lastLaneSampleMs>=100) {
    lastLaneSampleMs=millis();float error;bool oneWall;centerReference(error,oneWall);
  }

  // Esquina CW: referencia de pared, seguimiento temporal y reversa guiada por IMU.
  bool cornerMoving=motion==APPROACHING_FRONT || motion==WAIT_BEFORE_REVERSE || motion==REVERSING_TURN || motion==REVERSING_STRAIGHT;
  if(cornerMoving) {
    if(cornerWaiting){motionMs+=millis()-cornerWaitTick;cornerWaitTick=millis();}
    if(cornerWaiting && !ultimaMuestraUs && yawValido && lidarRestartMs && millis()-lidarRestartMs<100){stopMotor();return;}
    if(!yawValido || !ultimaMuestraUs || micros()-ultimaMuestraUs>50000){stopTrial("IMU invalida en esquina");return;}
    if(millis()-motionMs>CORNER_STAGE_TIMEOUT_MS){stopTrial("Tiempo agotado en etapa de esquina");return;}
    // La IMU debe poder finalizar el giro aun cuando falte el flujo LiDAR.
    if(motion==REVERSING_TURN && angleDifference(cornerLaneYaw,yaw)<=CORNER_TURN_LEAD_DEG) {
      float error=angleDifference(cornerLaneYaw,yaw);
      stopMotor();setSteering(0);clearCornerTracker();motion=REVERSING_STRAIGHT;motionMs=millis();
      cornerWaiting=false;
      Serial.print("[GIRO] Iniciando reversa guiada. Error rumbo grados=");Serial.println(error);
      return;
    }
    bool lidarFresh=lidarStarted && millis()-lastPointMs<=LIDAR_FRESH_MS;
    ApproachWall wall;
    if(lidarFresh && motion!=REVERSING_TURN)wall=cornerWallReading(motion==REVERSING_STRAIGHT);
    bool missing=!lidarFresh || (motion!=REVERSING_TURN && !wall.valid);
    if(missing) {
      stopMotor();setSteering(0); // No dejar ruedas a tope mientras el carro se detiene.
      if(!cornerWaiting){cornerWaitSince=cornerWaitTick=millis();cornerWaitLog=0;cornerWaiting=true;}
      if(!cornerWaitLog || millis()-cornerWaitLog>=1000) {
        cornerWaitLog=millis();Serial.print("[ESQUINA_ESPERA] causa=");
        Serial.print(!lidarFresh?"SIN_UART_LIDAR":motion==REVERSING_STRAIGHT?"AJUSTE_PARED_TRASERA":"AJUSTE_PARED_FRONTAL");
        Serial.print(" edad_uart_ms=");Serial.print(millis()-lastPointMs);
        Serial.print(" error_rumbo_deg=");Serial.println(angleDifference(cornerLaneYaw,yaw));
      }
      return;
    }
    if(cornerWaiting){Serial.println("[ESQUINA] Datos recuperados. Reanudando etapa.");cornerWaiting=false;}

    if(motion==WAIT_BEFORE_REVERSE) {
      stopMotor();setSteering(0);
      // Solo orientaciones nuevas, tomadas con el robot ya detenido.
      if(!wall.held && (int32_t)(wall.stamp-motionMs)>=200 && wall.stamp!=cornerHeadingStamp) {
        float normalHeading=yaw+wall.normalAngle;
        bool same=cornerHeadingHits && fabsf(angleDifference(normalHeading,cornerWallHeading))<3;
        cornerWallHeading=same?cornerWallHeading+0.5f*angleDifference(normalHeading,cornerWallHeading):normalHeading;
        cornerHeadingHits=same?cornerHeadingHits+1:1;cornerHeadingStamp=wall.stamp;
      }
      if(millis()-motionMs>=CORNER_DIRECTION_PAUSE_MS && cornerHeadingHits>=2) {
        Serial.print("[FRENTE_FINAL] hueco_chasis_mm=");Serial.print(wall.gap);
        Serial.print(" inclinacion_deg=");Serial.println(wall.normalAngle);
        cornerLaneYaw=cornerWallHeading+90.0f;turnStartYaw=yaw;
        float required=angleDifference(cornerLaneYaw,turnStartYaw);
        if(required<45 || required>135){stopTrial("Referencia de esquina incompatible");return;}
        clearCornerTracker();motion=REVERSING_TURN;motionMs=millis();
        Serial.print("[GIRO] Objetivo CW relativo grados=");Serial.println(required);
      }
      return;
    }
    if(motion==REVERSING_TURN) {
      setSteering(-40);digitalWrite(AIN1,LOW);digitalWrite(AIN2,HIGH);analogWrite(PWMA,CORNER_DRIVE_PWM);
      return;
    }
    bool rear=motion==REVERSING_STRAIGHT;
    float estimatedGap=estimatedWallGap(wall);
    if(millis()-approachLogMs>=250) {
      approachLogMs=millis();Serial.print(rear?"[ATRAS] hueco_mm=":"[FRENTE] hueco_mm=");Serial.print(wall.gap);
      Serial.print(" estimado_actual_mm=");Serial.print(estimatedGap);
      Serial.print(" inclinacion_deg=");Serial.print(wall.normalAngle);
      Serial.print(" edad_ms=");Serial.print(wall.age);
      Serial.print(" entre_barridos=");Serial.println(wall.held?"SI":"NO");
    }
    float stopGap=rear?REAR_CHASSIS_GAP_MM+REAR_STOP_RESERVE_MM:FRONT_CHASSIS_GAP_MM+FRONT_STOP_RESERVE_MM;
    if(estimatedGap<=stopGap) {
      stopMotor();setSteering(0);motionMs=millis();
      if(rear) {
        motion=FINISHED_SEQ;finalStopMs=millis();rearFinalLogged=false;
        Serial.println("[ESQUINA] Secuencia terminada. Verificando separacion trasera en reposo.");
      } else {
        approachStopGap=wall.gap;cornerHeadingHits=0;cornerHeadingStamp=0;
        motion=WAIT_BEFORE_REVERSE;
        Serial.println("[ESQUINA] Parada frontal. Pausa minima 500ms y referencia estable de pared.");
      }
    } else if(rear) {
// Al retroceder se invierte el signo de la correccion de direccion.
      float error=angleDifference(cornerLaneYaw,yaw);
      setSteering(limitFloat(-REVERSE_HEADING_KP*error+HEADING_KD*velocidadZ,-REVERSE_STEER_LIMIT,REVERSE_STEER_LIMIT));
      digitalWrite(AIN1,LOW);digitalWrite(AIN2,HIGH);analogWrite(PWMA,CORNER_DRIVE_PWM);
    } else {
      setSteering(0);digitalWrite(AIN1,HIGH);digitalWrite(AIN2,LOW);analogWrite(PWMA,CORNER_DRIVE_PWM);
    }
    return;
  }
  if(motion==FINISHED_SEQ) {
    stopMotor();
    if(!rearFinalLogged && millis()-finalStopMs>=500) {
      ApproachWall wall=fitCornerWall(true);
      if(wall.valid && wall.age<=150 && (int32_t)(wall.stamp-finalStopMs)>=300) {
        Serial.print("[ATRAS_FINAL] hueco_chasis_mm=");Serial.print(wall.gap);
        Serial.print(" error_rumbo_deg=");Serial.println(angleDifference(cornerLaneYaw,yaw));rearFinalLogged=true;
      } else if(millis()-finalStopMs>1500){Serial.println("[ATRAS_FINAL] Sin medida valida en reposo.");rearFinalLogged=true;}
    }
    if(completedCorners<TARGET_LAPS*CORNERS_PER_LAP && rearFinalLogged && millis()-finalStopMs>=500 && fabsf(velocidadZ)<3)prepareSecondLane();
    return;
  }

  if(motion==CORNER_HOLD){handleCornerHold();return;}
  if(motion==RECOVERING){serviceRecovery();return;}
  trackNextPillar();
  if(motion==DECIDE_NEXT){decideNext();return;}
  if(!drivingState())return;
  
  if(!yawValido || !ultimaMuestraUs || micros()-ultimaMuestraUs>50000) {stopTrial("IMU invalida");return;}
  if(!lidarStarted || millis()-lastPointMs>250){pauseMotion(RX_MISSING,"Sin datos LiDAR recientes");return;}
  if(motion!=CENTERING && millis()-motionMs>MAX_MOTION_MS){motionMs=millis();Serial.println("[CONTINUA] Evasion prolongada: conserva objetivo y busqueda.");}
  
  if((motion==CENTERING || motion==PARALLEL) && checkCornerCW()){
    if(motion==PARALLEL)Serial.println("[ESQUINA DESDE COSTADO] Pilar detras del LiDAR, margen lateral y geometria confirmados.");
    holdCorner();return;
  }
  
  if(frontTooClose()){
    bool ignorable=false;
    if(motion==MOVING && target.valid && millis()-target.stamp<=LIDAR_FRESH_MS) {
      float rot=angleDifference(yaw,initialYaw)*DEG;
      float fx=frontDiag.x*cosf(rot)-frontDiag.y*sinf(rot);
      float fy=frontDiag.x*sinf(rot)+frontDiag.y*cosf(rot);
      float knownClearance=-passSide*target.x;
      ignorable=knownClearance>=MIN_CENTER_OFFSET_MM && hypotf(fx-target.x,fy-target.y)<=70;
    }
    if(!ignorable) {
      bool known=false;
      if(target.valid && millis()-target.stamp<=LIDAR_FRESH_MS && passSide) {
        float rot=angleDifference(yaw,initialYaw)*DEG;
        float fx=frontDiag.x*cosf(rot)-frontDiag.y*sinf(rot);
        float fy=frontDiag.x*sinf(rot)+frontDiag.y*cosf(rot);
        known=hypotf(fx-target.x,fy-target.y)<=100;
      }
      if(TRY_CLOSE_EVASION && known) {
        closeOverride=true;closeSteering=passSide*40.0f;
        if(millis()-closeReportMs>=500){closeReportMs=millis();Serial.println("[EVASION CERCA] Pilar asociado: intentando paso a maxima direccion, sin parada por cercania.");}
      } else {
        closeOverride=true;closeSteering=frontDiag.x>=0?-40:40;
        if(millis()-closeReportMs>=500){closeReportMs=millis();Serial.println("[CORRIGE] Retorno frontal: alejandose del lado ocupado.");}
      }
    }
  }

  float relativeYaw=angleDifference(yaw,initialYaw);
  float yawLimit=passedPillars?MAX_CROSS_HEADING_DEG+10:55;
  if(fabsf(relativeYaw)>yawLimit){headingTarget=0;closeOverride=true;closeSteering=relativeYaw>0?-40:40;driveHeading(relativeYaw);return;}
  
  if(motion==CENTERING) {
    if(!laneGuaranteedEnd && (!nextVisionMs || millis()-nextVisionMs>VISION_FRESH_MS) && !viewAdjustmentAllowed()){
      pauseMotion(VISION_MISSING,"Foto pendiente; conservando control de carril");
    }
    float error;bool oneWall;
    if(centerReference(error,oneWall)){
      float limit=oneWall?12.0f:25.0f;
      float desired=limitFloat(error*0.15f,-limit,limit);
      float step=CENTER_SLEW_DEG_S*0.020f;
      headingTarget+=limitFloat(desired-headingTarget,-step,step);
    } else {
      headingTarget=0;
      if(!lastWallReferenceMs || millis()-lastWallReferenceMs>WALL_GRACE_MS){ pauseMotion(WALLS_MISSING,"Sin pared; manteniendo rumbo IMU"); }
    }
    driveHeading(relativeYaw);return;
  }

  if(millis()-lastTrackCheck>=30) {
    lastTrackCheck=millis();
    float expected=atan2f(target.x,target.y)/DEG-angleDifference(yaw,initialYaw);
    PillarReading c=findPillar(expected,trackingHalfWindow(),true);
    if(c.valid)target=c;
  }
  if(millis()-target.stamp>TRACK_LOST_MS){
    if(finishKnownPassedPillar())return;
    Serial.print("[PILAR_PERDIDO] x/y_mm=");Serial.print(target.x);Serial.print("/");Serial.print(target.y);
    Serial.print(" edad_ms=");Serial.println(millis()-target.stamp);
    // Color conservado. No fingir que el pilar fue pasado ni iniciar esquina.
    float turn=angleDifference(yaw,initialYaw);
    headingTarget=0;closeOverride=true;
    closeSteering=(passSide*turn<35)?passSide*40.0f:limitFloat(HEADING_KP*angleDifference(passSide*35.0f,turn)-HEADING_KD*velocidadZ,-40,40);
    // Ampliar la readquisicion con puntos nuevos; no integrar la posicion antigua como actual.
    reacquiring=true;uint32_t savedPause=pauseStartedMs;pauseStartedMs=millis()-400;
    PillarReading recovered=findPillar(-passSide*90.0f-turn,100,true);
    pauseStartedMs=savedPause;reacquiring=false;
    if(recovered.valid){target=recovered;lastTrackCheck=0;}
    driveHeading(turn);return;
  }
  float clearance=-passSide*target.x;
  if(!adaptPassToWall(clearance))return;
  if(target.y<100 && clearance<MIN_CENTER_OFFSET_MM){
    Serial.print("[PILAR] x/y_mm=");Serial.print(target.x);Serial.print("/");Serial.print(target.y);
    Serial.print(" separacion_centros_mm=");Serial.print(clearance);Serial.print(" edad_ms=");Serial.println(millis()-target.stamp);
    if(!TRY_CLOSE_EVASION){stopTrial("Margen lateral no logrado");return;}
    closeOverride=true;closeSteering=passSide*40.0f;
  }
  if(clearance>=fmaxf(MIN_CENTER_OFFSET_MM,plannedOffset-15))returningStraight=true;
  if(motion==MOVING)headingTarget=maneuverHeading(clearance);
  if(motion==MOVING && target.stamp!=sideStamp) {
    sideStamp=target.stamp;
    bool beside=target.y<=80 && clearance>=MIN_CENTER_OFFSET_MM && fabsf(relativeYaw)<8 && millis()-target.stamp<=LIDAR_FRESH_MS;
    sideConfirmations=beside?sideConfirmations+1:0;
    if(sideConfirmations>=1) {
      motion=PARALLEL;returningStraight=true;lastPhotoMs=0;
    }
  }
  if(motion==PARALLEL)updateRejoin(relativeYaw);
  
  float rearExtent=100*cosf(relativeYaw*DEG)+BODY_HALF_WIDTH_MM*fabsf(sinf(relativeYaw*DEG));
  if(motion==PARALLEL && target.y<-(rearExtent+PILLAR_HALF_MM+20) && clearance>=MIN_CENTER_OFFSET_MM && fabsf(relativeYaw)<=REJOIN_MAX_DEG+2) {
    ++passedPillars;
    ++lanePillars;
    if(lanePillars>=2 && !laneGuaranteedEnd) {
      laneGuaranteedEnd=true;
    }
    if(nextReady()) {
      stopMotor();setSteering(0);motion=DECIDE_NEXT;
    } else {
      enterCentering();
    }
    return;
  }
  driveHeading(relativeYaw);
}
void requestPhoto(bool retry=false) {
  if(!retry) {
    ++requestId;if(!requestId)++requestId;
    haveBox=haveResult=false;packedBox=0;result='E';boxQuality='U';sends=0;
    photoMask=0;photoFinished=false;photoStartedMs=millis();photoYaw=yaw;
    photoMoving=drivingState();
    for(int i=0;i<2;++i){photoBoxes[i]=0;photoColors[i]='E';}
  }
  camSend(Serial0,'Q',session,requestId,cameraBoot,'3');
  pending=true;sentMs=millis();++sends;
}
void serviceCamera() {
  CamFrame f;int budget=256;
  while(Serial0.available() && budget-->0) {
    ++debugCamBytes;
    char received=(char)Serial0.read();
    if(!cameraBoot && debugRawCount<sizeof(debugRawBytes))debugRawBytes[debugRawCount++]=(uint8_t)received;
    if(!reader.push(received,f))continue;
    ++debugCamFrames;
    if(f.session!=session){++debugForeignFrames;continue;}
    if((f.type=='U' && f.data=='R' && f.id==1) || (f.type=='B' && f.data=='B')) {
      if(!f.boot)continue;
      if(cameraBoot!=f.boot) {
        if(cameraBoot && drivingState()) pauseMotion(VISION_MISSING,"Camara reiniciada");
        resetNextVision();
        cameraBoot=f.boot;pending=false;stableFrames=0;previousColor='E';
      }
    } else if(f.type=='S' && f.boot==cameraBoot && cameraBoot && f.id && (f.data=='0'||f.data=='1')) {
      camSend(Serial0,'K',session,f.id,cameraBoot,'K');
      ++debugButtons;
      if(!AUTO_START_ENABLED && !enabled){enabled=true;buttonMs=millis();Serial.println("[ARRANQUE BOTON] Orden recibida y aceptada por C6. Espera minima: 500 ms.");}
    } else if(pending && f.id==requestId) {
      if((f.type=='X' || f.type=='Y') && (f.data=='R'||f.data=='V'||f.data=='r'||f.data=='v'||f.data=='N'||f.data=='E')) {
        unsigned i=f.type=='X'?0:1;photoBoxes[i]=f.boot;photoColors[i]=f.data;photoMask|=1u<<i;
      } else if(f.type=='Z' && f.boot==cameraBoot && f.data=='2') {
        photoFinished=true;
      } else if(f.type=='P' && (f.data=='V'||f.data=='U')) {
        packedBox=f.boot;boxQuality=f.data;haveBox=true;
      } else if(f.type=='R' && f.boot==cameraBoot && (f.data=='R'||f.data=='V'||f.data=='N'||f.data=='E')) {
        result=f.data;haveResult=true;
      }
    }
  }
  if(pending && photoFinished && photoMask==3) {
    packedBox=photoBoxes[0];result=photoColors[0];
    boxQuality=(result=='R'||result=='V')?'V':'U';
    ++debugPhotos;
    reportPhoto();
    Serial.print("[VISION DECISION] ");Serial.println(visionDecision);
    pending=false;lastPhotoMs=millis();
  }
  if((!enabled || !cameraBoot) && millis()-lastHelloMs>=500) {
    lastHelloMs=millis();camSend(Serial0,'T',session,1,0,'?');++debugHelloSends;
  }
  if(pending && millis()-sentMs>=RETRY_MS) {
    if(sends<3)requestPhoto(true);
    else {
      pending=false;lastPhotoMs=millis();stableFrames=0;previousColor='E';
      ++debugTimeouts;cameraBoot=0;
    }
  }
  MotionState phase=effectiveState();
  bool early=phase==MOVING && returningStraight && fabsf(angleDifference(yaw,initialYaw))<15;
  bool capturePhase=phase==PREPARE_SECOND_LANE || phase==WAIT_PILLAR || early || phase==PARALLEL || phase==DECIDE_NEXT || phase==CENTERING;
  if(enabled && cameraBoot && !pending && capturePhase && millis()-buttonMs>=500 && millis()-lastPhotoMs>=PHOTO_INTERVAL_MS)requestPhoto();
}
void setup() {
  pinMode(PWMA,OUTPUT);pinMode(AIN1,OUTPUT);pinMode(AIN2,OUTPUT);stopMotor();
  Serial.begin(115200);
  Serial0.setRxBufferSize(512);Serial0.begin(38400,SERIAL_8N1,CAM_RX,CAM_TX);
  ledcAttach(SERVO_PIN,50,16);setSteering(0);
  ledcAttach(LIDAR_MOTOR,500,8);ledcWrite(LIDAR_MOTOR,200);
  lidar.init(LIDAR_RX,LIDAR_TX);lidar.postParseCallback=lidarPoint;
  Serial.println("[ARRANQUE] Inicializando y calibrando IMU. Mantener quieto.");
  imuSetup();
  Serial.println("[ARRANQUE] IMU preparada. Iniciando escaneo LiDAR...");
  lidarStarted=lidar.startStandardScan();
  Serial.println(lidarStarted?"[ARRANQUE] Driver LiDAR inicio OK; falta comprobar puntos.":"[ARRANQUE] Driver LiDAR no inicio; se reintentara detenido.");
  ultimaMuestraUs=0;
  initialYaw=yaw;
  session=esp_random();if(!session)session=1;
  if(!yawValido)stopTrial("Calibracion fallida; reinicia quieto");
  Serial.println("CW-3VUELTAS-CONTINUA8: PWM150, hueco frontal/trasero objetivo 50mm, giro referido a pared.");
}
void serviceAutoStart() {
  if(AUTO_START_ENABLED && !enabled && millis()>=AUTO_START_DELAY_MS) {
    enabled=true;
    buttonMs=millis()-500; // El plazo automatico sustituye la espera posterior al boton.
    Serial.println("[ARRANQUE AUTO] Plazo de 2 s cumplido. Habilitado sin boton; esperando sensores y vision validos.");
  }
}
void loop() {
  updateLapFinish();
  serviceAutoStart();
  imuUpdate();
  serviceLidar();
  static uint32_t controlMs=0;
  if(millis()-controlMs>=20){controlMs=millis();updateMotion();}
  serviceLidar();serviceCamera();serviceLidar();
  static uint32_t startupDiagnosticMs=0;
  if(!debugStarted && millis()-startupDiagnosticMs>=1000){startupDiagnosticMs=millis();printStartupDiagnostic();}
  static uint32_t secondDiagnosticMs=0;
  if(currentLane>=2 && millis()-secondDiagnosticMs>=1000){secondDiagnosticMs=millis();printSecondLaneDiagnostic();}
  static uint32_t retryMs=0,statsMs=0;
  bool stoppedForRx=(motion==PREPARE_SECOND_LANE && millis()-secondLaneStartedMs>=600) || (motion==RECOVERING && millis()-pauseStartedMs>=600) ||
    (cornerWaiting && millis()-cornerWaitSince>=600);
  // Reinicio del escaneo solo con traccion parada y giro ya calmado.
  if((motion==WAIT_PILLAR || stoppedForRx) && (!lidarStarted || millis()-lastPointMs>600) &&
     millis()-retryMs>=2000 && fabsf(velocidadZ)<3 && yawValido) {
    stopMotor();setSteering(0);retryMs=millis();
    Serial.println("[LIDAR_RECUPERACION] Reiniciando escaneo con robot detenido.");
    lidar.stopScan();lidarStarted=lidar.startStandardScan();
    lidarReader.reset();bucketDegree=-1;lastLidarServiceMs=0;
    for(int i=0;i<360;++i){ranges[i]=0;rangeMs[i]=0;}
    clearCornerTracker();
    // La inicializacion del driver puede bloquear: no integrar ese intervalo parado.
    ultimaMuestraUs=0;lidarRestartMs=millis();
  }
}
bool centerReference(float &error,bool &oneWall) {
  float right=laneWallDistance(true),left=laneWallDistance(false);
  wallRightReport=right;wallLeftReport=left;
  oneWall=false;
  if(currentLane>=2 && !secondLaneSawRight && millis()-cornerRightCheckMs>=180) {
    cornerRightCheckMs=millis();
    float ahead=cornerRightWallAhead();
    bool wallsOK=right>BODY_HALF_WIDTH_MM+MIN_GAP_MM && left>0 && ahead>0 &&
      fabsf(ahead-right)<100 && fabsf(angleDifference(yaw,initialYaw))<12;
    cornerRightHits=wallsOK?cornerRightHits+1:0;
    if(cornerRightHits>=3)secondLaneSawRight=true;
  }
  if(right && left){
    if(currentLane>=2)secondLaneExit=false;
    laneWidthMM=right+left;laneWidthMs=millis();
    error=(right-left)*0.5f;lastWallReferenceMs=millis();return true;
  }
  if(currentLane>=2 && secondLaneExit && left>0 && secondLaneLeftReference>0) {
    // Mantiene la separacion real de salida, sin inventar un ancho de carril.
    error=secondLaneLeftReference-left;oneWall=true;lastWallReferenceMs=millis();return true;
  }
  if(laneWidthMM>0 && millis()-laneWidthMs<=LANE_WIDTH_HOLD_MS){
    if(right>0 && right<laneWidthMM){error=right-laneWidthMM*0.5f;oneWall=true;}
    else if(left>0 && left<laneWidthMM){error=laneWidthMM*0.5f-left;oneWall=true;}
    else return false;
    lastWallReferenceMs=millis();return true;
  }
  return false;
}
bool viewAdjustmentAllowed() {
  if(motion!=CENTERING || millis()-centeringStartedMs>VIEW_ADJUST_MS)return false;
  float error;bool oneWall;
  if(!centerReference(error,oneWall))return false;
  unsigned farFront=0;
  for(int a=-45;a<=45;++a) {
    int index=(a+360)%360;
    if(!ranges[index] || millis()-rangeMs[index]>LIDAR_FRESH_MS)continue;
    float x=ranges[index]*sinf(a*DEG),y=ranges[index]*cosf(a*DEG);
    if(fabsf(x)<250 && y>0 && y<450)return false;
    if(abs(a)<=10 && y>=600)++farFront;
  }
  return farFront>=5;
}
void enterCentering() {
  bool alreadyRejoining=motion==PARALLEL;
  motion=CENTERING;motionMs=millis();centeringStartedMs=millis();
  if(!alreadyRejoining)headingTarget=0;
  centerStableMs=0;
}
// Solo completa un paso YA demostrado: detras de toda la envolvente del robot,
// con historial reciente de avance casi recto. Nunca considera un cero como pilar pasado.
bool finishKnownPassedPillar() {
  if(!target.valid || millis()-target.stamp>650 || target.y>=-(hypotf(100,BODY_HALF_WIDTH_MM)+PILLAR_HALF_MM+20) || -passSide*target.x<MIN_CENTER_OFFSET_MM)return false;
  float bound=0;if(!recentHeadingBound(target.stamp,bound) || bound>22)return false;
  ++passedPillars;++lanePillars;laneGuaranteedEnd=lanePillars>=2;
  Serial.println("[PASO] Ultima posicion ya confirmaba pilar detras; continuando carril.");
  if(nextReady()){stopMotor();setSteering(0);motion=DECIDE_NEXT;motionMs=millis();}
  else enterCentering();
  return true;
}
bool sideRecoveryCandidate(const PillarReading &c,const PillarReading &old) {
  if(!old.valid || !passSide || old.y < -100 || old.y > 350 || -passSide*old.x<80)return false;
  float lateral=-passSide*c.x;
  return lateral>=MIN_CENTER_OFFSET_MM && lateral<=400 && c.y>=-150 && c.y<=100 &&
    c.y<=old.y+30 && fabsf(c.x-old.x)<=160 &&
    hypotf(c.x-old.x,c.y-old.y)<=SIDE_RECOVERY_MAX_SHIFT_MM;
}
void serviceRecovery() {
  stopMotor();setSteering(0);
  if(!yawValido){stopTrial("IMU perdio referencia");return;}
  if(millis()-recoveryLogMs>=1000) {
    recoveryLogMs=millis();Serial.print("[RECUPERACION] causa=");Serial.print((int)recoveryCause);
    Serial.print(" edad_uart_ms=");Serial.print(millis()-lastPointMs);
    Serial.print(" paquetes=");Serial.print(lidarReader.packets);
    Serial.print(" pilar_X/Y=");Serial.print(target.x);Serial.print("/");Serial.print(target.y);
    Serial.print(" edad_pilar_ms=");Serial.print(millis()-target.stamp);
    Serial.print(" confirmaciones=");Serial.println(recoveryHits);
  }
  if(millis()-recoveryCheckMs<60)return;
  recoveryCheckMs=millis();
  if(!ultimaMuestraUs || micros()-ultimaMuestraUs>50000 || !lidarStarted || millis()-lastPointMs>150){recoveryHits=0;return;}
  if(recoveryCause==VISION_MISSING && resumeState==CENTERING && laneGuaranteedEnd && checkCornerCW()) {
    Serial.println("[ESQUINA RECUPERADA] Dos pilares completados; geometria CW confirmada durante espera de vision.");
    holdCorner();return;
  }
  if(frontTooClose()){recoveryHits=0;return;}
  MotionState destination=resumeState;
  if(resumeState==MOVING || resumeState==PARALLEL){
    float expected=atan2f(target.x,target.y)/DEG-angleDifference(yaw,initialYaw);
    reacquiring=true;PillarReading c=findPillar(expected,100,true);reacquiring=false;
    if(!c.valid && recoveryCause==PILLAR_MISSING && target.valid && target.y>=-100 && target.y<=350 && -passSide*target.x>=80) {
      // Centro +/-90 grados del carril, convertido al marco actual del sensor.
      float sideBearing=-passSide*90.0f-angleDifference(yaw,initialYaw);
      reacquiring=true;reacquiringSide=true;
      c=findPillar(sideBearing,65,true);
      reacquiringSide=false;reacquiring=false;
    }
    if(!c.valid){recoveryHits=0;return;}
    if(recoveryCandidate.valid && c.stamp==recoveryCandidate.stamp)return;
    bool consistent=recoveryCandidate.valid && hypotf(c.x-recoveryCandidate.x,c.y-recoveryCandidate.y)<40;
    recoveryHits=consistent?recoveryHits+1:1;recoveryCandidate=c;
    if(recoveryHits<3)return;
    bool recoveredAtSide=sideRecoveryCandidate(c,target);
    target=c;sideConfirmations=0;sideStamp=0;
    if(recoveredAtSide) {
      // No declara paso completado ni salta margen/rumbo: retoma el control normal.
      Serial.println("[RECUPERACION COSTADO] Pilar confirmado en tres lecturas nuevas. Retomando evasion.");
    }
  } else {
    if(!nextVisionMs || (int32_t)(nextVisionMs-pauseStartedMs)<0 || millis()-nextVisionMs>VISION_FRESH_MS){recoveryHits=0;return;}
    if(resumeState==CENTERING){
      if(nextConfirmations>=1 && nextPillar.valid)destination=DECIDE_NEXT;
      else {
        float error;bool oneWall;
        if(nextColor!='N' || !centerReference(error,oneWall)){recoveryHits=0;return;}
      }
    }
    if(++recoveryHits<3)return;
  }
  motionMs+=millis()-pauseStartedMs;
  motion=destination;lastTrackCheck=0;
  if(motion==CENTERING){centeringStartedMs=millis();headingTarget=0;}
}

// No vuelve a ejecutar filtros de vision ni consultas que modifiquen el control.
void printStartupDiagnostic() {
  if(motion!=WAIT_PILLAR && motion!=FAULT){
    debugStarted=true;Serial.println("[ARRANQUE] Espera inicial superada; control de recorrido activo.");return;
  }
  if(!cameraBoot) {
    char rawLine[180];
    int n=snprintf(rawLine,sizeof(rawLine),"[UART CAM] saludos_T_enviados=%lu RX17/TX21 38400 8N1 | RX_HEX:",(unsigned long)debugHelloSends);
    for(unsigned i=0;i<debugRawCount;++i)n+=snprintf(rawLine+n,sizeof(rawLine)-n," %02X",(unsigned)debugRawBytes[i]);
    Serial.println(rawLine);
    char ascii[25];for(unsigned i=0;i<debugRawCount;++i)ascii[i]=debugRawBytes[i]>=32 && debugRawBytes[i]<=126?(char)debugRawBytes[i]:'.';
    ascii[debugRawCount]=0;Serial.print("[UART CAM] RX_TEXTO: ");Serial.println(ascii);
    debugRawCount=0;
  }
  bool imuFresh=yawValido && ultimaMuestraUs && micros()-ultimaMuestraUs<=50000;
  bool lidarFresh=lidarStarted && points && millis()-lastPointMs<=150;
  const char *cause=motion==FAULT?"FALLO: revisar mensaje [PARADA]":
    !imuFresh?"IMU NO LISTA / SIN MUESTRA FRESCA":
    !lidarFresh?"LIDAR SIN PUNTOS FRESCOS":
    !cameraBoot?"SIN ENLACE CONFIRMADO CON CAMARA":
    !enabled?(AUTO_START_ENABLED?"ESPERANDO PLAZO AUTOMATICO DE 2 s":"ESPERANDO ORDEN DEL BOTON DESDE CAMARA"):
    millis()-buttonMs<500?"ESPERA DE 500 ms":debugVision;
  char b[400];
  snprintf(b,sizeof(b),"[ARRANQUE DEBUG] %s | habilitado=%s IMU=%s LIDAR=%s enlace=%s\n",
    cause,enabled?"SI":"NO",imuFresh?"OK":"NO",lidarFresh?"OK":"NO",cameraBoot?"CONFIRMADO":"NO");Serial.print(b);
  snprintf(b,sizeof(b),"[ARRANQUE RX] CAM bytes=%lu tramas=%lu corruptas=%u otraSesion=%lu botones=%lu | LIDAR bytes=%lu puntos=%lu edad_ms=%lu\n",
    (unsigned long)debugCamBytes,(unsigned long)debugCamFrames,(unsigned)reader.bad,(unsigned long)debugForeignFrames,
    (unsigned long)debugButtons,(unsigned long)debugLidarBytes,(unsigned long)points,(unsigned long)(millis()-lastPointMs));Serial.print(b);
  snprintf(b,sizeof(b),"[ARRANQUE FOTO] completas=%lu pendiente=%s intento=%u partes=%u fin=%u timeouts=%lu pilar=%u/3 vacias=%u/3 | %s\n",
    (unsigned long)debugPhotos,pending?"SI":"NO",sends,photoMask,(unsigned)photoFinished,(unsigned long)debugTimeouts,
    stableFrames,(unsigned)emptyStartFrames,debugVision);Serial.print(b);
}

void updateLapFinish() {
  uint32_t now=millis(),elapsed=now-finalClockMs;finalClockMs=now;
  if(completedCorners!=TARGET_LAPS*CORNERS_PER_LAP || motion==DONE || motion==FAULT)return;
  if(finalForwardCommand && drivingState())finalAdvanceMs+=elapsed;
  if(finalAdvanceMs>=FINAL_APPROACH_MS) {
    stopMotor();setSteering(0);motion=DONE;pending=false;
    Serial.println("[FIN] 12 esquinas y tramo final completados. Robot detenido.");
  }
}
