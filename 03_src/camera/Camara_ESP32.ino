/* ESP32-CAM AI Thinker + OV2640. Arduino-ESP32 3.x.
   Enviar '1' a 115200 baudios. Visor: http://192.168.4.1
   Los cuatro archivos de esta carpeta deben permanecer juntos.
*/
#include <Arduino.h>
#include <esp_system.h>
#include "esp_camera.h"
#include "img_converters.h"
#include <WiFi.h>
#include <WebServer.h>
#include "detector.h"
#include "pagina.h"
#include "color_probe.h"

// RX13 <- C6 D3/GPIO21; TX14 -> C6 D7/GPIO17. Sin microSD.
static const int ROBOT_RX = 13;
static const int ROBOT_TX = 14;
bool pedidoDelRobot = false;

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

static uint32_t camLocalBoot=0;
static CamReader robotReader;
static uint32_t robotRxBytes=0,robotFrames=0,robotHellos=0,robotPhotos=0,robotCacheHits=0;
// Contadores de diagnostico: U cuenta llamadas de envio, no recepcion en C6.
static uint32_t startHellos=0,startReplies=0,startRejected=0;
static char lastFrameType='-';
struct CachedPhoto {
  uint32_t session=0,id=0,box=0;
  char result='E',boxQuality='U';
  uint32_t boxes[2]={}; char colors[2]={'E','E'};
};
static CachedPhoto photoCache[8];
static unsigned photoCacheNext=0;
static char robotResult='E', robotBoxQuality='U';
static uint32_t robotBox=0;
static uint32_t robotBoxes[2]={};
static char robotColors[2]={'E','E'};
void takePhoto();


static const int START_BUTTON_PIN=15;
// Boton entre 3.3 V e IO15: HIGH pulsado, LOW suelto.
// buttonRaw/buttonStable representan SUELTO, no el voltaje fisico.
// UART conserva 0=pulsado, 1=suelto para compatibilidad con la C6.
static uint32_t buttonSession=0, buttonSequence=0, buttonPending=0;
static uint32_t buttonChangeMs=0, buttonLastSend=0;
static bool buttonRaw=true, buttonStable=true, buttonReleased=false;
static bool buttonWaitingLink=false;
static uint32_t buttonWaitingSince=0;
static const uint32_t BUTTON_PRELINK_TTL_MS=30000;


void handleStartFrame(const CamFrame &f);
void handleStartFrame(const CamFrame &f) {
  if(f.type=='T' && f.session && f.id==1 && f.boot==0 && f.data=='?') {
    ++startHellos;
    Serial.println("[ENLACE] Saludo T recibido de C6");
    if(buttonSession!=f.session) {
      // Una orden de una sesion anterior no se transfiere a una C6 reiniciada.
      if(buttonSession)buttonWaitingLink=false;
      buttonSession=f.session;buttonPending=0;buttonSequence=0;
      buttonRaw=buttonStable=digitalRead(START_BUTTON_PIN)==LOW;
      buttonReleased=false;buttonChangeMs=millis();
    }
    camSend(Serial1,'U',buttonSession,1,camLocalBoot,'R');
    ++startReplies;
    Serial.println("[ENLACE] Respuesta U entregada a UART TX IO14 (sin confirmacion de recepcion)");
  } else if(f.type=='T') {
    ++startRejected;
    char detail[150];
    snprintf(detail,sizeof(detail),"[ENLACE] Saludo T rechazado: sesion=%08lX id=%lu boot=%lu dato=%c",
      (unsigned long)f.session,(unsigned long)f.id,(unsigned long)f.boot,f.data);
    Serial.println(detail);
  } else if(f.type=='K' && f.data=='K'  && f.session==buttonSession &&
            f.boot==camLocalBoot && buttonPending && f.id==buttonPending) {
    buttonPending=0;Serial.println("[BOTON] C6 confirmo el arranque.");
  }
}
void pollStartButton() {
  uint32_t now=millis();
  bool raw=digitalRead(START_BUTTON_PIN)==LOW;
  static bool printedInitial=false;
  if(raw!=buttonRaw){buttonRaw=raw;buttonChangeMs=now;}
  if(now-buttonChangeMs>=30) {
    bool changed=buttonStable!=buttonRaw;
    buttonStable=buttonRaw;
    if(changed || !printedInitial) {
      printedInitial=true;
      Serial.println(buttonStable ? "[BOTON IO15] SUELTO (LOW)" : "[BOTON IO15] PRESIONADO (HIGH)");
    }
    if(changed) {
      if(!buttonStable && buttonReleased && !buttonPending) {
        if(!buttonSession) {
          buttonWaitingLink=true;buttonWaitingSince=now;buttonReleased=false;
          Serial.println("[BOTON] Pulsacion guardada 30 s: esperando primer enlace con C6.");
        } else {
        ++buttonSequence;if(!buttonSequence)++buttonSequence;
        buttonPending=buttonSequence;
        camSend(Serial1,'S',buttonSession,buttonPending,camLocalBoot,'0');
        Serial.println("[BOTON] Orden de arranque enviada a la C6. Esperando confirmacion.");
        buttonLastSend=now;buttonReleased=false;
        }
      }
    }
    if(buttonStable)buttonReleased=true;
  }
  if(buttonWaitingLink) {
    if(now-buttonWaitingSince>=BUTTON_PRELINK_TTL_MS) {
      buttonWaitingLink=false;
      Serial.println("[BOTON] Espera de enlace agotada. Pulsa otra vez cuando haya enlace.");
    } else if(buttonSession) {
      buttonWaitingLink=false;
      ++buttonSequence;if(!buttonSequence)++buttonSequence;
      buttonPending=buttonSequence;buttonLastSend=now;
      camSend(Serial1,'S',buttonSession,buttonPending,camLocalBoot,'0');
      Serial.println("[BOTON] Enlace recuperado. Enviando pulsacion guardada; reintento cada 200 ms hasta ACK.");
    }
  }
  if(buttonPending && now-buttonLastSend>=200) {
    buttonLastSend=now;
    camSend(Serial1,'S',buttonSession,buttonPending,camLocalBoot,'0');
  }
  static uint32_t lastButtonReport=0;
  if(buttonSession && now-lastButtonReport>=500) {
    lastButtonReport=now;
    uint32_t flags=8u | (buttonStable?1u:0u) | (buttonReleased?2u:0u) | (buttonPending?4u:0u);
    camSend(Serial1,'D',buttonSession,flags,camLocalBoot,buttonStable?'1':'0');
  }
}

// P: x/y/w/h empaquetados en boot; correlacion por sesion y solicitud.
// R mantiene el identificador real de arranque. Solo Q con dato 2 pide geometria.
void sendPhotoReply(const CamFrame &f, const CachedPhoto &p);
void sendPhotoReply(const CamFrame &f, const CachedPhoto &p) {
  // Q3: dos posiciones, incluso si estan vacias. Minuscula = recortado.
  // Z confirma fin y arranque real; cada trama conserva sesion, id y CRC.
  if(f.data=='3') {
    camSend(Serial1,'X',f.session,f.id,p.boxes[0],p.colors[0]);
    camSend(Serial1,'Y',f.session,f.id,p.boxes[1],p.colors[1]);
    camSend(Serial1,'Z',f.session,f.id,camLocalBoot,'2');
    return;
  }
  if(f.data=='2')camSend(Serial1,'P',f.session,f.id,p.box,p.boxQuality);
  camSend(Serial1,'R',f.session,f.id,camLocalBoot,p.result);
}
void procesarPedidoRobot(const CamFrame &f) {
  if(f.type=='T' || f.type=='K'){handleStartFrame(f);return;}
  if(!f.session || !f.id)return;
  if(f.type=='H' && f.data=='?' && f.boot==0) {
    ++robotHellos;
    camSend(Serial1,'B',f.session,f.id,camLocalBoot,'B');return;
  }
  if(f.type!='Q' || (f.data!='1' && f.data!='2' && f.data!='3'))return;
  if(f.boot!=camLocalBoot) {
    camSend(Serial1,'B',f.session,f.id,camLocalBoot,'B');return;
  }
  uint32_t maxId=0;
  for(const CachedPhoto &p:photoCache) {
    if(p.session!=f.session)continue;
    if(p.id==f.id) {
      ++robotCacheHits;
      sendPhotoReply(f,p);return;
    }
    if(p.id>maxId)maxId=p.id;
  }
  if(f.id<maxId) {camSend(Serial1,'R',f.session,f.id,camLocalBoot,'E');return;}
  camSend(Serial1,'A',f.session,f.id,camLocalBoot,'A');
  // Este manejador es secuencial: duplicados esperan en UART y usan la cache.
  pedidoDelRobot=true;robotResult='E';robotBox=0;robotBoxQuality='U';
  for(int i=0;i<2;++i){robotBoxes[i]=0;robotColors[i]='E';}
  ++robotPhotos;
  takePhoto();
  pedidoDelRobot=false;
  CachedPhoto &p=photoCache[photoCacheNext];
  p.session=f.session;p.id=f.id;p.result=robotResult;p.box=robotBox;p.boxQuality=robotBoxQuality;
  for(int i=0;i<2;++i){p.boxes[i]=robotBoxes[i];p.colors[i]=robotColors[i];}
  photoCacheNext=(photoCacheNext+1)%8;
  sendPhotoReply(f,p);
}
void atenderRobot() {
  int budget=256;CamFrame f;
  while(Serial1.available() && budget-->0) {
    ++robotRxBytes;
    if(robotReader.push((char)Serial1.read(),f)){++robotFrames;lastFrameType=f.type;procesarPedidoRobot(f);}
  }
}


static const bool VISUAL_FEEDBACK = true; // false: desactiva Wi-Fi y visor.
static const bool FLIP_VERTICAL = true;
static const bool MIRROR_HORIZONTAL = false;
static const int WIDTH = 160, HEIGHT = 120;
static const size_t PIXELS = WIDTH * HEIGHT;
static const size_t JPEG_CAPACITY = 65536;
static const char *AP_NAME = "WRO-CAM";
static const char *AP_PASSWORD = "wrocam123";
WebServer server(80);
uint8_t *rgb = nullptr, *labels = nullptr, *jpg = nullptr;
uint16_t *queuePixels = nullptr;
Pillar objects[MAX_OBJECTS];
int objectCount = 0;
size_t jpgLength = 0;
uint32_t captureId = 0, captureMs = 0;
bool ready = false, uncertain = false;
bool decoderBGR = false;
String lastError, metadata;

const char *colorName(int c) { return c == 1 ? "ROJO" : "VERDE"; }

void reportError(const char *message) {
  lastError = message;
  Serial.print("ERROR: "); Serial.println(message);
  if (pedidoDelRobot) robotResult = 'E';
}

bool checkDecoder() {
  uint8_t sample[8 * 8 * 3];
  if (!fmt2rgb888(RED_PROBE, sizeof(RED_PROBE), PIXFORMAT_JPEG, sample)) {
    reportError("Fallo de prueba de color del decodificador"); return false;
  }
  int a=0, b=0, g=0;
  for (int i=0; i<64; ++i) {a+=sample[3*i];g+=sample[3*i+1];b+=sample[3*i+2];}
  if (a>64*180 && b<64*70 && g<64*70) decoderBGR=false;
  else if (b>64*180 && a<64*70 && g<64*70) decoderBGR=true;
  else {reportError("Orden de color no reconocido");return false;}
  Serial.println(decoderBGR ? "Decodificador verificado: BGR" : "Decodificador verificado: RGB");
  return true;
}

bool initCamera() {
  camera_config_t c = {};
  c.ledc_channel = LEDC_CHANNEL_0; c.ledc_timer = LEDC_TIMER_0;
  c.pin_d0 = 5; c.pin_d1 = 18; c.pin_d2 = 19; c.pin_d3 = 21;
  c.pin_d4 = 36; c.pin_d5 = 39; c.pin_d6 = 34; c.pin_d7 = 35;
  c.pin_xclk = 0; c.pin_pclk = 22; c.pin_vsync = 25; c.pin_href = 23;
  c.pin_sccb_sda = 26; c.pin_sccb_scl = 27;
  c.pin_pwdn = 32; c.pin_reset = -1;
  c.xclk_freq_hz = 20000000;
  c.pixel_format = PIXFORMAT_JPEG;
  c.frame_size = FRAMESIZE_VGA; // Reserva un buffer mayor al iniciar.
  c.jpeg_quality = 12;
  c.fb_count = 1;
  c.fb_location = CAMERA_FB_IN_PSRAM;
  c.grab_mode = CAMERA_GRAB_WHEN_EMPTY;
  esp_err_t err = esp_camera_init(&c);
  if (err != ESP_OK) {
    Serial.printf("Codigo de camara: 0x%x\n", (unsigned)err);
    reportError("No se pudo iniciar la camara"); return false;
  }
sensor_t *s = esp_camera_sensor_get();
if (s) {
  s->set_framesize(s, FRAMESIZE_QQVGA);
  s->set_vflip(s, FLIP_VERTICAL);
  s->set_hmirror(s, MIRROR_HORIZONTAL);
}
  // Dejar que la exposicion y el balance de blancos automaticos se adapten.
  for (int i = 0; i < 12; ++i) {
    camera_fb_t *fb = esp_camera_fb_get();
    if (fb) esp_camera_fb_return(fb);
    delay(80);
  }
  return true;
}

void buildMetadata() {
  metadata = "{\"id\":" + String(captureId) + ",\"w\":" + String(WIDTH) +
    ",\"h\":" + String(HEIGHT) + ",\"jpg\":" + String((unsigned)jpgLength) +
    ",\"ms\":" + String(captureMs) + ",\"uncertain\":" + (uncertain ? "true" : "false");
  metadata += ",\"roiTop\":" + String(detectionTop(HEIGHT));
  metadata += ",\"roiLeft\":" + String(detectionLeft(WIDTH));
  metadata += ",\"roiRight\":" + String(detectionRight(WIDTH));
  metadata += ",\"config\":\"H rojo: 0-" + String(RED_H_MAX) + " / " + String(RED_H_MIN) +
    "-359; H verde: " + String(GREEN_H_MIN) + "-" + String(GREEN_H_MAX) +
    "; S minimo: " + String(RED_MIN_S) + "/" + String(GREEN_MIN_S) + "; V minimo: " + String(MIN_V) + "\",\"objects\":[";
  for (int i = 0; i < objectCount; ++i) {
    const Pillar &p = objects[i];
    if (i) metadata += ',';
    metadata += "{\"c\":" + String(p.color) + ",\"x\":" + String(p.x) +
      ",\"y\":" + String(p.y) + ",\"w\":" + String(p.w) +
      ",\"h\":" + String(p.h) + ",\"area\":" + String(p.area) + "}";
  }
  metadata += "]}";
}

void takePhoto() {
  if (!ready) { reportError("Camara no disponible; revisa el arranque y reinicia"); return; }
  uint32_t started = millis();
  // Descartar frames pendientes para no analizar una imagen anterior al comando.
  for (int i = 0; i < 2; ++i) {
    camera_fb_t *old = esp_camera_fb_get();
    if (!old) { reportError("Fallo al actualizar la captura"); return; }
    esp_camera_fb_return(old);
  }
  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb) { reportError("Fallo de captura"); return; }
  if (fb->width != WIDTH || fb->height != HEIGHT || fb->format != PIXFORMAT_JPEG || fb->len > JPEG_CAPACITY) {
    esp_camera_fb_return(fb); reportError("Tamano o formato de foto inesperado"); return;
  }
  bool converted = fmt2rgb888(fb->buf, fb->len, fb->format, rgb);
  if (!converted) { esp_camera_fb_return(fb); reportError("Fallo al decodificar JPEG"); return; }
  // Orden determinado con un JPEG rojo conocido, no supuesto por version.
  // Limpiar la mascara completa, incluidos bits de visitado de la foto anterior.
  memset(labels, 0, PIXELS);
  for (int y = detectionTop(HEIGHT); y < HEIGHT; ++y)
    for (int x = detectionLeft(WIDTH); x < detectionRight(WIDTH); ++x) {
      size_t i = (size_t)y * WIDTH + x;
      labels[i] = classifyRGB(rgb[3*i+(decoderBGR?2:0)], rgb[3*i+1], rgb[3*i+(decoderBGR?0:2)]);
    }
  objectCount = findPillars(labels, queuePixels, WIDTH, HEIGHT, objects);
  if (objectCount < 0) {
    esp_camera_fb_return(fb);
    // La mascara cambio: invalidar la captura anterior para no mezclar datos.
    captureId = 0; jpgLength = 0;
    reportError("Demasiadas regiones; ajusta los filtros de color y tamano"); return;
  }
  memcpy(jpg, fb->buf, fb->len); jpgLength = fb->len;
  esp_camera_fb_return(fb);
  uncertain = false;
  for (int i = 0; i < objectCount; ++i) if (objects[i].clipped) uncertain = true;
  if (objectCount > 1 && (objects[0].h - objects[objectCount-1].h) * 100 <= objects[0].h * 10)
    uncertain = true;
  captureMs = millis() - started;
  // Identificador monotono incluso despues de una captura invalida.
  static uint32_t sequence = 0;
  captureId = ++sequence;
  lastError = "";
  buildMetadata();
  if(pedidoDelRobot) {
    for(int i=0;i<2;++i) {
      robotBoxes[i]=0;robotColors[i]='N';
      if(i>=objectCount)continue;
      const Pillar &p=objects[i];
      robotBoxes[i]=((uint32_t)p.x<<24)|((uint32_t)p.y<<16)|
                    ((uint32_t)p.w<<8)|(uint32_t)p.h;
      robotColors[i]=p.color==1?(p.clipped?'r':'R'):(p.clipped?'v':'V');
    }
  }
const char *respuesta = objectCount > 0 ? colorName(objects[0].color) : "NINGUNO";
  Serial.print("[CAM resultado] ");
  Serial.println(respuesta);
  if (pedidoDelRobot) robotResult = objectCount == 0 ? 'N' : objects[0].color == 1 ? 'R' : 'V';
  if(pedidoDelRobot && objectCount>0) {
    const Pillar &nearest=objects[0];
    robotBox=((uint32_t)nearest.x<<24)|((uint32_t)nearest.y<<16)|
             ((uint32_t)nearest.w<<8)|(uint32_t)nearest.h;
    robotBoxQuality=uncertain?'U':'V';
  }
}

void sendSnapshot() {
  if (!captureId) { server.send(404, "text/plain", "Sin captura"); return; }
  // Una sola respuesta contiene metadata, JPEG y mascara DE LA MISMA captura.
  uint32_t n = metadata.length();
  uint8_t prefix[4] = {(uint8_t)n, (uint8_t)(n>>8), (uint8_t)(n>>16), (uint8_t)(n>>24)};
  server.sendHeader("Cache-Control", "no-store");
  server.setContentLength(4 + n + jpgLength + PIXELS);
  server.send(200, "application/octet-stream", "");
  server.sendContent((const char *)prefix, 4);
  server.sendContent(metadata);
  server.sendContent((const char *)jpg, jpgLength);
  server.sendContent((const char *)labels, PIXELS);
}

String settingsJson() {
  return "{\"rs\":" + String(RED_MIN_S) + ",\"gs\":" + String(GREEN_MIN_S) +
    ",\"v\":" + String(MIN_V) + ",\"area\":" + String(MIN_AREA) +
    ",\"height\":" + String(MIN_HEIGHT) + ",\"rhmax\":" + String(RED_H_MAX) +
    ",\"rhmin\":" + String(RED_H_MIN) + ",\"ghmin\":" + String(GREEN_H_MIN) +
    ",\"ghmax\":" + String(GREEN_H_MAX) + "}";
}

void configureDetector() {
  const char *keys[] = {"rs", "gs", "v", "area", "height", "rhmax", "rhmin", "ghmin", "ghmax"};
  const int low[] = {0,0,0,1,1,0,270,40,40};
  const int high[] = {255,255,255,19200,120,60,359,200,200};
  int values[9];
  for (int i=0; i<9; ++i) {
    String value = server.arg(keys[i]);
    if (!value.length()) { server.send(400,"text/plain","Falta un parametro"); return; }
    for (unsigned j=0; j<value.length(); ++j) {
      if (value[j]<'0' || value[j]>'9' || value.length()>5) {
        server.send(400,"text/plain","Parametro no valido"); return;
      }
    }
    values[i] = value.toInt();
    if (values[i]<low[i] || values[i]>high[i]) {
      server.send(400,"text/plain","Parametro fuera de rango"); return;
    }
  }
  if (values[7]>values[8] || values[5]>=values[7]) {
    server.send(400,"text/plain","Revisa los rangos de tono"); return;
  }
  RED_MIN_S=values[0]; GREEN_MIN_S=values[1]; MIN_V=values[2];
  MIN_AREA=values[3]; MIN_HEIGHT=values[4]; RED_H_MAX=values[5];
  RED_H_MIN=values[6]; GREEN_H_MIN=values[7]; GREEN_H_MAX=values[8];
  server.send(200,"application/json",settingsJson());
}

void setup() {
  Serial.begin(115200);
  pinMode(START_BUTTON_PIN,INPUT_PULLDOWN);
  camLocalBoot=esp_random();if(!camLocalBoot)camLocalBoot=1;
  Serial1.setRxBufferSize(1024);
  Serial1.begin(38400, SERIAL_8N1, ROBOT_RX, ROBOT_TX);
  delay(1000);
  pinMode(4, OUTPUT); digitalWrite(4, LOW); // Flash apagado.
  if (psramFound()) {
    rgb = (uint8_t *)ps_malloc(PIXELS * 3);
    labels = (uint8_t *)ps_malloc(PIXELS);
    queuePixels = (uint16_t *)ps_malloc(PIXELS * sizeof(uint16_t));
    jpg = (uint8_t *)ps_malloc(JPEG_CAPACITY);
    if (rgb && labels && queuePixels && jpg) ready = checkDecoder() && initCamera();
    else reportError("No hay memoria suficiente");
  } else reportError("PSRAM no detectada; selecciona AI Thinker ESP32-CAM y habilita PSRAM");
  if (VISUAL_FEEDBACK) {
    WiFi.mode(WIFI_AP);
    if (!WiFi.softAPConfig(IPAddress(192,168,4,1), IPAddress(192,168,4,1), IPAddress(255,255,255,0)) ||
        !WiFi.softAP(AP_NAME, AP_PASSWORD)) {
      Serial.println("ERROR: No se pudo crear la red Wi-Fi");
    } else {
      server.on("/", HTTP_GET, [](){ server.send_P(200, "text/html; charset=utf-8", PAGE); });
      server.on("/status", HTTP_GET, [](){
        server.sendHeader("Cache-Control", "no-store");
        server.send(200, "application/json", "{\"id\":" + String(captureId) + ",\"error\":\"" + lastError + "\"}");
      });
      server.on("/snapshot", HTTP_GET, sendSnapshot);
      server.on("/link", HTTP_GET, [](){
        server.sendHeader("Cache-Control", "no-store");
        String info="Firmware: ID-CRC-VISION2-BOTON-COLA1 (dos cuadros + ROI; Q1/Q2 compatibles)\nUART: 38400, RX13, TX14\n";
        info+="Arranque: "+String(camLocalBoot)+"\n";
        info+="Camara lista: "+String(ready?"SI":"NO")+"\n";
        info+="Bytes recibidos: "+String(robotRxBytes)+"\n";
        info+="Mensajes validos: "+String(robotFrames)+"\n";
        info+="Mensajes corruptos: "+String(robotReader.bad)+"\n";
        info+="Saludos contestados: "+String(robotHellos+startReplies)+"\n";
        info+="Saludos H contestados: "+String(robotHellos)+"\n";
        info+="Saludos T aceptados: "+String(startHellos)+"\n";
        info+="Respuestas U entregadas a UART: "+String(startReplies)+"\n";
        info+="Saludos T rechazados: "+String(startRejected)+"\n";
        info+="Boton esperando primer enlace: "+String(buttonWaitingLink?"SI":"NO")+"\n";
        info+="Orden pendiente de confirmacion: "+String(buttonPending?"SI":"NO")+"\n";
        info+="Ultimo tipo valido: "+String(lastFrameType)+"\n";
        info+="Capturas solicitadas: "+String(robotPhotos)+"\n";
        info+="Resultados reenviados: "+String(robotCacheHits)+"\n";
        info+="Ultimo error: "+lastError+"\n";
        server.send(200,"text/plain; charset=utf-8",info);
      });
      server.on("/settings", HTTP_GET, [](){server.send(200,"application/json",settingsJson());});
      server.on("/settings", HTTP_POST, configureDetector);
      server.begin();
      Serial.println("Red: WRO-CAM | Clave: wrocam123 | Visor: http://192.168.4.1");
    }
  } else WiFi.mode(WIFI_OFF);
  // Reafirmar la entrada tras inicializar camara y Wi-Fi.
  pinMode(START_BUTTON_PIN,INPUT_PULLDOWN);
  if (ready) Serial.println("Listo. Envia 1 para tomar una foto.");
}

void loop() {
  // Atender la UART antes del visor web.
  atenderRobot();
  pollStartButton();
  if (VISUAL_FEEDBACK) server.handleClient();
  atenderRobot();
  pollStartButton();
  if (Serial.available() && Serial.read() == '1') {
    pedidoDelRobot=false;
    takePhoto();
  }
  delay(2);
}
