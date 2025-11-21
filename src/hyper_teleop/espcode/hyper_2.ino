/************************************************************
 * ESP32 UDP Control - 4x ESC (esc1_r, esc1_l, esc2_r, esc2_l)
 * Protocolo ASCII: <v1,v2,v3,cmd,id,crc>\n  (CRC-8 Dallas/Maxim)
 *
 * Cada motor usa dos canales:
 *   - _r : canal "adelante"  (se activa si v > 0)
 *   - _l : canal "atrás"     (se activa si v < 0)
 *
 * Movimiento:
 *   - Avance   : [ESC_FWD_MIN .. ESC_MAX] µs
 *   - Retroceso: [ESC_REV_MIN .. ESC_MAX] µs
 * Neutral/arming:
 *   - ESC_MIN = 1100 µs (motor parado)
 *
 * Escalado global por trama:
 *   - Si v1<0 o v2<0 → |v| se normaliza con 1.5 (NEG_INPUT_ABS_MAX)
 *   - Si v1>=0 y v2>=0 → |v| se normaliza con 32.0 (POS_INPUT_ABS_MAX)
 ************************************************************/

/* Prototipo/struct arriba para evitar error de prototipado automático */
struct ESCCommand { int ch_r; int ch_l; };
class Servo;
ESCCommand escWrite(Servo &esc_r, Servo &esc_l, float vel, const char* label);

#include <WiFi.h>
#include <WiFiUdp.h>
#include <ESP32Servo.h>
#include <math.h>
#include <TinyGPS++.h>
#include <TinyGPSPlus.h>

// ========================= WIFI CONFIG =========================
const char* WIFI_SSID = "Pixel 8a";
const char* WIFI_PASS = "";
TinyGPSPlus gps;

// UDP
const uint16_t UDP_LISTEN_PORT = 9000; // ESP32 escucha aquí
const uint16_t PC_LISTEN_PORT = 9001;  // Respuestas al PC
WiFiUDP udp;
IPAddress lastRemoteIP;
uint16_t lastRemotePort = 0;

// ================== ESC CONFIG (4 canales) ==================
Servo esc1_r; // motor 1 adelante
Servo esc1_l; // motor 1 atrás
Servo esc2_r; // motor 2 adelante
Servo esc2_l; // motor 2 atrás

// Ajusta estos pines según tu placa/hardware
const int ESC1_R_PIN = 25;
const int ESC1_L_PIN = 33;
const int ESC2_R_PIN = 26;
const int ESC2_L_PIN = 27;

// ---- Límites de pulso ----
const int ESC_MIN      = 1100; // Neutral / Stop / Arming
const int ESC_FWD_MIN  = 1300; // Mínimo cuando hay movimiento hacia adelante (canal *_r)
const int ESC_REV_MIN  = 1650; // Mínimo cuando hay movimiento hacia atrás (canal *_l)
const int ESC_MAX      = 1940;

// ---- Escalado de entradas ----
const float POS_INPUT_ABS_MAX = 27.5f; // escala para tramas no-negativas
const float NEG_INPUT_ABS_MAX = 5.0f;  // escala cuando hay cualquier velocidad negativa
const float DEADZONE = 0.01f;

// Escala activa global (se decide en cada trama recibida)
float g_input_abs_max = POS_INPUT_ABS_MAX;

/* =================== GPS =================== */
void sendGPSUDP() {
  static uint32_t lastSend = 0;
  uint32_t now = millis();

  // Enviar cada 1 s
  if (now - lastSend >= 1000) {
    lastSend = now;
    
    if (gps.location.isUpdated() && gps.location.isValid()) {
      char payload[128];
      snprintf(payload, sizeof(payload),
               "{\"lat\":%.7f,\"lon\":%.7f,\"alt\":%.2f}",
               gps.location.lat(), 
               gps.location.lng(), 
               gps.altitude.meters());
      
      // Si aún no conocemos la IP del PC, enviar por broadcast
      if (!lastRemoteIP) {
        IPAddress broadcast = WiFi.localIP();
        broadcast[3] = 255; // Convertir a broadcast (ej: 192.168.1.255)
        udp.beginPacket(broadcast, PC_LISTEN_PORT);
        udp.print(payload);
        udp.endPacket();
        Serial.printf("[GPS->BROADCAST] %s (esperando PC...)\n", payload);
      } else {
        // Ya conocemos la IP, enviar directo
        udp.beginPacket(lastRemoteIP, PC_LISTEN_PORT);
        udp.print(payload);
        udp.endPacket();
        Serial.printf("[GPS->PC %s] %s\n", lastRemoteIP.toString().c_str(), payload);
      }
    } else {
      // Diagnóstico mejorado
      Serial.printf("[GPS] Sats=%d | Chars=%lu | Sentences=%lu | Failed=%lu\n",
                    gps.satellites.value(),
                    gps.charsProcessed(),
                    gps.sentencesWithFix(),
                    gps.failedChecksum());
    }
  }
}
// ===============================================================
enum CmdCode { CMD_IDLE=0, CMD_ROLL=1, CMD_WALK=2, CMD_ESTOP=3, CMD_ERES=4, CMD_WALK_START=6 };

// ===== CRC-8 Dallas/Maxim =====
uint8_t crc8_maxim(const uint8_t* d, size_t n){
  uint8_t c = 0x00;
  for(size_t i=0;i<n;++i){
    c ^= d[i];
    for(uint8_t b=0;b<8;++b){
      c = (c & 1) ? ((c >> 1) ^ 0x8C) : (c >> 1);
    }
  }
  return c & 0xFF;
}

// ============== Función de control por motor (2 canales) ==============
ESCCommand escWrite(Servo &esc_r, Servo &esc_l, float vel, const char* label){
  float mag  = fabsf(vel);
  float norm = (g_input_abs_max > 0.0f) ? (mag / g_input_abs_max) : 0.0f;

  if (norm < DEADZONE) norm = 0.0f;
  if (norm > 1.0f)     norm = 1.0f;

  // Escala en 1000 pasos para mayor precisión
  int scaled = (int)round(norm * 1000.0f);

  int pulse_r = ESC_MIN;  // por defecto neutral
  int pulse_l = ESC_MIN;  // por defecto neutral

  if (vel > 0.0f && norm > 0.0f){
    // Avance: usar [ESC_FWD_MIN .. ESC_MAX] en canal R, L neutral
    int pulse_fwd = map(scaled, 0, 1000, ESC_FWD_MIN, ESC_MAX);
    pulse_r = pulse_fwd;
    pulse_l = ESC_MIN;
  }
  else if (vel < 0.0f && norm > 0.0f){
    // Retroceso: usar [ESC_REV_MIN .. ESC_MAX] en canal L, R neutral
    int pulse_rev = map(scaled, 0, 1000, ESC_REV_MIN, ESC_MAX);
    pulse_r = ESC_MIN;
    pulse_l = pulse_rev;
  }
  // vel==0 || norm==0 => ambos en ESC_MIN (neutral 1100)

  esc_r.writeMicroseconds(pulse_r);
  esc_l.writeMicroseconds(pulse_l);

  Serial.printf("[%s] R=%d L=%d (v=%.3f, norm=%.3f, scale=%.2f)\n", label, pulse_r, pulse_l, vel, norm, g_input_abs_max);
  return ESCCommand{pulse_r, pulse_l};
}

void stopAllESC(){
  esc1_r.writeMicroseconds(ESC_MIN); esc1_l.writeMicroseconds(ESC_MIN);
  esc2_r.writeMicroseconds(ESC_MIN); esc2_l.writeMicroseconds(ESC_MIN);
  Serial.printf("[STOP] esc1_r=%d esc1_l=%d | esc2_r=%d esc2_l=%d\n",
                ESC_MIN, ESC_MIN, ESC_MIN, ESC_MIN);
}

// ===== WALK sequence =====
bool estop_latched = false;
bool walk_sequence = false;
bool walk_phase_A  = true;
const uint32_t WALK_PHASE_MS = 10000;
const float WALK_VEL = +10.0f;  // valor equivalente a velocidad fija para WALK
uint32_t last_phase_ms = 0;

// ====== UDP helpers (STATUS al PC) ======
String buildFrame(float v1, float v2, float v3, int cmd, const char* id_hex){
  char payload[128];
  snprintf(payload, sizeof(payload), "%.3f,%.3f,%.3f,%d,%s", v1, v2, v3, cmd, id_hex);
  uint8_t crc = crc8_maxim((const uint8_t*)payload, strlen(payload));
  char frame[160];
  snprintf(frame, sizeof(frame), "<%s,%02X>\n", payload, crc);
  return String(frame);
}

void sendUDPStatus(float v1, float v2, float v3, int cmd){
  if (!lastRemoteIP) return;
  String f = buildFrame(v1, v2, v3, cmd, "0xSTA");
  udp.beginPacket(lastRemoteIP, PC_LISTEN_PORT);
  udp.print(f);
  udp.endPacket();
  Serial.print("[TX->PC] "); Serial.print(f);
}

void applyWalkPhase(bool phaseA){
  // WALK siempre es positivo → asegura escala positiva
  g_input_abs_max = POS_INPUT_ABS_MAX;

  if (phaseA) {
    // fase A: motor1 adelante (WALK_VEL), motor2 parado
    escWrite(esc1_r, esc1_l, +WALK_VEL, "WALK-A-M1");
    escWrite(esc2_r, esc2_l, 0.0f,      "WALK-A-M2");
  } else {
    // fase B: motor2 adelante (WALK_VEL), motor1 parado
    escWrite(esc1_r, esc1_l, 0.0f,      "WALK-B-M1");
    escWrite(esc2_r, esc2_l, +WALK_VEL, "WALK-B-M2");
  }
  float vA = phaseA ? +WALK_VEL : 0.0f;
  sendUDPStatus(vA, 0.0f, 0.0f, CMD_WALK);
}

// ===== WiFi =====
void wifiConnect(){
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.print("[WiFi] Conectando a "); Serial.println(WIFI_SSID);
  while (WiFi.status() != WL_CONNECTED) { delay(500); Serial.print("."); }
  Serial.println();
  Serial.print("[WiFi] IP: "); Serial.println(WiFi.localIP());
}

void setup(){
  Serial.begin(115200);
  Serial.println("\n[ESP32] UDP Control con 4x ESC (R/L por motor)");

  // Adjuntar ESCs (rango físico 1000–2000; neutral 1100)
  esc1_r.attach(ESC1_R_PIN, 1000, 2000);
  esc1_l.attach(ESC1_L_PIN, 1000, 2000);
  esc2_r.attach(ESC2_R_PIN, 1000, 2000);
  esc2_l.attach(ESC2_L_PIN, 1000, 2000);

  delay(2000); // Espera a que arranquen
  stopAllESC();
  delay(3000); // Arming delay típico de ESC

  wifiConnect();
  udp.begin(UDP_LISTEN_PORT);
  Serial.printf("[UDP] Escuchando en puerto %u\n", UDP_LISTEN_PORT);
  Serial.println("[ESC] Inicializados (neutral 1100 µs; escala global por trama)");
}

void loop(){
  int packetSize = udp.parsePacket();
  if (packetSize > 0){
    char buf[1024];
    int n = udp.read(buf, sizeof(buf)-1);
    if(n<0) n=0;
    buf[n]='\0';

    lastRemoteIP = udp.remoteIP();
    lastRemotePort = udp.remotePort();

    String s(buf);
    s.trim();

    if(s.length() && s.startsWith("<") && s.endsWith(">")){
      s.remove(0,1);
      s.remove(s.length()-1,1);

      int lastComma = s.lastIndexOf(',');
      if(lastComma > 0){
        String payload = s.substring(0,lastComma);
        String crc_hex = s.substring(lastComma+1);
        crc_hex.trim();

        uint8_t calc = crc8_maxim((const uint8_t*)payload.c_str(), payload.length());
        uint8_t recv = (uint8_t) strtoul(crc_hex.c_str(), nullptr, 16);

        if(calc == recv){
          int p1=payload.indexOf(',');
          int p2=payload.indexOf(',',p1+1);
          int p3=payload.indexOf(',',p2+1);
          int p4=payload.indexOf(',',p3+1);

          if(p1>=0 && p2>=0 && p3>=0 && p4>=0){
            float v1 = payload.substring(0,p1).toFloat();
            float v2 = payload.substring(p1+1,p2).toFloat();
            /*float v3 =*/ payload.substring(p2+1,p3).toFloat();
            int cmd = payload.substring(p3+1,p4).toInt();

            if(cmd==CMD_ESTOP){
              estop_latched = true;
              walk_sequence = false;
              stopAllESC();
              Serial.println("[SAFETY] E-STOP]");
              sendUDPStatus(0.0f, 0.0f, 0.0f, CMD_ESTOP);
            }
            else if(cmd==CMD_ERES){
              estop_latched = false;
              Serial.println("[SAFETY] RESET");
              sendUDPStatus(0.0f, 0.0f, 0.0f, CMD_ERES);
            }
            else if(estop_latched){
              // Ignorar comandos mientras esté latcheado
            }
            else if(cmd==CMD_ROLL){
              walk_sequence = false;

              // -------- Escala global por trama --------
              // Si cualquiera de las dos velocidades es negativa → usar 1.5
              bool any_negative = (v1 < 0.0f) || (v2 < 0.0f);
              g_input_abs_max = any_negative ? NEG_INPUT_ABS_MAX : POS_INPUT_ABS_MAX;

              Serial.println("[ROLL] =======");
              // Motor 1: esc1_r / esc1_l  (v1)
              escWrite(esc1_r, esc1_l, v1, "ESC1");
              // Motor 2: esc2_r / esc2_l  (v2)
              escWrite(esc2_r, esc2_l, v2, "ESC2");

              sendUDPStatus(v1, v2, 0.0f, CMD_ROLL);
            }
            else if(cmd==CMD_WALK){
              // WALK → escala positiva
              g_input_abs_max = POS_INPUT_ABS_MAX;
              Serial.println("[MODE] WALK (anuncio)");
              sendUDPStatus(0.0f, 0.0f, 0.0f, CMD_WALK);
            }
            else if(cmd==CMD_WALK_START){
              // WALK START → escala positiva
              g_input_abs_max = POS_INPUT_ABS_MAX;
              Serial.println("[WALK] START -> secuencia 10s A/B (ESC R/L)");
              walk_sequence = true;
              walk_phase_A = true;
              last_phase_ms = millis();
              applyWalkPhase(true);
            }
            else if(cmd==CMD_IDLE){
              Serial.println("[WALK] STOP -> todo a 1100 µs");
              walk_sequence = false;
              stopAllESC();
              sendUDPStatus(0.0f, 0.0f, 0.0f, CMD_IDLE);
            }
          }
        } else {
          Serial.println("[ERROR] CRC inválido");
        }
      }
    }
  }

  // WALK sequence automática
  if(!estop_latched && walk_sequence){
    uint32_t now = millis();
    if(now - last_phase_ms >= WALK_PHASE_MS){
      walk_phase_A = !walk_phase_A;
      last_phase_ms = now;
      applyWalkPhase(walk_phase_A);
      Serial.printf("[WALK] Alterna fase -> %s\n", walk_phase_A ? "A adelante" : "B adelante");
    }
  }
}
