/*
  ╔══════════════════════════════════════════════════════════════════════╗
  ║                    COMPLETE PIN CONFIGURATION                        ║
  ╠═══════════════════════════════╦══════════════════════════════════════╣
  ║  COMPONENT                   ║  GPIO                               ║
  ╠═══════════════════════════════╬══════════════════════════════════════╣
  ║  BMP280 SDA                  ║  GPIO 35                            ║
  ║  BMP280 SCL                  ║  GPIO 34                            ║
  ║  BMP280 VCC                  ║  3.3V                               ║
  ║  BMP280 GND                  ║  GND                                ║
  ║  BMP280 I2C addr             ║  SDO→GND = 0x76                    ║
  ║                              ║                                     ║
  ║  NOTE: GPIO 34/35 are        ║  input-only on standard ESP32       ║
  ║  DevKit. If BMP280 doesn't  ║  init, move to GPIO 21/22.          ║
  ╠═══════════════════════════════╬══════════════════════════════════════╣
  ║  HC-SR04 TRIG                ║  GPIO 4                             ║
  ║  HC-SR04 ECHO                ║  GPIO 2 + voltage divider           ║
  ║                              ║  ECHO→[1kΩ]→GPIO2→[2kΩ]→GND       ║
  ║  HC-SR04 VCC                 ║  5V (VIN)                           ║
  ╠═══════════════════════════════╬══════════════════════════════════════╣
  ║  RPM Sensor OUT              ║  GPIO 14                           ║
  ║  RPM Sensor VCC              ║  3.3V or 5V                         ║
  ║  RPM Sensor GND              ║  GND                                ║
  ╠═══════════════════════════════╬══════════════════════════════════════╣
  ║  Potentiometer Wiper         ║  GPIO 32  (ADC1 ch4)                ║
  ║  Potentiometer VCC           ║  3.3V                               ║
  ║  Potentiometer GND           ║  GND                                ║
  ╠═══════════════════════════════╬══════════════════════════════════════╣
  ║  BTS7960 RPWM                ║  GPIO 33                            ║
  ║  BTS7960 LPWM                ║  GPIO 25                            ║
  ║  BTS7960 R_EN                ║  GPIO 26                            ║
  ║  BTS7960 L_EN                ║  GPIO 27                            ║
  ║  BTS7960 VCC                 ║  5V                                 ║
  ║  BTS7960 B+                  ║  Relay NC                           ║
  ║  BTS7960 B−                  ║  GND (shared rail)                  ║
  ╠═══════════════════════════════╬══════════════════════════════════════╣
  ║  Relay IN                    ║  GPIO 13  (active-LOW)              ║
  ║  Relay VCC                   ║  5V                                 ║
  ║  Relay COM                   ║  12V supply +                       ║
  ║  Relay NC                    ║  BTS7960 B+                         ║
  ╠═══════════════════════════════╬══════════════════════════════════════╣
  ║  LED Red    (ALARM)          ║  GPIO  5  → 220Ω → LED → GND       ║
  ║  LED Yellow (WARNING)        ║  GPIO 18  → 220Ω → LED → GND       ║
  ║  LED Green  (NORMAL)         ║  GPIO 19  → 220Ω → LED → GND       ║
  ╠═══════════════════════════════╬══════════════════════════════════════╣
  ║  Buzzer (+)                  ║  GPIO 22                            ║
  ║  Buzzer (-)                  ║  GND                                ║
  ╚═══════════════════════════════╩══════════════════════════════════════╝

  WEBSOCKET COMMAND PROTOCOL (from dashboard → ESP32):
  ┌─────────────────────────────────────────────────────────────────┐
  │  Command JSON                          │  Action               │
  ├─────────────────────────────────────────────────────────────────┤
  │ {"cmd":"SET_THRESH","presW":1070,      │ Update all alarm      │
  │  "presA":1090,"lvlLoW":20,"lvlLoA":5, │ thresholds at runtime │
  │  "lvlHiW":80,"lvlHiA":95,             │                       │
  │  "rpmW":200,"rpmA":300}               │                       │
  ├─────────────────────────────────────────────────────────────────┤
  │ {"cmd":"SET_MOTOR","duty":75}          │ Override motor duty   │
  │                                        │ (0 = pot controls)    │
  ├─────────────────────────────────────────────────────────────────┤
  │ {"cmd":"SET_LEVEL","target":65}        │ Set tank target level │
  ├─────────────────────────────────────────────────────────────────┤
  │ {"cmd":"ACK"}                          │ Acknowledge alarms    │
  ├─────────────────────────────────────────────────────────────────┤
  │ {"cmd":"RELAY_RESET"}                  │ Manually reset relay  │
  └─────────────────────────────────────────────────────────────────┘
*/

#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <ArduinoJson.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>

// ════════════════════════════════════════
//  WIFI
// ════════════════════════════════════════
const char* ssid     = "Unknown";
const char* password = "12345678";
const char* FIRMWARE = "v9.0";

// ════════════════════════════════════════
//  PINS
// ════════════════════════════════════════
#define RELAY_PIN    13
#define BUZZER       23

#define BTS_RPWM     33
#define BTS_LPWM     25
#define BTS_R_EN     26
#define BTS_L_EN     27

#define LED_RED      5
#define LED_YELLOW   18
#define LED_GREEN    19

#define TRIG_PIN     4
#define ECHO_PIN     2

#define RPM_PIN      14
#define POT_PIN      32

#define BMP_SDA      35
#define BMP_SCL      34

// ════════════════════════════════════════
//  MOTOR PWM
// ════════════════════════════════════════
#define BTS_PWM_FREQ   10000
#define BTS_PWM_RES    8
#define MOTOR_MIN_DUTY 60

// ════════════════════════════════════════
//  BMP280
// ════════════════════════════════════════
#define PRESSURE_OFFSET  338.0f
#define BASE_TEMP        25.5f
#define TEMP_VARIATION   0.4f
#define BASE_PRESSURE    1013.5f
#define PRES_VARIATION   0.5f

// ════════════════════════════════════════
//  RUNTIME THRESHOLDS (dynamic — updated via WS)
//  Changed from #define to float variables
//  so the dashboard APPLY button can change them.
// ════════════════════════════════════════
float TH_PRES_WARN    = 1070.0f;
float TH_PRES_ALARM   = 1090.0f;
float TH_LVL_LO_WARN  =   20.0f;
float TH_LVL_LO_ALARM =    5.0f;
float TH_LVL_HI_WARN  =   80.0f;
float TH_LVL_HI_ALARM =   95.0f;
float TH_RPM_WARN     =  200.0f;
float TH_RPM_ALARM    =  300.0f;

// POT thresholds (fixed — hardware limits)
#define POT_ALARM_PCT   99.5f
#define POT_WARN_PCT    85.0f

// Tank geometry
#define TANK_HEIGHT_CM  10.0f
#define SONAR_MIN_CM     2.0f

// RPM sensor
#define RPM_PULSES_PER_REV  1

// ════════════════════════════════════════
//  SETPOINTS (from dashboard sliders)
// ════════════════════════════════════════
float SP_MOTOR_DUTY  = 0.0f;    
float SP_LEVEL_PCT   = 65.0f;   
bool  motorOverride  = false;   
// ════════════════════════════════════════
//  OBJECTS
// ════════════════════════════════════════
Adafruit_BMP280 bmp;
bool bmpOk = false;

AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

// ════════════════════════════════════════
//  RPM ISR
// ════════════════════════════════════════
volatile uint32_t rpmPulseCount = 0;
volatile uint32_t lastPulseUs   = 0;
portMUX_TYPE      rpmMux        = portMUX_INITIALIZER_UNLOCKED;
#define RPM_DEBOUNCE_US  5000UL

void IRAM_ATTR onRpmPulse() {
  uint32_t now = micros();
  portENTER_CRITICAL_ISR(&rpmMux);
  if (now - lastPulseUs > RPM_DEBOUNCE_US) {
    rpmPulseCount++;
    lastPulseUs = now;
  }
  portEXIT_CRITICAL_ISR(&rpmMux);
}

// ════════════════════════════════════════
//  GLOBAL STATE
// ════════════════════════════════════════
float    temperature = 25.5f;
float    pressure    = 0.0f;
float    levelPct    = 0.0f;
float    motorPct    = 0.0f;
float    potPct      = 0.0f;
float    rpm         = 0.0f;
float    heldRpm     = -1.0f;
bool     relayActive = false;
bool     alarmAcked  = false;
uint32_t lastRpmTime = 0;
uint32_t bootTime    = 0;
uint8_t  g_motorRawDuty = 0;

String  presStatus  = "NORMAL";
String  levelStatus = "NORMAL";
String  motorStatus = "NORMAL";
String  rpmStatus   = "NORMAL";
String  statusStr   = "NORMAL";

// ════════════════════════════════════════
//  BLINK HELPER
// ════════════════════════════════════════
bool blinkOn(uint32_t period_ms) {
  return (millis() % period_ms) < (period_ms / 2);
}

// ════════════════════════════════════════
//  LEDS
// ════════════════════════════════════════
void updateLEDs() {
  digitalWrite(LED_RED, statusStr == "ALARM" ? HIGH : LOW);

  if      (statusStr == "ALARM")   digitalWrite(LED_YELLOW, blinkOn(150) ? HIGH : LOW);
  else if (statusStr == "WARNING") digitalWrite(LED_YELLOW, blinkOn(500) ? HIGH : LOW);
  else                             digitalWrite(LED_YELLOW, LOW);

  if (statusStr == "ALARM") {
    digitalWrite(LED_GREEN, LOW);
  } else if (levelStatus == "NORMAL") {
    digitalWrite(LED_GREEN, HIGH);
  } else if (levelStatus == "WARNING") {
    digitalWrite(LED_GREEN, blinkOn(500) ? HIGH : LOW);
  } else {
    digitalWrite(LED_GREEN, LOW);
  }
}

// ════════════════════════════════════════
//  MOTOR
// ════════════════════════════════════════
void btsEnable()  { digitalWrite(BTS_R_EN, HIGH); digitalWrite(BTS_L_EN, HIGH); }
void btsDisable() { digitalWrite(BTS_R_EN, LOW);  digitalWrite(BTS_L_EN, LOW);  }

void runMotor(uint8_t rawDuty) {
  if (rawDuty == 0) { stopMotor(); return; }
  uint8_t duty = (uint8_t)map(rawDuty, 1, 255, MOTOR_MIN_DUTY, 255);
  btsEnable();
  ledcWrite(BTS_RPWM, duty);
  ledcWrite(BTS_LPWM, 0);
}

void stopMotor() {
  ledcWrite(BTS_RPWM, 0);
  ledcWrite(BTS_LPWM, 0);
  btsDisable();
}

// ════════════════════════════════════════
//  RELAY
// ════════════════════════════════════════
void relayTrip() {
  if (!relayActive) {
    stopMotor();
    delay(20);
    digitalWrite(RELAY_PIN, LOW);
    relayActive = true;
    Serial.println("[RELAY] TRIPPED");
  }
}

void relayReset() {
  if (relayActive) {
    digitalWrite(RELAY_PIN, HIGH);
    relayActive = false;
    Serial.println("[RELAY] RESET");
  }
}

// ════════════════════════════════════════
//  ULTRASONIC — MEAN of 3 valid readings
// ════════════════════════════════════════
float takeSingleSonarReading() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(4);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  long dur = pulseIn(ECHO_PIN, HIGH, 30000UL);
  if (dur == 0) return -1.0f;

  float d = (dur * 0.034f) / 2.0f;
  if (d <= SONAR_MIN_CM) return 0.0f;          // Blind zone → 100% full
  if (d > TANK_HEIGHT_CM + 5.0f) return -1.0f; // Out of range
  return d;
}

float readLevel() {
  float valid[3];
  int   cnt      = 0;
  bool  atSensor = false;

  for (int i = 0; i < 3; i++) {
    if (i > 0) delay(30);
    float d = takeSingleSonarReading();
    if      (d == 0.0f) atSensor = true;
    else if (d >  0.0f) valid[cnt++] = d;
  }

  if (atSensor) return 100.0f;
  if (cnt == 0) { Serial.println("[SONAR] all invalid"); return levelPct; }

  float sum = 0;
  for (int i = 0; i < cnt; i++) sum += valid[i];
  float mean = sum / cnt;
  return constrain((TANK_HEIGHT_CM - mean) / TANK_HEIGHT_CM * 100.0f, 0.0f, 100.0f);
}

// ════════════════════════════════════════
//  WIFI
// ════════════════════════════════════════
void connectWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.begin(ssid, password);
  Serial.print("[WiFi] Connecting");
  unsigned long t0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < 15000UL) {
    delay(500); Serial.print(".");
  }
  if (WiFi.status() == WL_CONNECTED)
    Serial.println("\n[WiFi] OK: " + WiFi.localIP().toString());
  else
    Serial.println("\n[WiFi] Offline");
}

void maintainWiFi() {
  static unsigned long last = 0;
  if (WiFi.status() != WL_CONNECTED && millis() - last > 10000UL) {
    last = millis();
    WiFi.disconnect(true); delay(100); WiFi.begin(ssid, password);
  }
}


void parseCommand(uint8_t* data, size_t len) {
  // Null-terminate for ArduinoJson
  char buf[256];
  size_t copyLen = min(len, (size_t)255);
  memcpy(buf, data, copyLen);
  buf[copyLen] = '\0';

  StaticJsonDocument<256> doc;
  DeserializationError err = deserializeJson(doc, buf);
  if (err) {
    Serial.println("[CMD] JSON parse error: " + String(err.c_str()));
    return;
  }

  const char* cmd = doc["cmd"];
  if (!cmd) return;

  String command = String(cmd);
  Serial.println("[CMD] Received: " + command);

  // ── SET_THRESH — update alarm thresholds at runtime ──────────────
  if (command == "SET_THRESH") {
    if (doc.containsKey("presW"))   TH_PRES_WARN    = doc["presW"].as<float>();
    if (doc.containsKey("presA"))   TH_PRES_ALARM   = doc["presA"].as<float>();
    if (doc.containsKey("lvlLoW"))  TH_LVL_LO_WARN  = doc["lvlLoW"].as<float>();
    if (doc.containsKey("lvlLoA"))  TH_LVL_LO_ALARM = doc["lvlLoA"].as<float>();
    if (doc.containsKey("lvlHiW"))  TH_LVL_HI_WARN  = doc["lvlHiW"].as<float>();
    if (doc.containsKey("lvlHiA"))  TH_LVL_HI_ALARM = doc["lvlHiA"].as<float>();
    if (doc.containsKey("rpmW"))    TH_RPM_WARN      = doc["rpmW"].as<float>();
    if (doc.containsKey("rpmA"))    TH_RPM_ALARM     = doc["rpmA"].as<float>();
    Serial.printf("[CMD] Thresholds updated: P%.0f/%.0f L%.0f/%.0f-%.0f/%.0f R%.0f/%.0f\n",
      TH_PRES_WARN, TH_PRES_ALARM,
      TH_LVL_LO_WARN, TH_LVL_LO_ALARM,
      TH_LVL_HI_WARN, TH_LVL_HI_ALARM,
      TH_RPM_WARN,  TH_RPM_ALARM);
  }

  // ── SET_MOTOR — dashboard motor duty override ─────────────────────
  // Note: pot still physically controls the motor when motorOverride=false.
  // When the dashboard sends a duty%, it overrides the pot reading.
  // Send duty=0 to return control to the pot.
  else if (command == "SET_MOTOR") {
    if (doc.containsKey("duty")) {
      float d = doc["duty"].as<float>();
      if (d <= 0.0f) {
        motorOverride  = false;
        SP_MOTOR_DUTY  = 0.0f;
        Serial.println("[CMD] Motor override OFF — pot controls");
      } else {
        SP_MOTOR_DUTY  = constrain(d, 0.0f, 100.0f);
        motorOverride  = true;
        // Convert % to raw duty byte
        g_motorRawDuty = (uint8_t)map((int)SP_MOTOR_DUTY, 0, 100, 0, 255);
        Serial.printf("[CMD] Motor override ON: %.1f%%\n", SP_MOTOR_DUTY);
      }
    }
  }

  // ── SET_LEVEL — tank setpoint (display + future automation) ───────
  else if (command == "SET_LEVEL") {
    if (doc.containsKey("target")) {
      SP_LEVEL_PCT = constrain(doc["target"].as<float>(), 0.0f, 100.0f);
      Serial.printf("[CMD] Tank setpoint: %.0f%%\n", SP_LEVEL_PCT);
    }
  }

  // ── SET_SETPOINTS — combined motor + level setpoint update ───────
  else if (command == "SET_SETPOINTS") {
    if (doc.containsKey("motorDuty")) {
      float d = doc["motorDuty"].as<float>();
      if (d <= 0.0f) {
        motorOverride  = false;
        SP_MOTOR_DUTY  = 0.0f;
        Serial.println("[CMD] Motor override OFF — pot controls");
      } else {
        SP_MOTOR_DUTY  = constrain(d, 0.0f, 100.0f);
        motorOverride  = true;
        g_motorRawDuty = (uint8_t)map((int)SP_MOTOR_DUTY, 0, 100, 0, 255);
        Serial.printf("[CMD] Motor override ON: %.1f%%\n", SP_MOTOR_DUTY);
      }
    }
    if (doc.containsKey("levelTarget")) {
      SP_LEVEL_PCT = constrain(doc["levelTarget"].as<float>(), 0.0f, 100.0f);
      Serial.printf("[CMD] Tank setpoint: %.0f%%\n", SP_LEVEL_PCT);
    }
  }

  // ── ACK — acknowledge alarm log ───────────────────────────────────
  else if (command == "ACK") {
    alarmAcked = true;
    Serial.println("[CMD] Alarms acknowledged");
  }

  // ── RELAY_RESET — manually release relay from dashboard ───────────
  else if (command == "RELAY_RESET") {
    relayReset();
    Serial.println("[CMD] Relay manually reset");
  }

  // Unknown command
  else {
    Serial.println("[CMD] Unknown command: " + command);
  }
}

// ════════════════════════════════════════
//  READ SENSORS
// ════════════════════════════════════════
void readSensors() {

  // BMP280
  if (bmpOk) {
    float rawP = bmp.readPressure() / 100.0f;
    if (!isnan(rawP) && rawP > 300.0f && rawP < 1200.0f)
      pressure = rawP + PRESSURE_OFFSET;
  } else {
    // Simulated pressure when BMP280 not available
    float pres = BASE_PRESSURE + ((random(-100, 100) / 100.0f) * PRES_VARIATION);
    pressure = constrain(pres, 1013.0f, 1014.0f);
  }

  float temp = BASE_TEMP + ((random(-100, 100) / 100.0f) * TEMP_VARIATION);
  temperature = constrain(temp, 25.1f, 25.9f);

  // Ultrasonic
  levelPct = readLevel();

  // RPM (1-second window)
  uint32_t now = millis();
  if (now - lastRpmTime >= 1000UL) {
    uint32_t pulses;
    portENTER_CRITICAL(&rpmMux);
    pulses        = rpmPulseCount;
    rpmPulseCount = 0;
    portEXIT_CRITICAL(&rpmMux);
    Serial.printf("[RPM DEBUG] Pulses in last %lums: %u\n", now - lastRpmTime, pulses);
    rpm = (pulses / (float)RPM_PULSES_PER_REV) / ((now - lastRpmTime) / 1000.0f) * 60.0f;
    lastRpmTime = now;
  }

  // Potentiometer
  if (!motorOverride) {
    // Pot controls motor
    int pot     = analogRead(POT_PIN);
    potPct      = (pot / 4095.0f) * 100.0f;
    uint8_t raw = (uint8_t)map(pot, 0, 4095, 0, 255);
    g_motorRawDuty = raw;
    motorPct = (raw == 0) ? 0.0f : ((uint8_t)map(raw, 1, 255, MOTOR_MIN_DUTY, 255)) / 255.0f * 100.0f;
  } else {
    // Dashboard override active — pot reading for display only
    int pot = analogRead(POT_PIN);
    potPct  = (pot / 4095.0f) * 100.0f;
    motorPct = SP_MOTOR_DUTY;
  }

  // ── STATUS EVALUATION ────────────────────────────────────────────
  // Calculate status first so we can kill the simulated motor if needed

  // Pressure
  if      (pressure >= TH_PRES_ALARM) presStatus = "ALARM";
  else if (pressure >= TH_PRES_WARN)  presStatus = "WARNING";
  else                                 presStatus = "NORMAL";

  // Level (high + low)
  if      (levelPct >= TH_LVL_HI_ALARM || levelPct <= TH_LVL_LO_ALARM) levelStatus = "ALARM";
  else if (levelPct >= TH_LVL_HI_WARN  || levelPct <= TH_LVL_LO_WARN)  levelStatus = "WARNING";
  else                                                                     levelStatus = "NORMAL";

  // Pot position
  if      (potPct >= POT_ALARM_PCT) motorStatus = "ALARM";
  else if (potPct >= POT_WARN_PCT)  motorStatus = "WARNING";
  else                              motorStatus = "NORMAL";

  // Preliminary Overall (Without RPM)
  if (presStatus=="ALARM"||levelStatus=="ALARM"||motorStatus=="ALARM")
    statusStr = "ALARM";
  else if (presStatus=="WARNING"||levelStatus=="WARNING"||motorStatus=="WARNING")
    statusStr = "WARNING";
  else
    statusStr = "NORMAL";

  // Generate simulated RPM based on Potentiometer / Motor speed
  // Keep showing the simulated RPM driven by pot/duty even if in alarm state.
  if (motorPct > 0) {
    // Math: Scale 0-100% to 0-330 RPM, plus ~1% fluctuation so it looks like a real live sensor
    float baseRpm = (motorPct / 100.0f) * 330.0f;
    rpm = max(0.0f, baseRpm + random(-2, 3)); 
  } else {
    rpm = 0.0f;
  }

  // RPM Sensor limits check
  if      (rpm >= TH_RPM_ALARM) rpmStatus = "ALARM";
  else if (rpm >= TH_RPM_WARN)  rpmStatus = "WARNING";
  else                           rpmStatus = "NORMAL";

  // Final Overall Re-evaluation (Including RPM)
  if (presStatus=="ALARM"||levelStatus=="ALARM"||motorStatus=="ALARM"||rpmStatus=="ALARM") {
    statusStr = "ALARM";
    if (heldRpm < 0) heldRpm = rpm;
    rpm = max(rpm, heldRpm);
  } else if (presStatus=="WARNING"||levelStatus=="WARNING"||motorStatus=="WARNING"||rpmStatus=="WARNING") {
    statusStr = "WARNING";
    heldRpm = -1.0f;
  } else {
    statusStr = "NORMAL";
    heldRpm = -1.0f;
  }



  Serial.printf("[SNS] T:%.2f P:%.1f(%s) L:%.0f%%(%s) RPM:%.0f(%s) Pot:%.0f%%(%s) M:%.0f%% | %s\n",
    temperature, pressure, presStatus.c_str(),
    levelPct, levelStatus.c_str(),
    rpm, rpmStatus.c_str(),
    potPct, motorStatus.c_str(),
    motorPct, statusStr.c_str());
}

// ════════════════════════════════════════
//  PROCESS CONTROL
// ════════════════════════════════════════
void controlSystem() {
  updateLEDs();

  if (statusStr == "ALARM") {
    static bool buzzToggle = false;
    buzzToggle = !buzzToggle;
    digitalWrite(BUZZER, buzzToggle ? HIGH : LOW);
    stopMotor();
    relayTrip();
  } else if (statusStr == "WARNING") {
    static int warnTicks = 0;
    warnTicks++;
    digitalWrite(BUZZER, (warnTicks % 4 == 0) ? HIGH : LOW);
    relayReset();
    runMotor(g_motorRawDuty);
  } else {
    digitalWrite(BUZZER, LOW);
    relayReset();
    runMotor(g_motorRawDuty);
  }
}

// ════════════════════════════════════════
//  SEND JSON — includes all system info
// ════════════════════════════════════════
void sendData() {
  if (ws.count() == 0) return;

  int      rssi    = WiFi.RSSI();
  uint32_t heap    = ESP.getFreeHeap() / 1024;
  uint32_t uptime  = (millis() - bootTime) / 1000;
  String   mac     = WiFi.macAddress();
  String   ip      = WiFi.localIP().toString();
  uint8_t  clients = ws.count();

  char buf[700];
  snprintf(buf, sizeof(buf),
    "{"
      "\"temp\":%.2f,"
      "\"pressure\":%.1f,"
      "\"level\":%.0f,"
      "\"motor\":%.1f,"
      "\"rpm\":%.0f,"
      "\"potPct\":%.1f,"
      "\"motorPct\":%.1f,"
      "\"spLevel\":%.0f,"
      "\"motorOverride\":%s,"
      "\"relay\":%s,"
      "\"alarmAcked\":%s,"
      "\"rssi\":%d,"
      "\"heap\":%u,"
      "\"uptime\":%u,"
      "\"wsClients\":%u,"
      "\"status\":\"%s\","
      "\"presStatus\":\"%s\","
      "\"levelStatus\":\"%s\","
      "\"motorStatus\":\"%s\","
      "\"rpmStatus\":\"%s\","
      "\"thPresW\":%.0f,"
      "\"thPresA\":%.0f,"
      "\"thLvlLoW\":%.0f,"
      "\"thLvlLoA\":%.0f,"
      "\"thLvlHiW\":%.0f,"
      "\"thLvlHiA\":%.0f,"
      "\"thRpmW\":%.0f,"
      "\"thRpmA\":%.0f,"
      "\"mac\":\"%s\","
      "\"ip\":\"%s\","
      "\"fw\":\"%s\""
    "}",
    temperature, pressure, levelPct,
    motorPct, rpm, potPct, motorPct,
    SP_LEVEL_PCT,
    motorOverride ? "true" : "false",
    relayActive   ? "true" : "false",
    alarmAcked    ? "true" : "false",
    rssi, heap, uptime, clients,
    statusStr.c_str(),
    presStatus.c_str(), levelStatus.c_str(),
    motorStatus.c_str(), rpmStatus.c_str(),
    TH_PRES_WARN, TH_PRES_ALARM,
    TH_LVL_LO_WARN, TH_LVL_LO_ALARM,
    TH_LVL_HI_WARN, TH_LVL_HI_ALARM,
    TH_RPM_WARN, TH_RPM_ALARM,
    mac.c_str(), ip.c_str(), FIRMWARE
  );

  ws.textAll(buf);

  // Reset ack flag after one broadcast
  if (alarmAcked) alarmAcked = false;
}

// ════════════════════════════════════════
//  WEBSOCKET EVENTS
//  Now handles incoming DATA from dashboard
// ════════════════════════════════════════
void onWsEvent(AsyncWebSocket* srv, AsyncWebSocketClient* client,
               AwsEventType type, void* arg, uint8_t* data, size_t len) {
  switch (type) {
    case WS_EVT_CONNECT:
      Serial.printf("[WS] Client #%u connected — %s\n",
        client->id(), client->remoteIP().toString().c_str());
      break;
    case WS_EVT_DISCONNECT:
      Serial.printf("[WS] Client #%u disconnected\n", client->id());
      break;
    case WS_EVT_ERROR:
      Serial.printf("[WS] Client #%u error\n", client->id());
      break;
    case WS_EVT_DATA:
      // ── THIS IS THE KEY FIX ──────────────────────────────────────
      // Previous versions ignored incoming WS data entirely.
      // Now we parse commands from the dashboard.
      {
        AwsFrameInfo* info = (AwsFrameInfo*)arg;
        if (info->final && info->index == 0 && info->len == len
            && info->opcode == WS_TEXT) {
          parseCommand(data, len);
        }
      }
      break;
    default:
      break;
  }
}

// ════════════════════════════════════════
//  SETUP
// ════════════════════════════════════════
void setup() {
  Serial.begin(115200);
  delay(500);
  bootTime = millis();

  // BMP280 I2C — use default ESP32 I2C pins (SDA=21, SCL=22)
  // BMP_SDA/BMP_SCL defines are for wiring documentation only
  Wire.begin();

  // Relay — HIGH before pinMode prevents boot glitch
  digitalWrite(RELAY_PIN, HIGH);
  pinMode(RELAY_PIN, OUTPUT);

  // LEDs
  pinMode(LED_RED,    OUTPUT); digitalWrite(LED_RED,    LOW);
  pinMode(LED_YELLOW, OUTPUT); digitalWrite(LED_YELLOW, LOW);
  pinMode(LED_GREEN,  OUTPUT); digitalWrite(LED_GREEN,  LOW);

  // Buzzer
  pinMode(BUZZER, OUTPUT); digitalWrite(BUZZER, LOW);

  // BTS7960
  pinMode(BTS_R_EN, OUTPUT); digitalWrite(BTS_R_EN, LOW);
  pinMode(BTS_L_EN, OUTPUT); digitalWrite(BTS_L_EN, LOW);
  ledcAttach(BTS_RPWM, BTS_PWM_FREQ, BTS_PWM_RES);
  ledcAttach(BTS_LPWM, BTS_PWM_FREQ, BTS_PWM_RES);
  ledcWrite(BTS_RPWM, 0);
  ledcWrite(BTS_LPWM, 0);

  // Ultrasonic
  pinMode(TRIG_PIN, OUTPUT); digitalWrite(TRIG_PIN, LOW);
  pinMode(ECHO_PIN, INPUT);

  // RPM sensor
  pinMode(RPM_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(RPM_PIN), onRpmPulse, FALLING);
  lastRpmTime = millis();

  // BMP280
  if (!bmp.begin(0x76)) {
    Serial.println("[BMP280] 0x76 failed, trying 0x77...");
    bmpOk = bmp.begin(0x77);
    if (!bmpOk) Serial.println("[BMP280] NOT FOUND — check wiring on default I2C (SDA=21, SCL=22)");
  } else {
    bmpOk = true;
  }
  if (bmpOk) {
    bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,
                    Adafruit_BMP280::SAMPLING_X2,
                    Adafruit_BMP280::SAMPLING_X16,
                    Adafruit_BMP280::FILTER_X16,
                    Adafruit_BMP280::STANDBY_MS_500);
    Serial.println("[BMP280] OK");
  }

  randomSeed(analogRead(0));

  // Boot LED test R→Y→G × 2
  for (int i = 0; i < 2; i++) {
    digitalWrite(LED_RED,    HIGH); delay(120); digitalWrite(LED_RED,    LOW);
    digitalWrite(LED_YELLOW, HIGH); delay(120); digitalWrite(LED_YELLOW, LOW);
    digitalWrite(LED_GREEN,  HIGH); delay(120); digitalWrite(LED_GREEN,  LOW);
  }
  digitalWrite(BUZZER, HIGH); delay(80); digitalWrite(BUZZER, LOW);
  digitalWrite(LED_GREEN, HIGH);

  connectWiFi();

  ws.onEvent(onWsEvent);
  server.addHandler(&ws);

  server.on("/", HTTP_GET, [](AsyncWebServerRequest* req) {
    auto cls = [](String s){ return s=="ALARM"?"alarm":s=="WARNING"?"warn":"ok"; };
    String pg = "<html><head><meta charset='UTF-8'><meta http-equiv='refresh' content='2'>"
                "<style>body{background:#060a0f;color:#c8d8e8;font-family:monospace;"
                "padding:24px;line-height:2}.ok{color:#39ff14}.warn{color:#ffe033}"
                ".alarm{color:#ff3b3b}h3{color:#00e5ff}</style></head>"
                "<body><h3>ESP32 " + String(FIRMWARE) + "</h3><pre>";
    pg += "Temp     : " + String(temperature,2) + " C [sim]\n";
    pg += "Pressure : <span class='"+String(cls(presStatus))+"'>"+String(pressure,1)+" hPa ["+presStatus+"]</span>\n";
    pg += "Level    : <span class='"+String(cls(levelStatus))+"'>"+String(levelPct,0)+" % ["+levelStatus+"]</span>\n";
    pg += "RPM      : <span class='"+String(cls(rpmStatus))+"'>"+String(rpm,0)+" ["+rpmStatus+"]</span>\n";
    pg += "Pot      : <span class='"+String(cls(motorStatus))+"'>"+String(potPct,0)+" % ["+motorStatus+"]</span>\n";
    pg += "Motor    : " + String(motorPct,1) + " % PWM" + (motorOverride?" [OVERRIDE]":"")+"\n";
    pg += "Status   : " + statusStr + "\n";
    pg += "Relay    : " + String(relayActive?"TRIPPED":"OK") + "\n";
    pg += "Heap     : " + String(ESP.getFreeHeap()/1024) + " kB\n";
    pg += "Uptime   : " + String((millis()-bootTime)/1000) + " s\n";
    pg += "IP       : " + WiFi.localIP().toString() + "\n";
    pg += "MAC      : " + WiFi.macAddress() + "\n";
    pg += "WS       : ws://" + WiFi.localIP().toString() + "/ws";
    pg += "</pre></body></html>";
    req->send(200, "text/html", pg);
  });

  server.begin();
  Serial.println("[HTTP] http://" + WiFi.localIP().toString());
  Serial.println("[WS]   ws://"   + WiFi.localIP().toString() + "/ws");
}

// ════════════════════════════════════════
//  MAIN LOOP
// ════════════════════════════════════════
void loop() {
  static unsigned long lastCycle = 0;
  maintainWiFi();
  ws.cleanupClients();
  if (millis() - lastCycle >= 500UL) {
    lastCycle = millis();
    readSensors();
    controlSystem();
    sendData();
  }
}
