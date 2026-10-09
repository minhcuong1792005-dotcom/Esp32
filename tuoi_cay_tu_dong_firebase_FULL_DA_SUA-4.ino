/*
 * SMART GARDEN - ESP32 + DHT11 + 2 cam bien dat + relay + Firebase RTDB
 *
 * QUY TAC DIEU KHIEN BOM
 *  - Do am dat = trung binh 2 cam bien (%).
 *  - NGUONG (chinh trong app, /control/threshold) vua la muc BAT vua la muc TAT:
 *      + TU DONG BAT : do am < NGUONG - AUTO_START_MARGIN
 *      + TU DONG TAT : do am >= NGUONG   (ap dung ca khi dang tuoi thu cong)
 *  - TUOI THU CONG (nut TUOI NGAY): bat bom ngay, tu tat khi do am >= NGUONG.
 *      Neu luc bam do am da >= NGUONG thi bom van chay (toi da 60 giay) hoac den khi bam DUNG.
 *  - DUNG TUOI: tat ngay o moi che do, khoa tu dong 2 phut.
 *  - An toan: moi lan bom chay toi da 60 giay.
 *
 * Database:
 *   /status  soil1 soil2 soilAvg soilRaw1 soilRaw2 temperature humidity pumpState pumpMode updatedAt
 *   /control threshold manualPump stopSeq
 *   /alerts/latest  type message timestamp
 */

#include <Arduino.h>
#include <WiFi.h>
#include <time.h>
#include <Firebase_ESP_Client.h>
#include "DHT.h"
#include <addons/TokenHelper.h>
#include <addons/RTDBHelper.h>

// ========================= WIFI / FIREBASE =========================
#define WIFI_SSID     "Bắt chùa clon"
#define WIFI_PASSWORD "cocaidaubuoi"

#define API_KEY "AIzaSyCIgA2b3SXkoUX3cid8v_HJdd8U-C2UW-M"
#define DATABASE_URL "https://tuoicayiot-cc958-default-rtdb.firebaseio.com"

// ============================ HARDWARE =============================
constexpr uint8_t PIN_DHT    = 4;
constexpr uint8_t PIN_RELAY  = 25;
constexpr uint8_t PIN_SOIL_1 = 32;
constexpr uint8_t PIN_SOIL_2 = 33;

#define DHTTYPE DHT11
DHT dht(PIN_DHT, DHTTYPE);

constexpr int RELAY_ON  = HIGH;
constexpr int RELAY_OFF = LOW;

// Hieu chuan cam bien: ADC cao = kho. Do lai bang Serial roi sua 2 so nay.
constexpr int SOIL_WET_RAW = 1200; // 100% am
constexpr int SOIL_DRY_RAW = 3300; // 0% am

constexpr unsigned long SENSOR_INTERVAL_MS   = 2000;
constexpr unsigned long WATERING_CHECK_MS    = 1000;
constexpr unsigned long CONTROL_INTERVAL_MS  = 1500;
constexpr unsigned long MAX_WATER_SAFETY_MS  = 60000;
constexpr unsigned long COOLDOWN_AFTER_STOP_MS   = 120000UL; // sau khi bam DUNG
constexpr unsigned long COOLDOWN_AFTER_SAFETY_MS = 300000UL; // sau khi het gio an toan
constexpr unsigned long COOLDOWN_AFTER_WET_MS    = 10000UL;  // sau khi du am
constexpr int AUTO_START_MARGIN = 3;  // tu bat khi do am < nguong - 3 (tranh bat/tat lien tuc)
constexpr int SOIL_SAMPLES = 10;
constexpr int DHT_MAX_FAILS_WARN = 5;

// ========================== FIREBASE ===============================
FirebaseData fbdo;
FirebaseAuth auth;
FirebaseConfig config;

bool signupOK = false;
bool firebaseDefaultsInitialized = false;
unsigned long lastAuthRetry = 0;
unsigned long lastSensorRead = 0;
unsigned long lastWaterCheck = 0;
unsigned long lastControlRead = 0;
unsigned long stateChangedAt = 0;
unsigned long autoBlockedUntil = 0;

enum SystemState { ST_MONITOR, ST_WATERING };
SystemState state = ST_MONITOR;
bool pumpIsManual = false;        // dang tuoi theo lenh thu cong?
bool ignoreMoistureStop = false;  // thu cong khi dat da du am luc bam

int  thresholdPercent = 40;
bool manualPump = false;          // ban sao /control/manualPump
bool pendingClearManual = false;  // can ghi manualPump=false len Firebase
long lastStopSeq = -1;
int  lastSoilAvg = -1;            // do am gan nhat (-1 = chua do)

float lastValidTemp = NAN;
float lastValidHumidity = NAN;
uint16_t dhtConsecutiveFails = 0;
String lastAlertMessage = "";

unsigned long epochMs() {
  time_t now = time(nullptr);
  if (now < 100000) return millis();
  return (unsigned long)now * 1000UL;
}

// ============================ HELPERS ==============================
int readSoilAveraged(uint8_t pin, int samples = SOIL_SAMPLES) {
  long sum = 0;
  for (int i = 0; i < samples; i++) {
    sum += analogRead(pin);
    delay(10);
  }
  return sum / samples;
}

int rawToPercent(int raw) {
  long pct = map(raw, SOIL_DRY_RAW, SOIL_WET_RAW, 0, 100);
  return constrain((int)pct, 0, 100);
}

void publishAlert(const char *type, const String &message) {
  if (!Firebase.ready() || message == lastAlertMessage) return;
  FirebaseJson json;
  json.set("type", type);
  json.set("message", message);
  json.set("timestamp", (double)epochMs());
  if (Firebase.RTDB.setJSON(&fbdo, "/alerts/latest", &json)) {
    lastAlertMessage = message;
  } else {
    Serial.print("Firebase alert error: ");
    Serial.println(fbdo.errorReason());
  }
}

void publishStatus(int raw1, int raw2, int soil1, int soil2, float temp, float humidity) {
  if (!Firebase.ready()) return;
  FirebaseJson json;
  json.set("soil1", soil1);
  json.set("soil2", soil2);
  json.set("soilAvg", (soil1 + soil2) / 2);
  json.set("soilRaw1", raw1);
  json.set("soilRaw2", raw2);
  if (!isnan(temp)) json.set("temperature", temp);
  if (!isnan(humidity)) json.set("humidity", humidity);
  json.set("pumpState", state == ST_WATERING ? "WATERING" : "IDLE");
  json.set("pumpMode", state == ST_WATERING ? (pumpIsManual ? "MANUAL" : "AUTO") : "-");
  json.set("updatedAt", (double)epochMs());
  if (!Firebase.RTDB.setJSON(&fbdo, "/status", &json)) {
    Serial.print("Firebase status error: ");
    Serial.println(fbdo.errorReason());
  }
}

// ---------- dieu khien bom ----------
void startWatering(bool manual, const String &reason) {
  if (state == ST_WATERING) return;
  digitalWrite(PIN_RELAY, RELAY_ON);
  state = ST_WATERING;
  pumpIsManual = manual;
  stateChangedAt = millis();
  lastWaterCheck = millis();
  // Thu cong ma dat da du am luc bam: khong tu tat theo do am (van co gioi han 60 giay).
  ignoreMoistureStop = manual && lastSoilAvg >= thresholdPercent;
  Serial.print(">> BAT BOM: ");
  Serial.println(reason);
}

void stopWatering(const String &reason, unsigned long cooldownMs) {
  digitalWrite(PIN_RELAY, RELAY_OFF);
  state = ST_MONITOR;
  stateChangedAt = millis();
  autoBlockedUntil = millis() + cooldownMs;
  Serial.print(">> TAT BOM: ");
  Serial.println(reason);
}

// ---------- doc lenh tu app ----------
void readControl() {
  if (!Firebase.ready()) return;

  if (Firebase.RTDB.getInt(&fbdo, "/control/threshold")) {
    thresholdPercent = constrain(fbdo.to<int>(), 10, 90);
  }

  // Lenh DUNG: app tang stopSeq moi lan bam nut.
  if (Firebase.RTDB.getInt(&fbdo, "/control/stopSeq")) {
    long seq = fbdo.to<int>();
    if (lastStopSeq < 0) {
      lastStopSeq = seq;                       // lan dau: chi dong bo
    } else if (seq != lastStopSeq) {
      lastStopSeq = seq;
      manualPump = false;
      if (state == ST_WATERING) {
        stopWatering("lenh dung tu app", COOLDOWN_AFTER_STOP_MS);
        publishAlert("INFO", "Đã dừng máy bơm theo lệnh từ ứng dụng.");
      } else {
        autoBlockedUntil = millis() + COOLDOWN_AFTER_STOP_MS;
      }
      return;
    }
  }

  // Dang cho ghi manualPump=false len Firebase thi bo qua gia tri cu doc ve.
  if (pendingClearManual) return;

  if (Firebase.RTDB.getBool(&fbdo, "/control/manualPump")) {
    bool newManual = fbdo.to<bool>();
    if (newManual != manualPump) {
      manualPump = newManual;
      if (manualPump) {
        if (state == ST_MONITOR) {
          startWatering(true, "lenh thu cong tu app");
          if (ignoreMoistureStop) {
            publishAlert("INFO", "Đất đã đủ ẩm nhưng vẫn tưới theo lệnh thủ công (tối đa 60 giây).");
          } else {
            publishAlert("INFO", "Đã bật máy bơm theo lệnh thủ công từ ứng dụng.");
          }
        } else {
          pumpIsManual = true;                 // tiep quan bom dang tu dong
        }
      } else if (state == ST_WATERING) {
        stopWatering("lenh dung tu app", COOLDOWN_AFTER_STOP_MS);
        publishAlert("INFO", "Đã dừng máy bơm theo lệnh từ ứng dụng.");
      }
    }
  }
}

void readDHT(float &temp, float &humidity) {
  humidity = dht.readHumidity();
  temp = dht.readTemperature();
  if (isnan(humidity) || isnan(temp)) {
    dhtConsecutiveFails++;
    temp = lastValidTemp;
    humidity = lastValidHumidity;
    Serial.print("CANH BAO: DHT11 doc loi, lan ");
    Serial.println(dhtConsecutiveFails);
    if (dhtConsecutiveFails == DHT_MAX_FAILS_WARN) {
      publishAlert("ERROR", "DHT11 lỗi liên tục 5 lần. Kiểm tra VCC/GND/DATA và điện trở pull-up 4.7k-10k.");
    }
  } else {
    dhtConsecutiveFails = 0;
    lastValidTemp = temp;
    lastValidHumidity = humidity;
  }
}

void handleSerialTestCommand() {
  if (Serial.available() <= 0) return;
  char c = Serial.read();
  while (Serial.available() > 0) Serial.read();
  if ((c == 'b' || c == 'B') && state == ST_MONITOR) {
    manualPump = true;
    startWatering(true, "lenh test Serial");
  }
}

// ============================== SETUP ==============================
void setup() {
  Serial.begin(115200);
  delay(300);

  dht.begin();
  pinMode(PIN_RELAY, OUTPUT);
  digitalWrite(PIN_RELAY, RELAY_OFF);

  Serial.println();
  Serial.println("====================================");
  Serial.println("     SMART GARDEN - ESP32");
  Serial.println("====================================");

  WiFi.mode(WIFI_STA);
  WiFi.setAutoReconnect(true);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("Dang ket noi Wi-Fi");

  unsigned long lastDot = 0;
  unsigned long lastWiFiRetry = millis();
  while (WiFi.status() != WL_CONNECTED) {
    delay(250);
    if (millis() - lastDot >= 1000) {
      lastDot = millis();
      Serial.print(".");
    }
    if (millis() - lastWiFiRetry >= 15000) {
      lastWiFiRetry = millis();
      Serial.println();
      Serial.println("Wi-Fi chua ket noi, thu lai...");
      WiFi.disconnect();
      delay(200);
      WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    }
  }
  Serial.println();
  Serial.print("Wi-Fi OK. IP: ");
  Serial.println(WiFi.localIP());

  configTime(7 * 3600, 0, "pool.ntp.org", "time.nist.gov");

  config.api_key = API_KEY;
  config.database_url = DATABASE_URL;
  config.token_status_callback = tokenStatusCallback;

  Firebase.reconnectNetwork(true);
  fbdo.setBSSLBufferSize(4096, 1024);
  fbdo.setResponseSize(2048);

  if (Firebase.signUp(&config, &auth, "", "")) {
    signupOK = true;
    Serial.println("Firebase Anonymous Auth: OK");
  } else {
    signupOK = false;
    Serial.print("Firebase Auth ERROR: ");
    Serial.println(config.signer.signupError.message.c_str());
    Serial.println("Se thu xac thuc lai trong loop().");
  }

  Firebase.begin(&config, &auth);
  Serial.println("Firebase.begin() da duoc goi; dang cho Firebase.ready().");
}

// =============================== LOOP ==============================
void loop() {
  handleSerialTestCommand();
  unsigned long now = millis();

  // Thu lai dang nhap an danh neu that bai.
  if (WiFi.status() == WL_CONNECTED && !signupOK &&
      (lastAuthRetry == 0 || now - lastAuthRetry >= 15000)) {
    lastAuthRetry = now;
    Serial.println("Dang thu Firebase Anonymous Auth lai...");
    if (Firebase.signUp(&config, &auth, "", "")) {
      signupOK = true;
      Serial.println("Firebase Anonymous Auth: OK (retry)");
    } else {
      Serial.print("Firebase Auth retry ERROR: ");
      Serial.println(config.signer.signupError.message.c_str());
    }
  }

  bool firebaseIsReady = signupOK && Firebase.ready();

  // Khoi tao gia tri mac dinh khi Firebase san sang.
  if (firebaseIsReady && !firebaseDefaultsInitialized) {
    if (!Firebase.RTDB.getInt(&fbdo, "/control/threshold")) {
      Firebase.RTDB.setInt(&fbdo, "/control/threshold", thresholdPercent);
    } else {
      thresholdPercent = constrain(fbdo.to<int>(), 10, 90);
    }
    // Luon dua manualPump ve false luc khoi dong (tranh lenh cu con sot lai).
    Firebase.RTDB.setBool(&fbdo, "/control/manualPump", false);
    manualPump = false;

    firebaseDefaultsInitialized = true;
    Serial.println("Firebase RTDB da san sang.");
    publishAlert("INFO", "ESP32 đã kết nối Firebase và sẵn sàng.");
  }

  // Ghi lai manualPump=false neu lan truoc ghi that bai.
  if (pendingClearManual && firebaseIsReady) {
    if (Firebase.RTDB.setBool(&fbdo, "/control/manualPump", false)) {
      pendingClearManual = false;
    }
  }

  // Doc lenh tu app.
  if (now - lastControlRead >= CONTROL_INTERVAL_MS) {
    lastControlRead = now;
    readControl();
  }

  // Doc cam bien + gui trang thai + tu dong BAT bom.
  if (now - lastSensorRead >= SENSOR_INTERVAL_MS) {
    lastSensorRead = now;

    float temp = NAN, humidity = NAN;
    readDHT(temp, humidity);

    int raw1 = readSoilAveraged(PIN_SOIL_1);
    int raw2 = readSoilAveraged(PIN_SOIL_2);
    int soil1 = rawToPercent(raw1);
    int soil2 = rawToPercent(raw2);
    int soilAvg = (soil1 + soil2) / 2;
    lastSoilAvg = soilAvg;

    Serial.print("Dat1="); Serial.print(soil1);
    Serial.print("% (ADC "); Serial.print(raw1);
    Serial.print(") | Dat2="); Serial.print(soil2);
    Serial.print("% (ADC "); Serial.print(raw2);
    Serial.print(") | Avg="); Serial.print(soilAvg);
    Serial.print("% | Nguong="); Serial.print(thresholdPercent);
    Serial.print("% | Nhiet="); Serial.print(temp, 1);
    Serial.print("C | KK="); Serial.print(humidity, 1);
    Serial.print("% | Bom=");
    Serial.println(state == ST_WATERING ? (pumpIsManual ? "ON(thu cong)" : "ON(tu dong)") : "OFF");

    publishStatus(raw1, raw2, soil1, soil2, temp, humidity);

    if (!manualPump && state == ST_MONITOR &&
        soilAvg < thresholdPercent - AUTO_START_MARGIN &&
        (long)(millis() - autoBlockedUntil) >= 0) {
      startWatering(false, "do am thap hon nguong");
      publishAlert("WARNING", "Đất khô: hệ thống đã tự bật máy bơm.");
    }
  }

  // Khi bom dang chay: kiem tra TAT (do am dat nguong hoac het gio an toan).
  if (state == ST_WATERING && now - lastWaterCheck >= WATERING_CHECK_MS) {
    lastWaterCheck = now;

    int p1 = rawToPercent(readSoilAveraged(PIN_SOIL_1));
    int p2 = rawToPercent(readSoilAveraged(PIN_SOIL_2));
    int soilNow = (p1 + p2) / 2;
    lastSoilAvg = soilNow;

    Serial.print("Dang tuoi: Dat1="); Serial.print(p1);
    Serial.print("% Dat2="); Serial.print(p2);
    Serial.print("% Avg="); Serial.print(soilNow);
    Serial.print("% / Nguong="); Serial.print(thresholdPercent);
    Serial.println("%");

    // Da tuoi dat muc nguong thi khong can ignore nua.
    if (ignoreMoistureStop && soilNow < thresholdPercent) ignoreMoistureStop = false;

    if (!ignoreMoistureStop && soilNow >= thresholdPercent) {
      stopWatering("do am dat nguong", COOLDOWN_AFTER_WET_MS);
      if (manualPump) {
        manualPump = false;
        pendingClearManual = true;             // ghi false len Firebase (co thu lai neu loi)
      }
      publishAlert("INFO", "Độ ẩm đã đạt ngưỡng: hệ thống đã tắt máy bơm.");
    } else if (millis() - stateChangedAt >= MAX_WATER_SAFETY_MS) {
      stopWatering("het thoi gian an toan", COOLDOWN_AFTER_SAFETY_MS);
      if (manualPump) {
        manualPump = false;
        pendingClearManual = true;
      }
      publishAlert("ERROR", "Bơm đã tự ngắt vì vượt thời gian an toàn 60 giây. Kiểm tra cảm biến đất.");
    }
  }

  // Wi-Fi tu ket noi lai.
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("Wi-Fi mat ket noi, dang thu ket noi lai...");
    WiFi.reconnect();
    delay(200);
  }
}
