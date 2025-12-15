/* TCL + MQTT + DHT11 (ESP32)
   - Attributes: Nhận lệnh từ dashboard → Phát IR → Publish telemetry ngay
   - IR Receive: Thu remote → Decode → Publish telemetry ngay
   - Sensor: Loop riêng, chỉ publish sensor data (không log AC state)
*/

#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>

#include <IRremoteESP8266.h>
#include <IRsend.h>
#include <IRrecv.h>
#include <IRutils.h>
#include <IRtext.h>
#include <ir_Tcl.h>

#include <DHT.h>

// ======= WiFi & MQTT cấu hình =======
#define WIFI_SSID  "ABC"
#define WIFI_PASS  "12345678"
#define TOKEN      "i8SsraTSjHajSmVC0snM"
#define MQTT_SERVER "thingsboard.cloud"
#define MQTT_PORT   1883

WiFiClient espClient;
PubSubClient mqtt(espClient);

// ======= IR pins & config =======
const uint16_t IR_RECV_PIN = 4;
const uint16_t IR_SEND_PIN = 15;

// IR receiver config (giống DumpV2 để đảm bảo thu được TCL)
const uint16_t kCaptureBufferSize = 1024;  // Buffer lớn cho AC codes
const uint8_t kTimeout = 50;               // Timeout cho AC protocols
const uint8_t kMinUnknownSize = 12;

IRrecv irrecv(IR_RECV_PIN, kCaptureBufferSize, kTimeout, true);
decode_results results;
IRTcl112Ac ac(IR_SEND_PIN);

// ======= DHT11 sensor =======
#define DHT_PIN 14
#define DHT_TYPE DHT11
DHT dht(DHT_PIN, DHT_TYPE);

// ======= STATE MẪU (14 byte) =======
uint8_t STATE_AUTO24[] = {0x23,0xCB,0x26,0x01,0x00,0x24,0x08,0x08,0x00,0x00,0x00,0x00,0x00,0x49};
uint8_t STATE_COOL24[] = {0x23,0xCB,0x26,0x01,0x00,0x24,0x03,0x07,0x00,0x00,0x00,0x00,0x80,0xC3};
uint8_t STATE_DRY24[]  = {0x23,0xCB,0x26,0x01,0x00,0x24,0x02,0x08,0x00,0x00,0x00,0x00,0x80,0xC3};
uint8_t STATE_HEAT24[] = {0x23,0xCB,0x26,0x01,0x00,0x24,0x01,0x07,0x00,0x00,0x00,0x00,0x00,0x41};
uint8_t STATE_OFF[]    = {0x23,0xCB,0x26,0x01,0x00,0x20,0x01,0x07,0x00,0x00,0x00,0x00,0x00,0x3D};

// ======= Internal state variables =======
bool powerState = false;
String modeState = "cool";
int tempState = 24;
String fanState = "auto";

// ======= Current raw TCL state (14 bytes) =======
uint8_t activeState[kTcl112AcStateLength];

// ======= WiFi connect =======
void connectWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.print("Connecting WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(500);
  }
  Serial.println("\n✔ WiFi connected");
}

// ======= MQTT connect & subscribe =======
void connectMQTT() {
  while (!mqtt.connected()) {
    Serial.print("MQTT connecting...");
    if (mqtt.connect("esp32", TOKEN, "")) {
      Serial.println("✔ MQTT connected");
      mqtt.subscribe("v1/devices/me/attributes");
      // Request attributes (tất cả đều dùng key thường)
      mqtt.publish("v1/devices/me/attributes/request/1",
                   "{\"sharedKeys\":\"power,mode,temp,fan\"}");
      mqtt.subscribe("v1/devices/me/attributes/response/+");
    } else {
      Serial.println("retry...");
      delay(1000);
    }
  }
}

// ======= Checksum helper =======
void updateChecksum(uint8_t *s) {
  s[13] = IRTcl112Ac::calcChecksum(s, kTcl112AcStateLength);
}

// ======= Build state từ biến dashboard =======
void buildStateFromDashboard() {
  if (!powerState) {
    memcpy(activeState, STATE_OFF, kTcl112AcStateLength);
  } else {
    if (modeState == "auto")       memcpy(activeState, STATE_AUTO24, kTcl112AcStateLength);
    else if (modeState == "cool")  memcpy(activeState, STATE_COOL24, kTcl112AcStateLength);
    else if (modeState == "dry")   memcpy(activeState, STATE_DRY24, kTcl112AcStateLength);
    else if (modeState == "heat")  memcpy(activeState, STATE_HEAT24, kTcl112AcStateLength);
    else memcpy(activeState, STATE_COOL24, kTcl112AcStateLength);

    // Temp encoding: 24°C = 0x07, mỗi +1°C => -1 byte
    int offset = 24 - tempState;
    int newTempByte = 0x07 + offset;
    if (newTempByte < 0) newTempByte = 0;
    if (newTempByte > 15) newTempByte = 15;
    activeState[7] = (uint8_t)newTempByte;

    // Fan encoding
    if (fanState == "auto")        activeState[8] = 0;
    else if (fanState == "low")    activeState[8] = 2;
    else if (fanState == "medium") activeState[8] = 3;
    else if (fanState == "high")   activeState[8] = 5;
    else activeState[8] = 0;
  }
  updateChecksum(activeState);
}

// ======= Publish telemetry helper =======
void publishTelemetry(const char* source) {
  DynamicJsonDocument doc(256);
  doc["power"] = powerState;
  doc["mode"] = modeState;
  doc["temp"] = tempState;
  doc["fan"]  = fanState;
  doc["source"] = source;

  String out;
  serializeJson(doc, out);
  bool ok = mqtt.publish("v1/devices/me/telemetry", out.c_str());
  if (ok) {
    Serial.println("📤 Telemetry published (" + String(source) + "): " + out);
  } else {
    Serial.println("❌ Telemetry publish failed");
    Serial.print("MQTT state: "); Serial.println(mqtt.state());
  }
}

// ======= SEND IR (từ attributes) =======
void sendACandPublishTelemetry() {
  buildStateFromDashboard();

  Serial.println("\n=== SEND IR (from ATTRIBUTES) ===");
  Serial.print("Power:"); Serial.print(powerState ? "ON" : "OFF");
  Serial.print(" Mode:"); Serial.print(modeState);
  Serial.print(" Temp:"); Serial.print(tempState);
  Serial.print(" Fan:"); Serial.println(fanState);

  // In ra raw state để debug
  Serial.print("Raw state: ");
  for (int i=0; i<kTcl112AcStateLength; i++) {
    if (activeState[i] < 0x10) Serial.print("0");
    Serial.print(activeState[i], HEX);
    Serial.print(" ");
  }
  Serial.println();

  // Phát IR
  ac.setRaw(activeState);
  ac.send();
  Serial.println("✅ IR signal sent");

  // Publish telemetry ngay
  publishTelemetry("attr");
}

// ======= PARSE attributes từ ThingsBoard =======
void parseAttributes(String json) {
  DynamicJsonDocument doc(512);
  DeserializationError err = deserializeJson(doc, json);
  if (err) {
    Serial.println("❌ JSON parse error in attributes!");
    return;
  }

  bool changed = false;

  // POWER - dùng key thường
  if (doc.containsKey("power")) {
    bool newp = doc["power"];
    if (newp != powerState) {
      powerState = newp;
      changed = true;
      Serial.print("→ Power changed to: "); Serial.println(newp ? "ON" : "OFF");
    }
  }

  // MODE - giữ nguyên key
  if (doc.containsKey("mode")) {
    String newm = doc["mode"].as<String>();
    if (newm != modeState) {
      modeState = newm;
      changed = true;
      Serial.print("→ Mode changed to: "); Serial.println(newm);
    }
  }

  // TEMP - giữ nguyên key
  if (doc.containsKey("temp")) {
    int nt = doc["temp"];
    if (nt != tempState) {
      tempState = nt;
      if (tempState < 16) tempState = 16;
      if (tempState > 31) tempState = 31;
      changed = true;
      Serial.print("→ Temp changed to: "); Serial.println(tempState);
    }
  }

  // FAN - giữ nguyên key
  if (doc.containsKey("fan")) {
    String nf = doc["fan"].as<String>();
    if (nf != fanState) {
      fanState = nf;
      changed = true;
      Serial.print("→ Fan changed to: "); Serial.println(nf);
    }
  }

  if (changed) {
    sendACandPublishTelemetry();
  } else {
    Serial.println("→ No change in attributes");
  }
}

// ======= MQTT callback =======
void mqttCallback(char* topic, byte* payload, unsigned int length) {
  String msg;
  for (unsigned int i=0; i<length; i++) msg += (char)payload[i];

  Serial.println("\n[MQTT] Topic: " + String(topic));
  Serial.println("[MQTT] Payload: " + msg);
  Serial.println("[MQTT] Current state BEFORE parse: power=" + String(powerState) + 
                 ", mode=" + modeState + ", temp=" + String(tempState) + ", fan=" + fanState);

  parseAttributes(msg);
  
  Serial.println("[MQTT] Current state AFTER parse: power=" + String(powerState) + 
                 ", mode=" + modeState + ", temp=" + String(tempState) + ", fan=" + fanState);
}

// ======= IR RECEIVE handler =======
void handleIR() {
  if (!irrecv.decode(&results)) return;

  // Hiển thị timestamp như DumpV2
  uint32_t now = millis();
  Serial.printf("\n🔔 === IR RECEIVED [%06u.%03u] ===\n", now / 1000, now % 1000);
  Serial.print("Protocol: "); Serial.println(typeToString(results.decode_type));
  Serial.print("Bits: "); Serial.println(results.bits);

  // Check buffer overflow
  if (results.overflow) {
    Serial.printf("⚠️ BUFFER OVERFLOW! Buffer size: %d\n", kCaptureBufferSize);
    irrecv.resume();
    return;
  }

  if (results.decode_type == TCL112AC || results.decode_type == MITSUBISHI112) {
    uint8_t *s = results.state;

    Serial.print("Raw state: ");
    for (int i=0; i<kTcl112AcStateLength; i++){
      if (s[i] < 0x10) Serial.print("0");
      Serial.print(s[i], HEX);
      Serial.print(" ");
    }
    Serial.println();

    // Validate checksum
    if (!IRTcl112Ac::validChecksum(s, kTcl112AcStateLength)) {
      Serial.println("⚠️ Invalid checksum - ignoring");
      irrecv.resume();
      return;
    }

    // Validate header
    if (s[0] != 0x23 || s[1] != 0xCB || s[2] != 0x26) {
      Serial.println("⚠️ Not TCL header - ignoring");
      irrecv.resume();
      return;
    }

    // Decode state
    bool newPower = (s[5] == 0x24);
    uint8_t modeByte = s[6];
    uint8_t tempByte = s[7];
    uint8_t fanByte = s[8];

    // Decode temp
    int newTemp = 24 - (tempByte - 0x07);
    if (newTemp < 16) newTemp = 16;
    if (newTemp > 31) newTemp = 31;

    // Decode mode
    String newMode;
    if (modeByte == 8)      newMode = "auto";
    else if (modeByte == 3) newMode = "cool";
    else if (modeByte == 2) newMode = "dry";
    else if (modeByte == 1) newMode = "heat";
    else newMode = "auto";

    // Decode fan
    String newFan;
    if (fanByte == 0)      newFan = "auto";
    else if (fanByte == 2) newFan = "low";
    else if (fanByte == 3) newFan = "medium";
    else if (fanByte == 5) newFan = "high";
    else newFan = "auto";

    // Check for changes
    bool changed = false;
    if (powerState != newPower) { powerState = newPower; changed = true; }
    if (modeState != newMode)   { modeState = newMode; changed = true; }
    if (tempState != newTemp)   { tempState = newTemp; changed = true; }
    if (fanState != newFan)     { fanState = newFan; changed = true; }

    Serial.println("Decoded from remote:");
    Serial.print(" Power:"); Serial.print(powerState ? "ON" : "OFF");
    Serial.print(" Mode:"); Serial.print(modeState);
    Serial.print(" Temp:"); Serial.print(tempState);
    Serial.print(" Fan:"); Serial.println(fanState);

    // Publish telemetry nếu có thay đổi
    if (changed) {
      publishTelemetry("remote");
    } else {
      Serial.println("→ No change from remote");
    }

  } else {
    Serial.print("⚠️ Unsupported protocol: ");
    Serial.println(typeToString(results.decode_type));
  }

  irrecv.resume();
}

// ======= DHT11 read & publish (loop riêng) =======
unsigned long lastSensorRead = 0;
const unsigned long SENSOR_INTERVAL_MS = 15000; // 15s

void readAndPublishSensor() {
  if (millis() - lastSensorRead < SENSOR_INTERVAL_MS) return;
  lastSensorRead = millis();

  float humidity = dht.readHumidity();
  float temperature = dht.readTemperature();

  if (isnan(humidity) || isnan(temperature)) {
    Serial.println("❌ DHT11 read failed");
    return;
  }

  Serial.print("🌡️ DHT11 - Temp: "); Serial.print(temperature);
  Serial.print("°C, Humidity: "); Serial.print(humidity); Serial.println("%");

  // Chỉ publish sensor data
  DynamicJsonDocument doc(128);
  doc["sensor_temp"] = temperature;
  doc["sensor_humidity"] = humidity;

  String out;
  serializeJson(doc, out);
  mqtt.publish("v1/devices/me/telemetry", out.c_str());
}

// ======= SETUP =======
void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n=== TCL MQTT IR Controller (DHT11) ===");

  // Init DHT11
  dht.begin();
  Serial.println("✔ DHT11 initialized");

  // Init IR
  irrecv.enableIRIn();
  Serial.println("✔ IR Receiver enabled on GPIO " + String(IR_RECV_PIN));
  
  ac.begin();
  Serial.println("✔ IR Sender initialized on GPIO " + String(IR_SEND_PIN));

  // Connect
  connectWiFi();
  mqtt.setServer(MQTT_SERVER, MQTT_PORT);
  mqtt.setCallback(mqttCallback);
  mqtt.setBufferSize(512); // Tăng buffer nếu cần
}

// ======= LOOP =======
void loop() {
  if (!mqtt.connected()) connectMQTT();
  mqtt.loop();

  handleIR();              // Thu IR từ remote
  readAndPublishSensor();  // Đọc DHT11 mỗi 15s
}