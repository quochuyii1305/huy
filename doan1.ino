#include <WiFi.h>
#include <ArduinoJson.h>
#include <PubSubClient.h>
#include <WiFiClientSecure.h>
#include <ESP32Servo.h>

// ======= Cảm biến độ ẩm đất (Sử dụng pins ADC) ========
#define MOISTURE_PIN_0 15    // Chuyển sang pins ADC trên ESP32
#define MOISTURE_PIN_1 17
#define MOISTURE_PIN_2 18    // ESP32 không có GPIO 37, sử dụng 39 thay thế

// ======= Cấu hình cây và tưới =======
#define SERVO_PIN 21
#define PUMP_PIN 16

#define SAMPLES_COUNT 10     // Số lần lấy mẫu cho mỗi cảm biến


#define servoPosition0 0    // góc quay đến cây 1
#define triggerValue0 3400  // ngưỡng cảm biến đất cho cây 1 (0-4095)
#define wateringTime0 20000   // ms tưới cây 1

#define servoPosition1 100
#define triggerValue1 3300
#define wateringTime1 20000

#define servoPosition2 180
#define triggerValue2 3000
#define wateringTime2 20000

Servo positionServo;
int moistureValue0 = 0;
int moistureValue1 = 0;
int moistureValue2 = 0;

// Biến theo dõi trạng thái đang tưới
bool isWatering = false;
int currentWateringZone = -1;  // -1 nghĩa là không tưới khu nào

//======= WiFi & MQTT HiveMQ =========
const char* ssid = "Trang T4";
const char* password = "688699688";

const char* mqtt_server = "d39af0392518478980c1e73505500d1b.s1.eu.hivemq.cloud";
const int mqtt_port = 8883;
const char* mqtt_username = "hivemq.webclient.1745571789362";
const char* mqtt_password = "M4hD@2rH9w3Qu:a#V<lK";

// ======= MQTT Topics =======
const char* TOPIC_DATA_SOIL = "data/soil";             // Topic gửi dữ liệu độ ẩm đất
const char* TOPIC_CONTROL_WATERING = "control/watering"; // Topic nhận lệnh tưới
const char* TOPIC_STATUS_WATERING = "status/watering";   // Topic gửi trạng thái tưới
const char* TOPIC_STOP_WATERING = "control/stop";      // Topic nhận lệnh dừng tưới
const char* TOPIC_CONTROL_REFRESH = "control/refresh"; // Topic nhận lệnh cập nhật dữ liệu

WiFiClientSecure espClient;
PubSubClient client(espClient);

// ======= Kết nối WiFi ==========
void setup_wifi() {
  delay(10);
  Serial.println("\nĐang kết nối WiFi...");
  WiFi.begin(ssid, password);
  
  unsigned long connectionStartTime = millis();
  while (WiFi.status() != WL_CONNECTED) {
    // Timeout sau 30 giây
    if (millis() - connectionStartTime > 30000) {
      Serial.println("Kết nối WiFi thất bại, khởi động lại...");
      ESP.restart();
    }
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nĐã kết nối WiFi - IP: ");
  Serial.println(WiFi.localIP());
}

// ======= Kết nối lại MQTT nếu bị ngắt ==========
void reconnect() {
  int attemptCount = 0;
  while (!client.connected() && attemptCount < 5) {
    Serial.print("Đang kết nối MQTT...");
    String clientID = "ESP32Client-";
    clientID += String(random(0xffff), HEX);
    if (client.connect(clientID.c_str(), mqtt_username, mqtt_password)) {
      Serial.println("đã kết nối");
      
      // Đăng ký nhận các topic điều khiển
      client.subscribe(TOPIC_CONTROL_WATERING);
      Serial.println("Đã đăng ký topic: " + String(TOPIC_CONTROL_WATERING));
      // Đăng ký topic cập nhật
      client.subscribe(TOPIC_CONTROL_REFRESH);
      Serial.println("Đã đăng ký topic: " + String(TOPIC_CONTROL_REFRESH));
      // Đăng ký topic dừng tưới mới
      client.subscribe(TOPIC_STOP_WATERING);
      Serial.println("Đã đăng ký topic: " + String(TOPIC_STOP_WATERING));
    } else {
      Serial.print("thất bại, lỗi=");
      Serial.print(client.state());
      Serial.println(" thử lại sau 5s");
      delay(5000);
      attemptCount++;
    }
  }
}

// Gửi thông báo trạng thái đang tưới
void sendWateringStatus(bool isWatering, int zone, int duration = 0) {
  DynamicJsonDocument doc(256);
  doc["isWatering"] = isWatering;
  doc["zone"] = zone;
  
  if (isWatering) {
    doc["duration"] = duration;
    doc["startTime"] = millis();
  }
  
  char mqtt_msg[256];
  serializeJson(doc, mqtt_msg);
  publishMessage(TOPIC_STATUS_WATERING, mqtt_msg, true);  // retained để client mới kết nối cũng biết trạng thái
}

// ========== Nhận tin nhắn MQTT ==========
void callback(char* topic, byte* payload, unsigned int length) {
  String msg = "";
  for (unsigned int i = 0; i < length; i++) msg += (char)payload[i];
  Serial.println("MQTT nhận [" + String(topic) + "]: " + msg);
  
  // Xử lý lệnh từ MQTT
  if (String(topic) == TOPIC_CONTROL_WATERING) {
    DynamicJsonDocument doc(256);
    DeserializationError error = deserializeJson(doc, msg);
    
    if (!error && doc.containsKey("plant") && doc.containsKey("duration")) {
      int plant = doc["plant"];
      int duration = doc["duration"];
      
      // Nếu đang tưới thì không thực hiện lệnh mới
      if (isWatering) {
        Serial.println("Đang tưới khu khác, không thể thực hiện lệnh");
        return;
      }
      
      Serial.print("Nhận lệnh tưới cho cây ");
      Serial.print(plant);
      Serial.print(" trong ");
      Serial.print(duration);
      Serial.println("ms");
      
      if (plant == 0) {
        simpleWatering(servoPosition0, duration, 0);
      } else if (plant == 1) {
        simpleWatering(servoPosition1, duration, 1);
      } else if (plant == 2) {
        simpleWatering(servoPosition2, duration, 2);
      }
    } else {
      Serial.println("Định dạng lệnh tưới không hợp lệ hoặc thiếu thông tin");
    }
  }
  else if (String(topic) == TOPIC_STOP_WATERING) {
    // Xử lý lệnh dừng tưới
    DynamicJsonDocument doc(256);
    DeserializationError error = deserializeJson(doc, msg);
    
    if (!error && doc.containsKey("zone")) {
      int zone = doc["zone"];
      
      // Chỉ dừng nếu đang tưới đúng khu được yêu cầu
      if (isWatering && currentWateringZone == zone) {
        Serial.print("Dừng tưới khu ");
        Serial.println(zone);
        stopWatering();
      }
    } else {
      // Nếu không chỉ rõ khu, dừng tưới tất cả
      if (isWatering) {
        stopWatering();
      }
    }
  }
  else if (String(topic) == TOPIC_CONTROL_REFRESH) {
    // Xử lý yêu cầu cập nhật dữ liệu
    Serial.println("Nhận lệnh cập nhật dữ liệu độ ẩm");
    
    // Đọc cảm biến độ ẩm ngay lập tức
    moistureValue0 = readMoistureSensor(MOISTURE_PIN_0);
    moistureValue1 = readMoistureSensor(MOISTURE_PIN_1);
    moistureValue2 = readMoistureSensor(MOISTURE_PIN_2);

    // Gửi dữ liệu độ ẩm qua MQTT
    DynamicJsonDocument doc(256);
    doc["moisture0"] = moistureValue0;
    doc["moisture1"] = moistureValue1;
    doc["moisture2"] = moistureValue2;
    doc["isWatering"] = isWatering;
    doc["wateringZone"] = currentWateringZone;
    
    char mqtt_msg[256];
    serializeJson(doc, mqtt_msg);
    publishMessage(TOPIC_DATA_SOIL, mqtt_msg, true);
    
    Serial.println("Đã gửi dữ liệu độ ẩm theo yêu cầu");
  }
}

// Dừng tưới
void stopWatering() {
  digitalWrite(PUMP_PIN, LOW);
  delay(200);
  initPosition();
  
  // Cập nhật trạng thái
  isWatering = false;
  
  // Gửi thông báo đã ngừng tưới
  sendWateringStatus(false, currentWateringZone);
  currentWateringZone = -1;
  
  Serial.println("Đã dừng tưới theo yêu cầu");
}

// ========== Gửi tin nhắn MQTT ==========
void publishMessage(const char* topic, String payload, boolean retained) {
  if (client.connected()) {
    if (client.publish(topic, payload.c_str(), retained)) {
      Serial.println("Đã gửi [" + String(topic) + "]: " + payload);
    } else {
      Serial.println("Gửi thất bại");
    }
  } else {
    Serial.println("Không thể gửi, MQTT chưa kết nối");
  }
}

// ========== Đọc cảm biến độ ẩm đất ==========
int readMoistureSensor(int pin) {
  long sum = 0;
  for (int i = 0; i < SAMPLES_COUNT; i++) {
    sum += analogRead(pin);
    delay(10);
  }
  return sum / SAMPLES_COUNT;
}

// ========== Đưa servo về vị trí ban đầu ==========
void initPosition() {
  positionServo.write(25);  // vị trí trung gian
  delay(500);
}

// ========== Tưới cây ==========
void simpleWatering(int servoPos, int duration, int zone) {
  // Cập nhật trạng thái tưới
  isWatering = true;
  currentWateringZone = zone;
  
  // Gửi thông báo bắt đầu tưới
  sendWateringStatus(true, zone, duration);
  
  Serial.print("Di chuyển servo tới: ");
  Serial.println(servoPos);
  positionServo.write(servoPos);
  delay(1000);

  Serial.print("Bật bơm trong ");
  Serial.print(duration);
  Serial.println(" ms");
  digitalWrite(PUMP_PIN, HIGH);
  
  // Sử dụng biến để đếm thời gian thay vì delay để có thể phản hồi lệnh dừng
  unsigned long startTime = millis();
  while (millis() - startTime < duration && isWatering) {
    // Vẫn xử lý các lệnh MQTT trong lúc tưới
    client.loop();
    delay(100);  // Đợi một chút để không dùng quá nhiều CPU
  }
  
  if (isWatering) {  // Nếu chưa bị dừng bởi lệnh
    digitalWrite(PUMP_PIN, LOW);
    delay(500);
    initPosition();
    
    // Cập nhật trạng thái kết thúc tưới
    isWatering = false;
    sendWateringStatus(false, zone);
    currentWateringZone = -1;
    
    Serial.println("Tưới xong");
  }
}

// ========== Kiểm tra kết nối MQTT ==========
unsigned long lastMqttCheckTime = 0;
void checkMqttConnection() {
  unsigned long currentTime = millis();
  // Kiểm tra kết nối MQTT mỗi 30 giây
  if (currentTime - lastMqttCheckTime >= 30000 || currentTime < lastMqttCheckTime) {
    if (!client.connected()) {
      Serial.println("Kết nối MQTT bị mất, đang kết nối lại...");
      reconnect();
    }
    lastMqttCheckTime = currentTime;
  }
}

// ========== Kiểm tra kết nối WiFi ==========
unsigned long lastWifiCheckTime = 0;
void checkWifiConnection() {
  unsigned long currentTime = millis();
  // Kiểm tra kết nối WiFi mỗi 60 giây
  if (currentTime - lastWifiCheckTime >= 60000 || currentTime < lastWifiCheckTime) {
    if (WiFi.status() != WL_CONNECTED) {
      Serial.println("Kết nối WiFi bị mất, đang kết nối lại...");
      WiFi.disconnect();
      WiFi.begin(ssid, password);
      
      unsigned long reconnectStartTime = millis();
      while (WiFi.status() != WL_CONNECTED) {
        // Timeout sau 30 giây
        if (millis() - reconnectStartTime > 30000) {
          Serial.println("Kết nối WiFi thất bại, khởi động lại...");
          ESP.restart();
        }
        delay(500);
        Serial.print(".");
      }
      Serial.println("\nĐã kết nối lại WiFi!");
    }
    lastWifiCheckTime = currentTime;
  }
}

// ========== Cài đặt ==========
void setup() {
  Serial.begin(115200);
  Serial.println("Bắt đầu khởi động hệ thống tưới tự động...");

  // Khởi tạo servo và bơm
  ESP32PWM::allocateTimer(0);
  positionServo.setPeriodHertz(50);
  positionServo.attach(SERVO_PIN);
  pinMode(PUMP_PIN, OUTPUT);
  digitalWrite(PUMP_PIN, LOW);
  
  // Khởi tạo pins cảm biến độ ẩm là input
  pinMode(MOISTURE_PIN_0, INPUT);
  pinMode(MOISTURE_PIN_1, INPUT);
  pinMode(MOISTURE_PIN_2, INPUT);

  initPosition();
  
  // Kết nối WiFi
  setup_wifi();
  
  // Cấu hình MQTT
  espClient.setInsecure();  // bỏ qua kiểm tra chứng chỉ SSL
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback);
  
  Serial.println("Hệ thống đã sẵn sàng");
}

// ========== Vòng lặp chính ==========
unsigned long lastUpdate = 0;
unsigned long lastDataSend = 0;
void loop() {
  // Kiểm tra kết nối WiFi và MQTT
  checkWifiConnection();
  checkMqttConnection();
  
  // Xử lý callbacks MQTT
  client.loop();

  // Kiểm tra thời gian hiện tại
  unsigned long currentMillis = millis();
  
  // Gửi dữ liệu cập nhật liên tục mỗi 5 giây
  if (currentMillis - lastDataSend >= 5000 || currentMillis < lastDataSend) {
    // Đọc độ ẩm với lấy mẫu nhiều lần để ổn định
    moistureValue0 = readMoistureSensor(MOISTURE_PIN_0);
    moistureValue1 = readMoistureSensor(MOISTURE_PIN_1);
    moistureValue2 = readMoistureSensor(MOISTURE_PIN_2);

    // Gửi lên MQTT
    DynamicJsonDocument doc(256);
    doc["moisture0"] = moistureValue0;
    doc["moisture1"] = moistureValue1;
    doc["moisture2"] = moistureValue2;
    doc["isWatering"] = isWatering;
    doc["wateringZone"] = currentWateringZone;
    
    char mqtt_msg[256];
    serializeJson(doc, mqtt_msg);
    publishMessage(TOPIC_DATA_SOIL, mqtt_msg, true);
    
    lastDataSend = currentMillis;
  }

  // Kiểm tra điều kiện tưới tự động mỗi 10 giây
  if (currentMillis - lastUpdate >= 10000 || currentMillis < lastUpdate) {
    Serial.print("Độ ẩm 0: "); Serial.print(moistureValue0);
    Serial.print(" | Độ ẩm 1: "); Serial.print(moistureValue1);
    Serial.print(" | Độ ẩm 2: "); Serial.println(moistureValue2);

    // Kiểm tra và tưới cây nếu cần và không đang trong tiến trình tưới
    if (!isWatering) {
      bool watered = false; // Biến để theo dõi xem có cây nào được tưới không
      
      if (moistureValue0 > triggerValue0) {
        Serial.println("Khu 1 cần tưới tự động, độ ẩm quá thấp");
        simpleWatering(servoPosition0, wateringTime0, 0);
        watered = true;
      }
      else if (moistureValue1 > triggerValue1) {
        Serial.println("Khu 2 cần tưới tự động, độ ẩm quá thấp");
        simpleWatering(servoPosition1, wateringTime1, 1);
        watered = true;
      }
      else if (moistureValue2 > triggerValue2) {
        Serial.println("Khu 3 cần tưới tự động, độ ẩm quá thấp");
        simpleWatering(servoPosition2, wateringTime2, 2);
        watered = true;
      }
      
      if (!watered) {
        Serial.println("Không cần tưới.");
      }
    }

    lastUpdate = currentMillis;
  }
}