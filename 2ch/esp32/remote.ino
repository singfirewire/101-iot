// โค้ดสำหรับบอร์ดรีโมท (Remote Board)
#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>

// WiFi credentials
const char* ssid = "101-IOT";         
const char* password = "10101010"; 

// MQTT Configuration
const char* mqtt_server = "test.mosquitto.org";
const int mqtt_port = 1883;
const char* mqtt_user = "";
const char* mqtt_password = "";
const char* device_id = "esp32_timer_relay_remote";

// MQTT Topics
const char* topicPrefix = "iot/timer-relay/";
const char* topic_remote = "iot/timer-relay/remote";
const char* topic_status = "iot/timer-relay/status";

// Pin definitions
const int LED_PIN = 2;           // LED สำหรับแสดงสถานะ WiFi/MQTT
const int SWITCH1_PIN = 22;      // สวิตช์ตัวที่ 1
const int SWITCH2_PIN = 23;      // สวิตช์ตัวที่ 2
const int LED1_PIN = 16;         // LED แสดงสถานะรีเลย์ 1
const int LED2_PIN = 19;         // LED แสดงสถานะรีเลย์ 2

// Timing configurations
const long EMERGENCY_BLINK = 100;      // กะพริบเร็วมาก (100ms) สำหรับ 30 วินาทีสุดท้าย
const long WARNING_BLINK = 500;        // กะพริบเร็ว (500ms) สำหรับ 3 นาทีสุดท้าย
const long WIFI_FAST_BLINK = 1000;     // กะพริบ 1 วินาที สำหรับไม่มี WiFi
const long WIFI_SLOW_BLINK = 3000;     // กะพริบช้า 3 วินาที สำหรับมี WiFi
const long WARNING_TIME = 3 * 60 * 1000;    // 3 นาทีสุดท้าย
const long URGENT_TIME = 30 * 1000;         // 30 วินาทีสุดท้าย
const long DEBOUNCE_DELAY = 50;
const long LONG_PRESS_TIME = 2500;

// Global variables
WiFiClient espClient;
PubSubClient mqtt(espClient);
unsigned long lastMqttReconnectAttempt = 0;
const long MQTT_RECONNECT_INTERVAL = 5000;

// Operating variables
unsigned long previousMillis = 0;
int connectionStatus = 0;
bool switch1LastState = HIGH;
bool switch2LastState = HIGH;
unsigned long lastDebounceTime1 = 0;
unsigned long lastDebounceTime2 = 0;
unsigned long switch1PressStart = 0;
unsigned long switch2PressStart = 0;
bool switch1LongPress = false;
bool switch2LongPress = false;

// สถานะรีเลย์และเวลา
bool relay1Status = false;
bool relay2Status = false;
unsigned long relay1StartTime = 0;
unsigned long relay2StartTime = 0;
unsigned long relay1Remaining = 0;
unsigned long relay2Remaining = 0;

void updateStatusLED(unsigned long currentMillis, int ledPin, unsigned long remaining) {
    if (remaining <= URGENT_TIME) {
        // 30 วินาทีสุดท้าย - กะพริบเร็วมาก
        digitalWrite(ledPin, (currentMillis % EMERGENCY_BLINK < EMERGENCY_BLINK / 2) ? HIGH : LOW);
    }
    else if (remaining <= WARNING_TIME) {
        // 3 นาทีสุดท้าย - กะพริบเร็ว
        digitalWrite(ledPin, (currentMillis % WARNING_BLINK < WARNING_BLINK / 2) ? HIGH : LOW);
    }
    else {
        // สถานะปกติ - ไฟติดค้าง
        digitalWrite(ledPin, HIGH);
    }
}

void mqttCallback(char* topic, byte* payload, unsigned int length) {
    String message;
    for (int i = 0; i < length; i++) {
        message += (char)payload[i];
    }

    if (String(topic) == topic_status) {
        StaticJsonDocument<400> doc;
        DeserializationError error = deserializeJson(doc, message);
        if (error) {
            Serial.println("Failed to parse JSON");
            return;
        }

        // อัพเดทสถานะรีเลย์ 1
        if (doc["relay1"]["active"]) {
            relay1Status = true;
            if (doc["relay1"]["remaining"]) {
                relay1Remaining = doc["relay1"]["remaining"].as<unsigned long>() * 1000; // แปลงวินาทีเป็นมิลลิวินาที
            }
        } else {
            relay1Status = false;
            digitalWrite(LED1_PIN, LOW);
        }

        // อัพเดทสถานะรีเลย์ 2
        if (doc["relay2"]["active"]) {
            relay2Status = true;
            if (doc["relay2"]["remaining"]) {
                relay2Remaining = doc["relay2"]["remaining"].as<unsigned long>() * 1000;
            }
        } else {
            relay2Status = false;
            digitalWrite(LED2_PIN, LOW);
        }
    }
}

void sendCommand(int relayNumber, String action) {
    StaticJsonDocument<200> doc;
    doc["command"] = "TOGGLE";
    doc["relay"] = relayNumber;
    doc["action"] = action;

    char buffer[256];
    serializeJson(doc, buffer);
    mqtt.publish(topic_remote, buffer);
    Serial.printf("Sent command: %s for relay %d\n", action.c_str(), relayNumber);
}

bool reconnectMQTT() {
    if (mqtt.connect(device_id, mqtt_user, mqtt_password)) {
        Serial.println("Connected to MQTT Broker");
        mqtt.subscribe(topic_status);
        return true;
    }
    return false;
}

void handleRelay1(bool isLongPress) {
    if (isLongPress) {
        if (relay1Status) {
            sendCommand(1, "OFF");
        }
    } else {
        if (!relay1Status) {
            sendCommand(1, "ON");
        } else if (relay1Remaining <= WARNING_TIME) {
            // ถ้ากดสั้นในช่วง 3 นาทีสุดท้าย จะเป็นการ Reset Timer
            sendCommand(1, "RESET");
        }
    }
}

void handleRelay2(bool isLongPress) {
    if (isLongPress) {
        if (relay2Status) {
            sendCommand(2, "OFF");
        }
    } else {
        if (!relay2Status) {
            sendCommand(2, "ON");
        } else if (relay2Remaining <= WARNING_TIME) {
            // ถ้ากดสั้นในช่วง 3 นาทีสุดท้าย จะเป็นการ Reset Timer
            sendCommand(2, "RESET");
        }
    }
}

void updateConnectionLED(unsigned long currentMillis) {
    if (WiFi.status() != WL_CONNECTED) {
        digitalWrite(LED_PIN, (currentMillis % WIFI_FAST_BLINK < WIFI_FAST_BLINK / 2) ? HIGH : LOW);
    } else if (!mqtt.connected()) {
        digitalWrite(LED_PIN, HIGH);
    } else {
        digitalWrite(LED_PIN, (currentMillis % WIFI_SLOW_BLINK < WIFI_SLOW_BLINK / 2) ? HIGH : LOW);
    }
}

void setup() {
    Serial.begin(115200);
    delay(1000);
    Serial.println("\nStarting Remote Control...");

    pinMode(LED_PIN, OUTPUT);
    pinMode(LED1_PIN, OUTPUT);
    pinMode(LED2_PIN, OUTPUT);
    pinMode(SWITCH1_PIN, INPUT_PULLUP);
    pinMode(SWITCH2_PIN, INPUT_PULLUP);

    digitalWrite(LED_PIN, LOW);
    digitalWrite(LED1_PIN, LOW);
    digitalWrite(LED2_PIN, LOW);

    WiFi.begin(ssid, password);
    Serial.print("Connecting to WiFi");
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println("\nWiFi connected");
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());

    mqtt.setServer(mqtt_server, mqtt_port);
    mqtt.setCallback(mqttCallback);
    
    Serial.println("Setup complete.");
}

void loop() {
    unsigned long currentMillis = millis();

    // MQTT Connection Management
    if (!mqtt.connected()) {
        if (currentMillis - lastMqttReconnectAttempt >= MQTT_RECONNECT_INTERVAL) {
            lastMqttReconnectAttempt = currentMillis;
            if (reconnectMQTT()) {
                lastMqttReconnectAttempt = 0;
            }
        }
    } else {
        mqtt.loop();
    }

    // อัพเดทการแสดงผล LED ตามสถานะ
    if (relay1Status) {
        updateStatusLED(currentMillis, LED1_PIN, relay1Remaining);
    }
    if (relay2Status) {
        updateStatusLED(currentMillis, LED2_PIN, relay2Remaining);
    }

    // Switch 1 handling
    bool switch1Reading = digitalRead(SWITCH1_PIN);
    if (switch1Reading != switch1LastState) {
        lastDebounceTime1 = currentMillis;
    }

    if ((currentMillis - lastDebounceTime1) > DEBOUNCE_DELAY) {
        if (switch1Reading == LOW && !switch1LongPress) {
            if (switch1PressStart == 0) {
                switch1PressStart = currentMillis;
            }
            else if ((currentMillis - switch1PressStart) >= LONG_PRESS_TIME) {
                handleRelay1(true);
                switch1LongPress = true;
            }
        }
        else if (switch1Reading == HIGH) {
            if (!switch1LongPress && switch1PressStart > 0) {
                handleRelay1(false);
            }
            switch1PressStart = 0;
            switch1LongPress = false;
        }
    }
    switch1LastState = switch1Reading;

    // Switch 2 handling
    bool switch2Reading = digitalRead(SWITCH2_PIN);
    if (switch2Reading != switch2LastState) {
        lastDebounceTime2 = currentMillis;
    }

    if ((currentMillis - lastDebounceTime2) > DEBOUNCE_DELAY) {
        if (switch2Reading == LOW && !switch2LongPress) {
            if (switch2PressStart == 0) {
                switch2PressStart = currentMillis;
            }
            else if ((currentMillis - switch2PressStart) >= LONG_PRESS_TIME) {
                handleRelay2(true);
                switch2LongPress = true;
            }
        }
        else if (switch2Reading == HIGH) {
            if (!switch2LongPress && switch2PressStart > 0) {
                handleRelay2(false);
            }
            switch2PressStart = 0;
            switch2LongPress = false;
        }
    }
    switch2LastState = switch2Reading;

    // Update WiFi/MQTT status LED
    updateConnectionLED(currentMillis);
}
