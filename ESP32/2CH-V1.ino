#include <WiFi.h>
#include <HTTPClient.h>
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
const char* device_id = "esp32_timer_relay_01";

// MQTT Topics
const char* topicPrefix = "iot/timer-relay/";
const char* topic_relay1_control = "iot/timer-relay/relay1/control";
const char* topic_relay2_control = "iot/timer-relay/relay2/control";
const char* topic_status = "iot/timer-relay/status";
const char* topic_command = "iot/timer-relay/command";

// Pin definitions
const int LED_PIN = 2;           // LED สำหรับแสดงสถานะทั้งหมด
const int SWITCH1_PIN = 22;     // สวิตช์ตัวที่ 1
const int SWITCH2_PIN = 23;     // สวิตช์ตัวที่ 2
const int RELAY1_PIN = 16;      // รีเลย์ตัวที่ 1
const int RELAY2_PIN = 19;      // รีเลย์ตัวที่ 2

// Timing configurations
const long EMERGENCY_BLINK = 100;    // กะพริบเร็วมาก (100ms) สำหรับ 30 วินาทีสุดท้าย
const long WARNING_BLINK = 500;      // กะพริบเร็ว (500ms) สำหรับ 3 นาทีสุดท้าย
const long WIFI_FAST_BLINK = 1000;   // กะพริบ 1 วินาที สำหรับไม่มี WiFi
const long WIFI_SLOW_BLINK = 3000;   // กะพริบช้า 3 วินาที สำหรับมี WiFi + Internet

const long CHECK_INTERVAL = 10000;          // ตรวจสอบสถานะทุก 10 วินาที
const long COUNTDOWN_TIME = 40 * 60 * 1000; // 40 นาที
const long WARNING_TIME = 3 * 60 * 1000;    // 3 นาทีสุดท้าย
const long URGENT_TIME = 30 * 1000;         // 30 วินาทีสุดท้าย
const long DEBOUNCE_DELAY = 50;             // ดีเลย์ป้องกันการกระเด้ง
const long LONG_PRESS_TIME = 2500;          // เวลากดค้าง 2.5 วินาที

// Global variables
WiFiClient espClient;
PubSubClient mqtt(espClient);
unsigned long lastMqttReconnectAttempt = 0;
const long MQTT_RECONNECT_INTERVAL = 5000;

// Operating variables
unsigned long previousMillis = 0;
bool ledState = LOW;
int connectionStatus = 0;
unsigned long relay1StartTime = 0;
unsigned long relay2StartTime = 0;
bool relay1Active = false;
bool relay2Active = false;
bool switch1LastState = HIGH;
bool switch2LastState = HIGH;
unsigned long lastDebounceTime1 = 0;
unsigned long lastDebounceTime2 = 0;
unsigned long switch1PressStart = 0;
unsigned long switch2PressStart = 0;
bool switch1LongPress = false;
bool switch2LongPress = false;

void publishStatus() {
    StaticJsonDocument<400> doc;

    // เพิ่ม debug timestamp
    doc["timestamp"] = millis();

    if (WiFi.status() == WL_CONNECTED) {
        doc["wifi_connected"] = true;
        Serial.println("WiFi connected - Publishing status");
    } else {
        doc["wifi_connected"] = false;
        Serial.println("WiFi disconnected - Publishing status");
    }

    doc["wifi_rssi"] = WiFi.RSSI();

    JsonObject relay1 = doc.createNestedObject("relay1");
    relay1["active"] = relay1Active;
    relay1["pin_state"] = digitalRead(RELAY1_PIN);
    if (relay1Active) {
        unsigned long remaining = (relay1StartTime + COUNTDOWN_TIME - millis()) / 1000;
        relay1["remaining"] = remaining;
        relay1["start_time"] = relay1StartTime;
    }

    JsonObject relay2 = doc.createNestedObject("relay2");
    relay2["active"] = relay2Active;
    relay2["pin_state"] = digitalRead(RELAY2_PIN);
    if (relay2Active) {
        unsigned long remaining = (relay2StartTime + COUNTDOWN_TIME - millis()) / 1000;
        relay2["remaining"] = remaining;
        relay2["start_time"] = relay2StartTime;
    }

    char buffer[400];
    serializeJson(doc, buffer);
    
    Serial.print("Publishing status: ");
    Serial.println(buffer);
    
    mqtt.publish(topic_status, buffer, true);
}

void mqttCallback(char* topic, byte* payload, unsigned int length) {
    String message;
    for (int i = 0; i < length; i++) {
        message += (char)payload[i];
    }

    Serial.print("Message arrived [");
    Serial.print(topic);
    Serial.print("] ");
    Serial.println(message);

    StaticJsonDocument<200> doc;
    DeserializationError error = deserializeJson(doc, message);
    if (error) {
        Serial.println("Failed to parse JSON");
        return;
    }

    String topicStr = String(topic);
    
    if (topicStr == topic_command) {
        String command = doc["command"];
        if (command == "status") {
            publishStatus();
        }
    }
    else if (topicStr == topic_relay1_control) {
        String action = doc["action"];
        Serial.printf("Relay 1 command received: %s\n", action.c_str());
        
        if (action == "ON") {
            activateRelay(1, true);
        } else if (action == "OFF") {
            activateRelay(1, false);
        } else if (action == "RESET") {
            resetTimer(1);
        }
    }
    else if (topicStr == topic_relay2_control) {
        String action = doc["action"];
        Serial.printf("Relay 2 command received: %s\n", action.c_str());
        
        if (action == "ON") {
            activateRelay(2, true);
        } else if (action == "OFF") {
            activateRelay(2, false);
        } else if (action == "RESET") {
            resetTimer(2);
        }
    }
}

void activateRelay(int relay, bool state) {
    unsigned long currentTime = millis();
    
    if (relay == 1) {
        if (state && !relay1Active) {
            Serial.println("Activating Relay 1...");
            digitalWrite(RELAY1_PIN, HIGH);  // เปิดรีเลย์ก่อน
            delay(50);  // รอให้รีเลย์เปิดสมบูรณ์
            relay1Active = true;
            relay1StartTime = currentTime;
            Serial.printf("Relay 1 activated at %lu ms\n", currentTime);
        } else if (!state && relay1Active) {
            Serial.println("Deactivating Relay 1...");
            relay1Active = false;
            digitalWrite(RELAY1_PIN, LOW);
            Serial.println("Relay 1 deactivated");
        }
    } else if (relay == 2) {
        if (state && !relay2Active) {
            Serial.println("Activating Relay 2...");
            digitalWrite(RELAY2_PIN, HIGH);  // เปิดรีเลย์ก่อน
            delay(50);  // รอให้รีเลย์เปิดสมบูรณ์
            relay2Active = true;
            relay2StartTime = currentTime;
            Serial.printf("Relay 2 activated at %lu ms\n", currentTime);
        } else if (!state && relay2Active) {
            Serial.println("Deactivating Relay 2...");
            relay2Active = false;
            digitalWrite(RELAY2_PIN, LOW);
            Serial.println("Relay 2 deactivated");
        }
    }
    publishStatus();
}

void handleRelay1(bool switchPressed) {
    if (switchPressed) {
        if (!relay1Active) {
            Serial.println("Switch 1 pressed - Activating relay 1");
            activateRelay(1, true);
        } else if (relay1Active && ((millis() - relay1StartTime) >= (COUNTDOWN_TIME - WARNING_TIME))) {
            Serial.println("Switch 1 pressed - Resetting timer for relay 1");
            resetTimer(1);
        }
    }
}

void handleRelay2(bool switchPressed) {
    if (switchPressed) {
        if (!relay2Active) {
            Serial.println("Switch 2 pressed - Activating relay 2");
            activateRelay(2, true);
        } else if (relay2Active && ((millis() - relay2StartTime) >= (COUNTDOWN_TIME - WARNING_TIME))) {
            Serial.println("Switch 2 pressed - Resetting timer for relay 2");
            resetTimer(2);
        }
    }
}

void resetTimer(int relay) {
    unsigned long currentTime = millis();
    
    if (relay == 1 && relay1Active) {
        Serial.printf("Resetting Relay 1 timer at %lu ms\n", currentTime);
        relay1StartTime = currentTime;
    } else if (relay == 2 && relay2Active) {
        Serial.printf("Resetting Relay 2 timer at %lu ms\n", currentTime);
        relay2StartTime = currentTime;
    }
    publishStatus();
}

bool reconnectMQTT() {
    if (mqtt.connect(device_id, mqtt_user, mqtt_password)) {
        Serial.println("Connected to MQTT Broker");
        mqtt.subscribe(topic_relay1_control);
        mqtt.subscribe(topic_relay2_control);
        mqtt.subscribe(topic_command);
        publishStatus();
        return true;
    }
    return false;
}

void updateLED(unsigned long currentMillis) {
    bool isEmergencyWarning = false;
    bool isWarning = false;

    if (relay1Active) {
        unsigned long remaining1 = COUNTDOWN_TIME - (currentMillis - relay1StartTime);
        if (remaining1 <= URGENT_TIME) {
            isEmergencyWarning = true;
        } else if (remaining1 <= WARNING_TIME) {
            isWarning = true;
        }
    }

    if (relay2Active && !isEmergencyWarning) {
        unsigned long remaining2 = COUNTDOWN_TIME - (currentMillis - relay2StartTime);
        if (remaining2 <= URGENT_TIME) {
            isEmergencyWarning = true;
        } else if (remaining2 <= WARNING_TIME) {
            isWarning = true;
        }
    }

    if (isEmergencyWarning) {
        digitalWrite(LED_PIN, (currentMillis % EMERGENCY_BLINK < EMERGENCY_BLINK / 2) ? HIGH : LOW);
    }
    else if (isWarning) {
        digitalWrite(LED_PIN, (currentMillis % WARNING_BLINK < WARNING_BLINK / 2) ? HIGH : LOW);
    }
    else {
        switch (connectionStatus) {
            case 0:  // No WiFi
                digitalWrite(LED_PIN, (currentMillis % WIFI_FAST_BLINK < WIFI_FAST_BLINK / 2) ? HIGH : LOW);
                break;
            case 1:  // WiFi connected
                digitalWrite(LED_PIN, LOW);
                break;
            case 2:  // WiFi + Internet
                digitalWrite(LED_PIN, (currentMillis % WIFI_SLOW_BLINK < WIFI_SLOW_BLINK / 2) ? HIGH : LOW);
                break;
            case 3:  // WiFi but no Internet
                digitalWrite(LED_PIN, HIGH);
                break;
        }
    }
}

void setup() {
    Serial.begin(115200);
    delay(1000);
    Serial.println("\nStarting Timer Relay Control...");

    pinMode(LED_PIN, OUTPUT);
    pinMode(RELAY1_PIN, OUTPUT);
    pinMode(RELAY2_PIN, OUTPUT);
    pinMode(SWITCH1_PIN, INPUT_PULLUP);
    pinMode(SWITCH2_PIN, INPUT_PULLUP);

    digitalWrite(RELAY1_PIN, LOW);
    digitalWrite(RELAY2_PIN, LOW);
    digitalWrite(LED_PIN, LOW);

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
            Serial.println("Attempting MQTT reconnection...");
            if (reconnectMQTT()) {
                lastMqttReconnectAttempt = 0;
                Serial.println("MQTT reconnected");
            }
        }
    } else {
        mqtt.loop();
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

            if ((currentMillis - switch1PressStart) >= LONG_PRESS_TIME) {
                if (relay1Active) {
                    Serial.println("Long press detected - Deactivating relay 1");
                    activateRelay(1, false);
                    switch1LongPress = true;
                }
            }
        }
        else if (switch1Reading == HIGH) {
            if (!switch1LongPress && switch1PressStart > 0) {
                handleRelay1(true);
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

            if ((currentMillis - switch2PressStart) >= LONG_PRESS_TIME) {
                if (relay2Active) {
                    Serial.println("Long press detected - Deactivating relay 2");
                    activateRelay(2, false);
                    switch2LongPress = true;
                }
            }
        }
        else if (switch2Reading == HIGH) {
            if (!switch2LongPress && switch2PressStart > 0) {
                handleRelay2(true);
            }
            switch2PressStart = 0;
            switch2LongPress = false;
        }
    }
    switch2LastState = switch2Reading;

    // Timer management for Relay 1
    if (relay1Active) {
        unsigned long relay1Elapsed = currentMillis - relay1StartTime;
        if (relay1Elapsed >= COUNTDOWN_TIME) {
            Serial.println("Relay 1 timer completed");
            activateRelay(1, false);
        } else {
            // Debug: print remaining time every 5 seconds
            if (currentMillis % 5000 == 0) {
                unsigned long remaining = (COUNTDOWN_TIME - relay1Elapsed) / 1000;
                Serial.printf("Relay 1 remaining: %lu seconds\n", remaining);
            }
        }
    }

    // Timer management for Relay 2
    if (relay2Active) {
        unsigned long relay2Elapsed = currentMillis - relay2StartTime;
        if (relay2Elapsed >= COUNTDOWN_TIME) {
            Serial.println("Relay 2 timer completed");
            activateRelay(2, false);
        } else {
            // Debug: print remaining time every 5 seconds
            if (currentMillis % 5000 == 0) {
                unsigned long remaining = (COUNTDOWN_TIME - relay2Elapsed) / 1000;
                Serial.printf("Relay 2 remaining: %lu seconds\n", remaining);
            }
        }
    }

    // Update LED status
    updateLED(currentMillis);

    // Periodic status update
    if (currentMillis - previousMillis >= CHECK_INTERVAL) {
        previousMillis = currentMillis;
        if (mqtt.connected()) {
            publishStatus();
        }
    }
}
