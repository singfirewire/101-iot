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
const char* device_id = "esp32_timer_relay_01";

// MQTT Topics
const char* topicPrefix = "iot/timer-relay/";
const char* topic_status = "iot/timer-relay/status";
const char* topic_command = "iot/timer-relay/command";

// Pin definitions
const int LED_PIN = 2;           // LED สำหรับแสดงสถานะทั้งหมด
const int SWITCH_PINS[] = {22, 23, 25, 26, 27, 32, 33, 34};  // สวิตช์ทั้ง 8 ตัว
const int RELAY_PINS[] = {16, 17, 18, 19, 21, 22, 23, 25};   // รีเลย์ทั้ง 8 ตัว

// Timing configurations
const long EMERGENCY_BLINK = 100;    // กะพริบเร็วมาก (100ms) สำหรับ 30 วินาทีสุดท้าย
const long WARNING_BLINK = 500;      // กะพริบเร็ว (500ms) สำหรับ 3 นาทีสุดท้าย
const long WIFI_FAST_BLINK = 1000;   // กะพริบ 1 วินาที สำหรับไม่มี WiFi
const long WIFI_SLOW_BLINK = 3000;   // กะพริบช้า 3 วินาที สำหรับมี WiFi + Internet

const long CHECK_INTERVAL = 10000;           // ตรวจสอบสถานะทุก 10 วินาที
const long COUNTDOWN_TIME = 40 * 60 * 1000;  // 40 นาที
const long WARNING_TIME = 3 * 60 * 1000;     // 3 นาทีสุดท้าย
const long URGENT_TIME = 30 * 1000;          // 30 วินาทีสุดท้าย
const long DEBOUNCE_DELAY = 50;              // ดีเลย์ป้องกันการกระเด้ง
const long LONG_PRESS_TIME = 2500;           // เวลากดค้าง 2.5 วินาที
const long RELAY_LOCK_TIME = 1000;           // เวลาล็อค 1 วินาที

// Global variables
WiFiClient espClient;
PubSubClient mqtt(espClient);
unsigned long lastMqttReconnectAttempt = 0;
const long MQTT_RECONNECT_INTERVAL = 5000;

// Operating variables
unsigned long previousMillis = 0;
bool ledState = LOW;
int connectionStatus = 0;

struct RelayState {
    bool active;
    unsigned long startTime;
    bool locked;
    unsigned long lockStartTime;
    bool switchLastState;
    unsigned long lastDebounceTime;
    unsigned long switchPressStart;
    bool switchLongPress;
};

RelayState relays[8];

// Function declarations
void publishStatus();
bool isRelayLocked(int relay);
void activateRelay(int relay, bool state);
void handleRelay(int relay, bool switchPressed);
void resetTimer(int relay);
void mqttCallback(char* topic, byte* payload, unsigned int length);
void updateLED(unsigned long currentMillis);
bool reconnectMQTT();

void publishStatus() {
    StaticJsonDocument<800> doc;

    // ข้อมูลการเชื่อมต่อ
    doc["timestamp"] = millis();
    doc["wifi_connected"] = (WiFi.status() == WL_CONNECTED);
    doc["wifi_rssi"] = WiFi.RSSI();

    // ข้อมูลรีเลย์ทั้ง 8 ตัว
    for (int i = 0; i < 8; i++) {
        JsonObject relay = doc.createNestedObject("relay" + String(i + 1));
        relay["active"] = relays[i].active;
        relay["pin_state"] = digitalRead(RELAY_PINS[i]);
        relay["locked"] = relays[i].locked;
        if (relays[i].active) {
            unsigned long remaining = (relays[i].startTime + COUNTDOWN_TIME - millis()) / 1000;
            relay["remaining"] = remaining;
        }
    }

    char buffer[800];
    serializeJson(doc, buffer);
    mqtt.publish(topic_status, buffer, true);
}

bool isRelayLocked(int relay) {
    if (relays[relay].locked) {
        if (millis() - relays[relay].lockStartTime < RELAY_LOCK_TIME) {
            return true;
        } else {
            relays[relay].locked = false;
        }
    }
    return false;
}

void activateRelay(int relay, bool state) {
    unsigned long currentTime = millis();
    
    if (state && !relays[relay].active) {
        Serial.printf("Activating Relay %d...\n", relay + 1);
        digitalWrite(RELAY_PINS[relay], HIGH);
        delay(50);
        relays[relay].active = true;
        relays[relay].startTime = currentTime;
        relays[relay].locked = true;
        relays[relay].lockStartTime = currentTime;
        Serial.printf("Relay %d locked for 1 second\n", relay + 1);
    } else if (!state && relays[relay].active) {
        if (!isRelayLocked(relay)) {
            Serial.printf("Deactivating Relay %d...\n", relay + 1);
            relays[relay].active = false;
            digitalWrite(RELAY_PINS[relay], LOW);
        } else {
            Serial.printf("Cannot deactivate Relay %d - Lock active\n", relay + 1);
        }
    }
    publishStatus();
}

void handleRelay(int relay, bool switchPressed) {
    if (switchPressed) {
        if (!relays[relay].active) {
            Serial.printf("Switch %d pressed - Activating relay %d\n", relay + 1, relay + 1);
            activateRelay(relay, true);
        } else if (relays[relay].active && ((millis() - relays[relay].startTime) >= (COUNTDOWN_TIME - WARNING_TIME))) {
            Serial.printf("Switch %d pressed - Resetting timer for relay %d\n", relay + 1, relay + 1);
            resetTimer(relay);
        }
    }
}

void resetTimer(int relay) {
    if (isRelayLocked(relay)) {
        Serial.printf("Cannot reset timer for Relay %d - Lock active\n", relay + 1);
        return;
    }

    unsigned long currentTime = millis();
    
    if (relays[relay].active) {
        Serial.printf("Resetting Relay %d timer at %lu ms\n", relay + 1, currentTime);
        relays[relay].startTime = currentTime;
    }
    publishStatus();
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
    else {
        int relay = topicStr.substring(topicStr.lastIndexOf('/') - 1, topicStr.lastIndexOf('/')).toInt() - 1;
        if (relay >= 0 && relay < 8) {
            String action = doc["action"];
            Serial.printf("Relay %d command received: %s\n", relay + 1, action.c_str());
            
            if (action == "ON") {
                activateRelay(relay, true);
            } else if (action == "OFF") {
                if (!isRelayLocked(relay)) {
                    activateRelay(relay, false);
                } else {
                    Serial.printf("Cannot turn off Relay %d - Lock active\n", relay + 1);
                }
            } else if (action == "RESET") {
                if (!isRelayLocked(relay)) {
                    resetTimer(relay);
                }
            }
        }
    }
}

void updateLED(unsigned long currentMillis) {
    bool isEmergencyWarning = false;
    bool isWarning = false;

    for (int i = 0; i < 8; i++) {
        if (relays[i].active) {
            unsigned long remaining = COUNTDOWN_TIME - (currentMillis - relays[i].startTime);
            if (remaining <= URGENT_TIME) {
                isEmergencyWarning = true;
            } else if (remaining <= WARNING_TIME) {
                isWarning = true;
            }
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

bool reconnectMQTT() {
    if (mqtt.connect(device_id, mqtt_user, mqtt_password)) {
        Serial.println("Connected to MQTT Broker");
        for (int i = 0; i < 8; i++) {
            mqtt.subscribe((topicPrefix + "relay" + String(i + 1) + "/control").c_str());
        }
        mqtt.subscribe(topic_command);
        publishStatus();
        return true;
    }
    return false;
}

void setup() {
    Serial.begin(115200);
    delay(1000);
    Serial.println("\nStarting Timer Relay Control...");

    pinMode(LED_PIN, OUTPUT);
    for (int i = 0; i < 8; i++) {
        pinMode(RELAY_PINS[i], OUTPUT);
        pinMode(SWITCH_PINS[i], INPUT_PULLUP);
        digitalWrite(RELAY_PINS[i], LOW);
        relays[i] = {false, 0, false, 0, HIGH, 0, 0, false};
    }
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

    // Switch handling
    for (int i = 0; i < 8; i++) {
        bool switchReading = digitalRead(SWITCH_PINS[i]);
        if (switchReading != relays[i].switchLastState) {
            relays[i].lastDebounceTime = currentMillis;
        }

        if ((currentMillis - relays[i].lastDebounceTime) > DEBOUNCE_DELAY) {
            if (switchReading == LOW && !relays[i].switchLongPress) {
                if (relays[i].switchPressStart == 0) {
                    relays[i].switchPressStart = currentMillis;
                }

                if ((currentMillis - relays[i].switchPressStart) >= LONG_PRESS_TIME) {
                    if (relays[i].active) {
                        Serial.printf("Long press detected - Deactivating relay %d\n", i + 1);
                        activateRelay(i, false);
                        relays[i].switchLongPress = true;
                    }
                }
            }
            else if (switchReading == HIGH) {
                if (!relays[i].switchLongPress && relays[i].switchPressStart > 0) {
                    handleRelay(i, true);
                }
                relays[i].switchPressStart = 0;
                relays[i].switchLongPress = false;
            }
        }
        relays[i].switchLastState = switchReading;
    }

    // Timer management
    for (int i = 0; i < 8; i++) {
        if (relays[i].active) {
            unsigned long elapsed = currentMillis - relays[i].startTime;
            if (elapsed >= COUNTDOWN_TIME) {
                Serial.printf("Relay %d timer completed\n", i + 1);
                activateRelay(i, false);
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
