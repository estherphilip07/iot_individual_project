#include <WiFi.h>
#include <DHT.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>

// Configuration
#define LED_R_PIN 16    // GPIO pin for Red LED
#define LED_G_PIN 17    // GPIO pin for Green LED
#define LED_B_PIN 5     // GPIO pin for Blue LED
#define DHT_PIN 4       // DHT sensor GPIO pin
#define DHTTYPE DHT11   // DHT sensor type

// Temperature threshold definitions
const float ALARM_COLD = 0.0;
const float WARN_COLD = 10.0;
const float WARN_HOT = 25.0;
const float ALARM_HOT = 30.0;

// WiFi Credentials
const char* ssid = "Esther";
const char* pass = "estherrr";

// MQTT Configuration
const char* MQTT_HOST = "broker.hivemq.com";
const int MQTT_PORT = 1883;
const char* MQTT_DEVICEID = "esp32-2793FC";
const char* MQTT_TOPIC_TEMP = "temp";
const char* MQTT_TOPIC_HUMIDITY = "humidity";
const char* MQTT_TOPIC_CONTROL = "esp32/led_control";
const char* MQTT_TOPIC_SET_INTERVAL = "esp32/set_interval";

DHT dht(DHT_PIN, DHTTYPE);

WiFiClient wifiClient;
PubSubClient mqtt(wifiClient);

unsigned long lastPublishTime = 0;
unsigned long publishInterval = 10000; // default 10 seconds
bool manualMode = false;
unsigned long manualModeStartTime;
const unsigned long manualModeDuration = 5000;

void setup() {
    Serial.begin(115200);
    setupWiFi();
    setupDHTSensor();
    setupLEDs();
    setupMQTT();
}

void setupWiFi() {
    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, pass);
    Serial.print("Connecting to WiFi");
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println("\nWiFi connected");
}

void setupDHTSensor() {
    dht.begin();
}

void setupLEDs() {
    pinMode(LED_R_PIN, OUTPUT);
    pinMode(LED_G_PIN, OUTPUT);
    pinMode(LED_B_PIN, OUTPUT);
    setColor(255, 255, 255); // LED off
}

void setupMQTT() {
    mqtt.setServer(MQTT_HOST, MQTT_PORT);
    mqtt.setCallback(mqttCallback);
    connectToMQTT();
}

void connectToMQTT() {
    while (!mqtt.connected()) {
        Serial.print("Attempting MQTT connection...");
        if (mqtt.connect(MQTT_DEVICEID)) {
            Serial.println("connected");
            mqtt.subscribe(MQTT_TOPIC_CONTROL);
            mqtt.subscribe(MQTT_TOPIC_SET_INTERVAL);
        } else {
            Serial.print("failed, rc=");
            Serial.print(mqtt.state());
            Serial.println(" try again in 5 seconds");
            delay(5000);
        }
    }
}

void mqttCallback(char* topic, byte* payload, unsigned int length) {
    String command;
    for (int i = 0; i < length; i++) {
        command += (char)payload[i];
    }

    if (String(topic) == MQTT_TOPIC_SET_INTERVAL) {
        setPublishInterval(command);
    } else if (String(topic) == MQTT_TOPIC_CONTROL) {
        setLEDColor(command);
    }
}

void setPublishInterval(String command) {
    DynamicJsonDocument doc(200);
    DeserializationError error = deserializeJson(doc, command);
    if (!error && doc.containsKey("Interval")) {
        publishInterval = doc["Interval"].as<unsigned long>() * 1000;
        Serial.print("New interval: ");
        Serial.print(publishInterval / 1000);
        Serial.println(" seconds");
    } else {
        Serial.println("Failed to parse interval command");
    }
}

void setLEDColor(String command) {
    manualMode = true;
    manualModeStartTime = millis();
    
    if (command == "red") setColor(0, 255, 255);
    else if (command == "green") setColor(255, 0, 255);
    else if (command == "blue") setColor(255, 255, 0);
    else if (command == "yellow") setColor(0, 0, 255);
    else if (command == "off") setColor(255, 255, 255); // LED off
}

void setColor(uint8_t r, uint8_t g, uint8_t b) {
    digitalWrite(LED_R_PIN, r ? LOW : HIGH);
    digitalWrite(LED_G_PIN, g ? LOW : HIGH);
    digitalWrite(LED_B_PIN, b ? LOW : HIGH);
}

void loop() {
    if (!WiFi.isConnected()) setupWiFi();
    if (!mqtt.connected()) connectToMQTT();

    mqtt.loop();
    handleSensorData();
    handleManualModeTimeout();
}

void handleSensorData() {
    if (!manualMode && millis() - lastPublishTime >= publishInterval) {
        lastPublishTime = millis();
        
        float humidity = dht.readHumidity();
        float temperature = dht.readTemperature();
        
        if (isnan(humidity) || isnan(temperature)) {
            Serial.println("Failed to read from DHT sensor!");
            return;
        }

        if (!mqtt.publish(MQTT_TOPIC_TEMP, String(temperature).c_str())) {
            Serial.println("Temperature publish failed");
        }
        if (!mqtt.publish(MQTT_TOPIC_HUMIDITY, String(humidity).c_str())) {
            Serial.println("Humidity publish failed");
        }

        setAutoLEDColor(temperature);
    }
}

void setAutoLEDColor(float temperature) {
    if (temperature < ALARM_COLD) {
        setColor(254, 254, 0); // Cold Alert
    } else if (temperature < WARN_COLD) {
        setColor(254, 0, 0);   // Cool
    } else if (temperature <= WARN_HOT) {
        setColor(254, 0, 254); // Normal
    } else if (temperature < ALARM_HOT) {
        setColor(0, 0, 254);   // Warm
    } else {
        setColor(0, 254, 254); // Hot Alert
    }
}

void handleManualModeTimeout() {
    if (manualMode && millis() - manualModeStartTime > manualModeDuration) {
        manualMode = false;
    }
}
