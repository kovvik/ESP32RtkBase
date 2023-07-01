#include <FS.h>
#include <WiFiClientSecure.h>
#include <WiFiManager.h>
#include <ArduinoJson.h>
#include "SPIFFS.h"
#include <PubSubClient.h>
#include "time.h"
#include "settings.h"

// Client ID for ESP Chip identification
#define CLIENT_ID "ESP_%06X"

// Chip ID
uint32_t chipId = 0;

// Device name
char host[50];

// MQTT Client
WiFiClientSecure espClient;
PubSubClient mqttClient(espClient);
WiFiManager wifiManager;

// MQTT reconnect
void reconnect() {
    Serial.println(F("Trying to connect mqtt..."));
    if (mqttClient.connect(host)) {
        Serial.println(F("Connected to MQTT broker"));
        mqttClient.subscribe(mqtt_command_topic);
        Serial.print(F("Subscribe to topic: "));
        Serial.println(mqtt_command_topic);
        logToMQTT("HELLO");
    }
}

// sends log to MQTT log topic
void logToMQTT(char message[256]) {
    StaticJsonDocument<256> logdata;
    logdata["time"] = getTime();
    logdata["host"] = host;
    logdata["message"] = message;
    char out[128];
    int b = serializeJson(logdata, out);
    Serial.print(F("sending bytes to MQTT log:"));
    Serial.println(b, DEC);
    mqttClient.publish(mqtt_log_topic, out);
}

// sends status information to MQTT log topic
void logStatus() {
    // Log data
    char wifiRSSI[5];
    char message[20] = "WiFi RSSI:";
    sprintf(wifiRSSI, "%d", WiFi.RSSI());
    strcat(message, wifiRSSI);
    logToMQTT(message);
}

void mqttCallback(char* topic, byte* payload, unsigned int length) {
    char message[length + 1];
    memcpy(message, payload, length);
    message[length] = '\0';
    Serial.print("Message arrived on command topic: ");
    Serial.println(message);
    if ( strcmp(message, "PING") == 0 ) {
        logToMQTT("PONG");
    } else if ( strcmp(message, "REBOOT") == 0 ) {
        logToMQTT("Rebooting");
        delay(1000);
        ESP.restart();
    } else if ( strcmp(message, "RESET") == 0 ) {
        logToMQTT("Reset settings");
        wifiManager.resetSettings();
        WiFi.disconnect(true);
        SPIFFS.format();
        delay(2000);
        ESP.restart();
    } else {
        Serial.println(F("Unknown command"));
    }
}

// Save config to file
bool save_config = true;

void getESPInfo() {
    for(int i=0; i<17; i=i+8) {
        chipId |= ((ESP.getEfuseMac() >> (40 - i)) & 0xff) << i;
    }
    Serial.println(chipId);
    sprintf(host, CLIENT_ID, chipId);
    Serial.println(host);
}

// Save config callback
void saveConfigToFile() {
    Serial.println("should save config");
    save_config = true;
}

unsigned long getTime() {
  time_t now;
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo)) {
    return(0);
  }
  time(&now);
  return now;
}

void setup() {
    // Starting serial
    Serial.begin(115200);

    // Waiting serial to be ready
    while(!Serial) { };

    // Print ESP info
    getESPInfo();

    //clean FS, for testing
    //SPIFFS.format();

    //read configuration from FS json
    Serial.println(F("mounting FS..."));
    if (SPIFFS.begin()) {
        Serial.println(F("mounted file system"));
        if (SPIFFS.exists("/config.json")) {
            //file exists, reading and loading
            Serial.println("reading config file");
            File configFile = SPIFFS.open("/config.json", "r");
            if (configFile) {
                Serial.println(F("opened config file"));
                size_t size = configFile.size();
                // Allocate a buffer to store contents of the file.
                std::unique_ptr<char[]> buf(new char[size]);
                configFile.readBytes(buf.get(), size);
                DynamicJsonDocument configJson(512);
                DeserializationError error = deserializeJson(configJson, configFile);
                if (!error) {
                    Serial.println(F("The parsed json:"));
                    strlcpy(mqtt_server, configJson["mqtt_server"], sizeof(mqtt_server));
                    strlcpy(mqtt_port, configJson["mqtt_port"], sizeof(mqtt_port));
                    strlcpy(mqtt_command_topic, configJson["mqtt_command_topic"], sizeof(mqtt_command_topic));
                    strlcpy(mqtt_ppp_topic, configJson["mqtt_ppp_topic"], sizeof(mqtt_ppp_topic));
                    strlcpy(mqtt_log_topic, configJson["mqtt_log_topic"], sizeof(mqtt_log_topic));
                    Serial.print(F("mqtt_server: "));
                    Serial.println(mqtt_server);
                    Serial.print(F("mqtt_port: "));
                    Serial.println(mqtt_port);
                    Serial.print(F("mqtt_command_topic: "));
                    Serial.println(mqtt_command_topic);
                    Serial.print(F("mqtt_ppp_topic: "));
                    Serial.println(mqtt_ppp_topic);
                    Serial.print(F("mqtt_log_topic: "));
                    Serial.println(mqtt_log_topic);
                } else {
                    Serial.println("Json parse error");
                }
            }
        }
    } else {
        Serial.println(F("Failed to mount FS, trying to format it..."));
        SPIFFS.begin(true);
        delay(3000);
        ESP.restart();
        delay(5000);
    }

    // Additional MQTT parameters to WiFiManager
    WiFiManagerParameter custom_mqtt_server("Server", "mqtt_server", mqtt_server, sizeof(mqtt_server));
    WiFiManagerParameter custom_mqtt_port("Port", "mqtt_port", mqtt_port, sizeof(mqtt_port));
    WiFiManagerParameter custom_mqtt_command_topic("mqtt_command_topic", "mqtt_command_topic", mqtt_command_topic, sizeof(mqtt_command_topic));
    WiFiManagerParameter custom_mqtt_ppp_topic("mqtt_ppp_topic", "mqtt_ppp_topic", mqtt_ppp_topic, sizeof(mqtt_ppp_topic));
    WiFiManagerParameter custom_mqtt_log_topic("mqtt_log_topic", "mqtt_log_topic", mqtt_log_topic, sizeof(mqtt_log_topic));

    WiFiManager wifiManager;
    wifiManager.setSaveConfigCallback(saveConfigToFile);
    wifiManager.setConfigPortalTimeout(240);

    wifiManager.addParameter(&custom_mqtt_server);
    wifiManager.addParameter(&custom_mqtt_port);
    wifiManager.addParameter(&custom_mqtt_command_topic);
    wifiManager.addParameter(&custom_mqtt_ppp_topic);
    wifiManager.addParameter(&custom_mqtt_log_topic);

    // Set WIFI connection timeout to avoid endless loop
    wifiManager.setConnectTimeout(60);

    // Set WIFI country
    wifiManager.setCountry("US");

    if (!wifiManager.autoConnect("ESP_TEMP", "changeit")) {
        Serial.println(F("Failed to connect, reset and try again ..."));
        delay(3000);
        ESP.restart();
        delay(5000);
    }

    Serial.println(F("Connected to WiFi"));
    Serial.print(F("My IP is: "));
    Serial.println(WiFi.localIP());

    strcpy(mqtt_server, custom_mqtt_server.getValue());
    strcpy(mqtt_port, custom_mqtt_port.getValue());
    strcpy(mqtt_command_topic, custom_mqtt_command_topic.getValue());
    strcpy(mqtt_ppp_topic, custom_mqtt_ppp_topic.getValue());
    strcpy(mqtt_log_topic, custom_mqtt_log_topic.getValue());

    if (save_config) {
        Serial.println("Saving config ...");
        DynamicJsonDocument jsonConfig(512);
        jsonConfig["mqtt_server"] = mqtt_server;
        jsonConfig["mqtt_port"] = mqtt_port;
        jsonConfig["mqtt_command_topic"] = mqtt_command_topic;
        jsonConfig["mqtt_ppp_topic"] = mqtt_ppp_topic;
        jsonConfig["mqtt_log_topic"] = mqtt_log_topic;

        File configFile = SPIFFS.open("/config.json", "w");
        if (configFile) {
            if (serializeJson(jsonConfig, configFile) == 0) {
                Serial.println(F("Failed to write config file!"));
            } else {
                Serial.println(F("Config saved."));
            }
        } else {
            Serial.println(F("Failed to open config file for writing!"));
        }
        configFile.close();
    }

    // Setup NTP
    configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);

    // Setup mqtt
    espClient.setCACert(mqtt_ca);
    espClient.setCertificate(mqtt_cert);
    espClient.setPrivateKey(mqtt_key);
    mqttClient.setServer(mqtt_server, atoi(mqtt_port));
    mqttClient.setCallback(mqttCallback);
    reconnect();
}

void loop() {
    // check Wifi connection every minute
    static uint32_t lastMillis = 0;
    if (millis() - lastMillis > 60000) {
        // Reconnect to mqtt server
        if (!mqttClient.connected()) {
            reconnect();
        }
        // Check wifi connection and reconnect if connection lost
        if (WiFi.status() != WL_CONNECTED) {
            Serial.println(F("Lost connection, reconnecting to WiFi..."));
            WiFi.disconnect();
            WiFi.reconnect();
        }
        logStatus();
        lastMillis = millis();
    } else {
        mqttClient.loop();
    }
}
