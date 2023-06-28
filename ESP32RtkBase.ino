#include <FS.h>
#include <WiFi.h>
#include <DNSServer.h>
#include <WebServer.h>
#include <WiFiManager.h>
#include <ArduinoJson.h>
#include "SPIFFS.h"

// Client ID for ESP Chip identification
#define CLIENT_ID "ESP_%06X"

// Chip ID
uint32_t chipId = 0;

// Device name
char dev_name[50];

// MQTT Settings
char mqtt_server[40] = "192.168.178.101";
char mqtt_port[6] = "8080";
char mqtt_username[32] = "esp";
char mqtt_password[32] = "esp_pass";
char mqtt_command_topic[32] = "rtk_command";
char mqtt_ppp_topic[32] = "rtk_ppp";
char mqtt_log_topic[32] = "rtk_log";

// Save config to file
bool save_config = true;

void getESPInfo() {
    for(int i=0; i<17; i=i+8) {
        chipId |= ((ESP.getEfuseMac() >> (40 - i)) & 0xff) << i;
    }
    Serial.println(chipId);
    sprintf(dev_name, CLIENT_ID, chipId);
    Serial.println(dev_name);
}

// Save config callback
void saveConfigToFile() {
    Serial.println("should save config");
    save_config = true;
}

void setup() {
    // Starting serial
    Serial.begin(115200);

    // Waiting serial to be ready
    while(!Serial) { };

    // Print ESP info
    getESPInfo();

    //clean FS, for testing
    SPIFFS.format();

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
                    strlcpy(mqtt_username, configJson["mqtt_username"], sizeof(mqtt_username));
                    strlcpy(mqtt_password, configJson["mqtt_password"], sizeof(mqtt_password));
                    strlcpy(mqtt_command_topic, configJson["mqtt_command_topic"], sizeof(mqtt_command_topic));
                    strlcpy(mqtt_ppp_topic, configJson["mqtt_ppp_topic"], sizeof(mqtt_ppp_topic));
                    strlcpy(mqtt_log_topic, configJson["mqtt_log_topic"], sizeof(mqtt_log_topic));
                    Serial.print(F("mqtt_server: "));
                    Serial.println(mqtt_server);
                    Serial.print(F("mqtt_port: "));
                    Serial.println(mqtt_port);
                    Serial.print(F("mqtt_username: "));
                    Serial.println(mqtt_username);
                    Serial.print(F("mqtt_password: "));
                    Serial.println(mqtt_password);
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
    WiFiManagerParameter custom_mqtt_username("Username", "mqtt_username", mqtt_username, sizeof(mqtt_username));
    WiFiManagerParameter custom_mqtt_password("Password", "mqtt_password", mqtt_password, sizeof(mqtt_password));
    WiFiManagerParameter custom_mqtt_command_topic("mqtt_command_topic", "mqtt_command_topic", mqtt_command_topic, sizeof(mqtt_command_topic));
    WiFiManagerParameter custom_mqtt_ppp_topic("mqtt_ppp_topic", "mqtt_ppp_topic", mqtt_ppp_topic, sizeof(mqtt_ppp_topic));
    WiFiManagerParameter custom_mqtt_log_topic("mqtt_log_topic", "mqtt_log_topic", mqtt_log_topic, sizeof(mqtt_log_topic));

    WiFiManager wifiManager;
    wifiManager.setSaveConfigCallback(saveConfigToFile);
    wifiManager.setConfigPortalTimeout(240);

    wifiManager.addParameter(&custom_mqtt_server);
    wifiManager.addParameter(&custom_mqtt_port);
    wifiManager.addParameter(&custom_mqtt_username);
    wifiManager.addParameter(&custom_mqtt_password);
    wifiManager.addParameter(&custom_mqtt_command_topic);
    wifiManager.addParameter(&custom_mqtt_ppp_topic);
    wifiManager.addParameter(&custom_mqtt_log_topic);

    // Set WIFI connection timeout to avoid endless loop
    wifiManager.setConnectTimeout(60);

    // Set WIFI country
    wifiManager.setCountry("US");
    wifiManager.resetSettings();

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
    strcpy(mqtt_username, custom_mqtt_username.getValue());
    strcpy(mqtt_password, custom_mqtt_password.getValue());
    strcpy(mqtt_command_topic, custom_mqtt_command_topic.getValue());
    strcpy(mqtt_ppp_topic, custom_mqtt_ppp_topic.getValue());
    strcpy(mqtt_log_topic, custom_mqtt_log_topic.getValue());

    if (save_config) {
        Serial.println("Saving config ...");
        DynamicJsonDocument jsonConfig(512);
        jsonConfig["mqtt_server"] = mqtt_server;
        jsonConfig["mqtt_port"] = mqtt_port;
        jsonConfig["mqtt_username"] = mqtt_username;
        jsonConfig["mqtt_password"] = mqtt_password;
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
}

void loop() {
    // check Wifi connection every minute
    static uint32_t lastMillis = 0;
    if (millis() - lastMillis > 60000) {
        // Check wifi connection and reconnect if connection lost
        if (WiFi.status() != WL_CONNECTED) {
            Serial.println("Lost connection, reconnecting to WiFi...");
            WiFi.disconnect();
            WiFi.reconnect();
        }
        // Publish values and print console
        lastMillis = millis();
    }
}
