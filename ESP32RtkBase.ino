#include <FS.h>
#include <WiFiClientSecure.h>
#include <WiFiManager.h>
#include <ArduinoJson.h>
#include "SPIFFS.h"
#include <PubSubClient.h>
#include "time.h"
#include <Wire.h>
#include <SparkFun_u-blox_GNSS_v3.h>
#include "settings.h"

// Client ID for ESP Chip identification
#define CLIENT_ID "ESP_%06X"

// Chip ID
uint32_t chipId = 0;

// Device name
char host[50];

// ZED-F9P Setup
SFE_UBLOX_GNSS myGNSS;
#define mqttSendMaxSize 128
#define fileBufferSize 4096
uint8_t *myBuffer;
int numSFRBX = 0;
int numRAWX = 0;
bool collectRAWX = false;

// RTK Variable
long lastSentRTCM_ms = 0;
int maxTimeBeforeHangup_ms = 10000;
uint32_t serverBytesSent = 0;
long lastReport_ms = 0;

// Location variables
char latitude[12];
char latitudeHP[3];
char longitude[12];
char longitudeHP[3];
char altitude[12];
char altitudeHP[3];

// MQTT Client
WiFiClientSecure espClient;
PubSubClient mqttClient(espClient);
WiFiManager wifiManager;

// MQTT reconnect
void reconnect() {
    Serial.println(F("Trying to connect mqtt..."));
    if (mqttClient.connect(host)) {
        Serial.println(F("Connected to MQTT broker"));
        mqttClient.subscribe(mqttCommandTopic);
        Serial.print(F("Subscribe to topic: "));
        Serial.println(mqttCommandTopic);
        logToMQTT("HELLO");
        logSettings();
    }
}

// sends log to MQTT log topic
void logToMQTT(char message[512]) {
    StaticJsonDocument<512> logdata;
    logdata["time"] = getTime();
    logdata["host"] = host;
    logdata["message"] = message;
    char out[512];
    int b = serializeJson(logdata, out);
    Serial.print(F("sending bytes to MQTT log:"));
    Serial.println(b, DEC);
    mqttClient.publish(mqttLogTopic, out);
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

void logSettings() {
    // Log data
    char out[512];
    StaticJsonDocument<512> logdata;
    logdata["time"] = getTime();
    logdata["host"] = host;
    logdata["settings"] = getConfigJson(); 
    serializeJson(logdata, out);
    mqttClient.publish(mqttLogTopic, out);
}

void mqttCallback(char* topic, byte* payload, unsigned int length) {
    char message[length + 1];
    memcpy(message, payload, length);
    message[length] = '\0';
    Serial.print("Message arrived on command topic: ");
    Serial.println(message);
    if ( strcmp(message, "PING") == 0 ) {
        logToMQTT("PONG");
        logSettings();
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
    } else if ( strcmp(message, "MODE_RTK") == 0 ) {
        logToMQTT("Setting main mode to RTK");
        strcpy(mainMode, "RTK");
        saveConfigToFile();
        delay(5000);
        ESP.restart();
    } else if ( strcmp(message, "MODE_PPP") == 0 ) {
        logToMQTT("Setting main mode to PPP");
        strcpy(mainMode, "PPP");
        saveConfigToFile();
        delay(5000);
        ESP.restart();
    } else if ( strcmp(message, "START_PPP") == 0 ) {
        logToMQTT("Starting RAWX collection");
        collectRAWX = true;
    } else if ( strcmp(message, "STOP_PPP") == 0 ) {
        logToMQTT("Stopping RAWX collection");
        collectRAWX = false;
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
void saveConfigToFileCallback() {
    Serial.println("should save config");
    save_config = true;
}

DynamicJsonDocument getConfigJson() {
    DynamicJsonDocument jsonConfig(1024);
    jsonConfig["mqttServer"] = mqttServer;
    jsonConfig["mqttPort"] = mqttPort;
    jsonConfig["mqttCommandTopic"] = mqttCommandTopic;
    jsonConfig["mqttPppTopic"] = mqttPppTopic;
    jsonConfig["mqttLogTopic"] = mqttLogTopic;
    jsonConfig["mainMode"] = mainMode;
    jsonConfig["latitude"] = latitude;
    jsonConfig["latitudeHP"] = latitudeHP;
    jsonConfig["longitude"] = longitude;
    jsonConfig["longitudeHP"] = longitudeHP;
    jsonConfig["altitude"] = altitude;
    jsonConfig["altitudeHP"] = altitudeHP;
    return jsonConfig;
}

void saveConfigToFile() {
    Serial.println("Saving config ...");
    DynamicJsonDocument jsonConfig = getConfigJson();
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
    save_config = false;
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

void newSFRBX(UBX_RXM_SFRBX_data_t *ubxDataStruct) {
    Serial.println(F("SFRBX data arrived"));
    numSFRBX++;
}

void newRAWX(UBX_RXM_RAWX_data_t *ubxDataStruct) {
    Serial.println(F("RAWX data arrived"));
    numRAWX++;
}

void setup() {
    delay(1000);
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
                // size_t size = configFile.size();
                // // Allocate a buffer to store contents of the file.
                // std::unique_ptr<char[]> buf(new char[size]);
                // configFile.readBytes(buf.get(), size);
                DynamicJsonDocument configJson(1024);
                DeserializationError error = deserializeJson(configJson, configFile);
                if (!error) {
                    Serial.println(F("The parsed json:"));
                    strlcpy(mqttServer, configJson["mqttServer"], sizeof(mqttServer));
                    strlcpy(mqttPort, configJson["mqttPort"], sizeof(mqttPort));
                    strlcpy(mqttCommandTopic, configJson["mqttCommandTopic"], sizeof(mqttCommandTopic));
                    strlcpy(mqttPppTopic, configJson["mqttPppTopic"], sizeof(mqttPppTopic));
                    strlcpy(mqttLogTopic, configJson["mqttLogTopic"], sizeof(mqttLogTopic));
                    strlcpy(mainMode, configJson["mainMode"], sizeof(mainMode));
                    strlcpy(longitude, configJson["longitude"], sizeof(longitude));
                    strlcpy(longitudeHP, configJson["longitudeHP"], sizeof(longitudeHP));
                    strlcpy(latitude, configJson["latitude"], sizeof(latitude));
                    strlcpy(latitudeHP, configJson["latitudeHP"], sizeof(latitudeHP));
                    strlcpy(altitude, configJson["altitude"], sizeof(altitude));
                    strlcpy(altitudeHP, configJson["altitudeHP"], sizeof(altitudeHP));
                    Serial.print(F("mqttServer: "));
                    Serial.println(mqttServer);
                    Serial.print(F("mqttPort: "));
                    Serial.println(mqttPort);
                    Serial.print(F("mqttCommandTopic: "));
                    Serial.println(mqttCommandTopic);
                    Serial.print(F("mqttPppTopic: "));
                    Serial.println(mqttPppTopic);
                    Serial.print(F("mqttLogTopic: "));
                    Serial.println(mqttLogTopic);
                    Serial.print(F("mainMode: "));
                    Serial.println(mainMode);
                    Serial.print(F("longitude: "));
                    Serial.println(longitude);
                    Serial.print(F("longitudeHP: "));
                    Serial.println(longitudeHP);
                    Serial.print(F("latitude: "));
                    Serial.println(latitude);
                    Serial.print(F("latitudeHP: "));
                    Serial.println(latitudeHP);
                    Serial.print(F("altitude: "));
                    Serial.println(altitude);
                    Serial.print(F("altitudeHP: "));
                    Serial.println(altitudeHP);
                } else {
                    Serial.print(F("Json parse error: "));
                    Serial.println(error.f_str());
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
    WiFiManagerParameter customMqttServer("mqttServer", "MQTT Server", mqttServer, sizeof(mqttServer));
    WiFiManagerParameter customMqttPort("mqttPort", "MQTT Port", mqttPort, sizeof(mqttPort));
    WiFiManagerParameter customMqttCommandTopic("mqttCommandTopic", "MQTT Command Topic", mqttCommandTopic, sizeof(mqttCommandTopic));
    WiFiManagerParameter customMqttPppTopic("mqttPppTopic", "MQTT PPP Topic", mqttPppTopic, sizeof(mqttPppTopic));
    WiFiManagerParameter customMqttLogTopic("mqttLogTopic", "MQTT Log Topic", mqttLogTopic, sizeof(mqttLogTopic));
    WiFiManagerParameter customLatitude("latitude", "Latitude", latitude, sizeof(latitude));
    WiFiManagerParameter customLatitudeHP("latitudeHP", "Latitude (high precision)", latitudeHP, sizeof(latitudeHP));
    WiFiManagerParameter customLongitude("longitude", "Longitude", longitude, sizeof(longitude));
    WiFiManagerParameter customLongitudeHP("longitudeHP", "Longitude (high precision)", longitude, sizeof(longitudeHP));
    WiFiManagerParameter customAltitude("altitude", "Altitude", altitude, sizeof(altitude));
    WiFiManagerParameter customAltitudeHP("altitudeHP", "Altitude (high precision)", altitude, sizeof(altitudeHP));

    WiFiManager wifiManager;
    wifiManager.setSaveConfigCallback(saveConfigToFileCallback);
    wifiManager.setConfigPortalTimeout(240);

    wifiManager.addParameter(&customMqttServer);
    wifiManager.addParameter(&customMqttPort);
    wifiManager.addParameter(&customMqttCommandTopic);
    wifiManager.addParameter(&customMqttPppTopic);
    wifiManager.addParameter(&customMqttLogTopic);
    wifiManager.addParameter(&customLatitude);
    wifiManager.addParameter(&customLatitudeHP);
    wifiManager.addParameter(&customLongitude);
    wifiManager.addParameter(&customLongitudeHP);
    wifiManager.addParameter(&customAltitude);
    wifiManager.addParameter(&customAltitudeHP);

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

    strcpy(mqttServer, customMqttServer.getValue());
    strcpy(mqttPort, customMqttPort.getValue());
    strcpy(mqttCommandTopic, customMqttCommandTopic.getValue());
    strcpy(mqttPppTopic, customMqttPppTopic.getValue());
    strcpy(mqttLogTopic, customMqttLogTopic.getValue());
    strcpy(latitude, customLatitude.getValue());
    strcpy(latitudeHP, customLatitudeHP.getValue());
    strcpy(longitude, customLongitude.getValue());
    strcpy(longitudeHP, customLongitudeHP.getValue());
    strcpy(altitude, customAltitude.getValue());
    strcpy(altitudeHP, customAltitudeHP.getValue());

    // Setup NTP
    configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);

    // Setup mqtt
    espClient.setCACert(mqtt_ca);
    espClient.setCertificate(mqtt_cert);
    espClient.setPrivateKey(mqtt_key);
    mqttClient.setServer(mqttServer, atoi(mqttPort));
    mqttClient.setCallback(mqttCallback);
    mqttClient.setBufferSize(512);
    reconnect();

    // GPS Setup
    Wire.begin();
    myGNSS.setFileBufferSize(fileBufferSize);

    while (myGNSS.begin() == false) {
        Serial.println(F("u-blox GNSS not detected at default I2C address. Retrying..."));
        delay(1000);
    }

    Serial.println(F("u-blox GNSS detected."));

    if (strcmp(mainMode, "PPP") == 0) {
        Serial.println(F("Main mode: PPP"));
        myGNSS.setI2COutput(COM_TYPE_UBX);
        myGNSS.saveConfigSelective(VAL_CFG_SUBSEC_IOPORT);
        myGNSS.setNavigationFrequency(1);
        myGNSS.setAutoRXMSFRBXcallbackPtr(&newSFRBX);
        myGNSS.logRXMSFRBX();
        myGNSS.setAutoRXMRAWXcallbackPtr(&newRAWX);
        myGNSS.logRXMRAWX();
    } else if (strcmp(mainMode, "RTK") == 0) {
        Serial.println(F("Main mode: RTK"));
        myGNSS.setI2COutput(COM_TYPE_UBX | COM_TYPE_NMEA | COM_TYPE_RTCM3);
        myGNSS.setNavigationFrequency(1);
        bool response = myGNSS.newCfgValset(VAL_LAYER_RAM); // Use cfgValset to disable individual NMEA messages
        // Disable NMEA messages
        response &= myGNSS.addCfgValset(UBLOX_CFG_MSGOUT_NMEA_ID_GLL_I2C, 0);
        response &= myGNSS.addCfgValset(UBLOX_CFG_MSGOUT_NMEA_ID_GSA_I2C, 0);
        response &= myGNSS.addCfgValset(UBLOX_CFG_MSGOUT_NMEA_ID_GSV_I2C, 0);
        response &= myGNSS.addCfgValset(UBLOX_CFG_MSGOUT_NMEA_ID_GST_I2C, 0);
        response &= myGNSS.addCfgValset(UBLOX_CFG_MSGOUT_NMEA_ID_RMC_I2C, 0);
        response &= myGNSS.addCfgValset(UBLOX_CFG_MSGOUT_NMEA_ID_VTG_I2C, 0);
        response &= myGNSS.addCfgValset(UBLOX_CFG_MSGOUT_NMEA_ID_GGA_I2C, 0);
        response &= myGNSS.addCfgValset(UBLOX_CFG_MSGOUT_NMEA_ID_ZDA_I2C, 0);
        response &= myGNSS.sendCfgValset();
        //Enable necessary RTCM sentences
        response &= myGNSS.newCfgValset(VAL_LAYER_RAM);
        response &= myGNSS.addCfgValset(UBLOX_CFG_MSGOUT_RTCM_3X_TYPE1005_I2C, 1);
        response &= myGNSS.addCfgValset(UBLOX_CFG_MSGOUT_RTCM_3X_TYPE1074_I2C, 1);
        response &= myGNSS.addCfgValset(UBLOX_CFG_MSGOUT_RTCM_3X_TYPE1084_I2C, 1);
        response &= myGNSS.addCfgValset(UBLOX_CFG_MSGOUT_RTCM_3X_TYPE1094_I2C, 1);
        response &= myGNSS.addCfgValset(UBLOX_CFG_MSGOUT_RTCM_3X_TYPE1124_I2C, 1);
        response &= myGNSS.addCfgValset(UBLOX_CFG_MSGOUT_RTCM_3X_TYPE1230_I2C, 10);
        response &= myGNSS.sendCfgValset();
        // Setup base station location
        response &= myGNSS.setStaticPosition(-128020830, -80, -471680384, -70, 408666581, 10, false, VAL_LAYER_RAM);
        if (response == false) {
            Serial.println(F("Setup failed. Freezing..."));
            while (1);
        }
    } else {
        Serial.println(F("No main mode set!"));
    }
    myBuffer = new uint8_t[mqttSendMaxSize];
    Serial.println(F("u-blox setup completed."));

}

void loop() {
    if (strcmp(mainMode, "PPP") == 0 && collectRAWX) {
        myGNSS.checkUblox();
        myGNSS.checkCallbacks();
        while (myGNSS.fileBufferAvailable() >= mqttSendMaxSize) {
            myGNSS.extractFileBufferData(myBuffer, mqttSendMaxSize);
            if (mqttClient.publish(mqttPppTopic, myBuffer, mqttSendMaxSize)) {
                Serial.println(F("RAWX data sent to MQTT"));
            } else {
                Serial.println(F("Couldn't send data to MQTT"));
            }
            myGNSS.checkUblox();
            myGNSS.checkCallbacks();
        }
    }
    // check Wifi connection every minute
    static uint32_t lastMillis = 0;
    if (millis() - lastMillis > 60000) {
        Serial.print(F("Number of message groups received: SFRBX: ")); // Print how many message groups have been received (see note above)
        Serial.print(numSFRBX);
        Serial.print(F(" RAWX: "));
        Serial.println(numRAWX);

        uint16_t maxBufferBytes = myGNSS.getMaxFileBufferAvail(); // Get how full the file buffer has been (not how full it is now)

        Serial.print(F("The maximum number of bytes which the file buffer has contained is: ")); // It is a fun thing to watch how full the buffer gets
        Serial.println(maxBufferBytes);

        if (maxBufferBytes > ((fileBufferSize / 5) * 4)) {
            Serial.println(F("Warning: the file buffer has been over 80% full. Some data may have been lost."));
        }

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
        if (strcmp(mainMode, "PPP") == 0 && !collectRAWX) {
            uint16_t remainingBytes = myGNSS.fileBufferAvailable();
            while (remainingBytes > 0) {
                uint16_t bytesToWrite = remainingBytes;
                if (bytesToWrite > mqttSendMaxSize) {
                    bytesToWrite = mqttSendMaxSize;
                }
                if (mqttClient.publish(mqttPppTopic, myBuffer, bytesToWrite)) {
                    Serial.println(F("RAWX data sent to MQTT"));
                } else {
                    Serial.println(F("Couldn't send data to MQTT"));
                }
                myGNSS.extractFileBufferData(myBuffer, bytesToWrite);
                remainingBytes -= bytesToWrite;
            }
        }
        mqttClient.loop();
    }
}
