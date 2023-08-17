/*
   ESP32RtkBase

   This ESP32 sketch creates an RTK Base Station. It needs an ublox ZED-F9P
   based GNSS module connected to the ESP32 with I2C. The sketch uses SparkFun's
   u-blox GNSS V3 Library and parts of the code based on the examples.

   Configuration is included from settings.h

   It uses MQTT connection for logging, accepting commands and saving PPP raw data.

   There are two main modes:
        * PPP:  It logs data to an MQTT topic that can be used to calculate the precise
                position of the base station. The topic to send can be set with
                mqttPppTopic setting.
        * RTK:  It serves as an RTK Base Station and sends NTRIP data to a caster.
                To use it as a Base, it should know the precise position of the
                antenna.
                The NTRIP caster settngs are:
                    ntripHost
                    ntripPort
                    ntripMountPoint
                    ntripPassword
                The position data settings are:
                    latitude
                    latitudeHP
                    longitude
                    longitudeHP
                    altitude
                    altitudeHP

    Commands can be sent through the MQTT command topic:
        * 'PING'        It is used to check the base station. It sends a response
                        to the MQTT log topic with the current settings.
        * 'REBOOT'      Reboots the ESP32
        * 'RESET'       Resets all ESP32 settings to defaults and formats the SPIFFS
        * 'MODE_RTK'    Sets the main mode to RTK and reboots the ESP32
        * 'MODE_PPP'    Sets the main mode to PPP and reboots the ESP32
        * 'START'       Starts the data collection. In PPP mode it sends the RAWX data
                        to an MQTT topic. In RTK mode it sends NTRIP data to a caster
        * 'STOP'        Stops the data collection
        * 'SETTINGS:{}' This command can be used to update any settings. The new settings
                        should be in json format.
                        Ex.:
                            'SETTINGS:{"ntripHost": "example.com"}'

    The sketch publishes logs and monitoring data (per minute) to an MQTT
    log topic (mqttLogTopic setting).
*/
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
bool mainModeRunning = false;

// RTK Variables
WiFiClient ntripCaster;
long lastSentRTCM_ms = 0;
int maxTimeBeforeHangup_ms = 10000;
uint32_t serverBytesSent = 0;

// MQTT Client
WiFiClientSecure espClient;
PubSubClient mqttClient(espClient);
WiFiManager wifiManager;

// Save config to file
bool save_config = true;

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
    Serial.println(out);
    mqttClient.publish(mqttLogTopic, out);
}

// sends status information to MQTT log topic
void logStatus() {
    // Log data
    char out[512];
    StaticJsonDocument<512> logdata;
    logdata["time"] = getTime();
    logdata["host"] = host;
    logdata["monitoring_data"]["wifi_rssi"] = WiFi.RSSI();
    logdata["monitoring_data"]["main_mode"] = mainMode;
    logdata["monitoring_data"]["main_mode_running"] = mainModeRunning;
    logdata["monitoring_data"]["num_sfrbx_messages"] = numSFRBX;
    logdata["monitoring_data"]["num_rawx_messages"] = numRAWX;
    logdata["monitoring_data"]["rtcm_bytes_sent"] = serverBytesSent;
    logdata["monitoring_data"]["antenna_status"] = myGNSS.getAntennaStatus();
    logdata["monitoring_data"]["satellites_in_view"] = myGNSS.getSIV(1000);
    logdata["monitoring_data"]["gnss_fix_type"] = myGNSS.getFixType(1000);
    logdata["monitoring_data"]["gnss_fix_ok"] = myGNSS.getGnssFixOk(1000);
    serializeJson(logdata, out);
    mqttClient.publish(mqttLogTopic, out);
}

// logs settings to MQTT log topic
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

// MQTT Callback, sets behavior based on commands
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
        mainModeRunning = false;
        saveConfigToFile();
        delay(5000);
        ESP.restart();
    } else if ( strcmp(message, "MODE_PPP") == 0 ) {
        logToMQTT("Setting main mode to PPP");
        strcpy(mainMode, "PPP");
        mainModeRunning = false;
        saveConfigToFile();
        delay(5000);
        ESP.restart();
    } else if ( strcmp(message, "START") == 0 ) {
        logToMQTT("Starting the data collection");
        mainModeRunning = true;
        saveConfigToFile();
    } else if ( strcmp(message, "STOP") == 0 ) {
        logToMQTT("Stopping the data collection");
        mainModeRunning = false;
        saveConfigToFile();
    } else if (strncmp(message, "SETTINGS:", 9) == 0) {
        logToMQTT("New settings received");
        int settingsLength = sizeof(message) - 9;
        char settingsJson[settingsLength];
        memcpy(settingsJson, &message[9], settingsLength);
        getConfigFromJson(settingsJson);
        saveConfigToFile();
        logSettings();
    } else {
        logToMQTT("Unknown command received");
    }
}

// Gets ESP info and prints to serial
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

// Returns a JSON document containing all config
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
    jsonConfig["ntripHost"] = ntripHost;
    jsonConfig["ntripPort"] = ntripPort;
    jsonConfig["ntripMountPoint"] = ntripMountPoint;
    jsonConfig["ntripPassword"] = ntripPassword;
    jsonConfig["mainModeRunning"] = mainModeRunning;
    return jsonConfig;
}

// Sets config from a serialized JSON string
void getConfigFromJson(char* configFile) {
    DynamicJsonDocument configJson(1024);
    DeserializationError error = deserializeJson(configJson, configFile);
    if (!error) {
        Serial.println(F("The parsed json:"));
        if (configJson["mqttServer"]) {
            strlcpy(mqttServer, configJson["mqttServer"], sizeof(mqttServer));
            Serial.print(F("mqttServer: "));
            Serial.println(mqttServer);
        }
        if (configJson["mqttPort"]) {
            strlcpy(mqttPort, configJson["mqttPort"], sizeof(mqttPort));
            Serial.print(F("mqttPort: "));
            Serial.println(mqttPort);
        }
        if (configJson["mqttCommandTopic"]) {
            strlcpy(mqttCommandTopic, configJson["mqttCommandTopic"], sizeof(mqttCommandTopic));
            Serial.print(F("mqttCommandTopic: "));
            Serial.println(mqttCommandTopic);
        }
        if (configJson["mqttCommandTopic"]) {
            strlcpy(mqttPppTopic, configJson["mqttPppTopic"], sizeof(mqttPppTopic));
            Serial.print(F("mqttPppTopic: "));
            Serial.println(mqttPppTopic);
        }
        if (configJson["mqttLogTopic"]) {
            strlcpy(mqttLogTopic, configJson["mqttLogTopic"], sizeof(mqttLogTopic));
            Serial.print(F("mqttLogTopic: "));
            Serial.println(mqttLogTopic);
        }
        if (configJson["mainMode"]) {
            strlcpy(mainMode, configJson["mainMode"], sizeof(mainMode));
            Serial.print(F("mainMode: "));
            Serial.println(mainMode);
        }
        if (configJson["longitude"]) {
            strlcpy(longitude, configJson["longitude"], sizeof(longitude));
            Serial.print(F("longitude: "));
            Serial.println(longitude);
        }
        if (configJson["longitudeHP"]) {
            strlcpy(longitudeHP, configJson["longitudeHP"], sizeof(longitudeHP));
            Serial.print(F("longitudeHP: "));
            Serial.println(longitudeHP);
        }
        if (configJson["latitude"]) {
            strlcpy(latitude, configJson["latitude"], sizeof(latitude));
            Serial.print(F("latitude: "));
            Serial.println(latitude);
        }
        if (configJson["latitudeHP"]) {
            strlcpy(latitudeHP, configJson["latitudeHP"], sizeof(latitudeHP));
            Serial.print(F("latitudeHP: "));
            Serial.println(latitudeHP);
        }
        if (configJson["altitude"]) {
            strlcpy(altitude, configJson["altitude"], sizeof(altitude));
            Serial.print(F("altitude: "));
            Serial.println(altitude);
        }
        if (configJson["altitudeHP"]) {
            strlcpy(altitudeHP, configJson["altitudeHP"], sizeof(altitudeHP));
            Serial.print(F("altitudeHP: "));
            Serial.println(altitudeHP);
        }
        if (configJson["ntripHost"]) {
            strlcpy(ntripHost, configJson["ntripHost"], sizeof(ntripHost));
            Serial.print(F("ntripHost: "));
            Serial.println(ntripHost);
        }
        if (configJson["ntripPort"]) {
            strlcpy(ntripPort, configJson["ntripPort"], sizeof(ntripPort));
            Serial.print(F("ntripPort: "));
            Serial.println(ntripPort);
        }
        if (configJson["ntripMountPoint"]) {
            strlcpy(ntripMountPoint, configJson["ntripMountPoint"], sizeof(ntripMountPoint));
            Serial.print(F("ntripMountPoint: "));
            Serial.println(ntripMountPoint);
        }
        if (configJson["ntripPassword"]) {
            strlcpy(ntripPassword, configJson["ntripPassword"], sizeof(ntripPassword));
            Serial.print(F("ntripPassword: "));
            Serial.println(ntripPassword);
        }
        if (configJson["mainModeRunning"]) {
            mainModeRunning = configJson["mainModeRunning"];
            Serial.print(F("mainModeRunning: "));
            if (configJson["mainModeRunning"])
                Serial.println("true");
            else
                Serial.println("false");
        }
    } else {
        Serial.print(F("Json parse error: "));
        Serial.println(error.f_str());
    }
}

// Saves config to SPIFFS filesystem as serialized JSON
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

// Returns current epoch
unsigned long getTime() {
    time_t now;
    struct tm timeinfo;
    if (!getLocalTime(&timeinfo))
        return(0);
    time(&now);
    return now;
}

void newSFRBX(UBX_RXM_SFRBX_data_t *ubxDataStruct) {
    numSFRBX++;
}

void newRAWX(UBX_RXM_RAWX_data_t *ubxDataStruct) {
    numRAWX++;
}

void reconnectAll() {
    // Reconnect to the mqtt server
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
                size_t size = configFile.size();
                // // Allocate a buffer to store contents of the file.
                std::unique_ptr<char[]> buf(new char[size]);
                configFile.readBytes(buf.get(), size);
                getConfigFromJson(buf.get());
            } else {
                Serial.println(F("File read error!"));
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
    WiFiManagerParameter customLongitudeHP("longitudeHP", "Longitude (high precision)", longitudeHP, sizeof(longitudeHP));
    WiFiManagerParameter customAltitude("altitude", "Altitude", altitude, sizeof(altitude));
    WiFiManagerParameter customAltitudeHP("altitudeHP", "Altitude (high precision)", altitudeHP, sizeof(altitudeHP));
    WiFiManagerParameter customNtripHost("ntripHost", "NTRIP Host:", ntripHost, sizeof(ntripHost));
    WiFiManagerParameter customNtripPort("ntripPort", "NTRIP Port:", ntripPort, sizeof(ntripPort));
    WiFiManagerParameter customNtripMountPoint("ntripMountPoint", "NTRIP Mount Point:", ntripMountPoint, sizeof(ntripMountPoint));
    WiFiManagerParameter customNtripPassword("ntripPassword", "NTRIP Password:", ntripPassword, sizeof(ntripPassword));

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
    wifiManager.addParameter(&customNtripHost);
    wifiManager.addParameter(&customNtripPort);
    wifiManager.addParameter(&customNtripMountPoint);
    wifiManager.addParameter(&customNtripPassword);

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
    strcpy(ntripHost, customNtripHost.getValue());
    strcpy(ntripPort, customNtripPort.getValue());
    strcpy(ntripMountPoint, customNtripMountPoint.getValue());
    strcpy(ntripPassword, customNtripPassword.getValue());


    // Setup NTP
    configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);

    // Setup mqtt
    espClient.setCACert(mqtt_ca);
    espClient.setCertificate(mqtt_cert);
    espClient.setPrivateKey(mqtt_key);
    mqttClient.setServer(mqttServer, atoi(mqttPort));
    mqttClient.setCallback(mqttCallback);
    mqttClient.setBufferSize(512);
    myBuffer = new uint8_t[mqttSendMaxSize];
    reconnect();

    // GPS Setup
    Wire.begin();
    myGNSS.setFileBufferSize(fileBufferSize);

    while (myGNSS.begin() == false) {
        logToMQTT("u-blox GNSS not detected at default I2C address. Retrying...");
        delay(1000);
    }
    logToMQTT("u-blox GNSS detected");

    if (strcmp(mainMode, "PPP") == 0) {
        logToMQTT("Main mode: PPP, setting up u-blox GNSS");
        myGNSS.setI2COutput(COM_TYPE_UBX);
        myGNSS.saveConfigSelective(VAL_CFG_SUBSEC_IOPORT);
        myGNSS.setNavigationFrequency(1);
        myGNSS.setAutoRXMSFRBXcallbackPtr(&newSFRBX);
        myGNSS.logRXMSFRBX();
        myGNSS.setAutoRXMRAWXcallbackPtr(&newRAWX);
        myGNSS.logRXMRAWX();
    } else if (strcmp(mainMode, "RTK") == 0) {
        logToMQTT("Main mode: RTK, setting up u-blox GNSS");
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
        response &= myGNSS.setStaticPosition(
                atoi(latitude),
                atoi(latitudeHP),
                atoi(longitude),
                atoi(longitudeHP),
                atoi(altitude),
                atoi(altitudeHP),
                false,
                VAL_LAYER_RAM
        );
        if (response == false) {
            logToMQTT("RTK mode setup failed");
            strcpy(mainMode, "ERR");
        }
    } else {
        logToMQTT("No main mode set");
        strcpy(mainMode, "ERR");
    }
    logToMQTT("u-blox setup completed");
}

void loop() {
    static uint32_t lastMillis = 0;
    if (strcmp(mainMode, "PPP") == 0 && mainModeRunning) {
        // main mode is PPP and data collection is started
        myGNSS.checkUblox();
        myGNSS.checkCallbacks();
        while (myGNSS.fileBufferAvailable() >= mqttSendMaxSize) {
            myGNSS.extractFileBufferData(myBuffer, mqttSendMaxSize);
            if (mqttClient.publish(mqttPppTopic, myBuffer, mqttSendMaxSize)) {
                logToMQTT("RAWX data sent to MQTT");
            } else {
                logToMQTT("Couldn't send data to MQTT");
            }
            myGNSS.checkUblox();
            myGNSS.checkCallbacks();
        }
    } else if (strcmp(mainMode, "RTK") == 0) {
        // main mode is RTK
        if (ntripCaster.connected() == false && mainModeRunning) {
            // Connecting to NTRIP caster
            mqttClient.loop();
            logToMQTT("Opening socket to NTRIP host");
            if (ntripCaster.connect(ntripHost, atoi(ntripPort)) == true) {
                logToMQTT("Connected to NTRIP host");
                const int SERVER_BUFFER_SIZE = 512;
                char serverRequest[SERVER_BUFFER_SIZE];
                snprintf(
                    serverRequest,
                    SERVER_BUFFER_SIZE,
                    "SOURCE %s /%s\r\nSource-Agent: NTRIP SparkFun u-blox Server v1.0\r\n\r\n",
                    ntripPassword, ntripMountPoint);
                logToMQTT("Sending NTRIP server request");
                ntripCaster.write(serverRequest, strlen(serverRequest));
                unsigned long timeout = millis();
                while (ntripCaster.available() == 0) {
                    if (millis() - timeout > 5000) {
                        logToMQTT("Caster timed out");
                        ntripCaster.stop();
                        return;
                    }
                    delay(10);
                }
                bool connectionSuccess = false;
                char response[512];
                int responseSpot = 0;
                while (ntripCaster.available()) {
                    response[responseSpot++] = ntripCaster.read();
                    if (strstr(response, "200") != nullptr)
                        connectionSuccess = true;
                    if (responseSpot == 512 - 1) {
                        logToMQTT("Connected to NTRIP caster");
                        break;
                    }
                }
                response[responseSpot] = '\0';
                if (connectionSuccess == false) {
                    logToMQTT("Failed to connect to NTRIP caster");
                    return;
                }
            } else {
                logToMQTT("Connection to host failed");
                return;
            }
        }
        if (ntripCaster.connected() == true) {
            // Sending data to NTRIP caster
            while (1) {
                if ( !mainModeRunning) {
                    logToMQTT("Closing connection to NTRIP caster");
                    ntripCaster.stop();
                    return;
                }
                myGNSS.checkUblox();
                if (millis() - lastSentRTCM_ms > maxTimeBeforeHangup_ms) {
                    logToMQTT("RTCM timeout, disconnecting");
                    ntripCaster.stop();
                    return;
                }
                delay(10);
                if (millis() - lastMillis > 60000) {
                    reconnectAll();
                    lastMillis = millis();
                }
                mqttClient.loop();
            }
            delay(10);
        }
    }
    // check Wifi connection every minute
    if (millis() - lastMillis > 60000) {
        // Send statistics if in PPP mode and collecting data
        if (strcmp(mainMode, "PPP") == 0 && mainModeRunning) {
            Serial.print(F("Number of message groups received: SFRBX: "));
            Serial.print(numSFRBX);
            Serial.print(F(" RAWX: "));
            Serial.println(numRAWX);
            uint16_t maxBufferBytes = myGNSS.getMaxFileBufferAvail();
            Serial.print(F("The maximum number of bytes which the file buffer has contained is: "));
            Serial.println(maxBufferBytes);
            if (maxBufferBytes > ((fileBufferSize / 5) * 4)) {
                Serial.println(F("Warning: the file buffer has been over 80% full. Some data may have been lost."));
            }
        }
        reconnectAll();
        lastMillis = millis();
    } else {
        if (strcmp(mainMode, "PPP") == 0 && !mainModeRunning) {
            uint16_t remainingBytes = myGNSS.fileBufferAvailable();
            while (remainingBytes > 0) {
                uint16_t bytesToWrite = remainingBytes;
                if (bytesToWrite > mqttSendMaxSize) {
                    bytesToWrite = mqttSendMaxSize;
                }
                if (mqttClient.publish(mqttPppTopic, myBuffer, bytesToWrite)) {
                    logToMQTT("RAWX data sent to MQTT");
                } else {
                    logToMQTT("Couldn't send data to MQTT");
                }
                myGNSS.extractFileBufferData(myBuffer, bytesToWrite);
                remainingBytes -= bytesToWrite;
            }
        }
        mqttClient.loop();
    }
}

void DevUBLOXGNSS::processRTCM(uint8_t incoming) {
    if (ntripCaster.connected() == true) {
        ntripCaster.write(incoming); //Send this byte to socket
        serverBytesSent++;
        lastSentRTCM_ms = millis();
    }
}
