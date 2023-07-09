// Save this file as settings.h

// MQTT Settings
char mqttServer[40];
char mqttPort[6];
char mqttCommandTopic[32];
char mqttPppTopic[32];
char mqttLogTopic[32];

// MQTT Cert
// const char* mqtt_ca = ;
// const char* mqtt_cert = ;
// const char* mqtt_key = ;

// Main mode:
//      PPP:
//          UBX_RAWX data logging. Data is sent to mqttPppTopic. It can be used
//          for offline PPP analysis.
//      RTK:
//          RTK Base station mode, data is sent to the configures NTRIP Caster.
//
char mainMode[4];

// NTP Settings
const char* ntpServer = "pool.ntp.org";
const long  gmtOffset_sec = 3600; 
const int   daylightOffset_sec = 3600;
