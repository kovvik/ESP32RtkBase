// MQTT Settings
char mqtt_server[40];
char mqtt_port[6];
char mqtt_command_topic[32];
char mqtt_ppp_topic[32];
char mqtt_log_topic[32];

// MQTT Cert
// const char* mqtt_ca = ;
// const char* mqtt_cert = ;
// const char* mqtt_key = ;


// NTP Settings
const char* ntpServer = "pool.ntp.org";
const long  gmtOffset_sec = 3600; 
const int   daylightOffset_sec = 3600;
