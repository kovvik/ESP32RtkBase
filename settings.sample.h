// MQTT Settings
char mqtt_server[40];
char mqtt_port[6];
char mqtt_username[32];
char mqtt_password[32];
char mqtt_command_topic[32];
char mqtt_ppp_topic[32];
char mqtt_log_topic[32];

// NTP Settings
const char* ntpServer = "pool.ntp.org";
const long  gmtOffset_sec = 3600; 
const int   daylightOffset_sec = 3600;
