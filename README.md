# ESP32RtkBase
RTK Base Station code for ESP32

## Hardware setup

You will need these:
- ESP32 Board
- ZED-F9P board with I2C interface
- Dual channel (L1/L2) GNSS antenna

The code uses SparkFuns u-blox GNSS V3 library and I used and tested it with a
`SparkFun GPS-RTK2 Board - ZED-F9P` board for my setup with.
Other ZED-F9P boards could work but I recommend the SparkFun products.

Connect the ESP board with I2C to the ZED-F9P board. Setup configuration in
`settings.h` and flash it to the ESP board.

## Architecture
```
   ----------------              ----------------
   | BASE STATION | -----------> | NTRIP CASTER |
   |              |              ----------------
   |              |
   |              |              ---------------          -------------------
   |              | -----------> | MQTT SERVER | -------> | LOG AND COMMAND |
   |              | <----------- |             | <------- |     SERVER      |
   ----------------              ---------------          -------------------
```

The Base Station connects to a MQTT Server through a WiFi connection. It sends
its log to a log topic (`rtk_log`) and accepts commands on a command topic
(`rtk_command`).

The connection to the MQTT server is using secure connection with client
authentication, so you have to setup the MQTT server accordingly with valid
certs. I use mosquitto as MQTT server/client in the exapmles but any other MQTT
compatible tools will work.

You need to setup the Wifi connection first. The ESP board will start in AP if
it cannot connect to Wifi and you can set up the Wifi SSID/password to your
network.

### Monitoring
You can monitor the logs by subscribing to the log topic:

```
export CA_CRT=<ca cert file location>
export CLIENT_CRT=<client cert file location>
export CLIENT_KEY=<client key file location>
export HOST=<MQTT server hostname or IP address>
export PORT=<MQTT server port>

mosquitto_sub \
    --cafile ${CA_CRT} \
    --cert ${CLIENT_CRT} \
    --key ${CLIENT_KEY} \
    -h ${HOST} \
    -p ${PORT} \
    -t rtk_log
```
### Sending comands

```
mosquitto_pub \
    --cafile ${CA_CRT} \
    --cert ${CLIENT_CRT} \
    --key ${CLIENT_KEY} \
    -h ${HOST} \
    -p ${PORT} \
    -t rtk_command \
    -m "PING"
```

See all commands and their description in the ino file.

### Update settings

All settings can be set up in the `settings.h` file but they can be overwritten
during operation by sending `SETTINGS:` followed by a json object with the
key/values you would like to update.

```
mosquitto_pub \
    --cafile ${CA_CRT} \
    --cert ${CLIENT_CRT} \
    --key ${CLIENT_KEY} \
    -h ${HOST} \
    -p ${PORT} \
    -t rtk_command \
    -m 'SETTINGS:{"ntripMountPoint":"exapmle"}'
```

# Main Modes

## PPP Mode

In PPP mode you will gather positional data to get the exact location of the
antenna. In the examples I use mosquitto as MQTT server and client with clinet
cert authentication.

Change main mode to PPP in another terminal and check the logs if its ready to
start PPP data collection:

```
mosquitto_pub \
    --cafile ${CA_CRT} \
    --cert ${CLIENT_CRT} \
    --key ${CLIENT_KEY} \
    -h ${HOST} \
    -p ${PORT} \
    -t rtk_command \
    -m "MODE_PPP"
```

Open an other terminal and save the output of the "rtp_ppp" topic to a binary
file without linebreaks:

```
mosquitto_sub \
    --cafile ${CA_CRT} \
    --cert ${CLIENT_CRT} \
    --key ${CLIENT_KEY} \
    -h ${HOST} \
    -p ${PORT} \
    -t "rtk_ppp" \
    -N > /tmp/ppp_data.ubx
```

Start the collection from the second terminal and let it run for 24 hours:
```
mosquitto_pub \
    --cafile ${CA_CRT} \
    --cert ${CLIENT_CRT} \
    --key ${CLIENT_KEY} \
    -h ${HOST} \
    -p ${PORT} \
    -t rtk_command \
    -m "START"
```

After 24 hours stop the data collection:
```
mosquitto_pub \
    --cafile ${CA_CRT} \
    --cert ${CLIENT_CRT} \
    --key ${CLIENT_KEY} \
    -h ${HOST} \
    -p ${PORT} \
    -t rtk_command \
    -m "STOP"
```
and hit crtl+c on the third terminal. You should have a `/tmp/ppp_data.ubx`
file with around 300MB of data.

We need to convert the ubx data to a RINEX format. I recommend this RTKLIB
fork: https://github.com/rtklibexplorer/RTKLIB

If you have a Linux machine you can use the console apps (On Windows you have
to build and use the GUI apps). Clone the repo and build the console apps:

```
git clone https://github.com/rtklibexplorer/RTKLIB.git
cd rtklib/app/consapp
make
```

Use the convbin app to convert the ubx binary data to RINEX data. You can scan
the file first to check if the data is correct and check the dates.

If it is more than one day we need to trim the data:

```
mkdir ~/ppp_rinex_data
convbin/gcc/convbin -scan -r ubx -d ~/ppp_rinex_data /tmp/ppp_data.ubx
tar czvf ppp_rinex_data.tar.gz -C ~/ppp_rinex_data .
```

Create an account on the Canadian Geodetic Survey site and upload the RINEX
data: https://webapp.csrs-scrs.nrcan-rncan.gc.ca/geod/tools-outils/ppp.php

You should get the position data back shortly (from 10-20min to 1 day in my
experience) by email. The results will contain a PDF with all the details.

We need the ITRF20 (2023.9) line from the estimated position data.

To convert the position data to ECEF coordinates first convert the longitude
and latitude values to decimal, using this formula:

```
decimal degree = degree + ( minutes / 60 ) + ( seconds / 3600 )
```

Open this page: https://www.convertecef.com/index

Fill in the decimal longitude, latitude and height information and convert them
to ECEF coordinates.

The Base Station expects the coordinates in cm with high precision extension,
so the ECEF coordinates that are in meters will be presented like this:

```
ECEF Longitude: 1234.5678 ==> Longitude: 123456, LongitudeHP: 78
```

Send the coordinates to the Base Station:

```
mosquitto_pub \
    --cafile ${CA_CRT} \
    --cert ${CLIENT_CRT} \
    --key ${CLIENT_KEY} \
    --insecure ${HOST} \
    -p ${PORT} \
    -t rtk_command \
    -m 'SETTINGS:{
            "latitude":"${LATITUDE}",
            "latitudeHP":"${LATITUDE_HP}",
            "longitude":"${LONGITUDE}",
            "longitudeHP":"${LONGITUDE_HP}",
            "altitude":"${ALTITUDE}",
            "altitudeHP":"${ALTITUDE_HP}"
    }'
```

## RTK Mode

This is the mode where you can use the Base Station as a RTK base. It will
gather GNSS data and send it to a NTRIP Caster using the NTRIP protocol. Then 
you can point your rover to the mount point on the NTRIP Caster.

To act as a NTRIP Server the Base Station needs to know it precise location 
(see PPP Mode).

After the precise location is set up you can set the main mode to RTK:

```
mosquitto_pub \
    --cafile ${CA_CRT} \
    --cert ${CLIENT_CRT} \
    --key ${CLIENT_KEY} \
    -h ${HOST} \
    -p ${PORT} \
    -t rtk_command \
    -m "MODE_RTK"
```

Set the NTRIP Caster information if you have not done it in the `settings.h`:

```
mosquitto_pub \
    --cafile ${CA_CRT} \
    --cert ${CLIENT_CRT} \
    --key ${CLIENT_KEY} \
    -h ${HOST} \
    -p ${PORT} \
    -t rtk_command \
    -m 'SETTINGS:{
        "ntripHost":"${NTRIP_HOST}",
        "ntripPort":"${NTRIP_PORT}",
        "ntripMountPoint":"${NTRIP_MOUNT_POINT}",
        "ntripPassword":"${PASSWORD}"
    }'
```

Start the NTRIP Server:
```
mosquitto_pub \
    --cafile ${CA_CRT} \
    --cert ${CLIENT_CRT} \
    --key ${CLIENT_KEY} \
    -h ${HOST} \
    -p ${PORT} \
    -t rtk_command \
    -m "STOP"
```

