-- Main configuration

-- Device name
NODENAME = "NodeMCU"

-- WiFi
wl_ap = {}
wl_ap["ssid"] = "pass"
wl_tries = 20

-- NTP server
ntp_server = "0.europe.pool.ntp.org"
ntp_interval = "765"
timezone = 2

-- Weather
ALTITUDE = 83
i2c_sda = 3
i2c_scl = 4
weather_interval = "60"

-- IoT MQTT
iot_server = "mqtt.example.com"
iot_port = 1883
iot_id = "MQTT_ID"
iot_user = "MQTT_USER"
iot_pass = "MQTT_PASS"

-- vim: set ft=lua ai ts=2 sts=2 et sw=2 sta nowrap nu :
