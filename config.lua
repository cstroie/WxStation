-- Main configuration
CFG = {}

-- Global device data
NODENAME = "NodeMCU"  -- device name
TZ = 0                -- timezone
ALT = 0               -- altitude
I2C_SDA = 3           -- i2c sda pin
I2C_SCL = 4           -- i2c scl pin

-- WiFi
CFG.WL = {}
CFG.WL.tries = 20
CFG.WL.AP = {}
CFG.WL.AP["ssid"] = "pass"

-- NTP server
CFG.NTP = {}
CFG.NTP.server = "0.europe.pool.ntp.org"
CFG.NTP.interval = 765

-- Weather
CFG.WX = {}
CFG.WX.interval = 60

-- IoT MQTT
CFG.IOT = {}
CFG.IOT.server = "mqtt.example.com"
CFG.IOT.port = 1883
CFG.IOT.ssl = 0
CFG.IOT.auto = 1
CFG.IOT.id = "MQTT_ID"
CFG.IOT.user = "MQTT_USER"
CFG.IOT.pass = "MQTT_PASS"

-- vim: set ft=lua ai ts=2 sts=2 et sw=2 sta nowrap nu :
