/**
  WxSta - Weather Station

  Copyright 2017 Costin STROIE <costinstroie@eridu.eu.org>

  This file is part of Weather Station.

  WxSta is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by the Free
  Software Foundation, either version 3 of the License, or (at your option) any
  later version.

  WxSta is distributed in the hope that it will be useful, but
  WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
  FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
  more details.

  You should have received a copy of the GNU General Public License along with
  WxSta.  If not, see <http://www.gnu.org/licenses/>.


  WiFi connected weather station, reading the athmospheric sensor BME280 and 
  the illuminance sensor TSL2561 and publishing the measured data along with
  various local telemetry.
*/

// Sensors are connected to I2C
#define SDA 0
#define SCL 2
#include <Wire.h>
#include <SparkFunBME280.h>
#include <SparkFunTSL2561.h>


// WiFi
#include <ESP8266WiFi.h>
#include <WiFiManager.h>
#include <WiFiUdp.h>

// NTP
#include <NTPClient.h>

// Timer
#include <AsyncDelay.h>

// MQTT
#include <PubSubClient.h>


// Device name
String NODENAME = "WxSta";
String LC_NODENAME = "wxsta";  // FIXME DNRY

// NTP
const int  TZ = 2;
const char NTP_SERVER[] = "europe.pool.ntp.org";
const int  NTP_INTERVAL = 765 * 1000;
WiFiUDP ntpUDP;
NTPClient NTP_Client(ntpUDP, NTP_SERVER, 3600 * TZ, NTP_INTERVAL);

// MQTT parameters
const char MQTT_ID[] = "wxsta-eridu-eu-org";
const char MQTT_SERVER[] = "eridu.eu.org";
const int  MQTT_PORT = 1883;
const int  MQTT_INTERVAL = 5000;
String MQTT_CMD  = "command/" + LC_NODENAME + "/";
String MQTT_REPORT = "report/" + LC_NODENAME;
String MQTT_REPORT_WIFI = MQTT_REPORT + "/wifi";
String MQTT_SENSOR = "sensor/outdoor"; // + LC_NODENAME;
WiFiClient WiFi_Client;
PubSubClient MQTT_Client(WiFi_Client);
AsyncDelay delayMQTT;

// Sensors
const int SNS_INTERVAL  = 60 * 1000;
BME280 atmo;
bool atmo_ok = false;
SFE_TSL2561 light;
boolean gain = false;
unsigned char shtr = 2;
unsigned int ms;
AsyncDelay delaySNS;

// Voltage
ADC_MODE(ADC_VCC);

/**
  Display the WiFi parameters
*/
void showWiFi() {
  if (WiFi.isConnected()) {
    String text = "\nWiFi connected to ";
    text += WiFi.SSID();
    text += " on channel ";
    text += WiFi.channel();
    text += ", RSSI ";
    text += WiFi.RSSI();
    text += " dBm.";
    text += "\n IP : ";
    text += WiFi.localIP().toString();
    text += "\n GW : ";
    text += WiFi.gatewayIP().toString();
    text += "\n DNS: ";
    text += WiFi.dnsIP().toString();
    text += "\n\n";
    Serial.print(text);
  }
}

/**
  Try to reconnect to MQTT server

  @return boolean reconnection success
*/
boolean mqttReconnect() {
  Serial.println(F("MQTT connecting..."));
  if (MQTT_Client.connect(MQTT_ID)) {
    // Publish the connection report
    MQTT_Client.publish(String(MQTT_REPORT_WIFI + "/hostname").c_str(), WiFi.hostname().c_str(), true);
    MQTT_Client.publish(String(MQTT_REPORT_WIFI + "/mac").c_str(), WiFi.macAddress().c_str(), true);
    MQTT_Client.publish(String(MQTT_REPORT_WIFI + "/ssid").c_str(), WiFi.SSID().c_str(), true);
    MQTT_Client.publish(String(MQTT_REPORT_WIFI + "/rssi").c_str(), String(WiFi.RSSI()).c_str(), true);
    MQTT_Client.publish(String(MQTT_REPORT_WIFI + "/ip").c_str(), WiFi.localIP().toString().c_str(), true);
    MQTT_Client.publish(String(MQTT_REPORT_WIFI + "/gw").c_str(), WiFi.gatewayIP().toString().c_str(), true);
    // Subscribe
    MQTT_Client.subscribe(String(MQTT_CMD + "#").c_str());
    // TODO
    MQTT_Client.subscribe("sensor/#");
    Serial.print(F("MQTT connected to "));
    Serial.println(MQTT_SERVER);
  }
  return MQTT_Client.connected();
}

/**
  Message arrived in MQTT subscribed topics

  @param topic the topic the message arrived on (const char[])
  @param payload the message payload (byte array)
  @param length the length of the message payload (unsigned int)
*/
void mqttCallback(char* topic, byte* payload, unsigned int length) {
  // Make a copy
  char message[100];
  if (length > 100) length = 100;
  memcpy(message, payload, length);
  message[length] = '\0';

  // Create string objects
  String strTopic = String(topic);
  String strMessage = String(message);
  Serial.println("MQTT " + strTopic + ": " + strMessage);

  // Decompose the topic
  String strRoot, strTrunk, strBranch;
  int idxSepOne = strTopic.indexOf('/');
  if (idxSepOne != -1) {
    strRoot = strTopic.substring(0, idxSepOne);
    //Serial.println("ROOT " + strRoot);
    int idxSepTwo = strTopic.indexOf('/', idxSepOne + 1);
    if (idxSepTwo != -1) {
      strTrunk = strTopic.substring(idxSepOne + 1, idxSepTwo);
      //Serial.println("TRNK " + strTrunk);
      strBranch = strTopic.substring(idxSepTwo + 1);
      //Serial.println("BRNC " + strBranch);

      // Dispatcher
      if (strRoot == "command") {
        if (strTrunk == LC_NODENAME) {
          if (strBranch == "restart") {
            ESP.restart();
          }
        }
      }
    }
  }
}

/**
  Feedback notification when SoftAP is started
*/
void wifiCallback (WiFiManager *wifiMgr) {
  String strMsg = "Connect to ";
  strMsg += wifiMgr->getConfigPortalSSID();
  Serial.println(strMsg);
}

void setup() {
  // Init the serial com
  Serial.println();
  Serial.begin(115200);
  Serial.print(NODENAME);
  Serial.print(" ");
  Serial.println(__DATE__);

  // Try to connect to WiFi
  WiFiManager wifiManager;
  // Reset settings
  //wifiManager.resetSettings();
  wifiManager.setTimeout(300);
  wifiManager.setAPCallback(wifiCallback);
  wifiManager.autoConnect(NODENAME.c_str());
  while (!wifiManager.autoConnect(NODENAME.c_str())) {
    delay(1000);
  }

  // Connected
  showWiFi();

  // Start the NTP client
  NTP_Client.begin();

  // Start the MQTT client
  MQTT_Client.setServer(MQTT_SERVER, MQTT_PORT);
  MQTT_Client.setCallback(mqttCallback);
  delayMQTT.start(MQTT_INTERVAL, AsyncDelay::MILLIS);

  // For I2C, the ESP8266-1 module uses the pin 0 for SDA and 2 for SCL
#if defined(ARDUINO_ARCH_ESP8266)
  Wire.pins(SDA, SCL);
#endif

  // BME280
  atmo.settings.commInterface = I2C_MODE;
  atmo.settings.I2CAddress = 0x77;
  atmo.settings.runMode = 3;
  atmo.settings.tStandby = 0;
  atmo.settings.filter = 0;
  atmo.settings.tempOverSample = 1;
  atmo.settings.pressOverSample = 1;
  atmo.settings.humidOverSample = 1;
  Serial.print("Starting BME280... result of .begin(): 0x");
  delay(10);
  Serial.println(atmo.begin(), HEX);
  Serial.println("BME280 ID, reset and ctrl regs");
  Serial.print("  ID(0xD0): 0x");
  Serial.println(atmo.readRegister(BME280_CHIP_ID_REG), HEX);
  if (atmo.readRegister(BME280_CHIP_ID_REG) == 0x60) {
    atmo_ok = true;
    Serial.println("BME280 sensor detected.");
  }
  else {
    atmo_ok = false;
    Serial.println("BME280 sensor missing.");
  }
  Serial.println();

  // TSL2561
  light.begin();
  light.setTiming(gain, shtr, ms);
  light.setPowerUp();

  // Sensor timer
  delaySNS.start(SNS_INTERVAL, AsyncDelay::MILLIS);
}

void loop() {
  // Process incoming MQTT messages and maintain connection
  if (!MQTT_Client.loop()) {
    // Not connected, try to reconnect every MQTT_INTERVAL seconds
    if (delayMQTT.isExpired()) {
      mqttReconnect();
      delayMQTT.repeat();
    }
  }

  // Read the sensors and publish telemetry
  if (delaySNS.isExpired()) {
    char text[10] = "";

    // Read BME280
    if (atmo_ok) {
      float temp = atmo.readTempC();
      float pres = atmo.readFloatPressure();
      float hmdt = atmo.readFloatHumidity();
      float dewp = 243.04 * (log(hmdt / 100.0) + ((17.625 * temp) / (243.04 + temp))) / (17.625 - log(hmdt / 100.0) - ((17.625 * temp) / (243.04 + temp)));

      dtostrf(temp, 7, 2, text);
      MQTT_Client.publish(String(MQTT_SENSOR + "/temperature").c_str(), text);
      dtostrf(hmdt, 7, 2, text);
      MQTT_Client.publish(String(MQTT_SENSOR + "/humidity").c_str(), text);
      dtostrf(dewp, 7, 2, text);
      MQTT_Client.publish(String(MQTT_SENSOR + "/dewpoint").c_str(), text);
      dtostrf(pres, 7, 2, text);
      MQTT_Client.publish(String(MQTT_SENSOR + "/pressure").c_str(), text);
    }

    // Read TSL2561
    unsigned int data0, data1;
    if (light.getData(data0, data1)) {
      double lux;
      boolean good = light.getLux(gain, ms, data0, data1, lux);
      if (good) {
        dtostrf(lux, 7, 2, text);
        MQTT_Client.publish(String(MQTT_SENSOR + "/illuminance").c_str(), text);
        dtostrf((float)data0, 7, 2, text);
        MQTT_Client.publish(String(MQTT_SENSOR + "/visible").c_str(), text);
        dtostrf((float)data1, 7, 2, text);
        MQTT_Client.publish(String(MQTT_SENSOR + "/infrared").c_str(), text);
      }
    }

    // Various telemetry
    MQTT_Client.publish(String(MQTT_REPORT_WIFI + "/rssi").c_str(), String(WiFi.RSSI()).c_str());
    MQTT_Client.publish(String(MQTT_REPORT + "/uptime").c_str(), String(millis() / 1000).c_str());
    MQTT_Client.publish(String(MQTT_REPORT + "/heap").c_str(), String(ESP.getFreeHeap()).c_str());
    int vcc = ESP.getVcc();
    MQTT_Client.publish(String(MQTT_REPORT + "/vcc").c_str(), String(String(vcc / 1000) + "." + String(vcc % 1000)).c_str());
    // Repeat
    delaySNS.repeat();
  }

  // NTP
  NTP_Client.update();
}
