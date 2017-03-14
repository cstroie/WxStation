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

// The DEBUG and DEVEL flag
#define DEBUG
#define DEVEL

// The sensors are connected to I2C
#define SDA 0
#define SCL 2
#include <Wire.h>
#include <SparkFunBME280.h>
#include <SparkFunTSL2561.h>


// WiFi
#include <ESP8266WiFi.h>
#include <WiFiManager.h>

// NTP
#include <NtpClientLib.h>
#include <TimeLib.h>

// Timer
#include <AsyncDelay.h>

// MQTT
#include <PubSubClient.h>


// Device name
#ifdef DEVEL
String NODENAME = "DevNode";
String LC_NODENAME = "devnode";
String VERSION = "0.1";
#else
String NODENAME = "WxSta";
String LC_NODENAME = "wxsta";  // FIXME DNRY
String VERSION = "0.3.1";
#endif

// Altitude
const int ALTI = 83; // meters
double ALTI_CORR = pow((double)(1.0 - 2.25577e-5 * ALTI), (double)(-5.25588));

// NTP
const int  TZ = 2;
String NTP_SERVER = "europe.pool.ntp.org";

// MQTT parameters
#ifdef DEVEL
String MQTT_ID = "devnode-eridu-eu-org";
#else
String MQTT_ID = "wxsta-eridu-eu-org";
#endif
String MQTT_SERVER = "eridu.eu.org";
int MQTT_PORT = 1883;
int MQTT_INTERVAL = 5000;
String MQTT_CMD  = "command/" + LC_NODENAME;
String MQTT_REPORT = "report/" + LC_NODENAME;
String MQTT_REPORT_WIFI = MQTT_REPORT + "/wifi";
String MQTT_SENSOR = "sensor/outdoor"; // + LC_NODENAME;
WiFiClient WiFi_Client;
PubSubClient MQTT_Client(WiFi_Client);
AsyncDelay delayMQTT;

// APRS parameters
String APRS_SERVER = "cwop5.aprs.net";
const int  APRS_PORT = 14580;
String APRS_CALLSIGN = "FW0690";
const int  APRS_PASSCODE = -1;
String APRS_LOCATION = "4427.67N/02608.03E";
const int  APRS_SNS_MAX = 5; // x SNS_INTERVAL
int APRS_SNS_COUNT = 0;
unsigned int aprsSeq = 0;
WiFiClient APRS_Client;

// Sensors
const int SNS_INTERVAL  = 60 * 1000;
BME280 atmo;
bool atmo_ok = false;
SFE_TSL2561 light;
bool light_ok = false;
boolean gain = false;
unsigned char shtr = 2;
unsigned int ms;
AsyncDelay delaySNS;

// Voltage
ADC_MODE(ADC_VCC);

// Zambretti forecaster
int zBaroTop  = 105000;
int zBaroBot  = 95000;
int zThreshold = 50;
int zDelay = 3600;
unsigned long zNextTime = 0;
float zPrevious = 0;

String zForecast[] = {"Settled fine", "Fine weather", "Becoming fine", "Fine, becoming less settled", "Fine, possible showers",
                      "Fairly fine, improving", "Fairly fine, possible showers early", "Fairly fine, showery later",
                      "Showery early, improving", "Changeable, mending", "Fairly fine, showers likely",
                      "Rather unsettled clearing later", "Unsettled, probably improving", "Showery, bright intervals",
                      "Showery, becoming less settled", "Changeable, some rain", "Unsettled, short fine intervals",
                      "Unsettled, rain later", "Unsettled, some rain", "Mostly very unsettled", "Occasional rain, worsening",
                      "Rain at times, very unsettled", "Rain at frequent intervals", "Rain, very unsettled",
                      "Stormy, may improve", "Stormy, much rain"
                     };
int zRising[]  = {25, 25, 25, 24, 24, 19, 16, 12, 11, 9, 8, 6, 5, 2, 1, 1, 0, 0, 0, 0, 0, 0};
int zSteady[]  = {25, 25, 25, 25, 25, 25, 23, 23, 22, 18, 15, 13, 10, 4, 1, 1, 0, 0, 0, 0, 0, 0};
int zFalling[] = {25, 25, 25, 25, 25, 25, 25, 25, 23, 23, 21, 20, 17, 14, 7, 3, 1, 1, 1, 0, 0, 0};

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
  if (MQTT_Client.connect(MQTT_ID.c_str())) {
    // Publish the connection report
    MQTT_Client.publish(String(MQTT_REPORT_WIFI + "/hostname").c_str(), WiFi.hostname().c_str(), true);
    MQTT_Client.publish(String(MQTT_REPORT_WIFI + "/mac").c_str(), WiFi.macAddress().c_str(), true);
    MQTT_Client.publish(String(MQTT_REPORT_WIFI + "/ssid").c_str(), WiFi.SSID().c_str(), true);
    MQTT_Client.publish(String(MQTT_REPORT_WIFI + "/rssi").c_str(), String(WiFi.RSSI()).c_str(), true);
    MQTT_Client.publish(String(MQTT_REPORT_WIFI + "/ip").c_str(), WiFi.localIP().toString().c_str(), true);
    MQTT_Client.publish(String(MQTT_REPORT_WIFI + "/gw").c_str(), WiFi.gatewayIP().toString().c_str(), true);
    // Subscribe
    MQTT_Client.subscribe(String(MQTT_CMD + "/#").c_str());
    // TODO
    //MQTT_Client.subscribe("sensor/#");
#ifdef DEBUG
    Serial.print(F("MQTT connected to "));
    Serial.println(MQTT_SERVER);
#endif
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
#ifdef DEBUG
  Serial.println("MQTT " + strTopic + ": " + strMessage);
#endif

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

/**
  Return time in APRS format: DDHHMMz

*/
String aprsTime() {
  time_t moment = now();
  int days = day(moment);
  String dayStr = days < 10 ? "0" + String(days) : String(days);
  int hours = hour(moment);
  String hoursStr = hours < 10 ? "0" + String(hours) : String(hours);
  int minutes = minute(moment);
  String minuteStr = minutes < 10 ? "0" + String(minutes) : String(minutes);
  return dayStr + hoursStr + minuteStr + "z";
}

/**
  Send APRS authentication data
  user FW0690 pass -1 vers WxSta 0.2"

*/
void aprsAuthenticate() {
  String pkt = "user " + APRS_CALLSIGN + " pass " + APRS_PASSCODE + " vers " + NODENAME + " " + VERSION;
  APRS_Client.println(pkt);
#ifdef DEBUG
  Serial.println("APRS: " + pkt);
#endif
}

/**
  Send APRS weather data
  FW0690>APRS,TCPIP*:@152457h4427.67N/02608.03E_.../...g...t044h86b10201L001WxSta

*/
void aprsSendWeather(float temp, float hmdt, float pres, float lux) {
  // Temperature will be in Fahrenheit
  float fahr = temp * 9 / 5 + 32;
  // Compose the APRS packet
  String pkt = APRS_CALLSIGN;
  pkt = pkt + ">APRS,TCPIP*:";
  pkt = pkt + "@" + aprsTime();
  pkt = pkt + APRS_LOCATION;
  // Wind
  pkt = pkt + "_.../...g...";
  // Temperature
  if (temp >= -273.15) {
    char txtTemp[4] = "";
    sprintf(txtTemp, "%03d", (int)fahr);
    pkt = pkt + "t" + txtTemp;
  }
  else {
    pkt = pkt + "t...";
  }
  // Humidity
  if (hmdt >= 0) {
    if (hmdt == 100) {
      pkt = pkt + "h00";
    }
    else {
      char txtHmdt[3] = "";
      sprintf(txtHmdt, "%02d", (int)hmdt);
      pkt = pkt + "h" + txtHmdt;
    }
  }
  // Athmospheric pressure
  if (pres >= 0) {
    char txtPres[6] = "";
    sprintf(txtPres, "%05d", (int)(pres / 10));
    pkt = pkt + "b" + txtPres;
  }
  // Illuminance, if valid
  if (lux >= 0) {
    char txtLux[4] = "";
    sprintf(txtLux, "%03d", (int)(lux * 0.0079));
    pkt = pkt + "L" + txtLux;
  }
  // Comment (device name)
  pkt = pkt + NODENAME;
  // Send the packet
  APRS_Client.println(pkt);
#ifdef DEBUG
  Serial.println("APRS: " + pkt);
#endif
}

/**
  Send APRS telemetry
  FW0690>APRS,TCPIP*:T#517,173,062,213,002,000,00000000

*/
void aprsSendTelemetry(int vcc, int rssi, int heap, unsigned int luxVis, unsigned int luxIrd, byte bits) {
  // Increment the telemetry sequence, reset it if exceeds 999
  aprsSeq += 1;
  if (aprsSeq > 999) {
    aprsSeq = 0;
  }
  // Send the setup, if the sequence number is 0
  if (aprsSeq == 0) {
    aprsSendTelemetrySetup();
  }

  // Compose the APRS packet
  String pkt = APRS_CALLSIGN;
  pkt = pkt + ">APRS,TCPIP*:";
  char text[27] = "";
  sprintf(text, "T#%03d,%03d,%03d,%03d,%03d,%03d,", aprsSeq, (vcc - 2500) / 4, -rssi, heap / 200, luxVis / 256, luxIrd / 256);
  pkt = pkt + text + String(bits, BIN);
  // Send the packet
  APRS_Client.println(pkt);
#ifdef DEBUG
  Serial.println("APRS: " + pkt);
#endif
}

/**
  Send APRS telemetry setup

*/
void aprsSendTelemetrySetup() {
  String pkt;
  String pad = String("         ").substring(APRS_CALLSIGN.length());
  // Parameter names
  pkt = APRS_CALLSIGN + ">APRS,TCPIP*::" + APRS_CALLSIGN + pad + ":PARM.Vcc,RSSI,Heap,IRed,Vis,B1,B2,B3,B4,B5,B6,B7,B8";
  APRS_Client.println(pkt);
#ifdef DEBUG
  Serial.println("APRS: " + pkt);
#endif
  // Equtions
  pkt = APRS_CALLSIGN + ">APRS,TCPIP*::" + APRS_CALLSIGN + pad + ":EQNS.0,0.004,2.5,0,-1,0,0,200,0,0,256,0,0,256,0";
  APRS_Client.println(pkt);
#ifdef DEBUG
  Serial.println("APRS: " + pkt);
#endif
  // Units
  pkt = APRS_CALLSIGN + ">APRS,TCPIP*::" + APRS_CALLSIGN + pad + ":UNIT.V,dBm,Bytes,units,units,N/A,N/A,N/A,N/A,N/A,N/A,N/A,N/A";
  APRS_Client.println(pkt);
#ifdef DEBUG
  Serial.println("APRS: " + pkt);
#endif
  // Bit sense and project name
  pkt = APRS_CALLSIGN + ">APRS,TCPIP*::" + APRS_CALLSIGN + pad + ":BITS.11111111," + NODENAME + "/" + VERSION;
  APRS_Client.println(pkt);
#ifdef DEBUG
  Serial.println("APRS: " + pkt);
#endif
}

/**
  Send APRS status
  FW0690>APRS,TCPIP*:>13:06 Fine weather

*/
void aprsSendStatus(String message) {
  // Send only if the message is not empty
  if (message != "") {
    // Compose the APRS packet
    String pkt = APRS_CALLSIGN + ">APRS,TCPIP*:>" + NTP.getTimeStr() + " " + message;
    // Send the packet
    APRS_Client.println(pkt);
#ifdef DEBUG
    Serial.println("APRS: " + pkt);
#endif
  }
}

/**
  Get the Zambretti forecast

*/
String zambretti(float zCurrent) {
  String result = "";
  if (zNextTime == 0) {
    // First run
    zNextTime = millis() + zDelay * 1000;
    zPrevious = zCurrent;
  }
  else {
    if (millis() >= zNextTime) {
      // Timer expired
      int trend = 0;
      int index = 0;
      float range = zBaroTop - zBaroBot;
      float pres = zCurrent;
      // Get the trend
      if ((zCurrent > zPrevious ) and (zCurrent - zPrevious > zThreshold)) {
        trend = 1;
      }
      else if ((zCurrent < zPrevious ) and (zPrevious - zCurrent > zThreshold)) {
        trend = -1;
      }
      else {
        trend = 0;
      }
      // Check if summer
      if ((month() >= 4) and (month() <= 9)) {
        if (trend > 0) {
          pres += range * 0.07;
        }
        else if (trend < 0) {
          pres -= range * 0.07;
        }
      }
      // Validate interval
      if (pres > zBaroTop) {
        pres = zBaroTop - 2 * zThreshold;
      }
      // Get the forecast
      index = (int)(22 * (pres - zBaroBot) / range);
      if ((index >= 0) and (index <= 22)) {
        if (trend > 0) {
          result = zForecast[zRising[index]];
        }
        else if (trend < 0) {
          result = zForecast[zFalling[index]];
        }
        else {
          result = zForecast[zSteady[index]];
        }
      }
      // Set the next timer
      zNextTime = zNextTime + zDelay * 1000;
      zPrevious = zCurrent;
    }
  }
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

  // Start the NTP sync
  NTP.begin(NTP_SERVER, TZ, true);

  // Start the MQTT client
  MQTT_Client.setServer(MQTT_SERVER.c_str(), MQTT_PORT);
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
  delay(10);
  if (atmo.begin() == 0x60) {
    atmo_ok = true;
    Serial.println("BME280 sensor detected.");
  }
  else {
    atmo_ok = false;
    Serial.println("BME280 sensor missing.");
  }

  // TSL2561
  light.begin();
  unsigned char ID;
  if (light.getID(ID)) {
    light.setTiming(gain, shtr, ms);
    light.setPowerUp();
    light_ok = true;
    Serial.println("TSL2561 sensor detected.");
  }
  else {
    light_ok = false;
    Serial.println("TSL2561 sensor missing.");
  }

  // Initialize the random number generator and set the APRS telemetry start sequence
  if (timeStatus() != timeNotSet) {
    randomSeed(now());
  }
  aprsSeq = random(1000);

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
    // Check if we need to send the APRS data
    APRS_SNS_COUNT++;
    if (APRS_SNS_COUNT > APRS_SNS_MAX) {
      APRS_SNS_COUNT = 1;
    }

    // Read TSL2561
    unsigned int luxVis = 0, luxIrd = 0;
    double lux = -1;
    if (light.getData(luxVis, luxIrd)) {
      boolean good = light.getLux(gain, ms, luxVis, luxIrd, lux);
      if (good) {
        MQTT_Client.publish(String(MQTT_SENSOR + "/illuminance").c_str(), String(lux, 2).c_str());
        MQTT_Client.publish(String(MQTT_SENSOR + "/visible").c_str(), String(luxVis).c_str());
        MQTT_Client.publish(String(MQTT_SENSOR + "/infrared").c_str(), String(luxIrd).c_str());
      }
      else {
        lux = -1;
      }
    }

    // Read BME280
    float temp, pres, seal, hmdt, dewp;
    if (atmo_ok) {
      temp = atmo.readTempC();
      pres = atmo.readFloatPressure();
      seal = pres * ALTI_CORR;
      hmdt = atmo.readFloatHumidity();
      dewp = 243.04 * (log(hmdt / 100.0) + ((17.625 * temp) / (243.04 + temp))) / (17.625 - log(hmdt / 100.0) - ((17.625 * temp) / (243.04 + temp)));

      MQTT_Client.publish(String(MQTT_SENSOR + "/temperature").c_str(), String(temp, 2).c_str());
      MQTT_Client.publish(String(MQTT_SENSOR + "/humidity").c_str(), String(hmdt, 2).c_str());
      MQTT_Client.publish(String(MQTT_SENSOR + "/dewpoint").c_str(), String(dewp, 2).c_str());
      MQTT_Client.publish(String(MQTT_SENSOR + "/pressure").c_str(), String(pres / 100, 2).c_str());
      MQTT_Client.publish(String(MQTT_SENSOR + "/sealevel").c_str(), String(seal / 100, 2).c_str());
    }

    // Various telemetry
    int rssi = WiFi.RSSI();
    int heap = ESP.getFreeHeap();
    int vcc = ESP.getVcc();
    MQTT_Client.publish(String(MQTT_REPORT_WIFI + "/rssi").c_str(), String(rssi).c_str());
    MQTT_Client.publish(String(MQTT_REPORT + "/uptime").c_str(), String(millis() / 1000).c_str());
    MQTT_Client.publish(String(MQTT_REPORT + "/uptime/text").c_str(), NTP.getUptimeString().c_str());
    MQTT_Client.publish(String(MQTT_REPORT + "/heap").c_str(), String(heap).c_str());
    MQTT_Client.publish(String(MQTT_REPORT + "/vcc").c_str(), String((float)vcc / 1000, 3).c_str());

    // APRS (every 5 minutes)
    if (APRS_SNS_COUNT == 1) {
      if (APRS_Client.connect(APRS_SERVER.c_str(), APRS_PORT)) {
        aprsAuthenticate();
        if (atmo_ok) {
          aprsSendWeather(temp, hmdt, seal, lux);
          aprsSendStatus(zambretti(seal));
        }
        aprsSendTelemetry(vcc, rssi, heap, luxVis, luxIrd, 0);
        APRS_Client.stop();
      };
    }

    // Repeat sensor reading
    delaySNS.repeat();
  }
}
