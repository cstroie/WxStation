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

// Statistics
#include <RunningMedian.h>
#include "lnregr.c"


// Device name
#ifdef DEVEL
String NODENAME = "DevNode";
String VERSION = "0.1";
#else
String NODENAME = "WxSta";
String VERSION = "0.3.2";
#endif
String LC_NODENAME = NODENAME;


// NTP
const int TZ = 0;
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
const int APRS_PORT = 14580;
#ifdef DEVEL
String APRS_CALLSIGN = "FW0727";
const int  APRS_PASSCODE = -1;
String APRS_LOCATION = "4455.29N/02527.08E";
const int ALTI = 282; // meters
#else
String APRS_CALLSIGN = "FW0690";
const int  APRS_PASSCODE = -1;
String APRS_LOCATION = "4427.67N/02608.03E";
const int ALTI = 83; // meters
#endif
double ALTI_CORR = pow((double)(1.0 - 2.25577e-5 * ALTI), (double)(-5.25588));
float ALTIF = ALTI * 3.28084;
const int APRS_SNS_MAX = 5; // x SNS_INTERVAL
int APRS_SNS_COUNT = 0;
unsigned int aprsSeq = 0;
WiFiClient APRS_Client;

// Statistics
RunningMedian rmTemp = RunningMedian(APRS_SNS_MAX);
RunningMedian rmHmdt = RunningMedian(APRS_SNS_MAX);
RunningMedian rmPres = RunningMedian(APRS_SNS_MAX);
RunningMedian rmLux  = RunningMedian(APRS_SNS_MAX);
RunningMedian rmVisi = RunningMedian(APRS_SNS_MAX);
RunningMedian rmIRed = RunningMedian(APRS_SNS_MAX);
RunningMedian rmVcc  = RunningMedian(APRS_SNS_MAX);
RunningMedian rmRSSI = RunningMedian(APRS_SNS_MAX);
RunningMedian rmHeap = RunningMedian(APRS_SNS_MAX);

// Linear regression
const int rgMax = 36;
int rgIdx = 0;
float rgX[rgMax];
float rgY[rgMax];
//float rgY[] = {1974, 1828, 1709, 1639, 1526, 1488, 1442, 1393, 1361, 1354, 1355, 1316, 1250, 1208, 1245, 1242, 1250, 1328, 1370, 1335, 1298, 1210, 1167, 1125, 1120, 1153, 1271, 1156, 1068, 988, 917, 865, 836, 801, 790, 82};
//float rgX[] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35};
float rgAB[] = {0, 0};

// Sensors
const int SNS_INTERVAL  = 60 * 1000;
BME280 atmo;
bool atmo_ok = false;
SFE_TSL2561 light;
bool light_ok = false;
boolean gain = false;
unsigned char shtr = 1;
unsigned int ms;
AsyncDelay delaySNS;

// Voltage
ADC_MODE(ADC_VCC);

// Zambretti forecaster
int zBaroTop   = 105000;
int zBaroBot   = 95000;
int zThreshold = 50;
int zDelay = 3600;
unsigned long zNextTime = 0;
float zPrevious[] = {0, 0, 0}; // last 1, 2 and 3 hours
int zPrevCount = 0;

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
#ifdef DEBUG
  Serial.println(F("MQTT connecting..."));
#endif
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
  yield();
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
void wifiCallback(WiFiManager *wifiMgr) {
  String strMsg = "Connect to ";
  strMsg += wifiMgr->getConfigPortalSSID();
  Serial.println(strMsg);
}

/**
  Convert to integer and pad with "0"

  @param x the number to pad
  @param digits number of characters
*/
String zeroPad(float x, int digits) {
  long y = (long)x;
  String result = String(y);
  String pad = "000000000";
  if (result.length() < digits) {
    pad = pad.substring(0, digits - result.length());
    if (y < 0) {
      result = "-" + pad + String(-y);
    }
    else {
      result = pad + result;
    }
  }
  return result;
}

/**
  Return time in APRS format: DDHHMMz
*/
String aprsTime() {
  time_t moment = now();
  return zeroPad(day(moment), 2) + zeroPad(hour(moment), 2) + zeroPad(minute(moment), 2) + "z";
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
  Send APRS weather data, then try to get the forecast
  FW0690>APRS,TCPIP*:@152457h4427.67N/02608.03E_.../...g...t044h86b10201L001WxSta

  @param temp temperature
  @param hmdt humidity
  @param pres athmospheric pressure
  @param lux illuminance
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
    pkt = pkt + "t" + zeroPad(fahr, 3);
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
      pkt = pkt + "h" + zeroPad(hmdt, 2);
    }
  }
  // Athmospheric pressure
  if (pres >= 0) {
    pkt = pkt + "b" + zeroPad(pres / 10, 5);
  }
  // Illuminance, if valid
  if (lux >= 0) {
    pkt = pkt + "L" + zeroPad(lux * 0.0079, 3);
  }
  // Comment (device name)
  pkt = pkt + NODENAME + "/" + VERSION;
  // Send the packet
  APRS_Client.println(pkt);
#ifdef DEBUG
  Serial.println("APRS: " + pkt);
#endif
  // Try to get and send the Zambretti forecast
  aprsSendStatus(zambretti(pres));
}

/**
  Send APRS telemetry and, periodically, send the telemetry setup
  FW0690>APRS,TCPIP*:T#517,173,062,213,002,000,00000000

  @param vcc voltage
  @param rssi wifi level
  @param heap free memory
  @param luxVis raw visible illuminance
  @param luxIrd raw infrared illuminance
  @bits digital inputs
*/
void aprsSendTelemetry(int vcc, int rssi, int heap, unsigned int luxVis, unsigned int luxIrd, byte bits) {
  // Increment the telemetry sequence number, reset it if exceeds 999
  aprsSeq += 1;
  if (aprsSeq > 999) aprsSeq = 0;
  // Send the setup, if the sequence number is 0
  if (aprsSeq == 0) aprsSendTelemetrySetup();
  // Compose the APRS packet
  String pkt = APRS_CALLSIGN +
               ">APRS,TCPIP*:T#" +
               zeroPad(aprsSeq, 3) + "," +
               zeroPad((vcc - 2500) / 4, 3) + "," +
               zeroPad(-rssi, 3) + "," +
               zeroPad(heap / 200, 3) + "," +
               zeroPad(luxVis / 256, 3) + "," +
               zeroPad(luxIrd / 256, 3) + "," +
               String(bits, BIN);
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
  pkt = APRS_CALLSIGN + ">APRS,TCPIP*::" + APRS_CALLSIGN + pad + ":PARM.Vcc,RSSI,Heap,IRd,Vis,B1,B2,B3,B4,B5,B6,B7,B8";
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

  @param message the status message to send
*/
void aprsSendStatus(String message) {
  // Send only if the message is not empty
  if (message != "") {
    // Compose the APRS packet
    String pkt = APRS_CALLSIGN + ">APRS,TCPIP*:>" + message;
    // Send the packet
    APRS_Client.println(pkt);
#ifdef DEBUG
    Serial.println("APRS: " + pkt);
#endif
  }
}

/**
  Send APRS position and alitude
  FW0690>APRS,TCPIP*:!DDMM.hhN/DDDMM.hhW$comments

  @param comment the comment to append
*/
void aprsSendPosition(String comment) {
  // Compose the APRS packet
  String pkt = APRS_CALLSIGN + ">APRS,TCPIP*:!" + APRS_LOCATION + "/000/000/A=" + zeroPad(ALTIF, 6) + comment;
  // Send the packet
  APRS_Client.println(pkt);
#ifdef DEBUG
  Serial.println("APRS: " + pkt);
#endif
}

/**
  Store previous athmospheric pressures for each 5 minute in the last 3 hours
  @param x timestamp
  @param y current value
*/
void rgAdd(float x, float y) {
  if (rgIdx == rgMax) {
    int s = sizeof(rgX[0]);
    memmove(rgX, rgX + s, (rgIdx - 1) * s);
    memmove(rgY, rgY + s, (rgIdx - 1) * s);
    rgX[rgIdx - 1] = x;
    rgY[rgIdx - 1] = y;
  }
  else {
    rgX[rgIdx] = x;
    rgY[rgIdx] = y;
    rgIdx++;
  }
}

/**
  Store the previous athmospheric pressures for the last 1, 2 and 3 hours
  @param pres current value of the atmospheric pressure
*/
void zPrevPres(float pres) {
  zPrevious[2] = zPrevious[1];
  zPrevious[1] = zPrevious[0];
  zPrevious[0] = pres;
  if (zPrevCount < 3) zPrevCount++;
}

/**
  Get the Zambretti forecast
  @param zCurrent current value of the atmospheric pressure
  @return the Zambretti forecast
*/
String zambretti(float zCurrent) {
  rgAdd((int)(millis() / 60000), zCurrent);
  String result = "";
  if (zNextTime == 0) {
    // First run, set the timeout to the next hour
    zNextTime = millis() + (60 - minute()) * 60000;
    zPrevPres(zCurrent);
  }
  else {
    if (millis() >= zNextTime) {
      // Timer expired
      int trend = 0;
      int index = 0;
      float range = zBaroTop - zBaroBot;
      float pres = zCurrent;
      // Get the trend
      if      ((zCurrent > zPrevious[zPrevCount - 1] ) and (zCurrent - zPrevious[zPrevCount - 1] > zThreshold)) trend = 1;
      else if ((zCurrent < zPrevious[zPrevCount - 1] ) and (zPrevious[zPrevCount - 1] - zCurrent > zThreshold)) trend = -1;
      // Check if summer
      if ((month() >= 4) and (month() <= 9)) {
        if      (trend > 0) pres += range * 0.07;
        else if (trend < 0) pres -= range * 0.07;
      }
      // Validate the interval
      if (pres > zBaroTop) pres = zBaroTop - 2 * zThreshold;
      // Get the forecast
      index = (int)(22 * (pres - zBaroBot) / range);
      if ((index >= 0) and (index <= 22)) {
        if      (trend > 0) result = zForecast[zRising[index]];
        else if (trend < 0) result = zForecast[zFalling[index]];
        else                result = zForecast[zSteady[index]];
      }
      lnRegr(rgX, rgY, rgAB, rgIdx);
      Serial.println(rgAB[0]);
      Serial.println(rgAB[1]);
      result = result + " (" + String(rgAB[0], 2) + ")";
      // Set the next timer and store the pressure
      zNextTime += zDelay * 1000;
      zPrevPres(zCurrent);
    }
  }
  return result;
}

void setup() {
  // Init the serial com
  Serial.println();
  Serial.begin(115200);
  Serial.println(NODENAME + "/" + VERSION + " [" + APRS_CALLSIGN + "] " + __DATE__);
  // Get the lowercase name
  LC_NODENAME.toLowerCase();

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
  //NTP.begin(NTP_SERVER, TZ, true);
  NTP.begin(NTP_SERVER, TZ, false);
  yield();

  // Start the MQTT client
  MQTT_Client.setServer(MQTT_SERVER.c_str(), MQTT_PORT);
  MQTT_Client.setCallback(mqttCallback);
  delayMQTT.start(MQTT_INTERVAL, AsyncDelay::MILLIS);
  yield();

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
  yield();

  // Initialize the random number generator and set the APRS telemetry start sequence
  if (timeStatus() != timeNotSet) randomSeed(now());
  aprsSeq = random(1000);
  yield();

  // Start the sensor timer
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
  yield();

  // Read the sensors and publish telemetry
  if (delaySNS.isExpired()) {
    // Check if we need to send the APRS data
    APRS_SNS_COUNT++;
    if (APRS_SNS_COUNT > APRS_SNS_MAX) {
      APRS_SNS_COUNT = 1;
    }

    // Read TSL2561
    // TODO Adapt the shutter
    unsigned int luxVis = 0, luxIrd = 0;
    double lux = -1;
    if (light.getData(luxVis, luxIrd)) {
      boolean good = light.getLux(gain, ms, luxVis, luxIrd, lux);
      if (good) {
        // Send to MQTT
        MQTT_Client.publish(String(MQTT_SENSOR + "/illuminance").c_str(), String(lux, 2).c_str());
        MQTT_Client.publish(String(MQTT_SENSOR + "/visible").c_str(), String(luxVis).c_str());
        MQTT_Client.publish(String(MQTT_SENSOR + "/infrared").c_str(), String(luxIrd).c_str());
      }
      else {
        lux = -1;
      }
    }
    // Running Median
    rmLux.add(lux);
    rmVisi.add(luxVis);
    rmIRed.add(luxIrd);
    yield();

    // Read BME280
    // TODO fake values if DEVEL
    float temp, pres, slvl, hmdt, dewp;
    if (atmo_ok) {
      // Get the weather parameters
      temp = atmo.readTempC();
      pres = atmo.readFloatPressure();
      slvl = pres * ALTI_CORR;
      hmdt = atmo.readFloatHumidity();
      dewp = 243.04 * (log(hmdt / 100.0) + ((17.625 * temp) / (243.04 + temp))) / (17.625 - log(hmdt / 100.0) - ((17.625 * temp) / (243.04 + temp)));
      // Running Median
      rmTemp.add(temp);
      rmHmdt.add(hmdt);
      rmPres.add(slvl);
      // Send to MQTT
      MQTT_Client.publish(String(MQTT_SENSOR + "/temperature").c_str(), String(temp, 2).c_str());
      MQTT_Client.publish(String(MQTT_SENSOR + "/humidity").c_str(), String(hmdt, 2).c_str());
      MQTT_Client.publish(String(MQTT_SENSOR + "/dewpoint").c_str(), String(dewp, 2).c_str());
      MQTT_Client.publish(String(MQTT_SENSOR + "/pressure").c_str(), String(pres / 100, 2).c_str());
      MQTT_Client.publish(String(MQTT_SENSOR + "/sealevel").c_str(), String(slvl / 100, 2).c_str());
    }
    yield();

    // Various telemetry
    int rssi = WiFi.RSSI();
    int heap = ESP.getFreeHeap();
    int vcc = ESP.getVcc();
    // Running Median
    rmVcc.add(vcc);
    rmRSSI.add(rssi);
    rmHeap.add(heap);
    // Send to MQTT
    MQTT_Client.publish(String(MQTT_REPORT_WIFI + "/rssi").c_str(), String(rssi).c_str());
    MQTT_Client.publish(String(MQTT_REPORT + "/uptime").c_str(), String(millis() / 1000).c_str());
    MQTT_Client.publish(String(MQTT_REPORT + "/uptime/text").c_str(), NTP.getUptimeString().c_str());
    MQTT_Client.publish(String(MQTT_REPORT + "/heap").c_str(), String(heap).c_str());
    MQTT_Client.publish(String(MQTT_REPORT + "/vcc").c_str(), String((float)vcc / 1000, 3).c_str());
    yield();

    // APRS (after the first minute, then every APRS_SNS_MAX minutes)
    if (APRS_SNS_COUNT == 1) {
      if (APRS_Client.connect(APRS_SERVER.c_str(), APRS_PORT)) {
        yield();
        aprsAuthenticate();
        //aprsSendPosition(" WxStaProbe");
        if (atmo_ok) {
          aprsSendWeather(rmTemp.getMedian(), rmHmdt.getMedian(), rmPres.getMedian(), rmLux.getMedian());
        }
        aprsSendTelemetry(rmVcc.getMedian(), rmRSSI.getMedian(), rmHeap.getMedian(), rmVisi.getMedian(), rmIRed.getMedian(), 0);
        yield();
        APRS_Client.stop();
      };
    }

    // Repeat sensor reading
    delaySNS.repeat();
  }
}
