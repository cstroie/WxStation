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

// Device name
#ifdef DEVEL
String NODENAME = "DevNode";
String LC_NODENAME = "devnode";
String VERSION = "0.3";
#else
String NODENAME = "WxSta";
String LC_NODENAME = "wxsta";
String VERSION = "0.3.3";
#endif

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
String APRS_TARGET = ">APRS,TCPIP*:";
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
int rgCnt = 0;
float rgY[rgMax];
//float rgY[] = {1974, 1828, 1709, 1639, 1526, 1488, 1442, 1393, 1361, 1354, 1355, 1316, 1250, 1208, 1245, 1242, 1250, 1328, 1370, 1335, 1298, 1210, 1167, 1125, 1120, 1153, 1271, 1156, 1068, 988, 917, 865, 836, 801, 790, 82};
float rgAB[] = {0, 0, 0};

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
int zThreshold = 100;
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
    String text;
    text.reserve(250);
    text  = "\nWiFi connected to ";
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

/** MQTT publishing wrapper, using strings, with retain flag on

  @param topic the MQTT topic
  @param payload the MQTT message to send to topic
  @return the publishig status
*/
boolean mqttPubRetain(const String &topic, const String &payload) {
  yield();
  return MQTT_Client.publish(topic.c_str(), payload.c_str(), true);
}

/** MQTT publishing wrapper, using strings

  @param topic the MQTT topic
  @param payload the MQTT message to send to topic
  @return the publishig status
*/
boolean mqttPub(const String &topic, const String &payload) {
  yield();
  return MQTT_Client.publish(topic.c_str(), payload.c_str(), false);
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
    mqttPubRetain(MQTT_REPORT_WIFI + "/hostname", WiFi.hostname());
    mqttPubRetain(MQTT_REPORT_WIFI + "/mac",      WiFi.macAddress());
    mqttPubRetain(MQTT_REPORT_WIFI + "/ssid",     WiFi.SSID());
    mqttPubRetain(MQTT_REPORT_WIFI + "/rssi",     String(WiFi.RSSI()));
    mqttPubRetain(MQTT_REPORT_WIFI + "/ip",       WiFi.localIP().toString());
    mqttPubRetain(MQTT_REPORT_WIFI + "/gw",       WiFi.gatewayIP().toString());
    // Subscribe
    MQTT_Client.subscribe((MQTT_CMD + "/#").c_str());
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
  String text;
  text.reserve(50);
  text  = "Connect to ";
  text += wifiMgr->getConfigPortalSSID();
  Serial.println(text);
}

/**
  Convert to integer and pad with "0"

  @param x the number to pad
  @param digits number of characters
*/
String zeroPad(float x, int digits) {
  long y = (long)x;
  String result;
  result.reserve(20);
  result = y;
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

void aprsDebug(const String &pkt) {
  String text;
  text.reserve(200);
  text  = "APRS: ";
  text += pkt;
  Serial.println(text);
}

void aprsSend(const String &pkt) {
  APRS_Client.println(pkt);
  yield();
#ifdef DEBUG
  Serial.print("APRS: ");
  Serial.println(pkt);
#endif
}

/**
  Return time in APRS format: DDHHMMz
*/
String aprsTime() {
  time_t moment = now();
  String result;
  result.reserve(20);
  result  = zeroPad(day(moment), 2);
  result += zeroPad(hour(moment), 2);
  result += zeroPad(minute(moment), 2);
  result += "z";
  return result;
}

/**
  Send APRS authentication data
  user FW0690 pass -1 vers WxSta 0.2"

*/
void aprsAuthenticate() {
  String pkt;
  pkt.reserve(200);
  pkt  = "user ";
  pkt += APRS_CALLSIGN;
  pkt += " pass ";
  pkt += APRS_PASSCODE;
  pkt += " vers ";
  pkt += NODENAME;
  pkt += " ";
  pkt += VERSION;
  aprsSend(pkt);
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
  String pkt;
  pkt.reserve(200);
  pkt  = APRS_CALLSIGN;
  pkt += APRS_TARGET;
  pkt += "@";
  pkt += aprsTime();
  pkt += APRS_LOCATION;
  // Wind (unavailable)
  pkt += "_.../...g...";
  // Temperature
  pkt += "t";
  if (temp >= -273.15) pkt += zeroPad(fahr, 3);
  else                 pkt += "...";
  // Humidity
  if (hmdt >= 0) {
    pkt += "h";
    if (hmdt == 100) pkt += "00";
    else             pkt += zeroPad(hmdt, 2);
  }
  // Athmospheric pressure
  if (pres >= 0) {
    pkt += "b";
    pkt += zeroPad(pres / 10, 5);
  }
  // Illuminance, if valid
  // For the SUN, there is an approximate conversion of 0.0079 W/m2 per Lux
  if (lux >= 0) {
    pkt += "L";
    pkt += zeroPad(lux * 0.0079, 3);
  }
  // Comment (device name)
  pkt += NODENAME;
  // Send the packet
  aprsSend(pkt);
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
  // Send the telemetry setup on power up (5 minutes) or if the sequence number is 0
  if ((aprsSeq == 0) or (millis() < 300000UL)) aprsSendTelemetrySetup();
  // Compose the APRS packet
  String pkt;
  pkt.reserve(200);
  pkt  = APRS_CALLSIGN;
  pkt += APRS_TARGET;
  pkt += "T#";
  pkt += zeroPad(aprsSeq, 3);
  pkt += ",";
  pkt += zeroPad((vcc - 2500) / 4, 3);
  pkt += ",";
  pkt += zeroPad(-rssi, 3);
  pkt += ",";
  pkt += zeroPad(heap / 200, 3);
  pkt += ",";
  pkt += zeroPad(luxVis / 256, 3);
  pkt += ",";
  pkt += zeroPad(luxIrd / 256, 3);
  pkt += ",";
  pkt += String(bits, BIN);
  // Send the packet
  aprsSend(pkt);
}

/**
  Send APRS telemetry setup
*/
void aprsSendTelemetrySetup() {
  // Space padding
  char padCallSign[10];
  sprintf(padCallSign, "%-9s", APRS_CALLSIGN.c_str());
  // The packet header
  String pktHeader;
  pktHeader.reserve(50);
  pktHeader  = APRS_CALLSIGN;
  pktHeader += APRS_TARGET;
  pktHeader += ":";
  pktHeader += padCallSign;
  // Compose the APRS packet
  String pkt;
  pkt.reserve(200);
  // Parameter names
  pkt  = pktHeader;
  pkt += ":PARM.Vcc,RSSI,Heap,IRed,Visb,PROBE,ATMO,LUX,SAT,BAT,B6,B7,B8";
  aprsSend(pkt);
  // Equtions
  pkt  = pktHeader;
  pkt += ":EQNS.0,0.004,2.5,0,-1,0,0,200,0,0,256,0,0,256,0";
  aprsSend(pkt);
  // Units
  pkt  = pktHeader;
  pkt += ":UNIT.V,dBm,Bytes,units,units,prb,on,on,sat,low,N/A,N/A,N/A";
  aprsSend(pkt);
  // Bit sense and project name
  pkt  = pktHeader;
  pkt += ":BITS.11111111,";
  pkt += NODENAME;
  pkt += "/";
  pkt += VERSION;
  aprsSend(pkt);
}

/**
  Send APRS status
  FW0690>APRS,TCPIP*:>Fine weather

  @param message the status message to send
*/
void aprsSendStatus(const String &message) {
  // Send only if the message is not empty
  if (message != "") {
    // Compose the APRS packet
    String pkt;
    pkt.reserve(200);
    pkt  = APRS_CALLSIGN;
    pkt += APRS_TARGET;
    pkt += ">";
    pkt += message;
    // Send the packet
    aprsSend(pkt);
  }
}

/**
  Send APRS position and alitude
  FW0690>APRS,TCPIP*:!DDMM.hhN/DDDMM.hhW$comments

  @param comment the comment to append
*/
void aprsSendPosition(const String &comment) {
  // Compose the APRS packet
  String pkt;
  pkt.reserve(200);
  pkt  = APRS_CALLSIGN;
  pkt += APRS_TARGET;
  pkt += "!";
  pkt += APRS_LOCATION;
  pkt += "/000/000/A=";
  pkt += zeroPad(ALTIF, 6);
  pkt += comment;
  // Send the packet
  aprsSend(pkt);
}

/**
  Store previous athmospheric pressures for each 5 minute in the last 3 hours
  @param y current value
*/
void rgAdd(float y) {
  rgY[rgIdx] = y;
  rgIdx++;
  if (rgIdx >= rgMax) rgIdx = 0; // wrap around
  if (rgCnt < rgMax) rgCnt++;    // count the stored values
}

/**
  Determine the linear regression
*/
void rgLinear() {
  int i, iy = 0;
  float denom, dy, x, y;
  float a1, a2, s, s1, s2, s3, s4;
  // Circular buffer, moving cursor
  if (rgCnt >= rgMax) iy = rgIdx;
  // Compute the sums
  s1 = s2 = s3 = s4 = s = 0;
  for (i = 0; i < rgCnt; i++) {
    int j = i + iy;
    if (j >= rgMax) j -= rgMax;
    x = i;
    y = rgY[j];
    s1 += x;
    s2 += x * x;
    s3 += y;
    s4 += x * y;
  }
  // Find alpha, beta and std deviation
  if ((denom = rgCnt * s2 - s1 * s1)) {
    a1 = (s3 * s2 - s1 * s4) / denom;
    a2 = (rgCnt * s4 - s3 * s1) / denom;
    for (i = 0; i < rgCnt; i++) {
      int j = i + iy;
      if (j >= rgMax) j -= rgMax;
      dy = rgY[j] - (a2 * i + a1);
      s += dy * dy;
    }
    rgAB[0] = a2;
    rgAB[1] = a1;
    rgAB[2] = sqrt(s / (rgCnt - 1));
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
  // Keep the current value in the circular buffer
  rgAdd(zCurrent);
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
      float pres  = zCurrent;
      // Get the trend
      if      ((zCurrent > zPrevious[zPrevCount - 1] ) and (zCurrent - zPrevious[zPrevCount - 1] > zThreshold)) trend =  1;
      else if ((zCurrent < zPrevious[zPrevCount - 1] ) and (zPrevious[zPrevCount - 1] - zCurrent > zThreshold)) trend = -1;
      // Corrections for summer
      int mon = month();
      if ((mon >= 4) and (mon <= 9)) {
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
      // Compute the linear regression
      rgLinear();
      Serial.println(rgAB[0]);
      Serial.println(rgAB[1]);
      Serial.println(rgAB[2]);
      result = result + " (" + String(rgAB[0], 6) + ")";
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
  atmo_ok = atmo.begin() == 0x60;
  if (atmo_ok) Serial.println("BME280 sensor detected.");
  else         Serial.println("BME280 sensor missing.");

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
    // Count to check if we need to send the APRS data
    if (APRS_SNS_COUNT++ > APRS_SNS_MAX) APRS_SNS_COUNT = 1;
    // Set the telemetry bits
#ifdef DEVEL
    int APRS_BITS = B10000000;
#else
    int APRS_BITS = B00000000;
#endif

    // Read TSL2561
    // TODO Adapt the shutter
    unsigned int luxVis = 0, luxIrd = 0;
    double lux = -1;
    if (!light_ok) {
      // Try to reinitialize the sensor
      light.begin();
      unsigned char ID;
      if (light.getID(ID)) {
        light.setTiming(gain, shtr, ms);
        light.setPowerUp();
        light_ok = true;
      }
    }
    if (light_ok) {
      // Set the bit to show the sensor is present
      APRS_BITS |= B00100000;
      if (light.getData(luxVis, luxIrd)) {
        boolean good = light.getLux(gain, ms, luxVis, luxIrd, lux);
        if (good) {
          // Send to MQTT
          mqttPub(MQTT_SENSOR + "/illuminance", String(lux, 2));
          mqttPub(MQTT_SENSOR + "/visible",     String(luxVis));
          mqttPub(MQTT_SENSOR + "/infrared",    String(luxIrd));
        }
        else {
          lux = -1;
          // Set the bit to show the sensor is saturated
          APRS_BITS |= B00010000;
        }
      }
    }
    // Running Median
    rmLux.add(lux);
    rmVisi.add(luxVis);
    rmIRed.add(luxIrd);
    yield();

    // Read BME280
    float temp, pres, slvl, hmdt, dewp;
    if (!atmo_ok) {
      // Try to reinitialize the sensor
      atmo_ok = atmo.begin() == 0x60;
    }
    if (atmo_ok) {
      // Set the bit to show the sensor is present
      APRS_BITS |= B01000000;
      // Get the weather parameters
      temp = atmo.readTempC();
      pres = atmo.readFloatPressure();
      slvl = pres * ALTI_CORR;
      hmdt = atmo.readFloatHumidity();
      dewp = 243.04 *
             (log(hmdt / 100.0) + ((17.625 * temp) / (243.04 + temp))) /
             (17.625 - log(hmdt / 100.0) - ((17.625 * temp) / (243.04 + temp)));
      // Running Median
      rmTemp.add(temp);
      rmHmdt.add(hmdt);
      rmPres.add(slvl);
      // Send to MQTT
      mqttPub(MQTT_SENSOR + "/temperature", String(temp, 2));
      mqttPub(MQTT_SENSOR + "/humidity",    String(hmdt, 2));
      mqttPub(MQTT_SENSOR + "/dewpoint",    String(dewp, 2));
      mqttPub(MQTT_SENSOR + "/pressure",    String(pres / 100, 2));
      mqttPub(MQTT_SENSOR + "/sealevel",    String(slvl / 100, 2));
    }

    // Various telemetry
    int rssi = WiFi.RSSI();
    int heap = ESP.getFreeHeap();
    int vcc  = ESP.getVcc();
    if (vcc < 3000) {
      // Set the bit to show the battery is low
      APRS_BITS |= B00001000;
    }
    // Running Median
    rmVcc.add(vcc);
    rmRSSI.add(rssi);
    rmHeap.add(heap);
    // Send to MQTT
    mqttPub(MQTT_REPORT_WIFI + "/rssi",   String(rssi));
    mqttPub(MQTT_REPORT + "/uptime",      String(millis() / 1000));
    mqttPub(MQTT_REPORT + "/uptime/text", NTP.getUptimeString());
    mqttPub(MQTT_REPORT + "/heap",        String(heap));
    mqttPub(MQTT_REPORT + "/vcc",         String((float)vcc / 1000, 3));

    // APRS (after the first minute, then every APRS_SNS_MAX minutes)
    if (APRS_SNS_COUNT == 1) {
      if (APRS_Client.connect(APRS_SERVER.c_str(), APRS_PORT)) {
        aprsAuthenticate();
        //aprsSendPosition(" WxStaProbe");
        if (atmo_ok) aprsSendWeather(rmTemp.getMedian(), rmHmdt.getMedian(), rmPres.getMedian(), rmLux.getMedian());
        //aprsSendWeather(16.3, 61.3, 102456, -1);
        aprsSendTelemetry(rmVcc.getMedian(), rmRSSI.getMedian(), rmHeap.getMedian(), rmVisi.getMedian(), rmIRed.getMedian(), APRS_BITS);
        //aprsSendStatus("Fine weather");
        //aprsSendTelemetrySetup();
        APRS_Client.stop();
      };
    }

    // Repeat sensor reading
    delaySNS.repeat();
  }
}
