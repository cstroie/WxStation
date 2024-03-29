/**
  WxSta - Weather probe based on ESP8266-1, WiFi connected

  Copyright (c) 2017-2021 Costin STROIE <costinstroie@eridu.eu.org>

  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, orl
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

// The DEBUG flag
#define DEBUG

// Select the light sensor
//#define BHLIGHT

// User settings
#include "config.h"

/** config.h should contain

  // Development
  #define DEVEL

  // APRS
  #define APRS_CALLSIGN "NOCALL"
  #define APRS_PASSCODE "-1"
  #define APRS_LAT      "ddmm.mmN"
  #define APRS_LON      "dddmm.mmE"
  #define APRS_ALTITUDE m

  // Weather Underground
  #define WU_ID   "WU ID"
  #define WU_PASS "WU PASS"

  // Secure connections
  #define USE_SSL
  #define USE_MQTT_SSL

  // MQTT
  #define MQTT_SERVER   "mqtt.server.exmaple"
  #define MQTT_ID       "your-mqtt-id"

  // NTP
  #define NTP_SERVER    "pool.ntp.org"
*/

// The sensors are connected to I2C, here we map the pins
#define SDA 0
#define SCL 2

// Include the sensors libraries
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#ifdef BHLIGHT
#include <BH1750.h>
#else
#include <SparkFunTSL2561.h>
#endif

// WiFi
#include <ESP8266WiFi.h>
#ifdef USE_SSL
#include <WiFiClientSecure.h>
#else
#include <WiFiClient.h>
#endif
#include <WiFiManager.h>
#include <WiFiUdp.h>
#include <ESP8266mDNS.h>
#include <ArduinoOTA.h>

// MQTT
#include <PubSubClient.h>

// Device name
#ifdef DEVEL
const char NODENAME[] = "WxSta-DEV";
const char nodename[] = "wxsta-dev";
#else
const char NODENAME[] = "WxSta";
const char nodename[] = "wxsta";
#endif
const char VERSION[]  = "4.5.5";
bool       PROBE      = true;                   // True if the station is being probed
const char DEVICEID[] = "tAEW4";                // t_hing A_rduino E_SP8266 W_iFi 4_

// OTA
int otaProgress       = 0;
int otaPort           = 8266;

// Timed debug reports
unsigned long dbgNext               = 0UL;                    // Next time to report

// Time synchronization and time keeping
const char    ntpServer[] PROGMEM   = NTP_SERVER;             // NTP server to connect to (RFC5905)
const int     ntpPort               = 123;                    // NTP port
unsigned long ntpNextSync           = 0UL;                    // Next time to syncronize
unsigned long ntpDelta              = 0UL;                    // Difference between real time and internal clock
bool          ntpOk                 = false;                  // Flag to know the time is accurate
const int     ntpTZ                 = 0;                      // Time zone

// Weather Underground parameters
const char wuServer[]       = "weatherstation.wunderground.com";
#ifdef USE_SSL
const unsigned int wuPort   = 443;
#else
const unsigned int wuPort   = 80;
#endif
const char wuGET[] PROGMEM  = "GET /weatherstation/updateweatherstation.php?"
                              "ID=" WU_ID "&PASSWORD=" WU_PASS "&dateutc=now"
                              "&tempf=%d&dewptf=%d&humidity=%d&baromin=%d.%d"
                              "&solarradiation=%d"
                              "&softwaretype=%s%%2F%s&action=updateraw"
                              " HTTP/1.1";

// MQTT parameters
#ifdef USE_MQTT_SSL
WiFiClientSecure    wifiClient;                                 // Secure WiFi TCP client for MQTT
#else
WiFiClient          wifiClient;                                 // Plain WiFi TCP client for MQTT
#endif
PubSubClient        mqttClient(wifiClient);                     // MQTT client, based on WiFi client
#ifdef DEVEL
const char          mqttId[]       = MQTT_ID "-dev";            // Development MQTT client ID
#else
const char          mqttId[]       = MQTT_ID;                   // Production MQTT client ID
#endif
const char          mqttServer[]   = MQTT_SERVER;               // MQTT server address to connect to
#ifdef USE_MQTT_SSL
const int           mqttPort       = 8883;                      // Secure MQTT port
#else
const int           mqttPort       = 1883;                      // Plain MQTT port
#endif
const unsigned long mqttDelay      = 5000UL;                    // Delay between reconnection attempts
unsigned long       mqttNextTime   = 0UL;                       // Next time to reconnect
// Various MQTT topics
const char          mqttTopicCmd[] = "command";
const char          mqttTopicSns[] = "sensor/outdoor";
const char          mqttTopicRpt[] = "report";

// APRS parameters
WiFiClient  aprsClient;                                                                     // WiFi TCP client for APRS
const char  aprsServer[]  = "cwop5.aprs.net";                                               // CWOP APRS-IS server address to connect to
const int   aprsPort      = 14580;                                                          // CWOP APRS-IS port
const int   altMeters     = APRS_ALTITUDE;                                                  // Altitude in meters
const long  altFeet       = (long)(altMeters * 3.28084);                                    // Altitude in feet
const float altCorr       = pow((float)(1.0 - 2.25577e-5 * altMeters), (float)(-5.25578));  // Altitude correction for QNH

const char aprsCallSign[] PROGMEM = APRS_CALLSIGN;
const char aprsPassCode[] PROGMEM = APRS_PASSCODE;
const char aprsPath[]     PROGMEM = ">APRS,TCPIP*:";
const char aprsLocation[] PROGMEM = APRS_LAT "/" APRS_LON "_";
const char aprsTlmPARM[]  PROGMEM = "PARM.Vcc,RSSI,Heap,IRed,Visb,ENVRN,BARO,LUX,SAT,VCC,HT,RB,TM";
const char aprsTlmEQNS[]  PROGMEM = "EQNS.0,0.004,2.5,0,-1,0,0,256,0,0,256,0,0,256,0";
const char aprsTlmUNIT[]  PROGMEM = "UNIT.V,dBm,Bytes,units,units,off,off,off,sat,bad,ht,rb,er";
const char aprsTlmBITS[]  PROGMEM = "BITS.11111111, ";
const char eol[]          PROGMEM = "\r\n";

// Reports and measurements
const int aprsRprtHour   = 12; // Number of APRS reports per hour
const int aprsMsrmMax    = 5;  // Number of measurements per report
int       aprsMsrmCount  = 1;  // Measurements counter (force one initial report)
int       aprsTlmSeq     = 0;  // Telemetry sequence mumber

// Telemetry bits
char      aprsTlmBits    = B00000000;

// The APRS packet buffer, largest packet is 82 for v2.1
char      aprsPkt[100]   = "";

// Statistics (round median filter for the last 3 values)
enum      rMedIdx {MD_TEMP, MD_DEWP, MD_HMDT, MD_PRES, MD_SRAD, MD_VISI, MD_IRED, MD_RSSI, MD_VCC, MD_HEAP, MD_ALL};
int       rMed[MD_ALL][4] = {0, -1, -1, -1};

// Sensors
const unsigned long snsDelay    = 3600000UL / aprsRprtHour / aprsMsrmMax; // Delay between sensor readings
unsigned long       snsNextTime = 0UL;                                    // Next time to read the sensors

// BME280
const byte          bmeAddr    = 0x76;                                    // The BME280 athmospheric sensor I2C address
Adafruit_BME280     bme;                                                  // The BME280 athmospheric sensor
bool                bmeOK      = false;                                   // The BME280 athmospheric sensor presence flag

#ifdef BHLIGHT
// BH1750
const byte          lightAddr   = 0x23;                                   // The illuminance sensor I2C address
BH1750              light(lightAddr);                                     // The illuminance sensor
#else // BHLIGHT
// TSL2561
const byte          lightAddr   = 0x23;                                   // The illuminance sensor I2C address
SFE_TSL2561         light;                                                // The illuminance sensor
boolean             lightGain   = false;                                  //    ~    ~    ~    ~    gain (true/false)
unsigned char       lightSHTR   = 0;                                      //    ~    ~    ~    ~    shutter (0, 1, 2)
unsigned int        lightMS;                                              //    ~    ~    ~    ~    integration timer
#endif // BHLIGHT
bool                lightOK     = false;                                  // The illuminance sensor presence flag

// Set ADC to Voltage
ADC_MODE(ADC_VCC);

// Zambretti forecaster (pressure in tenths of mB = decapascals dPa)
int           zbBaroTop   = 10400;                  // Highest athmospheric pressure
int           zbBaroBot   =  9800;                  // Lowest athmospheric pressure
int           zbBaroTrs   =    10;                  // Pressure threshold
const int     zbHours     = 3;                      // Need the last 3 hours for forecast
int           zbDelay     = 3600UL * 1000UL;        // Report hourly
unsigned long zbNextTime  = 0;                      // The next time to report, collect data till then

// Linear regression computer
const int     rgMax       = zbHours * aprsRprtHour; // The size of the circular buffer
int           rgIdx       = 0;                      // Index in circular buffer
int           rgCnt       = 0;                      // Counter of current values in buffer
int           rgY[rgMax]  = {0};                    // The circular buffer

// Forecast texts
const char zbFcA[] PROGMEM = "Settled fine";
const char zbFcB[] PROGMEM = "Fine weather";
const char zbFcC[] PROGMEM = "Becoming fine";
const char zbFcD[] PROGMEM = "Fine, becoming less settled";
const char zbFcE[] PROGMEM = "Fine, possible showers";
const char zbFcF[] PROGMEM = "Fairly fine, improving";
const char zbFcG[] PROGMEM = "Fairly fine, possible showers early";
const char zbFcH[] PROGMEM = "Fairly fine, showery later";
const char zbFcI[] PROGMEM = "Showery early, improving";
const char zbFcJ[] PROGMEM = "Changeable, mending";
const char zbFcK[] PROGMEM = "Fairly fine, showers likely";
const char zbFcL[] PROGMEM = "Rather unsettled clearing later";
const char zbFcM[] PROGMEM = "Unsettled, probably improving";
const char zbFcN[] PROGMEM = "Showery, bright intervals";
const char zbFcO[] PROGMEM = "Showery, becoming less settled";
const char zbFcP[] PROGMEM = "Changeable, some rain";
const char zbFcQ[] PROGMEM = "Unsettled, short fine intervals";
const char zbFcR[] PROGMEM = "Unsettled, rain later";
const char zbFcS[] PROGMEM = "Unsettled, some rain";
const char zbFcT[] PROGMEM = "Mostly very unsettled";
const char zbFcU[] PROGMEM = "Occasional rain, worsening";
const char zbFcV[] PROGMEM = "Rain at times, very unsettled";
const char zbFcW[] PROGMEM = "Rain at frequent intervals";
const char zbFcX[] PROGMEM = "Rain, very unsettled";
const char zbFcY[] PROGMEM = "Stormy, may improve";
const char zbFcZ[] PROGMEM = "Stormy, much rain";
const char* const zbFc[] PROGMEM = {zbFcA, zbFcB, zbFcC, zbFcD, zbFcE, zbFcF, zbFcG, zbFcH, zbFcI,
                                    zbFcJ, zbFcK, zbFcL, zbFcM, zbFcN, zbFcO, zbFcP, zbFcQ, zbFcR,
                                    zbFcS, zbFcT, zbFcU, zbFcV, zbFcW, zbFcX, zbFcY, zbFcZ
                                   };

// Forecast selectors for rising, steady and falling trend
byte zbRs[] = {25, 25, 25, 24, 24, 19, 16, 12, 11,  9,  8,  6,  5,  2,  1,  1,  0,  0,  0,  0,  0,  0};
byte zbSt[] = {25, 25, 25, 25, 25, 25, 23, 23, 22, 18, 15, 13, 10,  4,  1,  1,  0,  0,  0,  0,  0,  0};
byte zbFl[] = {25, 25, 25, 25, 25, 25, 25, 25, 23, 23, 21, 20, 17, 14,  7,  3,  1,  1,  1,  0,  0,  0};

// Various
const char pstrD[]  PROGMEM = "%d";
const char pstrDD[] PROGMEM = "%d.%d";
const char pstrSP[] PROGMEM = " ";
const char pstrCL[] PROGMEM = ":";
const char pstrSL[] PROGMEM = "/";

// Make sure some required macros are defined
#undef max
#define max(a,b) ((a)>(b)?(a):(b))
#undef min
#define min(a,b) ((a)<(b)?(a):(b))


/**
  Simple round median filter: get the median
  2014-03-25: started by David Cary

  @param idx the index in round median array
  @return the median
*/
int rMedOut(int idx) {
  // Return the last value if the buffer is not full yet
  if (rMed[idx][0] < 3)
    return rMed[idx][3];
  else {
    // Get the maximum and the minimum
    int the_max = max(max(rMed[idx][1], rMed[idx][2]), rMed[idx][3]);
    int the_min = min(min(rMed[idx][1], rMed[idx][2]), rMed[idx][3]);
    // Clever code: XOR the max and min, remaining the middle
    return the_max ^ the_min ^ rMed[idx][1] ^ rMed[idx][2] ^ rMed[idx][3];
  }
}

/**
  Simple median filter: add value to array
  @param idx the index in round median array
  @param x the value to add
*/
void rMedIn(int idx, int x) {
  // At index 0 there is the number of values stored
  if (rMed[idx][0] < 3) rMed[idx][0]++;
  // Shift one position
  rMed[idx][1] = rMed[idx][2];
  rMed[idx][2] = rMed[idx][3];
  rMed[idx][3] = x;
#ifdef DEBUG
  Serial.print(F("RMed "));
  Serial.print(idx);
  Serial.print(F(": "));
  Serial.println(x);
#endif
}

/**
  Convert IPAddress to char array
*/
char charIP(const IPAddress ip, char *buf, size_t len, boolean pad = false) {
  if (pad) snprintf_P(buf, len, PSTR("%3d.%3d.%3d.%3d"), ip[0], ip[1], ip[2], ip[3]);
  else     snprintf_P(buf, len, PSTR("%d.%d.%d.%d"),     ip[0], ip[1], ip[2], ip[3]);
}

/**
  Get current time as UNIX time (1970 epoch)

  @param sync flag to show whether network sync is to be performed
  @return current UNIX time
*/
unsigned long timeUNIX(bool sync = true) {
  // Check if we need to sync
  if (millis() >= ntpNextSync and sync) {
    // Try to get the time from Internet
    unsigned long utm = ntpSync();
    if (utm == 0) {
      // Time sync has failed, sync again over one minute
      ntpNextSync = millis() + 60000UL;
      ntpOk = false;
    }
    else {
      // Compute the new time delta
      ntpDelta = utm - (millis() / 1000);
      // Time sync has succeeded, sync again in 8 hours
      ntpNextSync = millis() + 28800000UL;
      ntpOk = true;
      Serial.print(F("Network UNIX Time: 0x"));
      Serial.println(utm, 16);
    }
  }
  // Get current time based on uptime and time delta,
  // or just uptime for no time sync ever
  return (millis() / 1000) + ntpDelta + ntpTZ * 3600;
}

/**
  © Francesco Potortì 2013 - GPLv3 - Revision: 1.13

  Send an NTP packet and wait for the response, return the Unix time

  To lower the memory footprint, no buffers are allocated for sending
  and receiving the NTP packets.  Four bytes of memory are allocated
  for transmision, the rest is random garbage collected from the data
  memory segment, and the received packet is read one byte at a time.
  The Unix time is returned, that is, seconds from 1970-01-01T00:00.
*/
unsigned long ntpSync() {
  // NTP UDP client
  WiFiUDP ntpClient;
  // Open socket on arbitrary port
  bool ntpOk = ntpClient.begin(12321);
  // NTP request header: Only the first four bytes of an outgoing
  // packet need to be set appropriately, the rest can be whatever.
  const long ntpFirstFourBytes = 0xEC0600E3;
  // Fail if UDP could not init a socket
  if (!ntpOk) return 0UL;
  // Clear received data from possible stray received packets
  ntpClient.flush();
  // Send an NTP request
  char ntpServerBuf[strlen_P((char*)ntpServer) + 1];
  strncpy_P(ntpServerBuf, (char*)ntpServer, sizeof(ntpServerBuf));
  if (!(ntpClient.beginPacket(ntpServerBuf, ntpPort) &&
        ntpClient.write((byte *)&ntpFirstFourBytes, 48) == 48 &&
        ntpClient.endPacket()))
    return 0UL;                             // sending request failed
  // Wait for response; check every pollIntv ms up to maxPoll times
  const int pollIntv = 150;                 // poll every this many ms
  const byte maxPoll = 15;                  // poll up to this many times
  int pktLen;                               // received packet length
  for (byte i = 0; i < maxPoll; i++) {
    if ((pktLen = ntpClient.parsePacket()) == 48) break;
    delay(pollIntv);
  }
  if (pktLen != 48) return 0UL;             // no correct packet received
  // Read and discard the first useless bytes (32 for speed, 40 for accuracy)
  for (byte i = 0; i < 40; ++i) ntpClient.read();
  // Read the integer part of sending time
  unsigned long ntpTime = ntpClient.read(); // NTP time
  for (byte i = 1; i < 4; i++)
    ntpTime = ntpTime << 8 | ntpClient.read();
  // Round to the nearest second if we want accuracy
  // The fractionary part is the next byte divided by 256: if it is
  // greater than 500ms we round to the next second; we also account
  // for an assumed network delay of 50ms, and (0.5-0.05)*256=115;
  // additionally, we account for how much we delayed reading the packet
  // since its arrival, which we assume on average to be pollIntv/2.
  ntpTime += (ntpClient.read() > 115 - pollIntv / 8);
  // Discard the rest of the packet and stop
  ntpClient.flush();
  ntpClient.stop();
  return ntpTime - 2208988800UL;            // convert to Unix time
}

/**
  Get the uptime

  @param buf character array to return the text to
  @param len the maximum length of the character array
  @return uptime in seconds
*/
unsigned long uptime(char *buf, size_t len) {
  // Get the uptime in seconds
  unsigned long upt = millis() / 1000;
  // Compute days, hours, minutes and seconds
  int ss =  upt % 60;
  int mm = (upt % 3600) / 60;
  int hh = (upt % 86400L) / 3600;
  int dd =  upt / 86400L;
  // Create the formatted time
  if (dd == 1) snprintf_P(buf, len, PSTR("%d day, %02d:%02d:%02d"),  dd, hh, mm, ss);
  else         snprintf_P(buf, len, PSTR("%d days, %02d:%02d:%02d"), dd, hh, mm, ss);
  // Return the uptime in seconds
  return upt;
}

/**
  Try to connect to WiFi
*/
void wifiConnect(int timeout = 300) {
  // Set the host name
  WiFi.hostname(NODENAME);
  // Set the mode
  WiFi.mode(WIFI_STA);
  // Try to connect to WiFi
#ifdef WIFI_SSID
  Serial.print(F("WiFi connecting "));
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  while (!WiFi.isConnected()) {
    Serial.print(".");
    delay(1000);
  };
  Serial.println(F(" done."));
#else
  WiFiManager wifiManager;
  wifiManager.setTimeout(timeout);
  wifiManager.setAPCallback(wifiCallback);
  while (!wifiManager.autoConnect(NODENAME))
    delay(1000);
#endif
}

/**
  Display the WiFi parameters
*/
void showWiFi() {
  Serial.println();
  if (WiFi.isConnected()) {
    char ipbuf[16] = "";
    char gwbuf[16] = "";
    char nsbuf[16] = "";

    // Get the IPs as char arrays
    charIP(WiFi.localIP(),   ipbuf, sizeof(ipbuf), true);
    charIP(WiFi.gatewayIP(), gwbuf, sizeof(gwbuf), true);
    charIP(WiFi.dnsIP(),     nsbuf, sizeof(nsbuf), true);

    // Print
    Serial.print(F("WiFi connected to ")); Serial.print(WiFi.SSID());
    Serial.print(F(" on channel "));       Serial.print(WiFi.channel());
    Serial.print(F(", RSSI "));            Serial.print(WiFi.RSSI());    Serial.println(F(" dBm."));
    Serial.print(F(" IP : "));             Serial.println(ipbuf);
    Serial.print(F(" GW : "));             Serial.println(gwbuf);
    Serial.print(F(" DNS: "));             Serial.println(nsbuf);
    Serial.println();
  }
  else {
    Serial.print(F("WiFi not connected"));
  }
}

/**
  Publish char array to topic
*/
boolean mqttPub(const char *payload, const char *lvl1, const char *lvl2 = NULL, const char *lvl3 = NULL, const boolean retain = false) {
  const int bufSize = 100;
  char buf[bufSize] = "";
  strncpy(buf, lvl1, bufSize);
  if (lvl2 != NULL) {
    strcat_P(buf, pstrSL);
    strncat(buf, lvl2, bufSize - strlen(buf) - 1);
  }
  if (lvl3 != NULL) {
    strcat_P(buf, pstrSL);
    strncat(buf, lvl3, bufSize - strlen(buf) - 1);
  }
  yield();
  return mqttClient.publish(buf, payload, retain);
}

/**
  Publish char array to topic and retain
*/
boolean mqttPubRet(const char *payload, const char *lvl1, const char *lvl2 = NULL, const char *lvl3 = NULL, const boolean retain = true) {
  return mqttPub(payload, lvl1, lvl2, lvl3, retain);
}

/**
  Publish integer to topic
*/
boolean mqttPub(const int payload, const char *lvl1, const char *lvl2 = NULL, const char *lvl3 = NULL, const boolean retain = false) {
  const int bufSize = 16;
  char buf[bufSize] = "";
  snprintf_P(buf, bufSize, pstrD, payload);
  return mqttPub(buf, lvl1, lvl2, lvl3, retain);
}

/**
  Publish integer to topic and retain
*/
boolean mqttPubRet(const int payload, const char *lvl1, const char *lvl2 = NULL, const char *lvl3 = NULL, const boolean retain = true) {
  return mqttPub(payload, lvl1, lvl2, lvl3, retain);
}

/**
  Subscribe to topic or topic/subtopic
*/
void mqttSubscribe(const char *lvl1, const char *lvl2 = NULL, bool all = false) {
  const int bufSize = 100;
  char buf[bufSize] = "";
  strncpy(buf, lvl1, bufSize);
  if (lvl2 != NULL) {
    strcat_P(buf, pstrSL);
    strncat(buf, lvl2, bufSize - strlen(buf) - 1);
  }
  if (all) strcat_P(buf, PSTR("/#"));
  mqttClient.subscribe(buf);
}

/**
  Try to reconnect to MQTT server

  @return boolean reconnection success
*/
boolean mqttReconnect() {
  Serial.println(F("MQTT connecting..."));
  const int bufSize = 64;
  char buf[bufSize] = "";
  // The report topic
  strncpy(buf, mqttTopicRpt, bufSize);
  strcat_P(buf, pstrSL);
  strcat(buf, nodename);
  // Connect and set LWM to "offline"
  if (mqttClient.connect(mqttId, buf, 0, true, "offline")) {
    // Publish the "online" status
    mqttPubRet("online", buf);

    // Publish the connection report
    strcat_P(buf, PSTR("/wifi"));
    mqttPubRet(WiFi.hostname().c_str(),   buf, "hostname");
    mqttPubRet(WiFi.macAddress().c_str(), buf, "mac");
    mqttPubRet(WiFi.SSID().c_str(),       buf, "ssid");
    mqttPubRet(WiFi.RSSI(),               buf, "rssi");
    // Buffer for IPs
    char ipbuf[16] = "";
    charIP(WiFi.localIP(),   ipbuf, sizeof(ipbuf));
    mqttPubRet(ipbuf, buf, "ip");
    charIP(WiFi.gatewayIP(), ipbuf, sizeof(ipbuf));
    mqttPubRet(ipbuf, buf, "gw");

    // Subscribe to command topic
    mqttSubscribe(mqttTopicCmd, nodename, true);

    Serial.print(F("MQTT connected to "));
    Serial.print(mqttServer);
    Serial.print(F(" port "));
    Serial.print(mqttPort);
    Serial.println(F("."));
  }
  yield();
  return mqttClient.connected();
}

/**
  Message arrived in MQTT subscribed topics

  @param topic the topic the message arrived on (const char[])
  @param payload the message payload (byte array)
  @param length the length of the message payload (unsigned int)
*/
void mqttCallback(char *topic, byte *payload, unsigned int length) {
  // Make a limited copy of the payload and make sure it ends with \0
  char message[100] = "";
  if (length > 100) length = 100;
  memcpy(message, payload, length);
  message[length] = '\0';

#ifdef DEBUG
  Serial.printf("MQTT %s: %s\r\n", topic, message);
#endif

  // Lowercase the topic
  while (*topic) {
    *topic = tolower((unsigned char) * topic);
    topic++;
  }

  // Decompose the topic
  char *pRoot = NULL, *pTrunk = NULL, *pBranch = NULL;
  if (pRoot = strtok(topic, "/"))
    if (pTrunk = strtok(NULL, "/"))
      if (pBranch = strtok(NULL, "/"));

  // Dispatcher
  if (strcmp(pRoot, "command") == 0) {
    if (strcmp(pTrunk, nodename) == 0) {
      if (strcmp(pBranch, "restart") == 0)
        ESP.restart();
    }
  }
}

/**
  Feedback notification when SoftAP is started
*/
void wifiCallback(WiFiManager *wifiMgr) {
  Serial.print(F("Connect to "));
  Serial.println(wifiMgr->getConfigPortalSSID());
}

/**
  Send an APRS packet and, eventually, print it to serial line

  @param *pkt the packet to send
*/
void aprsSend(const char *pkt) {
#ifndef DEVEL
  aprsClient.print(pkt);
  yield();
#endif
#ifdef DEBUG
  Serial.print(pkt);
#endif
}

/**
  Return time in zulu APRS format: HHMMSSh

  @param *buf the buffer to return the time to
  @param len the buffer length
*/
char aprsTime(char *buf, size_t len) {
  // Get the time, but do not open a connection to server
  unsigned long utm = timeUNIX(false);
  // Compute hour, minute and second
  int hh = (utm % 86400L) / 3600;
  int mm = (utm % 3600) / 60;
  int ss =  utm % 60;
  // Return the formatted time
  snprintf_P(buf, len, PSTR("%02d%02d%02dh"), hh, mm, ss);
}

/**
  Send APRS authentication data
  user FW0690 pass -1 vers WxSta 0.2"
*/
void aprsAuthenticate() {
  strcpy_P(aprsPkt, PSTR("user "));
  strcat_P(aprsPkt, aprsCallSign);
  strcat_P(aprsPkt, PSTR(" pass "));
  strcat_P(aprsPkt, aprsPassCode);
  strcat_P(aprsPkt, PSTR(" vers "));
  strcat  (aprsPkt, NODENAME);
  strcat_P(aprsPkt, pstrSP);
  strcat  (aprsPkt, VERSION);
  strcat_P(aprsPkt, eol);
  aprsSend(aprsPkt);
}

/**
  Send APRS status
  FW0690>APRS,TCPIP*:>Fine weather

  @param message the status message to send
*/
void aprsSendStatus(const char *message) {
  // Send only if the message is not empty
  if (message[0] != '\0') {
    // Send the APRS packet
    strcpy_P(aprsPkt, aprsCallSign);
    strcat_P(aprsPkt, aprsPath);
    strcat_P(aprsPkt, PSTR(">"));
    strcat  (aprsPkt, message);
    strcat_P(aprsPkt, eol);
    aprsSend(aprsPkt);
  }
}

/**
  Send APRS position and altitude
  FW0690>APRS,TCPIP*:!DDMM.hhN/DDDMM.hhW$/A=000000 comment

  @param comment the comment to append
*/
void aprsSendPosition(const char *comment = NULL) {
  // Compose the APRS packet
  strcpy_P(aprsPkt, aprsCallSign);
  strcat_P(aprsPkt, aprsPath);
  strcat_P(aprsPkt, PSTR("!"));
  strcat_P(aprsPkt, aprsLocation);
  strcat_P(aprsPkt, PSTR("/A="));
  // Altitude buffer
  char buf[10] = "";
  sprintf_P(buf, PSTR("%06d"), altFeet);
  strncat(aprsPkt, buf, sizeof(buf));
  strcat_P(aprsPkt, pstrSP);
  // Comment
  if (comment != NULL)
    strcat(aprsPkt, comment);
  else {
    strcat(aprsPkt, NODENAME);
    strcat_P(aprsPkt, pstrSL);
    strcat(aprsPkt, VERSION);
    if (PROBE) strcat_P(aprsPkt, PSTR(" [PROBE]"));
  }
  strcat_P(aprsPkt, eol);
  aprsSend(aprsPkt);
}

/**
  Send APRS weather data, then try to get the forecast
  FW0690>APRS,TCPIP*:@152457h4427.67N/02608.03E_.../...g...t044h86b10201L001WxSta

  @param temp temperature in Fahrenheit
  @param hmdt relative humidity in percents
  @param pres athmospheric pressure (QNH) in dPa
  @param srad solar radiation in W/m^2
*/
void aprsSendWeather(int temp, int hmdt, int pres, int srad) {
  const int bufSize = 10;
  char buf[bufSize] = "";
  // Weather report
  strcpy_P(aprsPkt, aprsCallSign);
  strcat_P(aprsPkt, aprsPath);
  strcat_P(aprsPkt, PSTR("@"));
  aprsTime(buf, bufSize);
  strncat(aprsPkt, buf, bufSize);
  strcat_P(aprsPkt, aprsLocation);
  // Wind (unavailable)
  strcat_P(aprsPkt, PSTR(".../...g..."));
  // Temperature
  if (temp >= -460) { // 0K in F
    snprintf_P(buf, bufSize, PSTR("t%03d"), temp);
    strncat(aprsPkt, buf, bufSize);
  }
  else
    strcat_P(aprsPkt, PSTR("t..."));
  // Humidity
  if (hmdt >= 0) {
    if (hmdt == 100)
      strcat_P(aprsPkt, PSTR("h00"));
    else {
      snprintf_P(buf, bufSize, PSTR("h%02d"), hmdt);
      strncat(aprsPkt, buf, bufSize);
    }
  }
  // Athmospheric pressure
  if (pres >= 0) {
    snprintf_P(buf, bufSize, PSTR("b%05d"), pres);
    strncat(aprsPkt, buf, bufSize);
  }
  // Solar radiation, if valid
  if (srad >= 0) {
    if (srad < 1000) snprintf_P(buf, bufSize, PSTR("L%03d"), srad);
    else             snprintf_P(buf, bufSize, PSTR("l%03d"), srad - 1000);
    strncat(aprsPkt, buf, bufSize);
  }
  // Comment (device name)
  strcat_P(aprsPkt, DEVICEID);
  strcat_P(aprsPkt, eol);
  aprsSend(aprsPkt);
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
void aprsSendTelemetry(int p1, int p2, int p3, int p4, int p5, byte bits) {
  // Increment the telemetry sequence number, reset it if exceeds 999
  if (++aprsTlmSeq > 999) aprsTlmSeq = 0;
  // Send the telemetry setup if the sequence number is 0
  if (aprsTlmSeq == 0) aprsSendTelemetrySetup();
  // Compose the APRS packet
  const int bufSize = 40;
  char buf[bufSize] = "";
  strcpy_P(aprsPkt, aprsCallSign);
  strcat_P(aprsPkt, aprsPath);
  snprintf_P(buf, bufSize, PSTR("T#%03d,%03d,%03d,%03d,%03d,%03d,"), aprsTlmSeq, p1, p2, p3, p4, p5);
  strncat(aprsPkt, buf, bufSize);
  uint8_t i = 0, b = 8;
  while (b--)
    buf[i++] = bitRead(bits, b) + '0';
  buf[i] = '\0';
  strncat(aprsPkt, buf, bufSize);
  strcat_P(aprsPkt, eol);
  aprsSend(aprsPkt);
}

/**
  Send APRS telemetry setup
*/
void aprsSendTelemetrySetup() {
  // The object's call sign has to be padded with spaces until 9 chars long
  const int padSize = 9;
  char padCallSign[padSize] = " ";
  // Copy the call sign from PROGMEM
  strcpy_P(padCallSign, aprsCallSign);
  // Pad with spaces, then make sure it ends with '\0'
  for (int i = strlen(padCallSign); i < padSize; i++)
    padCallSign[i] = ' ';
  padCallSign[padSize] = '\0';
  // Create the common header of the packet
  strcpy_P(aprsPkt, aprsCallSign);
  strcat_P(aprsPkt, aprsPath);
  strcat_P(aprsPkt, pstrCL);
  strncat(aprsPkt, padCallSign, padSize);
  strcat_P(aprsPkt, pstrCL);
  // At this point, keep the size of the packet header,
  // so we can trim the packet and append to it again
  int lenHeader = strlen(aprsPkt);
  // Parameter names
  strcat_P(aprsPkt, aprsTlmPARM);
  strcat_P(aprsPkt, eol);
  aprsSend(aprsPkt);
  // Trim the packet
  aprsPkt[lenHeader] = '\0';
  // Equations
  strcat_P(aprsPkt, aprsTlmEQNS);
  strcat_P(aprsPkt, eol);
  aprsSend(aprsPkt);
  // Trim the packet
  aprsPkt[lenHeader] = '\0';
  // Units
  strcat_P(aprsPkt, aprsTlmUNIT);
  strcat_P(aprsPkt, eol);
  aprsSend(aprsPkt);
  // Trim the packet
  aprsPkt[lenHeader] = '\0';
  // Bit sense and project name
  strcat_P(aprsPkt, aprsTlmBITS);
  strcat(aprsPkt, NODENAME);
  strcat_P(aprsPkt, pstrSL);
  strcat(aprsPkt, VERSION);
  strcat_P(aprsPkt, eol);
  aprsSend(aprsPkt);
}

/**
  Send the weather forecast as APRS status report
  FW0690>APRS,TCPIP*:>Showery, becoming less settled

  @param pres athmospheric pressure (QNH) in dPa
*/
int aprsSendForecast(int pres) {
  // Get the Zambretti code
  int zbCode = zbForecast(pres);
  // If valid, send the report
  if (zbCode >= 0) {
    // Need a larger buffer
    char buf[40] = "";
    strncpy_P(buf, zbFc[zbCode], sizeof(buf));
    // Report to APRS
    aprsSendStatus(buf);
    // Send to MQTT, too
    mqttPubRet(buf, mqttTopicSns, "forecast");
  }
  return zbCode;
}

/**
  Get the Zambretti forecast
  @param zbCurrent current value of the atmospheric pressure in dPa (0.1 mB)
  @return the Zambretti forecast
*/
int zbForecast(int zbCurrent) {
  // Prepare the result
  int result = -1;
  // Keep the current value in the circular buffer for linear regression
  rgStore(zbCurrent);
  // If first run, set the timeout to the next hour
  unsigned long utm = timeUNIX(false);
  unsigned long  mm = (utm % 3600) / 60;
  if (zbNextTime == 0)
    zbNextTime = millis() + (60UL - mm) * 60000UL;
  else if (millis() >= zbNextTime) {
    // Timer expired
    int trend = 0;
    int index = 0;
    int range = zbBaroTop - zbBaroBot;
    // Compute the linear regression
    float rgA, rgB, rgS;
    rgLnRegr(&rgA, &rgB, &rgS);
    // Compute the pressure variation and last pressure (according to the equation)
    int pVar = lround(rgA * rgCnt);
    int pLst = lround(pVar + rgB);
    // Report
    char buf[32] = "";
    snprintf_P(buf, 32, PSTR("PVar:%d, PLst:%d"), pVar, pLst);
    aprsSendMessage(NULL, "ZBRT.", buf);
    // Get the trend
    if      (pVar >  zbBaroTrs) trend =  1;
    else if (pVar < -zbBaroTrs) trend = -1;
    // Compute the approximate day of the year,
    // no great precision required
    int appday = lround(((timeUNIX(false) / 315576UL) % 100) * 365 / 100);
    // Corrections for summer (april..september)
    if ((appday >= 90) and (appday <= 270)) {
      if      (trend > 0) pLst += range * 7 / 100;
      else if (trend < 0) pLst -= range * 7 / 100;
    }
    // Validate the interval
    if (pLst > zbBaroTop) pLst = zbBaroTop - zbBaroTrs - zbBaroTrs;
    // Get the forecast
    index = lround(22 * (pLst - zbBaroBot) / range);
    if ((index >= 0) and (index <= 22)) {
      if      (trend > 0) result = (int)zbRs[index];
      else if (trend < 0) result = (int)zbFl[index];
      else                result = (int)zbSt[index];
    }
    // Set the next timer
    zbNextTime += zbDelay;
  }
  return result;
}

/**
  Send an APRS message

  @param dest the message destination, own call sign if empty
  @param title the message title, if not empty
  @param message the message body
*/
void aprsSendMessage(const char *dest, const char *title, const char *message) {
  // The object's call sign has to be padded with spaces until 9 chars long
  const int padSize = 9;
  char padCallSign[padSize] = " ";
  // Check if the destination is specified
  if (dest == NULL) strcpy_P(padCallSign, aprsCallSign);  // Copy the own call sign from PROGMEM
  else              strncpy(padCallSign, dest, padSize);  // Use the specified destination
  // Pad with spaces, then make sure it ends with '\0'
  for (int i = strlen(padCallSign); i < padSize; i++)
    padCallSign[i] = ' ';
  padCallSign[padSize] = '\0';
  // Create the header of the packet
  strcpy_P(aprsPkt, aprsCallSign);
  strcat_P(aprsPkt, aprsPath);
  strcat_P(aprsPkt, pstrCL);
  // Message destination
  strncat(aprsPkt, padCallSign, padSize);
  strcat_P(aprsPkt, pstrCL);
  // Message title
  if (title != NULL) strncat(aprsPkt, title, 8);
  // The body of the message, maximum size is 45, including the title
  strncat(aprsPkt, message, 40);
  strcat_P(aprsPkt, eol);
  aprsSend(aprsPkt);
}

/**
  Store previous athmospheric pressure values for each report in the last zbHours hours

  @param y current pressure value in dPa
*/
void rgStore(int y) {
  rgY[rgIdx] = y;
  rgIdx++;
  if (rgIdx >= rgMax) rgIdx = 0; // wrap around
  if (rgCnt <  rgMax) rgCnt++;   // count the stored values
}

/**
  Linear regression

  @param A the alpha coefficient (pass by pointer)
  @param B the beta coefficient (pass by pointer)
  @param S the standard deviation (pass by pointer)
*/
void rgLnRegr(float * A, float * B, float * S) {
  int i, iy = 0;
  float denom, dy, x, y;
  float a1 = 0, a2 = 0, s = 0, s1 = 0, s2 = 0, s3 = 0, s4 = 0;
  // Circular buffer, moving cursor
  if (rgCnt >= rgMax) iy = rgIdx;
  // Compute the sums
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
    // Return the coefficients
    *A = a2;
    *B = a1;
    *S = sqrt(s / (rgCnt - 1));
  }
}

/**
  Weather Underground

  @param temp temperature in Fahrenheit
  @param dewp dew point in Fahrenheit
  @param hmdt relative humidity in percents
  @param pres athmospheric pressure (QNH) in dPa
  @param srad solar radiation in W/m^2
*/
void wuUpdate(int temp, int dewp, int hmdt, int pres, int srad) {
  // Only on valid data
  if (temp >= -460 and dewp >= -460 and hmdt >= 0 and pres >= 0 and srad >= 0) {
    // Barometer in inHg
    int baro = lround(pres * 0.02953);

#ifdef USE_SSL
    WiFiClientSecure wuClient;  // The HTTPS client
#else
    WiFiClient       wuClient;  // The HTTP client
#endif
    wuClient.setTimeout(5000);
    //Serial.println(F("Connecting"));
    if (wuClient.connect(wuServer, wuPort)) {
      const int bufSize = 250;
      char buf[bufSize] = "";

      // Compose the WU request
      snprintf_P(buf, bufSize, wuGET, temp, dewp, hmdt, baro / 10, baro % 10, srad, NODENAME, VERSION);
      strcat_P(buf, eol);
      wuClient.print(buf);
      Serial.print(buf);
      yield();
      // Host
      strcpy_P(buf, PSTR("Host: "));
      strcat(buf, wuServer);
      strcat_P(buf, eol);
      wuClient.print(buf);
      Serial.print(buf);
      yield();
      // User agent
      strcpy_P(buf, PSTR("User-Agent: "));
      strcat  (buf, NODENAME);
      strcat_P(buf, pstrSP);
      strcat  (buf, VERSION);
      strcat_P(buf, eol);
      wuClient.print(buf);
      Serial.print(buf);
      yield();
      // Connection
      strcpy_P(buf, PSTR("Connection: close"));
      strcat_P(buf, eol);
      strcat_P(buf, eol);
      wuClient.print(buf);
      Serial.print(buf);
      yield();

      // Get the response
      while (wuClient.connected()) {
        int rlen = wuClient.readBytesUntil('\r', buf, bufSize);
        yield;
        buf[rlen] = '\0';
        Serial.print(buf);
      }

      // Close the connection
      wuClient.stop();
    }
    //Serial.println(F("Done"));
  }
}

/**
  Main Arduino setup function
*/
void setup() {
  // Init the serial com
  Serial.begin(115200);
  Serial.println();
  Serial.print(NODENAME);
  Serial.print(" ");
  Serial.print(VERSION);
  Serial.print(" ");
  Serial.println(__DATE__);

  // Try to connect
  wifiConnect();
  // Connected
  showWiFi();

  // OTA Update
  ArduinoOTA.setPort(otaPort);
  ArduinoOTA.setHostname(NODENAME);
  // ArduinoOTA.setPassword((const char *)"123");

  ArduinoOTA.onStart([]() {
    Serial.println(F("OTA Start"));
  });

  ArduinoOTA.onEnd([]() {
    Serial.println("\nOTA Finished");
  });

  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    int otaPrg = progress / (total / 100);
    if (otaProgress != otaPrg) {
      otaProgress = otaPrg;
      Serial.printf("Progress: %u%%\r", otaProgress * 100);
    }
  });

  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR)
      Serial.println(F("Auth Failed"));
    else if (error == OTA_BEGIN_ERROR)
      Serial.println(F("Begin Failed"));
    else if (error == OTA_CONNECT_ERROR)
      Serial.println(F("Connect Failed"));
    else if (error == OTA_RECEIVE_ERROR)
      Serial.println(F("Receive Failed"));
    else if (error == OTA_END_ERROR)
      Serial.println(F("End Failed"));
  });

  ArduinoOTA.begin();
  Serial.println(F("OTA Ready"));

  // Start time sync
  timeUNIX();
  yield();

  // Start the MQTT client
  mqttClient.setServer(mqttServer, mqttPort);
  mqttClient.setCallback(mqttCallback);
  mqttNextTime = millis();
  yield();

  // For I2C, the ESP8266-1 module uses the pin 0 for SDA and 2 for SCL
#if defined(ARDUINO_ARCH_ESP8266)
  Wire.pins(SDA, SCL);
#endif

  // BME280
  bmeOK = bme.begin((uint8_t)bmeAddr);
  if (bmeOK)  Serial.println(F("BME280 sensor detected"));
  else        Serial.println(F("BME280 sensor missing"));
  yield();

#ifdef BHLIGHT
  // BH1750
  Wire.beginTransmission(lightAddr);
  lightOK = Wire.endTransmission() == 0;
  if (lightOK) {
    light.begin(BH1750_CONTINUOUS_HIGH_RES_MODE);
    Serial.println(F("BH1750 sensor detected"));
  }
  else
    Serial.println(F("BH1750 sensor missing"));
#else
  // TSL2561
  light.begin();
  unsigned char ID;
  if (light.getID(ID)) {
    light.setTiming(lightGain, lightSHTR, lightMS);
    light.setPowerUp();
    lightOK = true;
    Serial.println(F("TSL2561 sensor detected"));
  }
  else {
    lightOK = false;
    Serial.println(F("TSL2561 sensor missing"));
  }
#endif
  yield();

  // Hardware data
  int hwVcc  = ESP.getVcc();
  Serial.print(F("Vcc : "));
  Serial.println((float)hwVcc / 1000, 3);

  // Initialize the random number generator and set the APRS telemetry start sequence
  randomSeed(timeUNIX(false) + hwVcc + millis());
  aprsTlmSeq = random(1000);
  Serial.print(F("TLM : "));
  Serial.println(aprsTlmSeq);

  // Start the sensor timer
  snsNextTime = millis();
  yield();
}

/**
  Main Arduino loop
*/
void loop() {
  // Handle OTA
  ArduinoOTA.handle();
  yield();

  // Process incoming MQTT messages and maintain connection
  if (!mqttClient.loop())
    // Not connected, check if it's time to try to reconnect
    if (millis() >= mqttNextTime)
      // Try to reconnect every mqttDelay seconds
      if (!mqttReconnect()) mqttNextTime = millis() + mqttDelay;
  yield();

  // Read the sensors and publish telemetry
  if (millis() >= snsNextTime) {
    // Count to check if we need to send the APRS data
    if (aprsMsrmCount-- <= 0)
      // Restart the counter
      aprsMsrmCount = aprsMsrmMax - 1;
    // Repeat sensor reading after the delay
    snsNextTime += snsDelay;

#ifdef DEBUG
    Serial.print(F("Sensor reading "));
    Serial.println(aprsMsrmCount);
#endif

    // Check the time and set the telemetry bit 0 if time is not accurate
    unsigned long utm = timeUNIX();
    if (!ntpOk) bitSet(aprsTlmBits, 0);
    else        bitClear(aprsTlmBits, 0);
    // Set the telemetry bit 1 if the uptime is less than one day (recent reboot)
    if (millis() < 86400000UL)  bitSet(aprsTlmBits, 1);
    else                        bitClear(aprsTlmBits, 1);
    yield();

    // Check again whether the BME280 sensor is present
    if (!bmeOK) bmeOK = bme.begin((uint8_t)bmeAddr);
    // Read the athmospheric sensor BME280
    if (bmeOK) {
      // Clear bit 7 to show the sensor is present
      bitClear(aprsTlmBits, 7);
      // Get the weather parameters
      float temp = bme.readTemperature();
      float hmdt = bme.readHumidity();
      float dewp = 243.04 *
                   (log(hmdt / 100.0) + ((17.625 * temp) / (243.04 + temp))) /
                   (17.625 - log(hmdt / 100.0) - ((17.625 * temp) / (243.04 + temp)));
      float pres = bme.readPressure();
      float slvl = pres * altCorr;
      // Store directly integer Fahrenheit
      rMedIn(MD_TEMP, lround(temp * 9 / 5 + 32));
      rMedIn(MD_DEWP, lround(dewp * 9 / 5 + 32));
      rMedIn(MD_HMDT, lround(hmdt));
      // Store directly sea level in dPa
      rMedIn(MD_PRES, lround(slvl / 10));
      // Compose and publish the telemetry
      mqttPubRet(lround(temp), mqttTopicSns, "temperature");
      mqttPubRet(lround(hmdt), mqttTopicSns, "humidity");
      mqttPubRet(lround(dewp), mqttTopicSns, "dewpoint");
      mqttPubRet(lround(pres / 100), mqttTopicSns, "pressure");
      mqttPubRet(lround(slvl / 100), mqttTopicSns, "sealevel");
    }
    else {
      // Set the bit 7 to show the sensor is absent
      bitSet(aprsTlmBits, 7);
      // Store invalid values if no sensor
      rMedIn(MD_TEMP, -500);
      rMedIn(MD_DEWP, -500);
      rMedIn(MD_HMDT, -1);
      rMedIn(MD_PRES, -1);
    }
    yield();

    // Check again whether the sensor is present
    if (!lightOK) {
#ifdef BHLIGHT
      Wire.beginTransmission(lightAddr);
      lightOK = Wire.endTransmission() == 0;
#else
      light.begin();
      unsigned char ID;
      if (light.getID(ID)) {
        light.setTiming(lightGain, lightSHTR, lightMS);
        light.setPowerUp();
        lightOK = true;
      }
#endif
    }
    // Read the light sensor
    if (lightOK) {
      // Clear bit 5 to show the sensor is present
      bitClear(aprsTlmBits, 5);
#ifdef BHLIGHT
      // Read BH1750, illuminance value in lux
      float lux = light.readLightLevel();
      // Calculate the solar radiation in W/m^2
      int solRad = lround(lux * 0.0079);
      // Compose and publish the telemetry
      mqttPubRet(lround(lux), mqttTopicSns, "illuminance");
      mqttPubRet(solRad,      mqttTopicSns, "solar");
      // Limit the reading to a maximum value accepted by APRS
      if (solRad > 1999) {
        solRad = 1999;
        // Set bit 4 to show the sensor is saturated
        bitSet(aprsTlmBits, 4);
      }
      else
        // Clear bit 4 to show the sensor is not saturated
        bitClear(aprsTlmBits, 4);
      // Add to round median filter
      rMedIn(MD_SRAD, solRad);
      rMedIn(MD_VISI, 0);
      rMedIn(MD_IRED, 0);
#else
      unsigned int luxVis = 0, luxIrd = 0;
      double lux = -1;
      int solRad = -1;
      if (light.getData(luxVis, luxIrd)) {
        // TODO Adapt the shutter if saturated
        if (light.getLux(lightGain, lightMS, luxVis, luxIrd, lux)) {
          // Calculate the solar radiation in W/m^2 and publish
          solRad = lround(lux * 0.0079);
          // Compose and publish the telemetry
          mqttPubRet(lround(lux), mqttTopicSns, "illuminance");
          mqttPubRet(luxVis,      mqttTopicSns, "visible");
          mqttPubRet(luxIrd,      mqttTopicSns, "infrared");
          mqttPubRet(solRad,      mqttTopicSns, "solar");
          // Limit the reading to a maximum value accepted by APRS
          if (solRad > 1999) solRad = 1999;
        }
        else {
          // Sensor is saturated
          lux = -1;
          // Limit the reading to a maximum value
          if (solRad > 1999) solRad = 1999;
          // Set the bit 4 to show the sensor is saturated
          aprsTlmBits |= B00010000;
        }
        // Add to round median filter
        rMedIn(MD_SRAD, solRad);
        rMedIn(MD_VISI, luxVis);
        rMedIn(MD_IRED, luxIrd);
      }
      else {
        // Store invalid values if erroneous data
        rMedIn(MD_SRAD, -1);
        rMedIn(MD_VISI, 0);
        rMedIn(MD_IRED, 0);
      }
#endif
    }
    else {
      // Set bit 5 if sensor is absent
      bitSet(aprsTlmBits, 5);
      // Store invalid values if no sensor
      rMedIn(MD_SRAD, -1);
      rMedIn(MD_VISI, 0);
      rMedIn(MD_IRED, 0);
    }
    yield();

    // Free Heap
    int heap = ESP.getFreeHeap();
    rMedIn(MD_HEAP, heap);
    // Read the Vcc (mV) and add to the round median filter
    int vcc  = ESP.getVcc();
    rMedIn(MD_VCC, vcc);
    // Set the bit 3 to show whether the battery is wrong (3.3V +/- 10%)
    if (vcc < 3000 or vcc > 3600) bitSet(aprsTlmBits, 3);
    else                          bitClear(aprsTlmBits, 3);
    // Get RSSI
    int rssi = WiFi.RSSI();
    rMedIn(MD_RSSI, rssi);

    // Create the reporting topic
    char topic[32] = "";
    char text[32] = "";
    strncpy(topic, mqttTopicRpt, sizeof(topic));
    strcat_P(topic, pstrSL);
    strcat(topic, nodename);
    // Uptime in seconds and text
    unsigned long ups = 0;
    char upt[32] = "";
    ups = uptime(upt, sizeof(upt));
    mqttPubRet(ups, topic, "uptime");
    mqttPubRet(upt, topic, "uptime", "text");
    // Free heap
    mqttPubRet(heap, topic, "heap");
    // Power supply
    snprintf_P(text, sizeof(text), pstrDD, vcc / 1000, vcc % 1000);
    mqttPubRet(text, topic, "vcc");
    // Add the WiFi topic and publish the RSSI value
    mqttPubRet(rssi, topic, "wifi", "rssi");

    // APRS (after the first 3600/(aprsMsrmMax*aprsRprtHour) seconds,
    //       then every 60/aprsRprtHour minutes)
    if (aprsMsrmCount == 0) {
      // Connect to APRS server
      if (aprsClient.connect(aprsServer, aprsPort)) {
        // Authentication
        aprsAuthenticate();
        // Send the position, altitude and comment in firsts minutes after boot
        if (millis() < snsDelay) aprsSendPosition();
        // Send weather data
        aprsSendWeather(rMedOut(MD_TEMP),
                        rMedOut(MD_HMDT),
                        rMedOut(MD_PRES),
                        rMedOut(MD_SRAD));
        // Send the telemetry setup if uptime is less than one minute
        if (ups < 60) aprsSendTelemetrySetup();
        // Send the telemetry
        aprsSendTelemetry((rMedOut(MD_VCC) - 2500) >> 2,
                          -rMedOut(MD_RSSI),
                          rMedOut(MD_HEAP) >> 8,
                          rMedOut(MD_VISI) >> 8,
                          rMedOut(MD_IRED) >> 8,
                          aprsTlmBits);
        // Send the forecast, if we have one
        aprsSendForecast(rMedOut(MD_PRES));
        // Hourly debug reports
        if (millis() >= dbgNext) {
          // Report again in one hour
          dbgNext += 3600000UL;
          // Send the uptime, as message
          aprsSendMessage(NULL, "UPTM.", upt);
        }
        //aprsSendStatus("Fine weather");
        // Close the connection
        aprsClient.stop();
      };
      // Send data to Weather Underground
      wuUpdate(rMedOut(MD_TEMP),
               rMedOut(MD_DEWP),
               rMedOut(MD_HMDT),
               rMedOut(MD_PRES),
               rMedOut(MD_SRAD));
    }
  }
}
