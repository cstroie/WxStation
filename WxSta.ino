/**
  WxSta - Weather probe based on ESP8266, WiFi connected

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


  WiFi connected weather probe, reading the athmospheric sensor BME280 and
  the illuminance sensor TSL2561, publishing the measured data along with
  various local telemetry.
*/

// The DEBUG and DEVEL flag
#define DEBUG
//#define DEVEL

// The sensors are connected to I2C
#define SDA 0
#define SCL 2
#include <Wire.h>
#include <SparkFunBME280.h>
#include <SparkFunTSL2561.h>

// WiFi
#include <ESP8266WiFi.h>
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
const char VERSION[]  = "4.0";
bool       PROBE      = true;                   // True if the station is being probed
const char DEVICEID[] = "tAEW4";                // t_hing A_rduino E_SP8266 W_iFi 4_

// OTA
int otaProgress       = 0;
int otaPort           = 8266;

// Time synchronization and time keeping
WiFiUDP       ntpClient;                                      // NTP UDP client
const char    ntpServer[] PROGMEM   = "europe.pool.ntp.org";  // NTP server to connect to (RFC5905)
const int     ntpPort               = 123;                    // NTP port
unsigned long ntpNextSync           = 0UL;                    // Next time to syncronize
unsigned long ntpDelta              = 0UL;                    // Difference between real time and internal clock
bool          ntpOk                 = false;                  // Flag to know the time is accurate
const int     ntpTZ                 = 0;                      // Time zone

// MQTT parameters
WiFiClient wifiClient;                                         // WiFi TCP client for MQTT
PubSubClient mqttClient(wifiClient);                           // MQTT client, based on WiFi client
#ifdef DEVEL
const char          mqttId[]       = "wxsta-dev-eridu-eu-org";  // Development MQTT client ID
#else
const char          mqttId[]       = "wxsta-eridu-eu-org";      // Production MQTT client ID
#endif
const char          mqttServer[]   = "eridu.eu.org";            // MQTT server address to connect to
const int           mqttPort       = 1883;                      // MQTT port
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
const int   altMeters     = 83;                                                             // Altitude in Bucharest
const long  altFeet       = (long)(altMeters * 3.28084);                                    // Altitude in feet
const float altCorr       = pow((float)(1.0 - 2.25577e-5 * altMeters), (float)(-5.25578));  // Altitude correction for QNH

const char aprsCallSign[] PROGMEM = "FW0690";
const char aprsPassCode[] PROGMEM = "-1";
const char aprsPath[]     PROGMEM = ">APRS,TCPIP*:";
const char aprsLocation[] PROGMEM = "4427.67N/02608.03E_";
const char aprsTlmPARM[]  PROGMEM = ":PARM.Vcc,RSSI,Heap,IRed,Visb,PROBE,ATMO,LUX,SAT,VCC,HT,RB,TM";
const char aprsTlmEQNS[]  PROGMEM = ":EQNS.0,0.004,2.5,0,-1,0,0,256,0,0,256,0,0,256,0";
const char aprsTlmUNIT[]  PROGMEM = ":UNIT.V,dBm,Bytes,units,units,prb,on,on,sat,bad,ht,rb,er";
const char aprsTlmBITS[]  PROGMEM = ":BITS.10011111, ";
const char eol[]          PROGMEM = "\r\n";

// Reports and measurements
const int aprsRprtHour   = 10; // Number of APRS reports per hour
const int aprsMsrmMax    = 5;  // Number of measurements per report (keep even)
int       aprsMsrmCount  = 0;  // Measurements counter
int       aprsTlmSeq     = 0;  // Telemetry sequence mumber

// Telemetry bits
char      aprsTlmBits    = B00000000;

// The APRS packet buffer, largest packet is 82 for v2.1
char      aprsPkt[100]   = "";

// Statistics (round median filter for the last 3 values)
enum      rMedIdx {MD_TEMP, MD_HMDT, MD_PRES, MD_SRAD, MD_VISI, MD_IRED, MD_RSSI, MD_VCC, MD_HEAP, MD_ALL};
int       rMed[MD_ALL][4] = {0, -1, -1, -1};

// Sensors
const unsigned long snsDelay    = 3600000UL / aprsRprtHour / aprsMsrmMax; // Delay between sensor readings
unsigned long       snsNextTime = 0UL;                                    // Next time to read the sensors
// BME280
const byte          atmoAddr    = 0x77;                                   // The athmospheric sensor I2C address
BME280              atmo;                                                 // The athmospheric sensor
bool                atmoOK      = false;                                  // The athmospheric sensor presence flag
// TSL2561
const byte          lightAddr   = 0x23;                                   // The illuminance sensor I2C address
SFE_TSL2561         light;                                                // The illuminance sensor
bool                lightOK     = false;                                  // The illuminance sensor presence flag
boolean             lightGain   = false;                                  //    ~    ~    ~    ~    gain (true/false)
unsigned char       lightSHTR   = 1;                                      //    ~    ~    ~    ~    shutter (0, 1, 2)
unsigned int        lightMS;                                              //    ~    ~    ~    ~    integration timer

// Set ADC to Voltage
ADC_MODE(ADC_VCC);

// Zambretti forecaster
int           zbBaroTop   = 105000;                 // Highest athmospheric pressure
int           zbBaroBot   = 95000;                  // Lowest athmospheric pressure
int           zbBaroTrs   = 100;                    // Pressure threshold
const int     zbHours     = 3;                      // Need the last 3 hours for forecast
int           zbDelay     = 3600000UL;              // Report hourly
unsigned long zbNextTime  = 0;                      // The next time to report, collect data till then

// Linear regression computer
const int     rgMax       = zbHours * aprsRprtHour; // The size of the circular buffer
int           rgIdx       = 0;                      // Index in circular buffer
int           rgCnt       = 0;                      // Counter of current values in buffer
float         rgY[rgMax]  = {0};                    // The circular buffer
float         rgAB[]      = {0, 0, 0};              // Coefficients: a, b and std dev in f(x)=ax+b

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
const char* const zbFc[] PROGMEM = {zbFcA, zbFcB, zbFcC, zbFcD, zbFcE, zbFcF, zbFcG,
                                    zbFcH, zbFcI, zbFcJ, zbFcK, zbFcL, zbFcM, zbFcN,
                                    zbFcO, zbFcP, zbFcQ, zbFcR, zbFcS, zbFcT, zbFcU,
                                    zbFcV, zbFcW, zbFcX, zbFcY, zbFcZ
                                   };

// Forecast selectors for rising, steady and falling trend
int zbRs[] = {25, 25, 25, 24, 24, 19, 16, 12, 11,  9,  8,  6,  5,  2,  1,  1,  0,  0,  0,  0,  0,  0};
int zbSt[] = {25, 25, 25, 25, 25, 25, 23, 23, 22, 18, 15, 13, 10,  4,  1,  1,  0,  0,  0,  0,  0,  0};
int zbFl[] = {25, 25, 25, 25, 25, 25, 25, 25, 23, 23, 21, 20, 17, 14,  7,  3,  1,  1,  1,  0,  0,  0};

// Various
const char pstrD[]  PROGMEM = "%d";
const char pstrDD[] PROGMEM = "%d.%d";
const char pstrSP[] PROGMEM = " ";
const char pstrCL[] PROGMEM = ":";
const char pstrSL[] PROGMEM = "/";


#undef max
#define max(a,b) ((a)>(b)?(a):(b))
#undef min
#define min(a,b) ((a)<(b)?(a):(b))


/**
  Simple median filter: get the median
  2014-03-25: started by David Cary

  @param idx the index in round median array
  @return the median
*/
int rMedOut(int idx) {
  // Return the last value if the buffer is not full yet
  if (rMed[idx][0] < 3) return rMed[idx][3];
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
  else     snprintf_P(buf, len, PSTR("%d.%d.%d.%d"), ip[0], ip[1], ip[2], ip[3]);
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
      ntpNextSync += 1UL * 60 * 1000;
      ntpOk = false;
      // Try to get old time from eeprom, if time delta is zero
    }
    else {
      // Compute the new time delta
      ntpDelta = utm - (millis() / 1000);
      // Time sync has succeeded, sync again in 8 hours
      ntpNextSync += 8UL * 60 * 60 * 1000;
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
  // Discard the rest of the packet
  ntpClient.flush();
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
  snprintf_P(buf, len, PSTR("%d days, %02d:%02d:%02d"), dd, hh, mm, ss);
  // Return the uptime in seconds
  return upt;
}


/**
  Display the WiFi parameters
*/
void showWiFi() {
  if (WiFi.isConnected()) {
    char ipbuf[16], gwbuf[16], nsbuf[16];

    charIP(WiFi.localIP(), ipbuf, sizeof(ipbuf), true);
    charIP(WiFi.gatewayIP(), gwbuf, sizeof(ipbuf), true);
    charIP(WiFi.dnsIP(), nsbuf, sizeof(ipbuf), true);

    Serial.println();
    Serial.print(F("WiFi connected to "));
    Serial.print(WiFi.SSID());
    Serial.print(F(" on channel "));
    Serial.print(WiFi.channel());
    Serial.print(F(", RSSI "));
    Serial.print(WiFi.RSSI());
    Serial.println(F(" dBm."));
    Serial.print(F(" IP : "));
    Serial.println(ipbuf);
    Serial.print(F(" GW : "));
    Serial.println(gwbuf);
    Serial.print(F(" DNS: "));
    Serial.println(nsbuf);
    Serial.println();
  }
  else {
    Serial.println();
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
  Publish char array from program memory to topic
*/
boolean mqttPub_P(const char *payload, const char *lvl1, const char *lvl2 = NULL, const char *lvl3 = NULL, const boolean retain = false) {
  const int bufSize = 64;
  char buf[bufSize] = "";
  strncpy_P(buf, payload, bufSize);
  return mqttPub(buf, lvl1, lvl2, lvl3, retain);
}

/**
  Publish char array from program memory to topic and retain
*/
boolean mqttPubRet_P(const char *payload, const char *lvl1, const char *lvl2 = NULL, const char *lvl3 = NULL, const boolean retain = true) {
  return mqttPub_P(payload, lvl1, lvl2, lvl3, retain);
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
  char buf[bufSize];
  strncpy(buf, mqttTopicRpt, bufSize);
  strcat_P(buf, pstrSL);
  strcat(buf, nodename);
  if (mqttClient.connect(mqttId, buf, 0, true, "offline")) {
    // Publish the "online" status
    mqttPubRet("online", buf);

    // Publish the connection report
    strcat_P(buf, PSTR("/wifi"));
    mqttPubRet(WiFi.hostname().c_str(), buf, "hostname");
    mqttPubRet(WiFi.macAddress().c_str(), buf, "mac");
    mqttPubRet(WiFi.SSID().c_str(), buf, "ssid");
    mqttPubRet(WiFi.RSSI(), buf, "rssi");
    char ipbuf[16];
    charIP(WiFi.localIP(), ipbuf, sizeof(ipbuf));
    mqttPubRet(ipbuf, buf, "ip");
    charIP(WiFi.gatewayIP(), ipbuf, sizeof(ipbuf));
    mqttPubRet(ipbuf, buf, "gw");

    // Subscribe to command topic
    mqttSubscribe(mqttTopicCmd, nodename, true);

    Serial.print(F("MQTT connected to "));
    Serial.println(mqttServer);
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
void mqttCallback(char* topic, byte* payload, unsigned int length) {
  // Make a limited copy of the payload and make sure it ends with \0
  char message[100];
  if (length > 100) length = 100;
  memcpy(message, payload, length);
  message[length] = '\0';

#ifdef DEBUG
  Serial.printf("MQTT %s: %s\r\n", topic, message);
#endif

  // Decompose the topic
  char *pRoot = NULL, *pTrunk = NULL, *pBranch = NULL;
  pRoot = topic;
  pTrunk = strchr(pRoot, '/');
  if (pTrunk != NULL)
    *pTrunk++ = '\0';
  pBranch = strchr(pTrunk, '/');
  if (pBranch != NULL)
    *pBranch++ = '\0';

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
  Send an APRS packet and, eventuall, print it to serial line

  @param *pkt the packet to send
*/
void aprsSend(const char *pkt) {
  aprsClient.print(pkt);
  yield();
  //aprsClient.write((uint8_t *)pkt, strlen(pkt));
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
  strcat_P(aprsPkt, VERSION);
  strcat_P(aprsPkt, eol);
  aprsSend(aprsPkt);
}

/**
  Send APRS weather data, then try to get the forecast
  FW0690>APRS,TCPIP*:@152457h4427.67N/02608.03E_.../...g...t044h86b10201L001WxSta

  @param temp temperature
  @param hmdt relative humidity
  @param pres athmospheric pressure (QNH)
  @param srad solar radiation
*/
void aprsSendWeather(int temp, int hmdt, int pres, int srad) {
  // Forecast as status report
  int zbCode = zbForecast((float)pres);
  if (zbCode >= 0) {
    const int bufSize = 40;
    char buf[bufSize] = "";
    strncpy_P(buf, (char*)pgm_read_word(&(zbFc[zbCode])), bufSize);
    aprsSendStatus(buf);
  }
  // Weather report
  const int bufSize = 8;
  char buf[bufSize] = "";
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
  else {
    strcat_P(aprsPkt, PSTR("t..."));
  }
  // Humidity
  if (hmdt >= 0) {
    if (hmdt == 100) {
      strcat_P(aprsPkt, PSTR("h00"));
    }
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
  // Illuminance, if valid
  if (srad >= 0 and srad <= 999) {
    snprintf_P(buf, bufSize, PSTR("L%03d"), srad);
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
  strcat_P(aprsPkt, PSTR("T"));
  // TODO
  snprintf_P(buf, bufSize, PSTR("#%03d,%03d,%03d,%03d,%03d,%03d,"), aprsTlmSeq, p1, p2, p3, p4, p5);
  strncat(aprsPkt, buf, bufSize);
  itoa(bits, buf, 2);
  strncat(aprsPkt, buf, bufSize);
  strcat_P(aprsPkt, eol);
  aprsSend(aprsPkt);
}

/**
  Send APRS telemetry setup
*/
void aprsSendTelemetrySetup() {
  const int padSize = 10;
  char padCallSign[padSize] = " ";
  strcpy_P(padCallSign, aprsCallSign);  // Workaround
  sprintf_P(padCallSign, PSTR("%-9s"), padCallSign);
  // Parameter names
  strcpy_P(aprsPkt, aprsCallSign);
  strcat_P(aprsPkt, aprsPath);
  strcat_P(aprsPkt, pstrCL);
  strncat(aprsPkt, padCallSign, padSize);
  strcat_P(aprsPkt, aprsTlmPARM);
  strcat_P(aprsPkt, eol);
  aprsSend(aprsPkt);
  // Equations
  strcpy_P(aprsPkt, aprsCallSign);
  strcat_P(aprsPkt, aprsPath);
  strcat_P(aprsPkt, pstrCL);
  strncat(aprsPkt, padCallSign, padSize);
  strcat_P(aprsPkt, aprsTlmEQNS);
  strcat_P(aprsPkt, eol);
  aprsSend(aprsPkt);
  // Units
  strcpy_P(aprsPkt, aprsCallSign);
  strcat_P(aprsPkt, aprsPath);
  strcat_P(aprsPkt, pstrCL);
  strncat(aprsPkt, padCallSign, padSize);
  strcat_P(aprsPkt, aprsTlmUNIT);
  strcat_P(aprsPkt, eol);
  aprsSend(aprsPkt);
  // Bit sense and project name
  strcpy_P(aprsPkt, aprsCallSign);
  strcat_P(aprsPkt, aprsPath);
  strcat_P(aprsPkt, pstrCL);
  strncat(aprsPkt, padCallSign, padSize);
  strcat_P(aprsPkt, aprsTlmBITS);
  strcat(aprsPkt, NODENAME);
  strcat_P(aprsPkt, pstrSL);
  strcat(aprsPkt, VERSION);
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
    strcat(aprsPkt, message);
    strcat_P(aprsPkt, eol);
    aprsSend(aprsPkt);
  }
}

/**
  Send APRS position and altitude
  FW0690>APRS,TCPIP*:!DDMM.hhN/DDDMM.hhW$comments

  @param comment the comment to append
*/
void aprsSendPosition(const char *comment = NULL) {
  // Compose the APRS packet
  strcpy_P(aprsPkt, aprsCallSign);
  strcat_P(aprsPkt, aprsPath);
  strcat_P(aprsPkt, PSTR("!"));
  strcat_P(aprsPkt, aprsLocation);
  strcat_P(aprsPkt, PSTR("/A="));
  char buf[7];
  sprintf_P(buf, PSTR("%06d"), altFeet);
  strncat(aprsPkt, buf, sizeof(buf));
  strcat_P(aprsPkt, pstrSP);
  if (comment != NULL) strcat(aprsPkt, comment);
  else strcat(aprsPkt, NODENAME);
  if (PROBE) strcat_P(aprsPkt, PSTR(" [PROBE]"));
  strcat_P(aprsPkt, eol);
  aprsSend(aprsPkt);
}

/**
  Get the Zambretti forecast
  @param zbCurrent current value of the atmospheric pressure
  @return the Zambretti forecast
*/
int zbForecast(float zbCurrent) {
  // Prepare the result
  int result = -1;
  // Keep the current value in the circular buffer for linear regression
  rgStore(zbCurrent);
  // If first run, set the timeout to the next hour
  unsigned long utm = timeUNIX(false);
  unsigned long  mm = (utm % 3600) / 60;
  if (zbNextTime == 0) zbNextTime = millis() + (60UL - mm) * 60000UL;
  else {
    if (millis() >= zbNextTime) {
      // Timer expired
      int trend = 0;
      int index = 0;
      float range = zbBaroTop - zbBaroBot;
      // Compute the linear regression
      rgLnRegr();
      Serial.print(F("Linear regression: "));
      Serial.print(rgAB[0]);
      Serial.print(" ");
      Serial.print(rgAB[1]);
      Serial.print(" ");
      Serial.print(rgAB[2]);
      Serial.println();
      // Compute the pressure variation and last pressure (according to equation)
      float pVar = rgAB[0] * rgCnt;
      float pLst = pVar + rgAB[1];
      // Get the trend
      if      (pVar >  zbBaroTrs) trend =  1;
      else if (pVar < -zbBaroTrs) trend = -1;
      // Corrections for summer
      // FIXME
      //int mon = month();
      //if ((mon >= 4) and (mon <= 9)) {
      //  if      (trend > 0) pLst += range * 0.07;
      //  else if (trend < 0) pLst -= range * 0.07;
      //}
      // Validate the interval
      if (pLst > zbBaroTop) pLst = zbBaroTop - 2 * zbBaroTrs;
      // Get the forecast
      index = (int)(22 * (pLst - zbBaroBot) / range);
      if ((index >= 0) and (index <= 22)) {
        if      (trend > 0) result = zbRs[index];
        else if (trend < 0) result = zbFl[index];
        else                result = zbSt[index];
      }
      // Set the next timer
      zbNextTime += zbDelay;
    }
  }
  return result;
}


/**
  Store previous athmospheric pressure values for each report in the last zbHours hours
  @param y current value
*/
void rgStore(float y) {
  rgY[rgIdx] = y;
  rgIdx++;
  if (rgIdx >= rgMax) rgIdx = 0; // wrap around
  if (rgCnt < rgMax)  rgCnt++;   // count the stored values
}

/**
  Linear regression
*/
void rgLnRegr() {
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
    // Store the coefficients
    rgAB[0] = a2;
    rgAB[1] = a1;
    rgAB[2] = sqrt(s / (rgCnt - 1));
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

  // Try to connect to WiFi
  WiFiManager wifiManager;
  // Reset settings
  //wifiManager.resetSettings();
  wifiManager.setTimeout(300);
  wifiManager.setAPCallback(wifiCallback);
  wifiManager.autoConnect(NODENAME);
  while (!wifiManager.autoConnect(NODENAME))
    delay(1000);

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
  atmo.settings.commInterface = I2C_MODE;
  atmo.settings.I2CAddress = atmoAddr;
  atmo.settings.runMode = 3;
  atmo.settings.tStandby = 0;
  atmo.settings.filter = 0;
  atmo.settings.tempOverSample = 1;
  atmo.settings.pressOverSample = 1;
  atmo.settings.humidOverSample = 1;
  delay(10);
  atmoOK = atmo.begin() == 0x60;
  if (atmoOK) Serial.println(F("BME280  sensor detected"));
  else        Serial.println(F("BME280  sensor missing"));
  yield();

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
    if (++aprsMsrmCount >= aprsMsrmMax)
      // Restart the counter
      aprsMsrmCount = 0;
    // Repeat sensor reading after the delay
    snsNextTime += snsDelay;
    // Set the telemetry bit 7 if the station is being probed
    if (PROBE) aprsTlmBits = B10000000;
    else       aprsTlmBits = B00000000;

#ifdef DEBUG
    Serial.print(F("Sensor reading "));
    Serial.println(aprsMsrmCount);
#endif

    // Check the time and set the telemetry bit 0 if time is not accurate
    unsigned long utm = timeUNIX();
    if (!ntpOk) aprsTlmBits |= B00000001;
    // Set the telemetry bit 1 if the uptime is less than one day (recent reboot)
    if (millis() < 86400000UL) aprsTlmBits |= B00000010;

    // Check again whether the sensor is present
    if (!atmoOK) atmoOK = atmo.begin() == 0x60;
    // Read the athmospheric sensor BME280
    if (atmoOK) {
      // Set the bit 6 to show the sensor is present (reverse)
      aprsTlmBits |= B01000000;
      // Get the weather parameters
      float temp = atmo.readTempC();
      float pres = atmo.readFloatPressure();
      float slvl = pres * altCorr;
      float hmdt = atmo.readFloatHumidity();
      float dewp = 243.04 *
                   (log(hmdt / 100.0) + ((17.625 * temp) / (243.04 + temp))) /
                   (17.625 - log(hmdt / 100.0) - ((17.625 * temp) / (243.04 + temp)));
      // Add to the round median filter
      rMedIn(MD_TEMP, (int)(temp * 9 / 5 + 32));      // Store directly integer Fahrenheit
      rMedIn(MD_PRES, (int)(slvl / 10));              // Store directly sea level in dPa
      rMedIn(MD_HMDT, (int)hmdt);                     // Humidity
      // Compose and publish the telemetry
      mqttPub((int)temp, mqttTopicSns, "temperature");
      mqttPub((int)hmdt, mqttTopicSns, "humidity");
      mqttPub((int)dewp, mqttTopicSns, "dewpoint");
      mqttPub((int)(pres / 100), mqttTopicSns, "pressure");
      mqttPub((int)(slvl / 100), mqttTopicSns, "sealevel");
    }
    else {
      // Store invalid values if no sensor
      rMedIn(MD_TEMP, -500);
      rMedIn(MD_PRES, -1);
      rMedIn(MD_HMDT, -1);
    }
    yield();

    // Check again whether the sensor is present
    if (!lightOK) {
      light.begin();
      unsigned char ID;
      if (light.getID(ID)) {
        light.setTiming(lightGain, lightSHTR, lightMS);
        light.setPowerUp();
        lightOK = true;
      }
    }
    // Read the light sensor TSL2561
    if (lightOK) {
      // Set the bit 5 to show the sensor is present (reverse)
      aprsTlmBits |= B00100000;
      unsigned int luxVis = 0, luxIrd = 0;
      double lux = -1;
      int solRad = -1;
      if (light.getData(luxVis, luxIrd)) {
        // TODO Adapt the shutter if saturated
        if (light.getLux(lightGain, lightMS, luxVis, luxIrd, lux)) {
          // Compose and publish the telemetry
          mqttPub((int)lux,    mqttTopicSns, "illuminance");
          mqttPub((int)luxVis, mqttTopicSns, "visible");
          mqttPub((int)luxIrd, mqttTopicSns, "infrared");
          // Calculate the solar radiation in W/m^2
          solRad = (int)(lux * 0.0079);
          // If the sensor is saturated, limit the reading to maximum value
          if (solRad > 999) solRad = 999;
        }
        else {
          // Saturated
          lux = -1;
          // If the sensor is saturated, limit the reading to maximum value
          if (solRad > 999) solRad = 999;
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
    }
    else {
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
    if (vcc < 3000 or vcc > 3600) aprsTlmBits |= B00001000;
    // Get RSSI
    int rssi = WiFi.RSSI();
    rMedIn(MD_RSSI, rssi);

    // Create the reporting topic
    char topic[32] = "";
    char text[32] = "";
    strncpy(topic, mqttTopicRpt, sizeof(topic));
    strcat_P(topic, pstrSL);
    strcat(topic, nodename);
    // Uptime
    unsigned long ups = 0;
    ups = uptime(text, sizeof(text));
    mqttPubRet(ups, topic, "uptime");
    mqttPubRet(text, topic, "uptime", "text");
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
        // Send the position, altitude and comment in firsts 10 minutes after boot
        if (millis() < 600000UL) aprsSendPosition();
        // Send weather data
        aprsSendWeather(rMedOut(MD_TEMP),
                        rMedOut(MD_HMDT),
                        rMedOut(MD_PRES),
                        rMedOut(MD_SRAD));
        // Send the telemetry
        //aprsSendTelemetrySetup();
        aprsSendTelemetry(rMedOut(MD_VCC) >> 2 - 625,
                          -rMedOut(MD_RSSI),
                          rMedOut(MD_HEAP) >> 8,
                          rMedOut(MD_VISI) >> 8,
                          rMedOut(MD_IRED) >> 8,
                          aprsTlmBits);
        //aprsSendStatus("Fine weather");
        // Close the connection
        aprsClient.stop();
      };
    }
  }
}
