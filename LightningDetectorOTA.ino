/*
  based on:
  LightningDetector.pde - AS3935 Franklin Lightning Sensor™ IC by AMS library demo code
  Copyright (c) 2012 Raivis Rengelis (raivis [at] rrkb.lv). All rights reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 3 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA


*/
#include <ESP8266WiFi.h>
#include <SPI.h>
#include <AS3935.h>
#include <ESP8266mDNS.h> // 2019-10-22
#include <WiFiUdp.h> // Needed for NTP as well as mDNSResolver
#include <ArduinoOTA.h>
#include <TimeLib.h>

#include <mDNSResolver.h> // Also needs WiFiUdp.h
#include <PubSubClient.h>
using namespace mDNSResolver;
WiFiUDP mdns;             // 2019-11-04
Resolver resolver(mdns);

#define WLAN_SSID       "MyAccessPoint"               // 343 main router
#define WLAN_PASS       "MyPassword"        
extern "C" {
  #include <user_interface.h>                    // to detect cold starts
}

const char *mqtt_servername = "MyComputer.local";
IPAddress mqtt_serveraddress;
WiFiClient espClient;
PubSubClient mqtt_client(espClient);
#define MQTT_PUBLISH "AS3935/messages"
#define MQTT_SUBSCRIBE "AS3935/commands"

long lastMsg = 0;
char msg[80];

IPAddress dns(8, 8, 8, 8);  //Google dns
IPAddress staticIP(192,168,0,30); // required for dns set 
IPAddress gateway(192,168,0,1);    
IPAddress subnet(255,255,255,0);

// NTP Servers:
static const char ntpServerName[] = "pool.ntp.org"; // 2019-03-24 from us.pool.ntp.org
const int timeZone = +8;     // Malaysia Time Time
WiFiUDP Udp;
unsigned int localPort = 8888;  // local port to listen for UDP packets
time_t getNtpTime();
void digitalClockDisplay();
void digitalClockStr();
void printDigits(int digits);
void sendNTPpacket(IPAddress &address);

void printAS3935Registers();

// Function prototype that provides SPI transfer and is passed to
// AS3935 to be used from within library, it is defined later in main sketch.
// That is up to user to deal with specific implementation of SPI
// Note that AS3935 library requires this function to have exactly this signature
// and it can not be member function of any C++ class, which happens
// to be almost any Arduino library
// Please make sure your implementation of choice does not deal with CS pin,
// library takes care about it on it's own
byte SPItransfer(byte sendByte);

// Iterrupt handler for AS3935 irqs
// and flag variable that indicates interrupt has been triggered
// Variables that get changed in interrupt routines need to be declared volatile
// otherwise compiler can optimize them away, assuming they never get changed
void AS3935Irq();
volatile int AS3935IrqTriggered;

// First parameter - SPI transfer function, second - Arduino pin used for CS
// and finally third argument - Arduino pin used for IRQ
// It is good idea to chose pin that has interrupts attached, that way one can use
// attachInterrupt in sketch to detect interrupt
// Library internally polls this pin when doing calibration, so being an interrupt pin
// is not a requirement

#define IRQpin 4
#define CSpin 15

AS3935 AS3935(SPItransfer,CSpin,IRQpin);


// 2019-10-21
//Hex command to send to serial for close relay
byte relON[]  = {0xA0, 0x01, 0x01, 0xA2};
//Hex command to send to serial for open relay
byte relOFF[] = {0xA0, 0x01, 0x00, 0xA1};
//Hex command to send to serial for close relay
byte rel2ON[]  = {0xA0, 0x02, 0x01, 0xA3};
//Hex command to send to serial for open relay
byte rel2OFF[] = {0xA0, 0x02, 0x00, 0xA2};

time_t LastDisconnect = now();
int disconnected = 0;

void publish_mqtt(const char *str) {
  snprintf (msg, 80, "%s", str);
  // does not mix well with mqtt publish // Serial.print("Publish message: ");
  //Serial.println(msg);
  mqtt_client.publish(MQTT_PUBLISH, msg);
  mqtt_client.loop(); // 2019-11-06
}

// 2019-11-04 PubSubClient
void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("MQTT message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();

  // Switch on the LED if an 1 was received as first character
  if ((char)payload[0] == '1') {
    digitalClockStr("Reconnecting the phone lines ...");
    Serial.println("Reconnecting the phone lines ...");
    Serial.write (relON, sizeof(relON)); 
    delay(100); // give relay cpu some time before the next command
    Serial.write (rel2ON, sizeof(rel2ON)); 
    delay(100); 
    disconnected = 0;           
  }
  else 
  {
    digitalClockStr("Disconnecting the phone lines ...");
    Serial.println("Disconnecting the phone lines ...");
    Serial.write (relOFF, sizeof(relOFF));
    delay(100);     
    Serial.write (rel2OFF, sizeof(rel2OFF));
    disconnected = 1; 
    LastDisconnect = now();
  }
}

long lastReconnectAttempt = 0; // 2019-11-05 Non-blocking reconnect

boolean reconnect() {
  Serial.print("Attempting MQTT reconnection...");
  // Create a random client ID
  String clientId = "heong"; // 2019-03-23
  clientId += String(random(0xffff), HEX);
  // Attempt to connect
  if (mqtt_client.connect(clientId.c_str())) {
    Serial.println("connected");
    // Once connected, publish an announcement...
    // client.publish("outTopic", "hello esp8266 mqtt world");
    publish_mqtt("AS3935 lightning detector reconnected\n");
    // mqtt_client.publish("AS3935 lightning detector reconnected\n");
    // ... and resubscribe
    mqtt_client.subscribe(MQTT_SUBSCRIBE);
    mqtt_client.loop();
  } else {
      Serial.print("failed, rc=");
      Serial.println(mqtt_client.state());
  }
  return mqtt_client.connected();
}

void setup()
{ // 2019-11-12 Try not to mqtt_publish() as it may not be set up yet
  // Serial.begin(9600);
  Serial.begin(115200); // 2019-10-19

  Serial.print("Connecting to ");
  
  Serial.println(WLAN_SSID);

  WiFi.mode(WIFI_OFF); // 2018-12-11 workaround from 2018-10-09
  WiFi.mode(WIFI_STA); // 2018-11-27 remove rogue AP
  WiFi.config(staticIP, dns, gateway, subnet);
  WiFi.begin(WLAN_SSID, WLAN_PASS); // 2018-12-11 changed to after WIFI_STA
  while (WiFi.waitForConnectResult() != WL_CONNECTED) { // 2019-10-17
    Serial.println("Connection Failed! Rebooting...");
    delay(5000);
    ESP.restart();
  }

  randomSeed(micros()); // 2019-11-04
  
  Serial.println();

  Serial.println("WiFi connected");
  Serial.println("IP address: "); 
  Serial.println(WiFi.localIP());

  rst_info *rinfo;
  rinfo = ESP.getResetInfoPtr();
  Serial.println(String("ResetInfo.reason = ") + (*rinfo).reason);
  // if ((*rinfo).reason == 0)  // power up    
  // There is a problem: 'int disconnected' needs to be saved as well. Until this is done do
  // not apply cold-start code
  // It would be nice to read the relays states from the 2nd cpu  
  Serial.write (relON, sizeof(relON)); // Connect phone line 
  delay(100); // give relay cpu some time before the next command
  Serial.write (rel2ON, sizeof(rel2ON)); 

  // OTA
  // Port defaults to 8266
  // ArduinoOTA.setPort(8266);

  // Hostname defaults to esp8266-[ChipID]
  ArduinoOTA.setHostname("LightningDetectorOTA");

  // No authentication by default
  ArduinoOTA.setPassword("drachenzahne");

  // Password can be set with it's md5 value as well
  // MD5(admin) = 21232f297a57a5a743894a0e4a801fc3
  // ArduinoOTA.setPasswordHash("21232f297a57a5a743894a0e4a801fc3");

  ArduinoOTA.onStart([]() {
    String type;
    if (ArduinoOTA.getCommand() == U_FLASH) {
      type = "sketch";
    } else { // U_SPIFFS
      type = "filesystem";
    }

    // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
    Serial.println("Start updating " + type);
  });
  ArduinoOTA.onEnd([]() {
    Serial.println("\nEnd");
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
  });
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) {
      Serial.println("Auth Failed");
    } else if (error == OTA_BEGIN_ERROR) {
      Serial.println("Begin Failed");
    } else if (error == OTA_CONNECT_ERROR) {
      Serial.println("Connect Failed");
    } else if (error == OTA_RECEIVE_ERROR) {
      Serial.println("Receive Failed");
    } else if (error == OTA_END_ERROR) {
      Serial.println("End Failed");
    }
  });
  ArduinoOTA.begin();
  Serial.println("Ready");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP()); 
    
  
  // first begin, then set parameters
  SPI.begin();
  // NB! chip uses SPI MODE1
  SPI.setDataMode(SPI_MODE1);
  // NB! max SPI clock speed that chip supports is 2MHz,
  // but never use 500kHz, because that will cause interference
  // to lightning detection circuit
  SPI.setClockDivider(SPI_CLOCK_DIV16);
  // and chip is MSB first
  SPI.setBitOrder(MSBFIRST);
  // reset all internal register values to defaults
  AS3935.reset();
  // and run calibration
  // if lightning detector can not tune tank circuit to required tolerance,
  // calibration function will return false
  
  
  if(!AS3935.calibrate())
    Serial.println("Tuning out of range, check your wiring, your sensor and make sure physics laws have not changed!");  

  outputCalibrationValues();
  recalibrate();

  AS3935.setNoiseFloor(3); // from 1
  AS3935.setSpikeRejection(0);
  AS3935.setWatchdogThreshold(5); // from 1
  
  outputCalibrationValues();
  recalibrate();

  // since this is demo code, we just go on minding our own business and ignore the fact that someone divided by zero

  // first let's turn on disturber indication and print some register values from AS3935
  // tell AS3935 we are indoors, for outdoors use setOutdoors() function
  //AS3935.setIndoors();
  AS3935.setOutdoors();
  // turn on indication of distrubers, once you have AS3935 all tuned, you can turn those off with disableDisturbers()
  AS3935.enableDisturbers();
  // AS3935.disableDisturbers();
  printAS3935Registers();
  AS3935IrqTriggered = 0; 
  // Using interrupts means you do not have to check for pin being set continiously, chip does that for you and
  // notifies your code
  // demo is written and tested on ChipKit MAX32, irq pin is connected to max32 pin 2, that corresponds to interrupt 1
  // look up what pins can be used as interrupts on your specific board and how pins map to int numbers

  // ChipKit Max32 - irq connected to pin 2
  // attachInterrupt(1,AS3935Irq,RISING);
  // uncomment line below and comment out line above for Arduino Mega 2560, irq still connected to pin 2
  attachInterrupt(digitalPinToInterrupt(IRQpin),AS3935Irq,RISING);

  Udp.begin(localPort);
  Serial.print("Local port: ");
  Serial.println(Udp.localPort());
  Serial.println("waiting for sync");
  setSyncProvider(getNtpTime);
  setSyncInterval(300);

  Serial.print("Resolving ");
  Serial.println(mqtt_servername);
  resolver.setLocalIP(WiFi.localIP());

  mqtt_serveraddress = resolver.search(mqtt_servername);

  if(mqtt_serveraddress != INADDR_NONE) {
    Serial.print("Resolved: ");
    Serial.println(mqtt_serveraddress);
  } 
  else {
    Serial.println("Not resolved");
    Serial.println("Connection Failed! Rebooting in 5s ...");
    delay(5000);
    ESP.restart();
  }

  mqtt_client.setServer(mqtt_serveraddress, 1883); // Use mdns name
  mqtt_client.setCallback(callback);
  lastReconnectAttempt = 0; // 2019-11-05
}

static int dots = 0;
static int val = 0; 
static int commas = 0;

void loop()
{
   ArduinoOTA.handle(); // 2019-10-22
  
  // here we go into loop checking if interrupt has been triggered, which kind of defeats
  // the whole purpose of interrupts, but in real life you could put your chip to sleep
  // and lower power consumption or do other nifty things
  if(AS3935IrqTriggered)  
  {
    // reset the flag
    AS3935IrqTriggered = 0;
    // wait 2 ms before reading register (according to datasheet?)
    delay(2);
    // first step is to find out what caused interrupt
    // as soon as we read interrupt cause register, irq pin goes low
    int irqSource = AS3935.interruptSource();
    // returned value is bitmap field, bit 0 - noise level too high, bit 2 - disturber detected, and finally bit 3 - lightning!
    if (irqSource & 0b0001)
    {
      Serial.println("Noise level too high, try adjusting noise floor");
      digitalClockStr("Noise level too high, try adjusting noise floor");
    }  
    if (irqSource & 0b0100)
    {
      digitalClockDisplay();
      Serial.println("Disturber detected");
      digitalClockStr("Disturber detected");
    }
    if (irqSource & 0b1000)
    {
      // need to find how far that lightning stroke, function returns approximate distance in kilometers,
      // where value 1 represents storm in detector's near victinity, and 63 - very distant, out of range stroke
      // everything in between is just distance in kilometers
      int strokeDistance = AS3935.lightningDistanceKm();
      if (strokeDistance == 1)
      {
        digitalClockStr("Storm overhead, watch out! Disconnecting the phone lines ..........");
        delay(100);
        LastDisconnect = now();
        disconnected = 1;
        digitalClockDisplay();
        Serial.println("Storm overhead, watch out!");
        Serial.println("Disconnecting the phone lines ..........");
        Serial.write (relOFF, sizeof(relOFF)); 
        delay(100); // give relay cpu some time before the next command
        Serial.write (rel2OFF, sizeof(rel2OFF));         
        delay(100); // give relay cpu some time before the next command
      }  
      if (strokeDistance == 63)
      {
        Serial.println("Out of range lightning detected.");
        digitalClockStr("Out of range lightning detected.");
      }  
      if (strokeDistance < 63 && strokeDistance > 1)
      {
        digitalClockDisplay();
        Serial.print("Lightning detected ");
        Serial.print(strokeDistance,DEC);
        Serial.println(" kilometers away.");

        char publishstr[80];
        sprintf(publishstr, "Lightning detected %d km away", strokeDistance);
        digitalClockStr(publishstr);
        
        if (disconnected && strokeDistance < 9) // do not be in a hurry to reconnect
          LastDisconnect = now();
      }
    }
  }

  if (disconnected && ((now() - LastDisconnect) > 1800))
  {
    disconnected = 0;
    digitalClockDisplay();
    Serial.println("Reconnecting the phone lines ...");
    digitalClockStr("Reconnecting the phone lines ...");
    Serial.write (relON, sizeof(relON)); 
    delay(100); // give relay cpu some time before the next command
    Serial.write (rel2ON, sizeof(rel2ON)); 
    delay(100);            
  }
  
  /*
  if(!AS3935.calibrate()) 
  {
    Serial.println("Tuning out of range, check your wiring, your sensor and make sure physics laws have not changed!");
  } 
  */
  // 2019-02-24 Check for broken wifi links
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("No WiFi! Initiating reset ...");
    Serial.write(relON, sizeof(relON)); // 2019-10-16 
    delay(5000);  // 2019-10-14
    ESP.restart();
  }

  // 2019-11-05 nonblocking reconnects
  if (!mqtt_client.connected()) { 
    long mqtt_now = millis();
    if (mqtt_now - lastReconnectAttempt > 5000) 
    {
      lastReconnectAttempt = mqtt_now;
      Serial.print("mqtt_client.connected() returns false ...");
      Serial.print("Retrying after 5s ...");
      if (reconnect())
      {
        lastReconnectAttempt = 0;
        Serial.println(" reconnected!");
      }
    }
  }
  else 
  {
    long mqtt_now = millis();
    if (mqtt_now - lastMsg > 3600000)
    {
      lastMsg = mqtt_now;
      

      //snprintf(publishstr, 80, "MQTT loop time is %ld\n", mqtt_now);
      //publish_mqtt(publishstr);

      //mqtt_client.publish(MQTT_PUBLISH, publishstr);
      
      //snprintf(publishstr, 80, "%02d:%02d:%02d %02d.%02d.%02d %s\n", hour(), minute(), second(), day(), month(), year(), "Woo hoo!");
      //publish_mqtt(publishstr);

      digitalClockStr("MQTT loop time");
      // publish_mqtt(publishstr);
    }  
  }
  mqtt_client.loop();
}

void printAS3935Registers()
{
  int noiseFloor = AS3935.getNoiseFloor();
  int spikeRejection = AS3935.getSpikeRejection();
  int watchdogThreshold = AS3935.getWatchdogThreshold();
  int minLightning = AS3935.getMinimumLightnings();
  Serial.print("Noise floor is: ");
  Serial.println(noiseFloor,DEC);
  Serial.print("Spike rejection is: ");
  Serial.println(spikeRejection,DEC);
  Serial.print("Watchdog threshold is: ");
  Serial.println(watchdogThreshold,DEC); 
  Serial.print("Minimum Lightning is: ");
  Serial.println(minLightning,DEC);   
}

// this is implementation of SPI transfer that gets passed to AS3935
// you can (hopefully) wrap any SPI implementation in this
byte SPItransfer(byte sendByte)
{
  return SPI.transfer(sendByte);
}

// this is irq handler for AS3935 interrupts, has to return void and take no arguments
// always make code in interrupt handlers fast and short
void ICACHE_RAM_ATTR AS3935Irq()
{
  AS3935IrqTriggered = 1;
}


void recalibrate() {
  delay(50);
  Serial.println();
  int calCap = AS3935.getBestTune();
  Serial.print("antenna calibration picks value:\t ");
  Serial.println(calCap);
  delay(50);
}

void outputCalibrationValues() {
   // output the frequencies that the different capacitor values set:
  delay(50);
  Serial.println();
  for (byte i = 0; i <= 0x0F; i++) {
    int frequency = AS3935.tuneAntenna(i);
    Serial.print("tune antenna to capacitor ");
    Serial.print(i);
    Serial.print("\t gives frequency: ");
    Serial.print(frequency);
    Serial.print(" = ");
    long fullFreq = (long) frequency*160;  // multiply with clock-divider, and 10 (because measurement is for 100ms)
    Serial.print(fullFreq,DEC);
    Serial.println(" Hz");
    delay(10);
  }
}

int digitalClockStr(const char *message)
{
  char clockstr[80];

  // output digital clock time to char buffer
  snprintf(clockstr, 80, "%02d:%02d:%02d %02d.%02d.%02d %s\n", hour(), minute(), second(), year(), month(), day(), message);
  publish_mqtt(clockstr);
  mqtt_client.loop();
}

void digitalClockDisplay()
{
  // digital clock display of the time
  Serial.print(hour());
  printDigits(minute());
  printDigits(second());
  Serial.print(" ");
  Serial.print(day());
  Serial.print(".");
  Serial.print(month());
  Serial.print(".");
  Serial.print(year());
  // Serial.println();
  // debugV("%d:%d:%d", hour(), minute(), second());
  Serial.print(" ");
}


void printDigits(int digits)
{
  // utility for digital clock display: prints preceding colon and leading 0
  Serial.print(":");
  if (digits < 10)
    Serial.print('0');
  Serial.print(digits);
}

/*-------- NTP code ----------*/

const int NTP_PACKET_SIZE = 48; // NTP time is in the first 48 bytes of message
byte packetBuffer[NTP_PACKET_SIZE]; //buffer to hold incoming & outgoing packets

time_t getNtpTime()
{
  IPAddress ntpServerIP; // NTP server's ip address

  while (Udp.parsePacket() > 0) ; // discard any previously received packets
  Serial.println("Transmit NTP Request");
  // get a random server from the pool
  WiFi.hostByName(ntpServerName, ntpServerIP);
  Serial.print(ntpServerName);
  Serial.print(": ");
  Serial.println(ntpServerIP);
  sendNTPpacket(ntpServerIP);
  uint32_t beginWait = millis();
  while (millis() - beginWait < 1500) {
    int size = Udp.parsePacket();
    if (size >= NTP_PACKET_SIZE) {
      Serial.println("Receive NTP Response");
      Udp.read(packetBuffer, NTP_PACKET_SIZE);  // read packet into the buffer
      unsigned long secsSince1900;
      // convert four bytes starting at location 40 to a long integer
      secsSince1900 =  (unsigned long)packetBuffer[40] << 24;
      secsSince1900 |= (unsigned long)packetBuffer[41] << 16;
      secsSince1900 |= (unsigned long)packetBuffer[42] << 8;
      secsSince1900 |= (unsigned long)packetBuffer[43];
      return secsSince1900 - 2208988800UL + timeZone * SECS_PER_HOUR;
    }
  }
  Serial.println("No NTP Response :-(");
  return 0; // return 0 if unable to get the time
}

// send an NTP request to the time server at the given address
void sendNTPpacket(IPAddress &address)
{
  // set all bytes in the buffer to 0
  memset(packetBuffer, 0, NTP_PACKET_SIZE);
  // Initialize values needed to form NTP request
  // (see URL above for details on the packets)
  packetBuffer[0] = 0b11100011;   // LI, Version, Mode
  packetBuffer[1] = 0;     // Stratum, or type of clock
  packetBuffer[2] = 6;     // Polling Interval
  packetBuffer[3] = 0xEC;  // Peer Clock Precision
  // 8 bytes of zero for Root Delay & Root Dispersion
  packetBuffer[12] = 49;
  packetBuffer[13] = 0x4E;
  packetBuffer[14] = 49;
  packetBuffer[15] = 52;
  // all NTP fields have been given values, now
  // you can send a packet requesting a timestamp:
  Udp.beginPacket(address, 123); //NTP requests are to port 123
  Udp.write(packetBuffer, NTP_PACKET_SIZE);
  Udp.endPacket();
}
