//#include "AH_MCP320x.h"
#include <SPI.h>
#include "vtx.h"
#include "HCT138.h"
#include "libraries/RunningAverage.h"

// WIFI testing
//#include "wifi.h"
//#include <ESP8266WiFi.h>

#define CS_PIN 0 //CS

//AH_MCP320x ADC_SPI(D8);

int readvalue;

const int DATAOUT = D7;
const int SPICLOCK = D5;
const int SELPIN = D8;
const int DATAIN = D6;
const long MIN_LAP_INTERVAL_MS = 5 * 1000;

const double ARM_THRESHOLD = 0.4;
  
boolean blockArming[] = {false, false, false, false};
boolean armed[] = {false, false, false, false};
double armedMaxRssi[] = {0, 0, 0, 0};
long armedMaxMs[] = {0, 0, 0, 0};

const int MIN_RSSI[] = {379, 372, 379, 335};
const int MAX_RSSI[] = {918, 912, 833, 734};
RunningAverage averages[] = {RunningAverage(10), RunningAverage(10), RunningAverage(10), RunningAverage(10)};

// WIFI testing
// declare telnet server (do NOT put in setup())
//WiFiServer telnetServer(23);
//WiFiClient serverClient;

void setup() {


  pinMode(D0, OUTPUT);
  pinMode(D1, OUTPUT);
  pinMode(D2, OUTPUT);
  pinMode(D5, OUTPUT);
  pinMode(D6, INPUT);
  pinMode(D7, OUTPUT);
  pinMode(D8, OUTPUT);

  Serial.begin(57600);
  Serial.println("#");
  Serial.println("#Setup begin");

  // WIFI testing
  //connectWifi();
  //telnetServer.begin();
  //telnetServer.setNoDelay(true);
  
  setupChannelList();

  delay(2000);

  //setChannelModule__(24);
  setPinHigh(0);
  setChannelModule(24);
  
  setPinHigh(1);
  setChannelModule(26);
  
  setPinHigh(2);
  setChannelModule(28);
  
  setPinHigh(3);
  setChannelModule(30);

  //set pin for adc
  setPinHigh(7);
}

double normalizeRssi(int channel, int rssi){
  if (rssi < MIN_RSSI[channel]){
    rssi = MIN_RSSI[channel];
  }else if (rssi > MAX_RSSI[channel]){
    rssi = MAX_RSSI[channel];
  }
  return (double)(rssi - MIN_RSSI[channel]) / (MAX_RSSI[channel] - MIN_RSSI[channel]);
}

double normalized = 0;
long now = 0;
long loopCount = 0;

/*
 * 
const double ARM_THRESHOLD = 0.5;
boolean armed[] = {false, false, false, false};
double armedMaxRssi[] = {0, 0, 0, 0};
long armedMaxMs[] = {0, 0, 0, 0};

 */
void handleChannel(int channel){
  readvalue = read_adc(channel);
  
  Serial.print("#channel: ");
  Serial.print(channel);
  Serial.print(", raw value: ");
  Serial.println(readvalue);
  
  normalized = normalizeRssi(channel, readvalue);
  averages[channel].addValue(normalized);

  now = millis();
  
  double avg = averages[channel].getAverage();
  if (!blockArming[channel] && !armed[channel] && avg >= ARM_THRESHOLD){
    if (now - armedMaxMs[channel] < MIN_LAP_INTERVAL_MS){
      Serial.print("#");
      Serial.print(now - armedMaxMs[channel]);
      Serial.print(" ms since last lap, blocking channel ");
      Serial.println(channel);
      blockArming[channel] = true;
    }else{
      Serial.print("#");
      Serial.print("Arming channel ");
      Serial.println(channel);
      armed[channel] = true;
      armedMaxRssi[channel] = avg;
    }
  }else if (armed[channel] && avg > armedMaxRssi[channel]){
    armedMaxMs[channel] = now;
    armedMaxRssi[channel] = avg;
  }else if (armed[channel] && avg < ARM_THRESHOLD){
    armed[channel] = false;
    Serial.print("#");
    Serial.print("New lap for channel ");
    Serial.print(channel);
    Serial.print(" at RSSI ");
    Serial.print(armedMaxRssi[channel]);
    Serial.print(" at MS ");
    Serial.println(armedMaxMs[channel]);

    Serial.print((char)1);
    Serial.print("TBD TBD ");
    Serial.print(channel);
    Serial.print(" ");
    Serial.println((double)armedMaxMs[channel]/1000);
    
    armedMaxRssi[channel] = 0;
  }else if (blockArming[channel] && avg < ARM_THRESHOLD){
    Serial.print("#");
    Serial.print("RSSI under threshold, unblocking channel ");
    Serial.println(channel);
    blockArming[channel] = false;
  }

  if (loopCount % 10 == 0){
    Serial.print("#;");
    Serial.print(millis());
    Serial.print(";");
    Serial.print(channel + 1);
    Serial.print(";");
    Serial.print(normalized, DEC);
    Serial.print(";");
    Serial.println(averages[channel].getAverage(), DEC);  
  }
}

void loop() {
  loopCount++;
  handleChannel(0);
  handleChannel(1);
  handleChannel(2);
  handleChannel(3);

  delay(5);
}


int read_adc(int channel) {
  int adcvalue = 0;
  byte commandbits = B11000000; //command bits - start, mode, chn (3), dont care (3)

  //allow channel selection
  commandbits |= ((channel) << 3);

  digitalWrite(SELPIN, LOW); //Select adc
  // setup bits to be written
  for (int i = 7; i >= 3; i--) {
    digitalWrite(DATAOUT, commandbits & 1 << i);
    //cycle clock
    digitalWrite(SPICLOCK, HIGH);
    digitalWrite(SPICLOCK, LOW);
  }

  digitalWrite(SPICLOCK, HIGH);   //ignores 2 null bits
  digitalWrite(SPICLOCK, LOW);
  digitalWrite(SPICLOCK, HIGH);
  digitalWrite(SPICLOCK, LOW);

  //read bits from adc
  for (int i = 11; i >= 0; i--) {
    adcvalue += digitalRead(DATAIN) << i;
    //cycle clock
    digitalWrite(SPICLOCK, HIGH);
    digitalWrite(SPICLOCK, LOW);
  }
  digitalWrite(SELPIN, HIGH); //turn off device
  return adcvalue;
}
//See more at: http://www.esp8266.com/viewtopic.php?p=38908#sthash.WbEX2gJE.dpuf


/*
void handleWifi(){
  
  if (telnetServer.hasClient()) {
    if (!serverClient || !serverClient.connected()) {
      if (serverClient) {
        serverClient.stop();
        Serial.println("Telnet Client Stop");
      }
      serverClient = telnetServer.available();
      Serial.println("New Telnet client");
      serverClient.flush();  // clear input buffer, else you get strange characters 
    }
  }

  //while(serverClient.available()) {  // get data from Client
  //  Serial.write(serverClient.read());
  //}
  
  if (loopCount % 100 == 0){
    if (serverClient && serverClient.connected()) {  // send data to Client
      serverClient.print("Telnet Test, millis: ");
      serverClient.println(millis());
      //serverClient.print("Free Heap RAM: ");
      //serverClient.println(ESP.getFreeHeap());
    }
  }
}
}

*/
