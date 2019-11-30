/*
 * Copyright (c) 2014 by Valery De Smedt <valery@indri.be>
 *
 * This file is free software; you can redistribute it and/or modify
 * it under the terms of either the GNU General Public License version 2
 * or the GNU Lesser General Public License version 2.1, both as
 * published by the Free Software Foundation.
 */

//This is made based on owsome work from Felix Risu
//It allows to reveive from RFM69 and communicate the payload added with the issuer nodeId to R-Pi
//It also allow for remote programming the Rfm69 node in the network
#include <version.h>
#include <Wire.h>
#include <ssd1306.h>
#include <Arduino.h>
#include <RFM69.h>
#include <SPI.h>
#include <SPIFlash.h>

#define PIN_PRO     6
#define PIN_HW      5
#define NODEID      254
#define FREQUENCY   RF69_868MHZ
#define ENCRYPTKEY  "passiondesfruits" //(16 bytes of your choice - keep the same on all encrypted nodes)
#define ACK_TIME    50  // # of ms to wait for an ack
#define TIMEOUT     3000
#define LOGSIZE     6

#ifdef __AVR_ATmega1284P__
  #define LED           15 // Moteino MEGAs have LEDs on D15
#else
  #define LED           9 // Moteinos have LEDs on D9
#endif
#define SERIAL_BAUD       115200

RFM69 radio;
uint8_t networkId = 0;
bool rfm69_hw = false;

char gtwBuffer[100];
byte gtwBufferLength;
char serBuffer[100];
byte serBufferLength;
byte targetID=0;


//Logging
char netId[20]; //Node identification to be displayed
char screenLogLine[20];
char screenLog[LOGSIZE][20];
unsigned long screenLogTimes[LOGSIZE];

unsigned long lastLogMillis = 0;
int screenLogIndex = 0;

void addToScreenLog(const char * log);
bool draw(void);
bool readDipSwitches();


void setup() {
  pinMode(PIN_HW, INPUT_PULLUP);
  pinMode(PIN_PRO, INPUT_PULLUP);
  pinMode(LED, OUTPUT);
  readDipSwitches();

  radio.initialize(RF69_868MHZ, NODEID, networkId);
  radio.encrypt(ENCRYPTKEY);
  radio.setHighPower(rfm69_hw);
  snprintf(netId, 20, "%s-%s [%s]", rfm69_hw?"HW":"W", networkId==50?"50(DEV)":"51(PRO)", VERSION);

  Serial.begin(SERIAL_BAUD);
  while(!Serial);

  ssd1306_128x64_i2c_init();
  ssd1306_fillScreen(0x00);
}

void loop() {
    if (radio.receiveDone()) {
      digitalWrite(LED,HIGH);
      int rssi = radio.RSSI;
      if(radio.DATALEN > 97) {
        gtwBufferLength = sprintf(gtwBuffer, "//ERR:Message.Size %u > 97 bytes", radio.DATALEN);
      }
      else {
        gtwBuffer[0] = radio.SENDERID;
        memcpy(gtwBuffer+1, (const void *)&rssi, 2);
        memcpy(gtwBuffer+3, (const void *)radio.DATA, radio.DATALEN);
        gtwBufferLength = 3+radio.DATALEN;
      }
      if (radio.ACKRequested())
        radio.sendACK();

      Serial.write(gtwBuffer, gtwBufferLength);
      Serial.println("");
      snprintf(screenLogLine, 20, "< %02d %03d %03d", (unsigned char)gtwBuffer[3], (uint8_t)gtwBuffer[0], (int)gtwBuffer[1]);
      addToScreenLog(screenLogLine);
      digitalWrite(LED,LOW);
    }

    while(Serial.available() > 0) {
      
      serBuffer[serBufferLength++] = Serial.read();
      if(serBufferLength >=2 && serBuffer[serBufferLength-2] == 13 && serBuffer[serBufferLength-1] == 10) {
        digitalWrite(LED, HIGH);
        uint8_t destNode = (int)serBuffer[0];
        Serial.print("//Sending to node "); Serial.println(destNode);
        bool success = radio.sendWithRetry(destNode, serBuffer+1, serBufferLength-3, 3, 40);
        digitalWrite(LED, LOW);        
        snprintf(screenLogLine, 20, "%s %02d %03d %03d", success?">":"x", serBuffer[1], (uint8_t)serBuffer[0], success?radio.RSSI:0);
        serBufferLength = 0;
        addToScreenLog(screenLogLine);
      }
    }

    //Detect dip switch changes
    if(readDipSwitches()) {
      radio.setNetwork(networkId);
      radio.setHighPower(rfm69_hw);
      snprintf(netId, 20, "%s-%s [%s]", rfm69_hw?"HW":"W", networkId==50?"50(DEV)":"51(PRO)", VERSION);
    }
    draw();
}

bool readDipSwitches() {
  uint8_t nnid = digitalRead(PIN_PRO)==LOW?51:50;
  bool nhw = digitalRead(PIN_HW) == LOW;
  if(nnid != networkId || nhw != rfm69_hw)
  {
    delay(50); //Debounce
    networkId = digitalRead(PIN_PRO)==LOW?51:50;
    rfm69_hw = digitalRead(PIN_HW) == LOW;
    return true;
  }
  return false;
}

void addToScreenLog(const char * log) {
    strncpy(screenLog[screenLogIndex], log, 20);
    screenLogTimes[screenLogIndex] = millis();
    screenLogIndex = ++screenLogIndex % LOGSIZE;
}

unsigned long lastPrint = millis();
bool draw(void ) {
  //Draw only every 500ms
  if(millis()-lastPrint < 1000)
    return false;

  ssd1306_charF6x8(0, 0, netId);
  ssd1306_charF6x8(0, 1, "      d mt rfi rss");
  
  int logIndex = 0;
  for(logIndex = 1 ; logIndex <= LOGSIZE ; logIndex++) 
  {
    int i = (screenLogIndex-logIndex+2*LOGSIZE) % LOGSIZE;
    unsigned long s = (millis()-screenLogTimes[i])/1000L;
    uint8_t sec = (uint8_t)(s % 60L);
    uint8_t min = (uint8_t)(((s - sec) / 60L) % 60L);
    uint8_t hou = (uint8_t)(((s - sec - 60 * min) / 3600L));
    
    if(hou > 0) 
      snprintf(screenLogLine, 20, "%02u:%02u", hou, min);
    else 
      snprintf(screenLogLine, 20, "%02u:%02u", min, sec);

    ssd1306_charF6x8(0, LOGSIZE-logIndex+2, screenLogLine);
    ssd1306_charF6x8(35, LOGSIZE-logIndex+2, screenLog[i]);
  }
  lastPrint = millis();
  return true;
}
