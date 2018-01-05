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

#define VER_MAJOR   1
#define VER_MINOR   2
#define NETWORKID   50  //50:DEV 150:TST 250:PRO
#define NODEID      254
#define FREQUENCY   RF69_868MHZ
#define ENCRYPTKEY  "passiondesfruits" //(16 bytes of your choice - keep the same on all encrypted nodes)
//#define IS_RFM69HW
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

void setup() {
  radio.initialize(FREQUENCY,NODEID,NETWORKID);
  radio.encrypt(ENCRYPTKEY);
#ifdef IS_RFM69HW
  radio.setHighPower();
#endif

  pinMode(LED, OUTPUT);
  Serial.begin(SERIAL_BAUD);
  while(!Serial);

#ifdef IS_RFM69HW
  snprintf(netId, 20, "HW-%d [%s]", NETWORKID, VERSION);
#else
  snprintf(netId, 20, "W-%d [%s]", NETWORKID, VERSION);
#endif

  ssd1306_128x64_i2c_init();
  ssd1306_fillScreen(0x00);
  ssd1306_charF6x8(0, 0, netId);
}

void loop() {
    if (radio.receiveDone()) {
      digitalWrite(LED,HIGH);
      int rssi = radio.RSSI;
      if(radio.DATALEN > 97) {
        gtwBufferLength = sprintf(gtwBuffer, "//ERR:Message.Size %i > 97 bytes", radio.DATALEN);
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
      snprintf(screenLogLine, 20, "%d fr %d (%d)", gtwBuffer[3], gtwBuffer[0], (int)gtwBuffer[1]);
      addToScreenLog(screenLogLine);
      digitalWrite(LED,LOW);
    }

    while(Serial.available() > 0) {
      serBuffer[serBufferLength++] = Serial.read();
      if(serBufferLength >=2 && serBuffer[serBufferLength-2] == 13 && serBuffer[serBufferLength-1] == 10) {
        digitalWrite(LED, HIGH);
        bool success = radio.sendWithRetry(serBuffer[0], serBuffer+1, serBufferLength-2, 3, 40);
        digitalWrite(LED, LOW);
        snprintf(screenLogLine, 20, "%d to %d (%s-%d)", serBuffer[1], serBuffer[0], success?"OK":"NOK", radio.RSSI);
        serBufferLength = 0;
        addToScreenLog(screenLogLine);
      }
    }

    draw();
}

void addToScreenLog(const char * log) {
    strncpy(screenLog[screenLogIndex], log, 20);
    screenLogTimes[screenLogIndex] = millis();
    screenLogIndex = ++screenLogIndex % LOGSIZE;
}

unsigned long lastPrint = millis();
bool draw(void ) {
  if(millis()-lastPrint < 500)
    return false;
  int logIndex = 0;
  for(logIndex = 1 ; logIndex <= LOGSIZE ; logIndex++) 
  {
    int i = (screenLogIndex-logIndex+8) % LOGSIZE;
    unsigned long s = (millis()-screenLogTimes[i])/1000;
    unsigned int sec = s % 60L;
    unsigned int min = (s / 60L) % 60L;
    unsigned int hou = (s % 86400L) / 3600L;
    
    if(hou > 0) {
      snprintf(screenLogLine, 20, "%02i:%02i", hou, min);
    }
    else {
      snprintf(screenLogLine, 20, "%02i:%02i", min, sec);
    }

    ssd1306_charF6x8(0, logIndex+1, screenLogLine);
    ssd1306_charF6x8(35, logIndex+1, screenLog[i]);
  }
  lastPrint = millis();
  return true;
}
