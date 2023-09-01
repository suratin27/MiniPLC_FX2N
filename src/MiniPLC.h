#ifndef MINIPLC_H
#define MINIPLC_H
#include <Arduino.h>
#include <SPIFFS.h>
#include <StreamUtils.h>
#include <ArduinoJson.h>
#include <ESP32Time.h>
#include <WiFi.h>
#include <Wire.h>
#include <FRAM.h>

//------------------- Main Function Parts -------------------
void     initPLC(uint16_t boardver = 1);
bool     getM(uint16_t addr);
void     setM(uint16_t addr);
void     resetM(uint16_t addr);
uint8_t  getU8D(uint16_t addr);
uint16_t getU16D(uint16_t addr);
uint32_t getU32D(uint16_t addr);
uint16_t getT(uint16_t _t);
uint16_t getC(uint16_t _t);
float    getFD(uint16_t addr);
void     setU8D(uint16_t addr,uint8_t val);
void     setU16D(uint16_t addr,uint16_t val);
void     setU32D(uint16_t addr,uint32_t val);
void     setFD(uint16_t addr,float val);

void     initRTC();
void     RTCSettime(int sc,int min,int hr,int da,int mt,int yr);
void     updateRTC();

//------------------- LED Status function ------------------

extern void  RUN_ON();
extern void  RUN_OFF();
extern void  TOGGLE_RUN();
extern void  NET_ON();
extern void  NET_OFF();
extern void  TOGGLE_NET();
extern void  WAR_ON();
extern void  WAR_OFF();
extern void  TOGGLE_WAR();
extern void  ERR_ON();
extern void  ERR_OFF();
extern void  TOGGLE_ERR();
extern void  statusRUN(bool _sts,int delay);

#endif