/*
  หลังจากโหลดลงไฟล์นี้ลงไปใน ESP32 Control 2.0RXO แล้วสามารถ Monitor โดยใช้ GXWork2 ได้เลย 
*/

#include "MiniPLC.h"

long lastTime;

void setup(){
  initPLC();
}

void loop(){
  if(millis() - lastTime > 500){    //- Random value and assing to D100
    //setU16D(100,random(0,999));
  }
}
