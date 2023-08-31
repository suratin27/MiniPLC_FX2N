#include "MiniPLC.h"

long lastTime;
bool m0;

void setup(){
  initPLC();
}

void loop(){
  if(millis() - lastTime > 500){    //- Random value and assing to D100
    setU16D(100,random(0,999));
    m0 = getM(0);
  }
}
