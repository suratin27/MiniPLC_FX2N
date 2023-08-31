#include "MiniPLC.h"
#include <WiFi.h>
#include <WiFiClient.h>
#include <BlynkSimpleEsp32.h>

long lastTime;
char auth[] = "-----------------------------";
char ssid[] = "-------";
char pass[] = "-------";

BLYNK_WRITE(V10){                           //- Received v10 data to be used to control M100
  bool v10_cmd  = param.asInt();
  if(v10_cmd)
    setM(100);
  else
    resetM(100);
}
BLYNK_WRITE(V2){                           //- Received v10 data to be used to control D4
  uint16_t v2_cmd  = param.asInt();
  setU16D(4,v2_cmd);
}

void setup(){
  initPLC();
  Blynk.begin(auth,ssid,pass,"blynk.iot-cm.com", 8080);
}

void loop(){
  if(millis() - lastTime > 500){    
    setU16D(100,random(0,999));           //- Random value and assing to D100
    Blynk.virtualWrite(V0,getU16D(100));  //- Set to Blynk V0
  }
  Blynk.run();
}
