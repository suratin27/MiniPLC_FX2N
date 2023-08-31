#define MINIPLC_32U   1
#include "ModbusClientRTU.h"

ModbusClientRTU MB(Serial2);
#define NUM_VALUES 21
float values[NUM_VALUES];

void handleData(ModbusMessage response, uint32_t token){
  // First value is on pos 3, after server ID, function code and length byte
  uint16_t offs = 3;
  // The device has values all as IEEE754 float32 in two consecutive registers
  // Read the requested in a loop
  for (uint8_t i = 0; i < NUM_VALUES; ++i) {
    offs = response.get(offs, values[i]);
  }
  // Signal "data is complete"
  //request_time = token;
  //data_ready = true;
}

void handleError(Error error, uint32_t token){
  // ModbusError wraps the error code and provides a readable error message for it
  ModbusError me(error);
  Serial.printf("Error response: %02X - %s\n", (int)me, (const char *)me);
}

void setup(){
  //initPLC(MINIPLC_32U);
  Serial.begin(115200);
  Serial2.begin(9600,SERIAL_8N1,22,23);

}

int last;

void loop(){
  if (Serial2.available()) {
    char c = Serial2.read();
    Serial.print(c); //ออก USB ไป debug
  } 
}