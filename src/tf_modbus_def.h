#include <Arduino.h>

typedef struct modebusStack{
  uint8_t slave = 0;
  uint8_t func;
  uint8_t data[2];
  uint8_t count[2];
  uint16_t dataPos;
  uint8_t stackPos;
};
