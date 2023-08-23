#ifndef PLCMODBUS_H
#define PLCMODBUS_H

#include "ModbusServerRTU.h"
#include "ModbusClientRTU.h"

ModbusServerRTU MBRTUServer(2000);
ModbusClientRTU MBRTUClient(Serial1);

#endif