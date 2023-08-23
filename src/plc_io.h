/****************************************************************************
PLC-related special registers
Dedicated Auxiliary Relay Description
M8126       global flag
M8127       Communication request handshake signal
M8128       Error flag
M8129       Communication request switching

Dedicated Data Register Description
D8000   = 200; scan time
D8001   = 0X5EF6; Model version FX2N(C)
D8101   = 0X5EF6; Model version FX2N(C)
D8002   = 8; memory capacity
D8102   = 8; memory capacity
D8003   = 0x0010; memory type, register type
D8006 CPU battery voltage
D8010   = 10; scan current value
D8011   = 20; scan minimum time (0.1MS)
D8012   = 140; maximum scan time (0.1MS)

D8120     = 0X4096 Communication format
D8121     Slave number (up to 16)
D8127     The first address of exchange data
D8128     Exchange data volume
D8129     Network communication timeout time confirmation value
D8000     Watchdog

Detailed explanation of communication format (D8120)
----------------------------------------------------------------------
Tag | Meaning | Description
-------------+-------------+------------------------------------------------
b0 | data length | 0: 7 bits 1: 8 bits
-------------+-------------+------------------------------------------------
b2b1 | Check mode | 00: Not used 01: Odd check 11: Even check
-------------+-------------+------------------------------------------------
b3 | stop bits | 0: 1 bit 1: 2 bits
-------------+-------------+------------------------------------------------
| | 0001:300 0111:4800
b7b6b5b4 | Baud Rate | 0100:600 1000:9600
| | 0101:1200 1001:19200
| | 0110:2400
-------------+-------------+------------------------------------------------
b8 | | 0: Not used Note: Only for non-protocol communication
-------------+-------------+------------------------------------------------
b9 | | 0: Not required Same as above
-------------+-------------+------------------------------------------------
b12b11b10 | Communication interface | 000: RS485 (RS422) interface
| | 010: RS232C interface
-------------+-------------+------------------------------------------------
b13 | Summation check | 0: No summation code is added 1: A summation code is added automatically
-------------+-------------+-----------------------------------------------
b14 | Protocol | 0: No protocol communication 1: Special communication protocol
-------------+-------------+------------------------------------------------
b15 | Protocol Format | 0: Format 1 1: Format 4
----------------------------------------------------------------------

Example: D8120 = 0X4096 The communication baud rate is 19200

*********************************************************************************/
#ifndef _PLC_IO_H 
#define _PLC_IO_H 

#include "tftype_def.h"
#ifdef __cplusplus
extern "C"  {
#endif
#ifdef __cplusplus
}
#endif
#include <Arduino.h>

// PLC Structure
typedef struct plc_io{
  uint8_t X0 = 0;   uint8_t X1 = 0;   uint8_t X2 = 0;   uint8_t X3 = 0;
  uint8_t X4 = 0;   uint8_t X5 = 0;   uint8_t X6 = 0;   uint8_t X7 = 0; 
  uint8_t Y0 = 0;   uint8_t Y1 = 0;   uint8_t Y2 = 0;   uint8_t Y3 = 0;
  uint8_t Y4 = 0;   uint8_t Y5 = 0;   uint8_t Y6 = 0;   uint8_t Y7 = 0; 
  uint8_t STS0 = 0; uint8_t STS1 = 0; uint8_t STS2 = 0; uint8_t STS3 = 0;
  uint16_t PLC_Model = 0;
};

#define RUN             1
#define STOP            0
#define PLC_RUN         1 

void RUN_ON();
void RUN_OFF();
void TOGGLE_RUN();
void NET_ON();
void NET_OFF();
void TOGGLE_NET();
void WAR_ON();
void WAR_OFF();
void TOGGLE_WAR();
void ERR_ON();
void ERR_OFF();
void TOGGLE_ERR();
void statusRUN(bool _sts,int delay);

#endif
