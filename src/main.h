
#ifndef _MAIN_H_
#define _MAIN_H_
#include "tftype_def.h"
#include "plc_io.h"
#include <stdio.h>
#include <string.h>
#include <stddef.h>

extern void PLC_ProInstructParse(void);
extern void RTC_Init(void);
extern void TIM3_PWM_Init(void);
extern void pulse_Init(void);
extern void RTC_Get(void);
extern void RST_C(void);
extern void RTC_Get_Init(void);
extern void PLC_Timer(void);
extern void A_B_Init(void);
extern void ADC_init(void);
extern void red_init(void);
extern void backup_data(void);
extern void Recover_data(void);                     //data recovery from power failure
extern void PLC_IO_config(void);
extern u8   Send_out;
extern void TX_Process(void);                       //Send serial data
extern void RX_Process();
extern void PLC_DATA_KEEP(void); 	
extern void data_init(void);

extern void usart(u16 i);
extern void USART1_Configuration(void);
extern void USART2_Configuration(void);
extern void data_init(void);                        //d8000~d8126 initialization

#endif
