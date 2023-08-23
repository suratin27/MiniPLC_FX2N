#include <Arduino.h>
#include "ESP32_FX1N.h"
#include "plc_dialogue.h"
#include "plc_io.h"
#include "main.h"
#include "bitoper.h"

//------------ Realtime --------------
const char* ntpServer = "pool.ntp.org";
const long  gmtOffset_sec = 3600;
const int   daylightOffset_sec = 3600;
ESP32Time rtc(3600*5);  // offset in seconds GMT+5

bool RTC_ready;
long lastRTCMill;

void initRTC(){
  RTC_ready = false;
	if(!RTC_ready){
		configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
		struct tm timeinfo;
		if(WiFi.isConnected()){
      if(getLocalTime(&timeinfo)){
        rtc.setTimeStruct(timeinfo); 
        RTC_ready = true;
        Serial.println("Online NTP Server ready");
		  }
    }else{
			Serial.println("Online NTP Server not ready.Set time by default");
      rtc.setTime(30, 24, 15, 17, 1, 2022);  // 17th Jan 2021 15:24:30
      RTC_ready = true;
		}
	}
}

void RTCSettime(int sc,int min,int hr,int da,int mt,int yr){
  rtc.setTime(sc,min,hr,da,mt,yr);
}

void updateRTC(){
  if(millis() - lastRTCMill >= 1000){
    lastRTCMill = millis();
    if(RTC_ready){
      RAM_16BIT(0x0E1A)	= rtc.getSecond();				//D8013
      RAM_16BIT(0x0E1C)	= rtc.getMinute();				//D8014
      RAM_16BIT(0x0E1E) = rtc.getHour(true);			//D8015
      RAM_16BIT(0x0E20)	= rtc.getDay();						//D8016
      RAM_16BIT(0x0E22)	= rtc.getMonth()+1;				//D8017										
      RAM_16BIT(0x0E24)	= rtc.getYear();					//D8018
      RAM_16BIT(0x0E26) = rtc.getDayofWeek();			//D8019
	  }
  }
}

/*---------------------------------------------------------------
													  Extern parts
---------------------------------------------------------------*/
extern u8 PLC_FLASH[];
extern u8 PLC_RAM[];
extern void initCrypto();
extern void createTask();
extern void PLC_IO_config(uint16_t _board);
extern void statusRUN(bool _sts,int delay);
extern void initSPIFFS();
extern void PLC_ProInstructParse(void);

/*---------------------------------------------------------------
													  Variable parts
---------------------------------------------------------------*/

/*---------------------------------------------------------------
													  Constant parts
---------------------------------------------------------------*/
//- Ladder task instant
TaskHandle_t t0 = NULL;
TaskHandle_t t10 = NULL;
const TickType_t xDelay10ms = pdMS_TO_TICKS(10);
const TickType_t xDelay60ms = pdMS_TO_TICKS(60);
const TickType_t xDelay100ms = pdMS_TO_TICKS(100);
const TickType_t xDelay1ms = pdMS_TO_TICKS(1);
const TickType_t xDelay5ms = pdMS_TO_TICKS(5);
const TickType_t xDelay20ms = pdMS_TO_TICKS(20);
const TickType_t xDelay30ms = pdMS_TO_TICKS(30);
const TickType_t xDelay50ms = pdMS_TO_TICKS(50);
const TickType_t xDelay500ms = pdMS_TO_TICKS(500);

/*---------------------------------------------------------------
													System Tasks 
---------------------------------------------------------------*/
void LADDER_Task(void *pvParam){
  while(1){
		PLC_ProInstructParse();																			//- PLC instruction parsing	
    vTaskDelay(xDelay10ms);   																	//- Delay Task ??? 50 ms
  }  
	vTaskDelete(NULL);  
}

void RTC_Task(void *pvParam){
  while(1){
		updateRTC();
    vTaskDelay(xDelay500ms);   																	//- Delay Task ??? 50 ms
  }  
	vTaskDelete(NULL); 
}

/*---------------------------------------------------------------
													  Function parts
---------------------------------------------------------------*/
void initPLC_Internal(uint16_t _board){
  USART1_Configuration();	//- Init USB communication 
  Serial.println();
  Serial.println("Begin set PLC IO");
  PLC_IO_config(_board);        //- Init PLC IO
  Serial.println("Begin PLC Data and Ladder");
  initSPIFFS();	          //- Init SPIFFS
  Serial.println("Init PLC Data");
  data_init();            //- Init PLC Ladder 
  Serial.println("Init PLC Timer");
  PLC_Timer();
  Serial.println("Begin PLC Internal task");
  xTaskCreatePinnedToCore(LADDER_Task,"LADDER_Task",5000,(void*) NULL,1,&t0,1);
  Serial.println("Begin PLC RTC task");
  initRTC();
  //xTaskCreatePinnedToCore(RTC_Task,"RTC_Task",1000,(void*) NULL,1,&t10,1);
  Serial.println("End init All PLC FX1N Configurations");
}

void initPLC(){
  Serial.begin(19200,SERIAL_7E1);
  initPLC_Internal(0);
}

void initPLC(uint16_t boardver){
  Serial.begin(19200,SERIAL_7E1);
  initPLC_Internal(boardver);
}

bool getM(uint16_t addr){
  uint16_t _dataPos,data;
	bool ret;

	data = addr + 0x800;
	_dataPos = data/8;
	uint16_t m_data = RAM_16BIT(_dataPos);							//- data address
	uint16_t m_bit = data%8;														//- data bit
	if((m_data&(1<<m_bit))==(1<<m_bit)){
		return true;
	}else{
		return false;
	}
}

void setM(uint16_t addr){
  uint16_t _dataPos,data,bit;
	data = addr + 0x800;
  _dataPos  = data/8;
  bit       = data%8;
  SETBIT(PLC_RAM[_dataPos],bit);
}

void resetM(uint16_t addr){
  uint16_t _dataPos,data,bit;
	data = addr + 0x800;
  _dataPos  = data/8;
  bit       = data%8;
  CLEARBIT(PLC_RAM[_dataPos],bit);
}

uint8_t getU8D(uint16_t addr){
  return RAM_8BIT(0x3000+addr*2);
}

uint16_t getU16D(uint16_t addr){
  if(addr >= 8000){
    return RAM_16BIT(0x0E00+addr*2);
  }else{
    return RAM_16BIT(0x3000+addr*2);
  }
}

uint32_t getU32D(uint16_t addr){
  return RAM_32BIT(0x3000+addr*2);
}

uint16_t getT(uint16_t _t){
  return RAM_16BIT(0x1000+_t*2);
}

uint16_t getC(uint16_t C_number){
  return RAM_16BIT(0x2000+C_number*2);
}

float getFD(uint16_t addr){
  return RAM_32BITF(0x3000+addr*2);
}

void setU8D(uint16_t addr,uint8_t val){
  RAM_8BIT(0x3000+addr*2) = val;
}

void setU16D(uint16_t addr,uint16_t val){
  RAM_16BIT(0x3000+addr*2) = val;
}

void setU32D(uint16_t addr,uint32_t val){
  RAM_32BIT(0x3000+addr*2) = val;
}

void setFD(uint16_t addr,float val){
  RAM_32BITF(0x3000+addr*2) = val;
}