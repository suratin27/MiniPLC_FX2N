/**********************************************************/
//CPU needs: STM32F103--RAM memory not less than 64K Flash memory not less than 128K
//This code has been tested on STM32F103RDT6 and VET6
//Edit Date: 20150909
//editor by Xiao Xiaosheng
//Online shop: shop182385147.taobao.com
/**********************************************************/
#include <stdio.h> 
#include "plc_dialogue.h"
#include "plc_conf.h"
#include "plc_io.h"

// Select a Timer Clock
//#define USING_TIM_DIV1                false           // for shortest and most accurate timer
//#define USING_TIM_DIV16               false           // for medium time and medium accurate timer
//#define USING_TIM_DIV256              true            // for longest timer but least accurate. Default
//#include "ESP8266TimerInterrupt.h"

extern void filter(void);															//ad conversion filter
//extern u16 PLC_16BIT[12100];	 												//
extern u8 PLC_RAM[];
extern u8 p_PLC_16BIT[];		   												//
extern u16 After_filter[],phase;											//ad convert data buffer
extern void X_filter(void);														//The input x signal is filtered once for 1 ms
extern void DAC_data(void);

u16 PLC_RUN_TIME;																			//Scan time
u16 temp[5];
u16 *p_data_given,*p_value;
extern bool receive_flag;
extern u8 Rx_header;

volatile int interruptCounter;
int totalInterruptCounter;
hw_timer_t * timer = NULL;															//ESP32 Timer
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;		//ESP32 Timer
//#define 		 TIMER_INTERVAL_MS       1
//ESP8266Timer ITimer;


/*******************************************************************************
*Function name: Delay
*Description: Delay function (ms)
*Input: d: delay coefficient, in milliseconds
*output: none
*return : none
*Description: The delay is realized by using the 1ms count generated by the Timer2 timer
*******************************************************************************/
void Delay(u16 time){    
   u16 i=0;  
   while(time--)
   {i=12000; while(i--) ; }
}

void timer_enable(u16 timer_number){     
	p_data_given = (u16*)PLC_RAM+0x0900+timer_number;    		//0x0900+timer_number;								//counter value address
	p_value = (u16*)PLC_RAM+0x0800+timer_number;			   		//
	if(*p_value<*p_data_given)
	{
		if(PLC_BIT_TEST(0x600+timer_number)) 
			PLC_BIT_ON(0x3800+timer_number);										//reset coil
		else 
		{
			PLC_BIT_OFF(0x600+timer_number);										//off when the value is less than the set value
			if(PLC_BIT_TEST(0x3800+timer_number))
			{
				*p_value=0;PLC_BIT_OFF(0x3800+timer_number);
			}
		}
	}
	else
	PLC_BIT_ON(0x600+timer_number);													//on when the value reaches the set value
}

void timer_disble(u16 timer_number){     
	p_data_given = (u16*)PLC_RAM+0x0900+timer_number;				//counter value address
	p_value = (u16*)PLC_RAM+0x0800+timer_number;			    	//
	PLC_BIT_OFF(0x600+timer_number);												//overflow coil
	PLC_BIT_OFF(0x1600+timer_number);												//enable coil
	PLC_BIT_OFF(0x3800+timer_number);												//reset coil
	*p_data_given=0;
	*p_value=0;
}

void T_100MS(void){
	u16 timer_count;
  for(timer_count=0;timer_count<200;timer_count++)
	{ 
		p_data_given = (u16*)PLC_RAM+0x0900+timer_count;
		p_value = (u16*)PLC_RAM+0x0800+timer_count;
		if(PLC_BIT_TEST(0x1600+timer_count))								//Coil Status
		{
			if(*p_value<*p_data_given)												//value state
			{
				if(PLC_BIT_TEST(0x600+timer_count)) ;
				else *p_value+=1;
			}
		}
	 }
}

void T_10MS(void){  
   u16 timer_count;
   for(timer_count=200;timer_count<246;timer_count++)
   { 
		p_data_given = (u16*)PLC_RAM+0x0900+timer_count;
	    p_value = (u16*)PLC_RAM+0x0800+timer_count;
	    if(PLC_BIT_TEST(0x1600+timer_count))								//coil state
		  {
          if(*p_value<*p_data_given)											//value state
				  {
				     if(PLC_BIT_TEST(0x600+timer_count)) ;
				     else *p_value+=1;
				  }
		  }
   }
}

void T_1MS(void){ 
	 u16 timer_count;  
     for(timer_count=246;timer_count<250;timer_count++)
	   { 
		  p_data_given = (u16*)PLC_RAM+0x0900+timer_count;
	      p_value = (u16*)PLC_RAM+0x0800+timer_count;
	      if(PLC_BIT_TEST(0x1600+timer_count))							//Coil Status
		    {  
           if(*p_value<*p_data_given)											//value state
				   {
				     if(PLC_BIT_TEST(0x600+timer_count)) ;
				     else *p_value+=1;
				  }
		    }		
	  }	
	  
}

void T_H100MS(void){ 
	u16 timer_count;
	for(timer_count=250;timer_count<256;timer_count++)
	{ 
		p_data_given = (u16*)PLC_RAM+0x0900+timer_count;
			p_value = (u16*)PLC_RAM+0x0800+timer_count;
			if(PLC_BIT_TEST(0x1600+timer_count))							//coil state
			{
					if(*p_value<*p_data_given)										//value state
					{
						if(PLC_BIT_TEST(0x600+timer_count)) ;
						else *p_value+=1;
					}
			}
		}
}

//ESP32 Timer interrupt routine
void IRAM_ATTR onTimer(){
  portENTER_CRITICAL_ISR(&timerMux);
  static u8 all_clock;
	static u16 minute;
	//TIM2->SR=0; 	
	PLC_RUN_TIME+=10;
	all_clock++;
	X_filter();																						//check x input state value
	if(all_clock>99){ 	              										//m8011 10MS, m8012 100MS,  m8013 1SEC, m8014 1minute   
		all_clock=0,PLC_BIT_ON(M8012);T_1MS();
	}	             
	if((all_clock%10)==7){																//10ms timer design refreshes every five times
		T_10MS(),PLC_BIT_OFF(M8011);
	}
	if((all_clock%10)==2){
		PLC_BIT_ON(M8011);
	}
	if(all_clock==50){																		//The two 100 ms timers are refreshed separately
		T_100MS(),PLC_BIT_OFF(M8012);	
	}
	if(all_clock==90){							//90									//every 100ms seconds minute timer
		T_H100MS(),minute++;
		//m8013Count++;
		//receive_flag=FALSE;  															//ADD
		//Rx_header=FALSE;
	}
	if((all_clock%0x10)==0x02){														//Update dac data once
		//DAC_data();
		//filter();																					//adc sends updates every ten milliseconds
	}
	if(minute%10==5){								//10									//Refresh seconds 8013
		PLC_BIT_OFF(M8013);
		//STOP_WAR;
	}
	if(minute%10==0){								//10
		PLC_BIT_ON(M8013);
		//RUN_WAR;
	}
	if(minute==300){									//300								//refresh minute 8014
		PLC_BIT_OFF(M8014);
	}
	if(minute==0){
		PLC_BIT_ON(M8014);
	}
	if(minute>599){
		minute=0;	
	}	
	//RAM_16BIT(0x3000) = minute;  
  portEXIT_CRITICAL_ISR(&timerMux);
}

void PLC_Timer(void){	
	timer = timerBegin(0, 80, true);
  timerAttachInterrupt(timer, &onTimer, true);
  timerAlarmWrite(timer, 1000, true);
  timerAlarmEnable(timer);
}
