#include "plc_io.h"
#include <stdio.h>
#include "PLC_Dialogue.h"
#include "bitoper.h"

extern u8 power_down;									//power failure detection program
extern u8 PLC_RAM[];
extern void saveDMemory();

u8  X_DIY=10;													//The instruction modifies the filter time x000 x007
#define D8020 RAM_16BIT(0x0E14)				//After the filter time X010
long lastRUNLamp,lastNETLamp,lastWARLamp,lastERRLamp;
int board = 0;
//#define TOGGLE_RUN      digitalWrite(4,!digitalRead(4)) 

/*--------------------------------------------------------------------
													PLC IO Declarations
--------------------------------------------------------------------*/
plc_io ESP32_20RXO,MINIPLC_32U,DINO_PLC;
plc_io *PLC_IO;

void setupPLCIO(){
	// ESP32 Control 2.0RXO
	Serial.println("Load ESP32 Control 2.0RXO IO Mapping");
	ESP32_20RXO.X0	=	21;		ESP32_20RXO.X1	=	19;		ESP32_20RXO.X2	=	18;		ESP32_20RXO.X3	=	5;		
	ESP32_20RXO.X4	=	36;		ESP32_20RXO.X5	=	39;		ESP32_20RXO.X6	=	34;		ESP32_20RXO.X7	=	35;
	ESP32_20RXO.Y0	=	13;		ESP32_20RXO.Y1	=	14;		ESP32_20RXO.Y2	=	17;		ESP32_20RXO.Y3	=	26;
	ESP32_20RXO.Y4	=	25;		ESP32_20RXO.Y5	=	33;		ESP32_20RXO.Y6	=	32;		ESP32_20RXO.Y7	=	2;
	ESP32_20RXO.STS0	=	4;
	// Mini PLC 32u
	Serial.println("Load Mini PLC 32u IO Mapping");
	MINIPLC_32U.X0	=	39;		MINIPLC_32U.X1	=	36;		MINIPLC_32U.X2	=	35;		MINIPLC_32U.X3	=	34;		
	MINIPLC_32U.X4	=	21;		MINIPLC_32U.X5	=	19;		MINIPLC_32U.X6	=	18;		MINIPLC_32U.X7	=	4;
	MINIPLC_32U.Y0	=	13;		MINIPLC_32U.Y1	=	14;		MINIPLC_32U.Y2	=	27;		MINIPLC_32U.Y3	=	26;
	MINIPLC_32U.Y4	=	0;		MINIPLC_32U.Y5	=	0;		MINIPLC_32U.Y6	=	0;		MINIPLC_32U.Y7	=	0;
	MINIPLC_32U.STS0	=	32;	MINIPLC_32U.STS1	=	33;	MINIPLC_32U.STS2	=	25;	MINIPLC_32U.STS3	=	2;
	// Dino PLC
	Serial.println("Load DINO PLC IO Mapping");
	DINO_PLC.X0	=	36;				DINO_PLC.X1	=	39;				DINO_PLC.X2	=	34;				DINO_PLC.X3	=	35;		
	DINO_PLC.X4	=	2;				DINO_PLC.X5	=	15;				DINO_PLC.X6	=	0;				DINO_PLC.X7	=	0;
	DINO_PLC.Y0	=	27;				DINO_PLC.Y1	=	26;				DINO_PLC.Y2	=	14;				DINO_PLC.Y3	=	12;
	DINO_PLC.Y4	=	0;				DINO_PLC.Y5	=	0;				DINO_PLC.Y6	=	0;				DINO_PLC.Y7	=	0;
	DINO_PLC.STS0	=	25;			DINO_PLC.STS1	=	33;			DINO_PLC.STS2	=	0;			DINO_PLC.STS3	=	32;
}

void PLC_Mode_config(void){ 	
	if(PLC_IO->STS0 > 0){
		pinMode(PLC_IO->STS0,OUTPUT);
	}
	if(PLC_IO->STS1 > 0){
		pinMode(PLC_IO->STS1,OUTPUT);
	}
	if(PLC_IO->STS2 > 0){
		pinMode(PLC_IO->STS2,OUTPUT);
	}
	if(PLC_IO->STS3 > 0){
		pinMode(PLC_IO->STS3,OUTPUT);
	}
	/*
	#ifdef ESP32_20RXO
		pinMode(4,OUTPUT);						//Working LED		
	#endif	
	*/						
}

void PLC_X_config(void){ 
	if(PLC_IO->X0 > 0){
		pinMode(PLC_IO->X0,INPUT);
	}
	if(PLC_IO->X1 > 0){
		pinMode(PLC_IO->X1,INPUT);
	}
	if(PLC_IO->X2 > 0){
		pinMode(PLC_IO->X2,INPUT);
	}
	if(PLC_IO->X3 > 0){
		pinMode(PLC_IO->X3,INPUT);
	}
	if(PLC_IO->X4 > 0){
		if(PLC_IO->PLC_Model == 100){
			pinMode(PLC_IO->X4,INPUT_PULLUP);
		}else{
			pinMode(PLC_IO->X4,INPUT);
		}
	}
	if(PLC_IO->X5 > 0){
		if(PLC_IO->PLC_Model == 100){
			pinMode(PLC_IO->X5,INPUT_PULLUP);
		}else{
			pinMode(PLC_IO->X5,INPUT);
		}
	}
	if(PLC_IO->X6 > 0){
		pinMode(PLC_IO->X6,INPUT);
	}
	if(PLC_IO->X7 > 0){
		pinMode(PLC_IO->X7,INPUT);
	}
}

void PLC_Y_config(void){ 
	if(PLC_IO->Y0 > 0){
		pinMode(PLC_IO->Y0,OUTPUT);
	}
	if(PLC_IO->Y1 > 0){
		pinMode(PLC_IO->Y1,OUTPUT);
	}
	if(PLC_IO->Y2 > 0){
		pinMode(PLC_IO->Y2,OUTPUT);
	}
	if(PLC_IO->Y3 > 0){
		pinMode(PLC_IO->Y3,OUTPUT);
	}
	if(PLC_IO->Y4 > 0){
		pinMode(PLC_IO->Y4,OUTPUT);
	}
	if(PLC_IO->Y5 > 0){
		pinMode(PLC_IO->Y5,OUTPUT);
	}
	if(PLC_IO->Y6 > 0){
		pinMode(PLC_IO->Y6,OUTPUT);
	}
	if(PLC_IO->Y7 > 0){
		pinMode(PLC_IO->Y7,OUTPUT);
	}
}

void X_filter(void)										//Need to be called once every 1ms for X filtering, tentatively set to 20MS
{
	static signed char x_buffer[33];		//Tentatively refresh 16 x
	/*
	(X00) ? (x_buffer[0]=0,PLC_16BIT[64].bits.bit0=0) : (x_buffer[0]<X_DIY) ? (x_buffer[0]++) : (PLC_16BIT[64].bits.bit0=1);
	(X01) ? (x_buffer[1]=0,PLC_16BIT[64].bits.bit1=0) : (x_buffer[1]<X_DIY) ? (x_buffer[1]++) : (PLC_16BIT[64].bits.bit1=1);	
	(X02) ? (x_buffer[2]=0,PLC_16BIT[64].bits.bit2=0) : (x_buffer[2]<X_DIY) ? (x_buffer[2]++) : (PLC_16BIT[64].bits.bit2=1);	
	(X03) ? (x_buffer[3]=0,PLC_16BIT[64].bits.bit3=0) : (x_buffer[3]<X_DIY) ? (x_buffer[3]++) : (PLC_16BIT[64].bits.bit3=1);	
	(X04) ? (x_buffer[4]=0,PLC_16BIT[64].bits.bit4=0) : (x_buffer[4]<X_DIY) ? (x_buffer[4]++) : (PLC_16BIT[64].bits.bit4=1);	
	(X05) ? (x_buffer[5]=0,PLC_16BIT[64].bits.bit5=0) : (x_buffer[5]<X_DIY) ? (x_buffer[5]++) : (PLC_16BIT[64].bits.bit5=1);	
	(X06) ? (x_buffer[6]=0,PLC_16BIT[64].bits.bit6=0) : (x_buffer[6]<X_DIY) ? (x_buffer[6]++) : (PLC_16BIT[64].bits.bit6=1);	
	(X07) ? (x_buffer[7]=0,PLC_16BIT[64].bits.bit7=0) : (x_buffer[7]<X_DIY) ? (x_buffer[7]++) : (PLC_16BIT[64].bits.bit7=1);
	*/
	if(PLC_IO->X0 > 0){
		(digitalRead(PLC_IO->X0)) ? (x_buffer[0]=0,CLEARBIT(PLC_RAM[128],0)) : (x_buffer[0]<X_DIY)?(x_buffer[0]++):SETBIT(PLC_RAM[128],0);
	}
	if(PLC_IO->X1 > 0){
		(digitalRead(PLC_IO->X1)) ? (x_buffer[1]=0,CLEARBIT(PLC_RAM[128],1)) : (x_buffer[1]<X_DIY)?(x_buffer[1]++):SETBIT(PLC_RAM[128],1);
	}
	if(PLC_IO->X2 > 0){
		(digitalRead(PLC_IO->X2)) ? (x_buffer[2]=0,CLEARBIT(PLC_RAM[128],2)) : (x_buffer[2]<X_DIY)?(x_buffer[2]++):SETBIT(PLC_RAM[128],2);
	}
	if(PLC_IO->X3 > 0){
		(digitalRead(PLC_IO->X3)) ? (x_buffer[3]=0,CLEARBIT(PLC_RAM[128],3)) : (x_buffer[3]<X_DIY)?(x_buffer[3]++):SETBIT(PLC_RAM[128],3);
	}
	if(PLC_IO->X4 > 0){
		(digitalRead(PLC_IO->X4)) ? (x_buffer[4]=0,CLEARBIT(PLC_RAM[128],4)) : (x_buffer[4]<X_DIY)?(x_buffer[4]++):SETBIT(PLC_RAM[128],4);
	}
	if(PLC_IO->X5 > 0){
		(digitalRead(PLC_IO->X5)) ? (x_buffer[5]=0,CLEARBIT(PLC_RAM[128],5)) : (x_buffer[5]<X_DIY)?(x_buffer[5]++):SETBIT(PLC_RAM[128],5);
	}
	if(PLC_IO->X6 > 0){
		(digitalRead(PLC_IO->X6)) ? (x_buffer[6]=0,CLEARBIT(PLC_RAM[128],6)) : (x_buffer[6]<X_DIY)?(x_buffer[6]++):SETBIT(PLC_RAM[128],6);
	}
	if(PLC_IO->X7 > 0){
		(digitalRead(PLC_IO->X7)) ? (x_buffer[7]=0,CLEARBIT(PLC_RAM[128],7)) : (x_buffer[7]<X_DIY)?(x_buffer[7]++):SETBIT(PLC_RAM[128],7);
	}
	/*
	(X00) ? (x_buffer[0]=0,CLEARBIT(PLC_RAM[128],0)) : (x_buffer[0]<X_DIY)?(x_buffer[0]++):SETBIT(PLC_RAM[128],0);
	(X01) ? (x_buffer[1]=0,CLEARBIT(PLC_RAM[128],1)) : (x_buffer[1]<X_DIY)?(x_buffer[1]++):SETBIT(PLC_RAM[128],1);
	(X02) ? (x_buffer[2]=0,CLEARBIT(PLC_RAM[128],2)) : (x_buffer[2]<X_DIY)?(x_buffer[2]++):SETBIT(PLC_RAM[128],2); 
	(X03) ? (x_buffer[3]=0,CLEARBIT(PLC_RAM[128],3)) : (x_buffer[3]<X_DIY)?(x_buffer[3]++):SETBIT(PLC_RAM[128],3);
	(X04) ? (x_buffer[4]=0,CLEARBIT(PLC_RAM[128],4)) : (x_buffer[4]<X_DIY)?(x_buffer[4]++):SETBIT(PLC_RAM[128],4);
	(X05) ? (x_buffer[5]=0,CLEARBIT(PLC_RAM[128],5)) : (x_buffer[5]<X_DIY)?(x_buffer[5]++):SETBIT(PLC_RAM[128],5);
	(X06) ? (x_buffer[6]=0,CLEARBIT(PLC_RAM[128],6)) : (x_buffer[6]<X_DIY)?(x_buffer[6]++):SETBIT(PLC_RAM[128],6);
	(X07) ? (x_buffer[7]=0,CLEARBIT(PLC_RAM[128],7)) : (x_buffer[7]<X_DIY)?(x_buffer[7]++):SETBIT(PLC_RAM[128],7);
	*/
// 	if(!PVD) {if(power_down<=7)power_down++;}else{if(power_down>0)power_down--;}
}

//Refresh output, one address is 32 points
void PLC_IO_Refresh(void){
	if(PLC_IO->Y0 > 0){
		digitalWrite(PLC_IO->Y0,GETBIT(PLC_RAM[160],0));
	}
	if(PLC_IO->Y1 > 0){
		digitalWrite(PLC_IO->Y1,GETBIT(PLC_RAM[160],1));
	}
	if(PLC_IO->Y2 > 0){
		digitalWrite(PLC_IO->Y2,GETBIT(PLC_RAM[160],2));
	}
	if(PLC_IO->Y3 > 0){
		digitalWrite(PLC_IO->Y3,GETBIT(PLC_RAM[160],3));
	}
	if(PLC_IO->Y4 > 0){
		digitalWrite(PLC_IO->Y4,GETBIT(PLC_RAM[160],4));
	}
	if(PLC_IO->Y5 > 0){
		digitalWrite(PLC_IO->Y5,GETBIT(PLC_RAM[160],5));
	}
	if(PLC_IO->Y6 > 0){
		digitalWrite(PLC_IO->Y6,GETBIT(PLC_RAM[160],6));
	}
	if(PLC_IO->Y7 > 0){
		digitalWrite(PLC_IO->Y7,GETBIT(PLC_RAM[160],7));
	}
	
}

void PLC_IO_config(uint16_t _board){
		Serial.println("Begin init PLC IO");
		Serial.println("Set PLC Mapping");
	setupPLCIO();								// Setup PLC All IO
	if(_board == 0){						// _board = 0 >> ESP32 Control 2.0RXO
		Serial.println("PLC mapping type ---> ESP32 Control 2.0RXO");
		PLC_IO = &ESP32_20RXO;
		PLC_IO->PLC_Model = _board;
	}else if(_board == 1){			// _board = 1 >> Mini PLC 32u
		Serial.println("PLC mapping type ---> Mini PLC 32u");
		PLC_IO = &MINIPLC_32U;
		PLC_IO->PLC_Model = _board;
	}else if(_board == 100){		// _board = 100 >> Dino PLC
		Serial.println("PLC mapping type ---> Dino PLC");
		PLC_IO = &DINO_PLC;
		PLC_IO->PLC_Model = _board;
	}
		delay(200);
	PLC_Y_config(); 
		TOGGLE_RUN();
		Serial.println("Init PLC Y Output");
		delay(200);
	PLC_IO_Refresh();						//refresh y output()
		TOGGLE_RUN();
		Serial.println("Init X0 - X7");
		delay(200);
	PLC_X_config();
		TOGGLE_RUN();
		Serial.println("Other IO Mode init");
		delay(200);
	PLC_Mode_config();
		delay(200);
	NET_OFF();
	WAR_OFF();
	ERR_OFF();
}

void statusRUN(bool _sts,int delay){
	if(_sts){
		if(millis()-lastRUNLamp > delay){
			TOGGLE_RUN();
			lastRUNLamp = millis();
		}
	}
}

void RUN_ON(){
	if(PLC_IO->STS0 > 0){
		if(PLC_IO->PLC_Model == 100){
			digitalWrite(PLC_IO->STS0,LOW);
		}else{
			digitalWrite(PLC_IO->STS0,HIGH);
		}
	}
}

void RUN_OFF(){
	if(PLC_IO->STS0 > 0){
		if(PLC_IO->PLC_Model == 100){
			digitalWrite(PLC_IO->STS0,HIGH);
		}else{
			digitalWrite(PLC_IO->STS0,LOW);
		}
	}
}

void TOGGLE_RUN(){
	if(PLC_IO->STS0 > 0){
		digitalWrite(PLC_IO->STS0,!digitalRead(PLC_IO->STS0));
	}
}

void NET_ON(){
	if(PLC_IO->STS1 > 0){
		if(PLC_IO->PLC_Model == 100){
			digitalWrite(PLC_IO->STS1,LOW);
		}else{
			digitalWrite(PLC_IO->STS1,HIGH);
		}
	}
}

void NET_OFF(){
	if(PLC_IO->STS1 > 0){
		if(PLC_IO->PLC_Model == 100){
			digitalWrite(PLC_IO->STS1,HIGH);
		}else{
			digitalWrite(PLC_IO->STS1,LOW);
		}
	}
}

void TOGGLE_NET(){
	if(PLC_IO->STS1 > 0){
		digitalWrite(PLC_IO->STS1,!digitalRead(PLC_IO->STS1));
	}
}

void WAR_ON(){
	if(PLC_IO->STS2 > 0){
		if(PLC_IO->PLC_Model == 100){
			digitalWrite(PLC_IO->STS2,LOW);
		}else{
			digitalWrite(PLC_IO->STS2,HIGH);
		}
	}
}

void WAR_OFF(){
	if(PLC_IO->STS2 > 0){
		if(PLC_IO->PLC_Model == 100){
			digitalWrite(PLC_IO->STS2,HIGH);
		}else{
			digitalWrite(PLC_IO->STS2,LOW);
		}
	}
}

void TOGGLE_WAR(){
	if(PLC_IO->STS2 > 0){
		digitalWrite(PLC_IO->STS2,!digitalRead(PLC_IO->STS2));
	}
}

void ERR_ON(){
	if(PLC_IO->STS3 > 0){
		if(PLC_IO->PLC_Model == 100){
			digitalWrite(PLC_IO->STS3,LOW);
		}else{
			digitalWrite(PLC_IO->STS3,HIGH);
		}
	}
}

void ERR_OFF(){
	if(PLC_IO->STS3 > 0){
		if(PLC_IO->PLC_Model == 100){
			digitalWrite(PLC_IO->STS3,HIGH);
		}else{
			digitalWrite(PLC_IO->STS3,LOW);
		}
	}
}

void TOGGLE_ERR(){
	if(PLC_IO->STS3 > 0){
		digitalWrite(PLC_IO->STS3,!digitalRead(PLC_IO->STS3));
	}
}