/**********************************************************/
//CPU needs: STM32F103--RAM memory not less than 64K Flash memory not less than 128K
//This code has been tested on STM32F103RDT6 and VET6
//Edit Date: 20150909
//editor by Xiao Xiaosheng
//Online shop: shop182385147.taobao.com
/**********************************************************/
//#include "stm32f10x.h"
//#include "stm32f10x_flash.h"
#include <stdio.h>
#include "plc_dialogue.h"
#include "plc_conf.h"
#include "plc_io.h"
#include <StreamUtils.h>
#include <ArduinoJson.h>

extern const u16 *PLC_Addr;					//address lookup table pointer


extern u8 FLASH_ErasePage(u32 Page_Address);
extern u8 FLASH_ProgramHalfWord(u32 Address,u16 Data);
/***********************************************
Function: PLC communication error handler
D8063, M8063=Link, communication error
0000=No exception;
6301=Parity error
6302=Communication character error
6303 = Communication and verification error
6304=Data format error
6305=Instruction error
***********************************************/
void PLC_COMM_ERROR(u16 err_id)
{
	if(err_id!=0)PLC_BIT_ON(M8063);//error flag
		D8063=err_id;
}

void write_data(u16 number)
{ 
	u16 temp,appoint_address;
	/*
	if(number<12)												//The block for writing parameters must be 10 blocks
	{
		//FLASH_Unlock();																			//flash off protection
		FLASH_ErasePage(PLC_RAM_ADDR+number*0x800);						//erase page A piece of data occupies 2K
		for(temp=0;temp<1024;temp++)													//The operation is 16bit, only 1024 program operations are required to complete
		{
			appoint_address=PLC_RAM_ADDR+number*0x800+temp*2;												//The starting address plus the block address plus the small address of the block is equal to the target location
			FLASH_ProgramHalfWord(appoint_address,prog_write_buffer[temp]);					//Wait for the program to finish writing
		}
		//FLASH_Lock();																				//End program writing and turn on flash protection
	}
	*/
}

void Read_data(u16 number)						//Program block backup, the purpose is to enter the previous program backup before writing the program
{ 
	u16 temp,appoint_address;
	if(number<12)
	{
		for(temp=0;temp<1024;temp++)
		{
			appoint_address=number*0x800+temp*2;																									//Start address plus block address plus block's small address
			//prog_write_buffer[temp]=PLC_ROM[appoint_address]+(PLC_ROM[appoint_address+1]*0x100);	//back up the program
		}
	}
}
/***********************************************
Function: Power off data backup
***********************************************/
//------------ EEPROM Allocation --------------
#define MEM_EPOS						1000
#define MEM_ESIZE						1000

void backupDMemory(){
  DynamicJsonDocument mJson(3000);
	for(uint16_t i=400;i<501;i++){						//- Load D Data to buffer
		mJson["D"+String(i)] = RAM_16BIT(0x3000+i*2);
	}
  EepromStream eeprombStream(MEM_EPOS, MEM_ESIZE);
  serializeJson(mJson, eeprombStream);
  eeprombStream.flush();
}

void loadBackupDMemory(){
	DynamicJsonDocument mJson(3000);
  EepromStream eepromStream(MEM_EPOS, MEM_ESIZE);
  deserializeJson(mJson, eepromStream);

	for(int i=400;i<501;i++){
		if(mJson.containsKey("D"+String(i))){																			//- Load saved D Data back
    	RAM_16BIT(0x3000+i*2) = mJson["D"+String(i)].as<uint16_t>();
		}
	}
}

void PLC_DATA_KEEP(void)                                                  
{  
	backupDMemory();
	u16 temp=0,backup_addr; 
	for(backup_addr=(0x1000+500);backup_addr<(0x1000+950);backup_addr++)				//Backup D400 unit to D500
	//[temp]=PLC_16BIT[backup_addr],temp++;

	for(backup_addr=(0x0500+100);backup_addr<(0x0500+150);backup_addr++)				//Backup T50 cells to T100
	//prog_write_buffer[temp]=PLC_16BIT[backup_addr],temp++;

	for(backup_addr=(0x0800+100);backup_addr<(0x0800+150);backup_addr++)				//Backup C50 Unit from C100
	//prog_write_buffer[temp]=PLC_16BIT[backup_addr],temp++;

	for(backup_addr=(0x0020);backup_addr<(0x0020+32);backup_addr++)							//Backup M512-M1023
	//prog_write_buffer[temp]=PLC_16BIT[backup_addr],temp++;

	write_data(10);
}

/***********************************************
Function function: power-on data reset
***********************************************/
void Recover_data(void)                                                
{ 
	u16 temp=0,backup_addr;
	Read_data(10);
/*
	for(backup_addr=(0x1000+500);backup_addr<(0x1000+950);backup_addr++)				//Backup D400 unit to D500
		//PLC_16BIT[backup_addr]=prog_write_buffer[temp],temp++;

	for(backup_addr=(0x0500+100);backup_addr<(0x0500+150);backup_addr++)				//Backup T50 cells to T100
		//PLC_16BIT[backup_addr]=prog_write_buffer[temp],temp++;

	for(backup_addr=(0x0800+100);backup_addr<(0x0800+150);backup_addr++)				//Backup C50 Unit from C100
		//PLC_16BIT[backup_addr]=prog_write_buffer[temp],temp++;

	for(backup_addr=(0x0020);backup_addr<(0x0020+32);backup_addr++)							//Backup M512-M1023
		//PLC_16BIT[backup_addr]=prog_write_buffer[temp],temp++;
*/
	//write_data(10);
}

void RST_T_D_C_M_data(void)//ADD Xiao Xiaosheng 20151214
{ 
	u16 backup_addr;
	for(backup_addr=(0x0500+0);backup_addr<(0x0500+256);backup_addr++)		 //Reset	C0-C256
		RAM_16BIT(backup_addr)=0;
	for(backup_addr=(0x0800+0);backup_addr<(0x0800+256);backup_addr++)		 //Reset	T0-T256
		RAM_16BIT(backup_addr)=0;/*
	for(backup_addr=(0x3000+0);backup_addr<(0x3000+7999);backup_addr++)		 //Reset 	D0000-D7999
		RAM_16BIT(backup_addr)=0;
	for(backup_addr=(0x0100);backup_addr<(0x0100+191);backup_addr++)	       //Reset	M0-M3096
		RAM_16BIT(backup_addr)=0;*/
	
	//Serial.println("All C and All T data was cleared");
}
