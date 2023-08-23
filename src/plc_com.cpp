/***********************************************************/
//CPU needs: STM32F103--RAM memory not less than 64K Flash memory not less than 128K
//This code has been tested on STM32F103RDT6 and VET6
//Edit Date: 20150909
//editor by Xiao Xiaosheng
//Online shop: shop182385147.taobao.com
/********************************************************
PLC-related special registers
Dedicated Auxiliary Relay Description
M8126 global flag
M8127 Communication request handshake signal
M8128 Error flag
M8129 Communication request switching

Dedicated Data Register Description
D8000 = 200; scan time
D8001 = 0X5EF6; Model version FX2N(C)
D8101 = 0X5EF6; Model version FX2N(C)
D8002 = 8; memory capacity
D8102 = 8; memory capacity
D8003 = 0x0010; memory type, register type
D8006 CPU battery voltage
D8010 = 10; scan current value
D8011 = 20; scan minimum time (0.1MS)
D8012 = 140; maximum scan time (0.1MS)
D6030 D6031 D6032 D6033 is the analog input
D8080 D8081 is the analog output



D8120 = 0X4096 Communication format
D8121 Slave number (up to 16)
D8127 The first address of exchange data
D8128 Exchange data volume
D8129 Network communication timeout time confirmation value
D8000 Watchdog
D8019 corresponds to the week
D8018 corresponds to the year
D8017 corresponds to the month
D8016 corresponding date
D8015 corresponds to the hour
D8014 corresponds to minutes
D8013 corresponds to seconds
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
| | 010: (RS232) interface
-------------+-------------+------------------------------------------------
b13 | Summation check | 0: No summation code is added 1: A summation code is added automatically
-------------+-------------+-----------------------------------------------
b14 | Protocol | 0: No protocol communication 1: Special communication protocol
-------------+-------------+------------------------------------------------
b15 | Protocol Format | 0: Format 1 1: Format 4
----------------------------------------------------------------------
Example: D8120 = 0X4096 The communication baud rate is 19200
*********************************************************************************/

#include <Arduino.h>
#include "tftype_def.h"
#include <stdio.h>
#include "plc_dialogue.h"
#include "plc_io.h"
#include "plc_conf.h"
#include <SPIFFS.h>

u16 *PLC_Addri;	
extern char buffer_rx[];
extern char buffer_tx[];
extern void deleteFile(fs::FS &fs, const char * path);
extern void readFile(fs::FS &fs, const char * path);
extern void writeFile(fs::FS &fs, const char * path, const char * message);
extern void listDir(fs::FS &fs, const char * dirname, uint8_t levels);

static intr_handle_t handle_console;
// Receive buffer to collect incoming data
uint8_t rxbuf[256];
// Register to collect data length
uint16_t urxlen;

//Function prototype
void uart0Config(int baud);
static void IRAM_ATTR uart_intr_handle(void *arg);
void USART1_Configuration(void);
void RX_Process();
void TX_Process();
void TX_Task(void *pvParam);
void RX_Task(void *pvParam);

TaskHandle_t t1 = NULL;
TaskHandle_t t2 = NULL;

void suspendTask12(){
	vTaskSuspend(t1);
	vTaskSuspend(t2);
}

void resumeTask12(){
	vTaskResume(t1);
	vTaskResume(t2);
}

const TickType_t xDelay10ms = pdMS_TO_TICKS(10);
const TickType_t xDelay5ms = pdMS_TO_TICKS(5);
const TickType_t xDelay1ms = pdMS_TO_TICKS(1);
const TickType_t xDelay20ms = pdMS_TO_TICKS(20);
const TickType_t xDelay30ms = pdMS_TO_TICKS(30);
const TickType_t xDelay40ms = pdMS_TO_TICKS(40);
const TickType_t xDelay50ms = pdMS_TO_TICKS(50);
const TickType_t xDelay60ms = pdMS_TO_TICKS(60);
const TickType_t xDelay70ms = pdMS_TO_TICKS(70);
const TickType_t xDelay100ms = pdMS_TO_TICKS(100);
const TickType_t xDelay120ms = pdMS_TO_TICKS(120);
const TickType_t xDelay200ms = pdMS_TO_TICKS(200);

u8 PLC_ROM[] = {//-- Must be edited later by Tom					
//The starting address of the Flash is the plc information ****************************
//The first 0x02 indicates that the plc is a program step of 16 k, the password area and the difference Number area ****************
//- First byte original = 0x02
	0x04,0x00,0xC0,0xE0,0x01,0x00,0x01,0x00,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0xD3,0xD7,0x16,0x59,0x80,0x3E,0x00,
	0x00,0x00,0x00,0x00,0x00,0x40,0x47,0x61,0x74,0x69,0x50,0x52,0x4F,0x4A,0x45,0x43,0x54,0x54,0x45,0x53,0x54,0x00,0x00,
	0x00,0x00,0xF4,0x09,0xFF,0x0B,0xF4,0x01,0xE7,0x03,0x64,0x0E,0xC7,0x0E,0xDC,0x0E,0xFF,0x0E,0x90,0x01,0xFE,0x03,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x83,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x0F,0x00,					//end command
	0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
};
	
const u16 special_d[256]={
	0x00C8,0x6662,0x0008,0x0010,0x0000,0x0000,0x001E,0x0000,0x0000,0x0000,
	0x0032,0x0082,0x0032,0x0032,0x0033,0x0004,0x00B0,0x0002,0x00052,0x0004,
	0x000A,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,
	0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,
	0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0x0000,0x0000,
	0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,
	0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,
	0x01F4,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,
	0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,
	0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,
	0x0000,0x0000,0x0010,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,
	0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0xFF4F,0x0EFF,0x0000,0x0000,
	0x0000,0x0000,0x0000,0x0000,0x0002,0x0003,0x0000,0x0000
};

const char Ascll[20]={0x30,0x31,0x32,0x33,0x34,0x35,0x36,0x37,0x38,0x39,0X41,0X42,0X43,0X44,0X45,0X46};

const char hex[]={
0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
0,1,2,3,4,5,6,7,8,9,0,0,0,0,0,0,
0,10,11,12,13,14,15,0,0,0,0,0,0,};

u8*  step_status 		= (u8*)PLC_RAM+0XC200;			//__at(RAM_ADDR+0xC200);					//__at(RAM_ADDR+0xC200); 	-- Must edit later by Tom
const u8*   p_x		  = (u8*)PLC_FLASH;					  //__at(PLC_RAM_ADDR)={0x08,0x00};		// PLC_RAM_ADDR					-- Must edit later by Tom

static u8 Flag_Uart_Send=1;							//send flag
u16 rx_count,tx_count;									//data calculation
char tx_data[500];											//Send buffer 143
char* tx_tbuffer;												//pointer for use to store data and send
char rx_data[500];											//Receive buffer 143
u16 prog_address,data_address;					//Calculate the data operation start address buffer

u8  Step_Address[2000];									//The write step state is 0. The rising delay and falling edge use a total of 2 k bytes 1600bit
u8  data_size,block_contol[2];													
extern u8  edit_prog;
extern void RTC_Set(u16 syear,u8 smon,u8 sday,u8 hour,u8 min,u8 sec);													
u8 Send_out;
u8 Write_Pro_flag = 0;
int ladder_byte;

/*----------------------------------------------------------------
													PLC_FLASH By TOM
----------------------------------------------------------------*/
#ifdef ESP32
  #include "SPIFFS.h"
#else
  #include #include <FS.h>
#endif
u8 PLC_FLASH[34000];
u8 PLC_RAM[32000];

const char* Ladder_path = "/ladder.txt";

void initSPIFFS(){
	//Serial.println("Init ladder memory");
	if(!SPIFFS.begin(true)){
		Serial.println("An Error has occurred while mounting SPIFFS");
		return;
	}else{
		Serial.println("SPIFFS init successfully");
	}
}

void loadFromSPIFFS(){
	Serial.println();
	Serial.printf("Load PLC file from: %s\n",Ladder_path);
	File file = SPIFFS.open(Ladder_path);
	if(!file){													//- If ladder do not exist
		Serial.println("There was an error opening - may be file not exist");
		//return;
		Serial.println("Trying to create new ladder file");
		File file0 = SPIFFS.open(Ladder_path,FILE_WRITE);
		if (!file0) {
			Serial.println("There was an error to create new ladder file");
			return;
		}
		for(int i=0;i<sizeof(PLC_ROM);i++){
			file0.write(PLC_ROM[i]);
		}
		Serial.println("File template written");
		file0.close();
	}else{															//- If ladder found
		bool err=false;
		Serial.printf("Ladder file size: %d bytes\n",file.size());
		if(file.size() >= 34000){
			PLC_FLASH[0] = 0x00;
			err = true;
		}else{
			Serial.println("Ladder file size - OK");
		}

		Serial.println("Init default Ladder contents");
		for(int u =0;u<34000;u++){
			PLC_FLASH[u] = 0xFF;
		}
		Serial.println("End init default Ladder contents");

		u16 pos=0;
		if(!err){
			while(file.available()){
				PLC_FLASH[pos] = file.read();
				Serial.print(PLC_FLASH[pos],HEX);
				if(pos>=93&&PLC_FLASH[pos]==0x00&&PLC_FLASH[pos-1]==0x0F){				//- Read until found 0x00,0x0F
					break;
				}
				pos++;
			}
		}
		file.close();
		Serial.println();
		Serial.printf("Last pos: %d\n",pos);
		Serial.printf("Last pos 92 value: 0x%X\n",PLC_FLASH[92]);
		Serial.printf("Last pos 93 value: 0x%X\n",PLC_FLASH[93]);
		Serial.printf("Last pos 0 value: 0x%X\n",PLC_FLASH[0]);
		pos = 0;
		if((PLC_FLASH[0]==0x00)||(PLC_FLASH[0]>0x10)||(PLC_FLASH[0]<0x02)||
				(err)||(PLC_FLASH[92]==0xF&&PLC_FLASH[93]!=0x0)){		//- Handle function when missing ladder
			Serial.println("File content missing");
			if(PLC_FLASH[0]==0x00){
				Serial.println("PLC_FLASH[0] == 0x00");
			}else if(PLC_FLASH[0]>0x10){
				Serial.println("PLC_FLASH[0] > 0x10");
			}else if(PLC_FLASH[0]<0x02){
				Serial.println("PLC_FLASH[0] < 0x08");
			}else if(err){
				Serial.println("Ladder size is over 34000 bytes");
			}else if(PLC_FLASH[92]!=0xF){
				Serial.println("Ladder format is wrong0");
			}else if(PLC_FLASH[93]!=0x0){
				Serial.println("Ladder format is wrong1");
			}else{
				Serial.println("Another Error");
			}

			Serial.printf("PLC file missing. Trying to write default ladder to %s\n",String(Ladder_path));
			deleteFile(SPIFFS,Ladder_path);
			Serial.printf("Trying to write %x Btyes\n ",sizeof(PLC_ROM));
			File file_miss = SPIFFS.open(Ladder_path,FILE_WRITE);
			if(file_miss){
				for(int i=0;i<sizeof(PLC_ROM);i++){
					file_miss.write(PLC_ROM[i]);
				}
			}
			file_miss.close();
			Serial.println("Default PLC ladder written");
		}
		Serial.println();
		Serial.println("PLC file data load finished");
	}	
}

void writetoFlash(){
	//- Write PLC template
	if(ladder_byte >=117){
		Serial2.println("Write PLC FLASH");
		File fileT = SPIFFS.open(Ladder_path,FILE_WRITE);
		if(fileT){
			for(int i=0;i<ladder_byte;i++){
				fileT.write(PLC_FLASH[i]);
				//Serial2.print(PLC_FLASH[i],HEX);
			}
			fileT.close();
		}
		Serial2.printf("\nLadder size: %d\n",ladder_byte);
		ladder_byte = 0;			//- Reset ladder byte value;
	}else{
		Serial2.println("Missing ladder contents");
		Serial2.printf("\nLadder size: %d\n",ladder_byte);
		ladder_byte = 0;			//- Reset ladder byte value;
	}
}

void write_PLC(u16 _prog_address,u16 _data_size){
	RUN_ON();
	//Serial2.println("Begin write PLC ladder");
	
	for(u16 i=0;i<_data_size;i++){
		PLC_FLASH[_prog_address+i]	= tx_data[5+i];
		//Serial2.println(PLC_FLASH[i],HEX);
	}
	ladder_byte += data_size;
	//Serial2.println(ladder_byte,HEX);
	RUN_ON();
}

void loadPLCProg(){

	loadFromSPIFFS();				//- Load ladder content from file

	Serial.println("Show last PLC running ROM");
	Serial.println("Begin PLC Parameter");
	for(uint8_t i=0;i<92;i++){
		Serial.print(PLC_FLASH[i],HEX);
	}
	Serial.println();
	Serial.println("Begin PLC Ladder");
	Serial.printf("PLC capacity : %d bytes\n",sizeof(PLC_FLASH));
	uint16_t total_ladder;
	for(uint16_t i=92;i<sizeof(PLC_FLASH);i++){
		Serial.printf("0x%X ",PLC_FLASH[i],HEX);
		if(PLC_FLASH[i] == 0x00 && PLC_FLASH[i-1] == 0x0F){
			total_ladder = i;
			break;
		}
	}
	Serial.printf("PLC ladder size : %d steps\n",total_ladder/2);
	Serial.println();
	Serial.println("End of all ladder");
	//Serial.println("D8000 - D8126");
	u16 temp_address;
	temp_address=0x0E00;
	for(uint8_t i=0;i<126;i++){
		//Serial.print(RAM_16BIT(temp_address + i*2),HEX);
	}
	//Serial.println();
}

void data_init(void)																//d8000~d8126 initialization
{                                                 	//
	u16 temp;                                        	//
	u16 temp_address;                                	//
	prog_address=0x52;                               	//
	//diag("Load D8000-D8126",0,"0",false);
	for(temp=0;temp<126;temp++)                      	
	{                                                	
		temp_address=0x0E00;																		//Take the starting address of D8000 0CC8/2=0X664 and use 32-bit transmission																							
		RAM_16BIT(temp_address+(temp*2)) = special_d[temp];			//pass in the user data backed up by the system flash
	} 
	//Serial.printf("8 Bit 0x0E00: %X\n",RAM_8BIT(0x0E00));
	//Serial.printf("16 Bit 0x0E00: %X\n",RAM_16BIT(0x0E00));

	RAM_16BIT(0x2000)	=	PLC_ROM[prog_address]|PLC_ROM[prog_address] >> 8;

	//diag("Load data to 0x2000",RAM_16BIT(0x2000),"",true);
	//Serial.printf("8 Bit 0x2000: %X\n",RAM_8BIT(0x2000));
	//Serial.printf("16 Bit 0x2000: %X\n",RAM_16BIT(0x2000));
										
	RAM_8BIT(0x01E0) = 0x09;													//Make M8000 M8003 ON
	//diag("set Mode: ",RAM_8BIT(0x01E0),"",true);

	loadPLCProg();																		//- Load ladder program to PLC_RAM_ADDRESS (Simulate flash) by TOM
/*
	PLC_Addri = PLC_START_Address;
	for(uint8_t i=0;i<10;i++){
		Serial.printf("PLC ADD: %X ,Value: %X\n",PLC_Addri,*PLC_Addri);
		PLC_Addri++;
	}
*/
	
	block_contol[0]=200;															//Prevent program loss when writing parameters
	block_contol[1]=200;                             	
	
}                                                  	

void write_block(u16 number)						//write to flash
{ 
	u16 temp,wait_write,appoint_address;
	/*
	if(number<17)																			//The block for writing parameters must be 10 blocks
	{
		FLASH_ErasePage(PLC_RAM_ADDR+number*0x800);			//Erasing a piece of data takes 2 k
		for(temp=0;temp<1024;temp++)										//The operation is 16bit, only 1024 program operations are required to complete
		{
			appoint_address=PLC_RAM_ADDR+number*0x800+temp*2;			//The starting address plus the block address plus the small address of the block is equal to the target location
			wait_write=prog_write_buffer[temp*2]+prog_write_buffer[temp*2+1]*0X100;			//Write 16bit to flash
			FLASH_ProgramHalfWord(appoint_address,wait_write);		//Wait for the program to finish writing
		}
	}
	*/
}

void backup_block(u16 number)						//Program block backup, the purpose is to enter the previous program backup before writing the program
{
	u16 temp,appoint_address;
	if(number<17)
	{
		for(temp=0;temp<2048;temp++)
		{
			appoint_address=number*0x800+temp;										//Start address plus block address plus block's small address
			//prog_write_buffer[temp]=PLC_ROM[appoint_address];			//back up the program
		}
	}
}

//===================================================== ===================================================== ===
//Function name: ErasurePLC
//Function description: PLC erases FLASH space
//Input: mode mode
//output: void
//global variable:
//Calling the module:
//Author: Xiao Xiaosheng
//Date: May 18, 2014
//Remark:
//-------------------------------------------------------------------------------------------------------
//Modified by:
//date:
//Remark:
//-------------------------------------------------------------------------------------------------------
//=======================================================================================================
void ErasurePLC(u8 mode)
{
	u16 temp=0,Erasure_PLC_16BIT;
/********************************************plc storage memory cleanup ***************************************************/
	if(mode==1)
	{
		PLC_FLASH[92] = 0x0F;
		PLC_FLASH[93] = 0x00;
		for(temp=94;temp<2048;temp++)														//from 0x5E
		{ 
			PLC_FLASH[temp] = 0xFF;
		}
	}
/************************************************plc clear data element bits**********************************************/
	if(mode==2)
	{
		for(Erasure_PLC_16BIT=0x3000;Erasure_PLC_16BIT<0x7E7E;Erasure_PLC_16BIT++)					//clear D0000-D7999
		{
			RAM_16BIT(Erasure_PLC_16BIT*2) = 0x0000;
		}
	}
/********************************************plc clean bit components ***************************************************/
	if(mode==3)
	{
		for(Erasure_PLC_16BIT=0x0000;Erasure_PLC_16BIT<0x00BE;Erasure_PLC_16BIT++)					//clear M0000-M3071
		{
			RAM_16BIT(Erasure_PLC_16BIT*2) = 0x0000;
		}
	}
	tx_data[0]=0x06,tx_count=1;																														//Clear the report to the host computer
}
				
/*******************************************************************************
Function: Calculate checksum
*******************************************************************************/
u8 check(char *MyD_str)																		//Calculate Reception Area and Checksum
{ 
	u16 temp; 
	u8 sum;
	sum=0;																									//Please divide and calculator
	MyD_str+=1;																							//calculate and start from the third bit (May be byte by TOM)
	for(temp=0;temp<(rx_count-3);temp++)										//Calculate and
	{ 
		sum+=*MyD_str;																				//start adding
		MyD_str++;																						//increment the pointer by one
	}
	return sum;																							//data is normal
}

/*******************************************************************************
Function name: void switch_read_data(void)
Function: Convert ASCII code to HEX code, occupy data transmission register
Export parameters: none
********************************************************************************/
void switch_read_data(void)             
{ 
	u16 temp;
	for(temp=2;temp<(rx_count-3);temp++)
	{
		tx_data[temp/2]=hex[rx_data[temp]]*0x10;
		tx_data[temp/2]+=hex[rx_data[temp+1]];      
		temp++;
	}
}

void setup_HL(void)														//High and low bit exchange and then convert, little endian conversion
{                                                                     
	u8 temp;                                                            
	temp=tx_data[2];														//The high bit of the address is sent to the 16-bit data area
	prog_address=temp*0x100+tx_data[1];					//Calculate the start address of the program operation
}	                                                                     

void setup_LH(void)														//Normal address translation
{ 
	u8 temp;
	temp=tx_data[2];														//The high bit of the address is sent to the 16-bit data area
	data_address=temp*0x100+tx_data[3];					//Calculate the starting address of the data operation
}

typedef union                           
{
	int data;
	char data1[2];
} usart_data;

void read_plc_tyte(u8 addr)				//- Tom proof						//Read PLC model //Command "30"
{
	u16 temp;
	u8 temp_sum; 
	usart_data plc_type;
	plc_type.data = special_d[addr];							//plc model
	tx_data[0]=0x02;															//start of message 02
	temp_sum=0;
	for(temp=0;temp<data_size;temp+=1)
	{ 
		tx_data[temp*2+1]=Ascll[plc_type.data1[temp]/0x10];				//Byte0 get high byte
		tx_data[temp*2+2]=Ascll[plc_type.data1[temp]%0x10];				//Byte0 get the low byte of the byte
		temp_sum+=tx_data[temp*2+1]+tx_data[temp*2+2];
	}
	tx_data[temp*2+1]=0x03;												//end of message 03
	temp_sum+=0x03;
	tx_data[temp*2+2]=Ascll[temp_sum/0x10];
	tx_data[temp*2+3]=Ascll[temp_sum%0x10]; 
	tx_count=temp*2+4;
}

/*******************************************************************************
Function name?PPLC_Comm_Byte
Function: redefine and execute communication byte address
Entry parameter: Cmd command
Exit parameter: the actual address of the mapping (16BIT address)
********************************************************************************/
u16 PLC_Comm_Byte(u16 comm_add)
{ //Serial2.printf("ComAddr: %X\n",comm_add);
	   																									 //0x2000
	if(comm_add>=0x4000&&comm_add<=0x7E7F){return comm_add-0x1000;}					//D0000-D7999 					--> From PLC_RAM[12288 - 28287]
	else if(comm_add>=0x0280&&comm_add<=0x02FC){return comm_add-0x0280;}		//S000-S999   					--> From PLC_RAM[0 		- 124]
	else if(comm_add>=0x0240&&comm_add<=0x0256){return comm_add-0x01C0;}		//X000-X277							--> From PLC_RAM[128 	- 150]
	else if(comm_add>=0x0180&&comm_add<=0x0196){return comm_add-0x00E0;}		//Y000-Y267 						--> From PLC_RAM[160  - 182]
	else if(comm_add>=0x0200&&comm_add<=0x021F){return comm_add-0x0140;}		//T00-T255 OVER contact	--> From PLC_RAM[192  - 223]
	else if(comm_add>=0x0500&&comm_add<=0x051F){return comm_add-0x0240;}		//T00-T255 Enable coil	--> From PLC_RAM[704  - 735]
	else if(comm_add>=0x0A00&&comm_add<=0x0B00){return comm_add+0x1600;}		//C00-C255 value				--> From PLC_RAM[704  - 735] -- Tom
	else if(comm_add>=0x01E0&&comm_add<=0x01FF){return comm_add-0x0020;}		//C00-C255 OVER contact	-->	From PLC_RAM[448  - 479]
	else if(                  comm_add<=0x00BF){return comm_add+0x0100;}		//M0000-M1535 					--> From PLC_RAM[256 - 447]
	else if(comm_add>=0x00C0&&comm_add<=0x017F){return comm_add+0x0440;}		//M1536-M3071						--> From PLC_RAM[1280 - 1471]
	else if(comm_add>=0x01C0&&comm_add<=0x01DF){return comm_add+0x0020;}		//M8000-M8255 					--> From PLC_RAM[480  - 511]
	else{return comm_add;}																									//invalid address
}

/*******************************************************************************
Function name: PLC_Com_BIT
Function: redefine and execute communication bit address
Entry parameter: Cmd command
Exit parameter: the actual address of the mapping (BIT address)
********************************************************************************/
u16 PLC_Com_BIT(u16 addr)        
{     	
	if((addr<=0x05FF))                      return (addr+0x800); 						//M0000-M1535						--> Min 0 + 0x800/8 = 256
	else if((addr>=0x0600)&&(addr<=0x0BFF)) return (addr+0x2200); 					//M1536-M3071						--> 0x2800/8 = 1280
	else if((addr>=0x0C00)&&(addr<=0x0CB7)) return (addr-0x0700); 					//Y00-Y267 							--> 0x500/8  = 160
	else if((addr>=0x1200)&&(addr<=0x12BF)) return (addr-0x0E00); 					//X00-X267 							--> 0x400/8  = 128	
	else if((addr>=0x1400)&&(addr<=0x17E7)) return (addr-0X1400); 					//S00-S999 							--> 0x00/8   = 0 ,0x12/8 = 2	
	else if((addr>=0x2800)&&(addr<=0x28FF)) return (addr-0X1200);						//T00-T255 Enable coil	--> 0x1600/8 = 704
	else if((addr>=0x1000)&&(addr<=0x10FF)) return (addr-0X0A00);						//T00-T255 OVER contact --> 0x600/8  = 192
	else if((addr>=0x0E00)&&(addr<=0x0EFF)) return (addr+0X0100); 					//M8000-M8255						--> 0xF00/8  = 480
	else if((addr>=0x0F00)&&(addr<=0x0FFF)) return (addr-0X0100);						//C00-C255 OVER contact --> 0xE00/8  = 448
	else  return addr;
}

//===================================================== ===================================================== ===
//Function name: void READ_data(void)
//Function description: Read data X,Y,M,S,T,C,D
//Input: void
//output: void
//global variable:
//Calling the module:
//Author: Xiao Xiaosheng
//Date: June 10, 2014
//Remark:
//-------------------------------------------------------------------------------------------------------
//Modified by:
//date:
//Remark:
//===================================================== ===================================================== ===
void read_other_data(void)																			//Instruction "30"
{
	u16 temp;
	u8 temp_sum;
	tx_data[0]=0x02;																									//start of message
	temp_sum=0;
	for(temp=0;temp<data_size;temp++)
	{ 
		tx_data[temp*2+1]=Ascll[PLC_RAM[PLC_Comm_Byte(temp+prog_address)]/0x10];				//Byte0 get high byte
		tx_data[temp*2+2]=Ascll[PLC_RAM[PLC_Comm_Byte(temp+prog_address)]%0x10];				//Byte0 Take the low byte of the byte
		temp_sum+=tx_data[temp*2+1]+tx_data[temp*2+2];
	}
	tx_data[temp*2+1]=0x03;																						//end of message 03
	temp_sum+=0x03;
	tx_data[temp*2+2]=Ascll[temp_sum/0x10];
	tx_data[temp*2+3]=Ascll[temp_sum%0x10]; 
	tx_count=temp*2+4;
}

//===================================================== ===================================================== ===
//Function name: void READ_data(void)
//Function description: Write data X,Y,M,S,T,C,D
//Input: void
//output: void
//global variable:
//Calling the module:
//Author: Xiao Xiaosheng
//Date: June 10, 2014
//Remark:
//-------------------------------------------------------------------------------------------------------
//Modified by:
//date:
//Remark:
//===================================================== ===================================================== ===
void PC_WRITE_byte(void)//write
{ 
	u16 temp;
	prog_address=tx_data[1]*0x100+tx_data[2]+4;											//Calculate data operation start address
	//Serial2.println("Write bytes");
	//Serial2.printf("Prog add: %X\n",prog_address);
	//Serial2.printf("Data size: %X\n",data_size);
	for(temp=0;temp<data_size;temp++)
	{
		//MEMORY_8BIT(PLC_Comm_Byte(temp+prog_address))=tx_data[4+temp];
		PLC_RAM[PLC_Comm_Byte(temp+prog_address)]	=	tx_data[4+temp];
	}
	tx_data[0]=0x06,tx_count=1;																			//report to the host computer
}

//===================================================== ===================================================== ===
//Function name: void FORCE_ON_data(void)
//Function description: FORCE ON X,Y,M,S,T,C
//Input: void
//output: void
//global variable:
//Calling the module:
//Author: Xiao Xiaosheng
//Date: June 10, 2014
//Remark:
//-------------------------------------------------------------------------------------------------------
//Modified by:
//date:
//Remark:
//===================================================== ===================================================== ===
void PC_FORCE_ON(void)																					//Force 38 ON
{ 
	//Serial2.printf("E81: %X\n",PLC_Com_BIT(tx_data[1]*0x100+tx_data[2]));
	PLC_BIT_ON(PLC_Com_BIT(tx_data[1]*0x100+tx_data[2]));						//Calculate data operation start address
	tx_data[0]=0x06,tx_count=1;  
}

//===================================================== ===================================================== ==
//Function name: void FORCE_ON_data(void)
//Function description: FORCE OFF X,Y,M,S,T,C
//Input: void
//output: void
//global variable:
//Calling the module:
//Author: Xiao Xiaosheng
//Date: June 10, 2014
//Remark:
//------------------------------------------------------------------------------------------------------
//Modified by:
//date:
//Remark:
//===================================================== ===================================================== ===
void PC_FORCE_OFF(void)																					//Force 37 OFF
{ 
	//Serial2.printf("E71: %X\n",PLC_Com_BIT(tx_data[1]*0x100+tx_data[2]));
	PLC_BIT_OFF(PLC_Com_BIT(tx_data[1]*0x100+tx_data[2]));				//Calculate data operation start address
	tx_data[0]=0x06,tx_count=1;   
}

void PC_READ_byte(void)																					//read word
{
	prog_address=tx_data[1]*0x100+tx_data[2];											//Calculate the starting address of the data operation
	switch(prog_address)
	{ 
		//Serial2.println(prog_address);
		case 0x0ECA: read_plc_tyte(101);  break;										//Read PLC TYPE
		case 0x0E02: read_plc_tyte(1);    break;										//Read PLC TYPE
		default: read_other_data();       break;
	}
}

void EPC_FORCE_ON(void)																					//Force on using extension function "e"
{ 
	//Serial.printf("E7: %X\n",prog_address); 
	switch(prog_address) 
	{
		case 0x0E23:   break;																							//Whether the remote operation request can be made
		case 0x0E24:  RAM_8BIT(0x01E0)=0x09; Write_Pro_flag = 0;  break;	//Remote operation needs to run
		case 0x0E25:  RAM_8BIT(0x01E0)=0x0A; break;												//The remote operation needs to be stopped
		default:      RAM_8BIT(0x01E0)=0x09; PLC_BIT_ON(PLC_Com_BIT(prog_address));  break;		//other operating areas
	}
	tx_data[0]=0x06,tx_count=1;																		//report to the host computer
}

void EPC_FORCE_OFF(void)																				//Force off using extension function "e"
{ 
	//Serial.printf("E8: %X\n",prog_address);
	PLC_BIT_OFF(PLC_Com_BIT(prog_address));
	tx_data[0]=0x06,tx_count=1;																		//report to the host computer
}

void PC_READ_Parameter(void)																		//read configuration E00
{
	u16 temp,temp_bit,temp_addr,mov_bit,temp1;
	u8 temp_sum;
	u8 send,monitor,monitor1,monitor2; 
	tx_data[0]=0x02;																							//start of message
	temp_sum=0;
	prog_address=PLC_Comm_Byte(tx_data[2]*0x100+tx_data[3]);			//Calculate the starting address of the data operation
	if((prog_address==0x1790)||(prog_address==0x17D0))						//Request to read the monitoring data area 0X1790 and 0X17D0 addresses
	{
		if(prog_address==0x1790)
		{
			monitor1=RAM_16BIT(0x1400);																//Read the number of monitoring words needed 0X1400/2=0xA00
			//Serial2.printf("Monitor1: %X\n",monitor1);
			for(temp1=monitor=0;monitor<monitor1;monitor++)						//Read monitor word data
			{ 
				temp_bit = PLC_Comm_Byte(RAM_16BIT(0x1404+monitor*2));
				//Serial2.printf("PC temp_bit-word: 0x%X\n",RAM_16BIT(0x1404+monitor*2));
				//Serial2.printf("Actual temp_bit-word: 0x%X\n",temp_bit);
				//Serial2.printf("Temp_bit value: 0x%X\n",RAM_16BIT(temp_bit));
				//Serial2.printf("RAM 0x2006: 0x%X\n",RAM_16BIT(0x2006));
				RAM_16BIT(0x1790+temp1) = RAM_16BIT(temp_bit);
				//Serial2.printf("Bit monitor: 0x%X\n",0x1790+temp1);	
				temp1+=2;			//Transfer the required data to the cache 0X1790/2=0xBC8
				if((temp_bit>=0xC00)&&(temp_bit<=0xCDC))								//Mainly the address after c200 c255 is 32 bits
				{   
					RAM_16BIT(0x1790+temp1) = RAM_16BIT(temp_bit+2);					//pass the required data to the cache
					temp1+=2;
				}			              
			}

			monitor2=RAM_16BIT(0x1402);																//Read the number of bits to be monitored
			//Serial2.printf("Monitor2: %X\n",monitor2);
			for(monitor1=0;monitor1<monitor2;monitor1++)							//read monitor bit data
			{  
				temp_addr=PLC_Com_BIT(RAM_16BIT(0x1404+(monitor*2)+(monitor1*2)));
				temp_bit=RAM_16BIT((temp_addr*2)/0x10);
				//Serial2.printf("PC temp_addr-bit: %X\n",RAM_16BIT(0x1404+(monitor*2)+(monitor1*2)));
				//Serial2.printf("Actual data address: %X\n",PLC_Com_BIT(RAM_16BIT(0x1404+(monitor*2)+(monitor1*2))));
				//Serial2.printf("Actual data RAM address: %X\n",(temp_addr*2)/0x10);
				//Serial2.printf("Actual data value: %X\n",temp_bit);

				mov_bit = temp_addr%0x08;
				//Serial2.printf("Bit position: %X\n",mov_bit);
				//Serial2.printf("Device bit value: %X\n",temp_bit&(1<<mov_bit));
				//Serial2.printf("Return bit pos: %X\n",monitor1%0x08);
				if((temp_bit&(1<<mov_bit))==(1<<mov_bit))
					RAM_16BIT(0x1790+temp1+monitor1/0x8) |=  (1<< (monitor1%0x08));				//Serial buffer data - Got some ploblem if over 8 
				else
					RAM_16BIT(0x1790+temp1+monitor1/0x8) &=~ (1<<(monitor1%0x08));		
			}
		}
	}

	for(temp=0;temp<data_size;temp++)															//read ram
	{ 
		//send=PLC_8BIT(prog_address+temp);
		tx_data[temp*2+1]=Ascll[PLC_RAM[prog_address+temp]/0x10];		//get high byte
		tx_data[temp*2+2]=Ascll[PLC_RAM[prog_address+temp]%0x10];		//Get the low byte of the byte
		temp_sum+=tx_data[temp*2+1]+tx_data[temp*2+2];
	}

	tx_data[temp*2+1]=0x03;    
	temp_sum+=0x03;
	tx_data[temp*2+2]=Ascll[temp_sum/0x10];
	tx_data[temp*2+3]=Ascll[temp_sum%0x10]; 
	tx_count=temp*2+4;
}

void PC_WRITE_Parameter(void)																		//write configuration E10
{  
	u16 temp;
	prog_address=tx_data[2]*0x100+tx_data[3];											//Calculate the starting address of the data operation
	
	//Serial2.println("Write parameter");
	//Serial2.printf("Prog add: %X\n",prog_address);
	//Serial2.printf("Actual Prog add: %X\n",PLC_Comm_Byte(prog_address));
	//Serial2.printf("Data size: %X\n",data_size);

	for(temp=0;temp<data_size;temp++)	         									//write  RAM
	{
	  //RAM_8BIT(PLC_Comm_Byte(prog_address+temp))=tx_data[5+temp];
		PLC_RAM[PLC_Comm_Byte(prog_address+temp)]=tx_data[5+temp];
		//Serial2.print(PLC_RAM[PLC_Comm_Byte(prog_address+temp)]);
	}
	//Serial2.println();
	tx_data[0]=0x06,tx_count=1;																		//report to the host computer
}
 
void PC_READ_PORG(void)																					//Read program E01  -- Must be edited later by Tom
{
	u16 temp;
	u8 temp_sum; 
	tx_data[0]=0x02;																							//start of message
	temp_sum=0;
	data_address-=0x8000;																					//Read the FLASH address minus 0x8000 equals the actual position

	//Serial2.println("Read program");
	//Serial2.printf("Prog add: %X\n",data_address);
	//Serial2.printf("Data size: %X\n",data_size);

	for(temp=0;temp<=data_size-1;temp++)
	{ 
		tx_data[temp*2+1]=Ascll[PLC_FLASH[data_address+temp]/0x10];		//Byte0 get high byte
		tx_data[temp*2+2]=Ascll[PLC_FLASH[data_address+temp]%0x10];		//Byte0 get the low byte of the byte
		temp_sum+=tx_data[temp*2+1]+tx_data[temp*2+2];
	}
	tx_data[temp*2+1]=0x03;    //
	temp_sum+=0x03;
	tx_data[temp*2+2]=Ascll[temp_sum/0x10];
	tx_data[temp*2+3]=Ascll[temp_sum%0x10]; 
	tx_count=temp*2+4;
}

void PC_WRITE_PORG(void)															//ADD Xiao Xiaosheng 20151118
{ 
	u16 temp;
	prog_address=(tx_data[2]*0x100+tx_data[3]);

	//Serial2.println("Write program");
	//Serial2.printf("Prog add: %X\n",prog_address);
	//Serial2.printf("Data size: %X\n",data_size);

	RAM_8BIT(0x01E0)=0x0A; 
	Write_Pro_flag = 1;																	//Prevent the program from running ADD Xiao Xiaosheng during the download process
	if(prog_address>0x7fff)
	{ 
		edit_prog=0;                       
		prog_address-=0x8000;

		write_PLC(prog_address,data_size);
	}
	else
	{
		prog_address+=0x04;
		for(temp=0;temp<data_size;temp++)	  
		{ 
      PLC_RAM[PLC_Comm_Byte(temp+prog_address)]=tx_data[5+temp];
		}
	} 
	tx_data[0]=0x06,tx_count=1;
	Write_Pro_flag = 0;			//- Uncomment by tom
}

static u16 find_data(u16 addr,u16 find_data)					//Find the data address and return the found data address
{
	u8 find_ok,data_H,data_L;
	find_ok=5;
	data_H=find_data/0x100;
	data_L=find_data%0x100;
	addr-=0x8000;
	do{
		if((PLC_ROM[addr]==data_L)&&(PLC_ROM[addr+1]==data_H))
		find_ok=0;																				//find the required command
		else
		addr+=2;
		if(addr>(0xdedb-0x8000))
		find_ok=1;																				//The end instruction was not found in the valid range
	   }while(find_ok>3);
	addr+=0X8000;
	return addr;
}

void find_data_address(void)													//find the command address required by the host computer
{ 
	u8 temp_sum,data_H,data_L;                                    
	data_L=tx_data[4];																	//Need to find the low bit of the content of the data
	data_H=tx_data[5];																	//Need to find the content high bit of the data
	data_address=find_data(data_address,data_H*0X100+data_L);			//read flash
	tx_data[0]=0x02;																		//start of message
	temp_sum=0;
	tx_data[1]=0x31;
	temp_sum+=tx_data[1];
	data_H=data_address/0x100;
	data_L=data_address%0x100;
	tx_data[2]=Ascll[data_H/0X10];
	tx_data[3]=Ascll[data_H%0X10];
	tx_data[4]=Ascll[data_L/0X10];
	tx_data[5]=Ascll[data_L%0X10];
	tx_data[6]=0X03;
	temp_sum+=tx_data[2];
	temp_sum+=tx_data[3];
	temp_sum+=tx_data[4];
	temp_sum+=tx_data[5];
	temp_sum+=tx_data[6];
	tx_data[7]=Ascll[temp_sum/0x10];
	tx_data[8]=Ascll[temp_sum%0x10]; 
	tx_count=9;
}

void backup_mov_block(u16 number)
{	/*
	u16 appoint_address;
	if(number<10)
	{
		for(u16 temp=0;temp<2048;temp++)
		{
			appoint_address=number*0x800+temp;
			prog_write_buffer[temp]=p_x[appoint_address];
		}
	}*/
}

void mov_flash(u16 addr,u8 mov_addr) 
{
	u16 start_addr,end_addr,backup_addr,temp,temp1,temp2,mov_byte,addr_mov; 
	static u8 offset;
	offset=mov_addr;
	end_addr=find_data(addr+0x8000,0x000f)+mov_addr-0x8000; 
	start_addr=end_addr;
	addr_mov=addr;

	if(addr>0x5B)      
	{ 
		addr_mov-=0X5C;	   
		end_addr-=0x5C;
		addr_mov/=2;		   
		end_addr/=2;
		addr_mov/=8;			   
		end_addr/=8;
		offset/=2;
		mov_byte=offset/8;
		offset%=8;
		while(!(end_addr==addr_mov))	   
		{
			temp=step_status[end_addr]*0x100+step_status[end_addr-1];	 
			temp<<=offset;   
			step_status[end_addr+mov_byte]=temp/0x100;		  
			end_addr--;							 
		}
		temp=step_status[end_addr]*0x100+step_status[end_addr-1];	 
		temp<<=offset;   
		step_status[end_addr+mov_byte]=temp/0x100;		  
	}
	end_addr=start_addr;   
	temp=start_addr;
	do{
		if((end_addr/0x800)==(addr/0x800))  
		start_addr=addr%0x800;			   
		else
		start_addr=0;					   
		if((temp/0x800)==(end_addr/0x800))  
		temp1=end_addr%0x800+1; 
		else
		temp1=2048;					   
		backup_block(end_addr/0x800);
		for(temp2=start_addr;temp2<temp1+1;temp2++)
		{
			backup_addr=(end_addr/0x800)*0x800+temp2-mov_addr;   
			//prog_write_buffer[temp2]=p_x[backup_addr];				
		}
		write_block(end_addr/0x800);    
		end_addr-=(temp1-start_addr);   
	}while(end_addr>addr+mov_addr);   
}

void online_write_data(void) 
{ 
	u16 temp;
	u8 temp1,temp2;
	temp1=tx_data[4];      
	temp2=tx_data[5];     
	temp2-=temp1;
	if(temp2>0)      
	{ 
		mov_flash(data_address-0x8000,temp2);      
	}
	edit_prog=0;                       
	block_contol[0]=100;
	block_contol[1]=100;
	prog_address=(tx_data[2]*0x100+tx_data[3])-0x8000;
	data_size=tx_data[5];
	for(temp=0;temp<data_size;temp++)
	{
		block_contol[0]=(prog_address+temp)/0x800;	
		if(block_contol[0]==block_contol[1])			 
		{
			//prog_write_buffer[(prog_address+temp)-block_contol[0]*0x800]=tx_data[6+temp];  
		}
		else							  
		{
			write_block(block_contol[1]);   
			backup_block(block_contol[0]);  
			block_contol[1]=block_contol[0];
			//prog_write_buffer[(prog_address+temp)-block_contol[0]*0x800]=tx_data[6+temp];
		}
	} 
	write_block(block_contol[0]);   
	tx_data[0]=0x06,tx_count=1;
}

void all_flash_unlock(void)																		//flash all unlocked
{
	block_contol[1]=200;
	block_contol[0]=200;
	tx_data[0]=0x06,tx_count=1;    
}

void all_flash_lock(void)																			//flash all lock
{
	writetoFlash();																							//- Write PLC ladder to file --> by Tom
	//loadDMemory();																							//- Load D Datat back					
	//write_block(block_contol[1]);															//Write the data to be written to flash before locking
	//block_contol[1]	=200;
	//block_contol[0]=200;
	//FLASH_Lock();
	tx_data[0]=0x06,tx_count=1;   
}

void PC_OPTION_PROG(void)																			//Extended function "E" code
{ 	
	u16 temp;
	if((rx_count==10)&&((rx_data[2]==0x37)||(rx_data[2]==0x38)))//Is it mandatory
	{	
		prog_address=hex[rx_data[3]]*0x10+hex[rx_data[4]]+hex[rx_data[5]]*0x1000+hex[rx_data[6]]*0x100;
		if(rx_data[2]==0x37)
		  EPC_FORCE_ON();	
		else
			EPC_FORCE_OFF();
	}
	else
	{
		setup_LH();																							//call the function to calculate the address
		temp=tx_data[1];
		switch(temp) 
		{ 
			case 0x00: PC_READ_Parameter();						break;		//read configuration E00
			case 0x10: PC_WRITE_Parameter(); 					break;		//write configuration E10
			case 0x01: PC_READ_PORG();       					break;		//Read program E01
			case 0x11: PC_WRITE_PORG(); 							break;		//write program E11
			case 0x77: all_flash_unlock();   					break;		//Use the e command to write a program write request 77
			case 0x87: all_flash_lock();     					break;		//Use the e command to write the program end request 87
			case 0x41: find_data_address();  					break;		//find the address of the end command
			case 0x61: all_flash_unlock();   					break;		//PLC storage memory cleaning should be issued many times, I unlock all of them in this FLASH
			case 0x60: ErasurePLC(1);  								break;		//PLC storage memory cleanup
			case 0x63: ErasurePLC(2);  								break;		//PLC clears data element bits
			case 0x64: ErasurePLC(2);  								break;		//PLC clears data element bytes
			case 0x62: ErasurePLC(3); 								break;		//plc clean up bit element
			case 0xD1: online_write_data();  					break;  
			default: tx_data[0]=0x15,tx_count=1;			break;		//encountered an unsupported command
		} 
	}
}

void find_end(void)																						//Find if there is an end instruction in the program,
{
	if(rx_count==13)
	tx_data[0]=0x06,tx_count=1;  
	else
	tx_data[0]=0x06,tx_count=1; 
}

void Process_switch(void)
{                                                                                                                      
	u8 temp;
	switch_read_data();										//Convert the ascii code starting from the third digit into hex, and the address is the data sending area
	temp=rx_data[1];

	switch(temp){ 
		case 0x30: data_size=tx_data[3],PC_READ_byte();  	break;		//Substitute the requested data length bit "tx_data[4]" to read data
		case 0x31: data_size=tx_data[3],PC_WRITE_byte(); 	break;		//Substitute the request data length bit "tx_data[4]" to write data
		case 0x34: find_end();                           	break;		//find command, if there is data found, return 6
		case 0x37: setup_HL(),PC_FORCE_ON();             	break;		//PLC start remote "0x37"
		case 0x38: setup_HL(),PC_FORCE_OFF();            	break;		//PLC stop remote "0x38"
		case 0x42: all_flash_lock();                     	break;		//Write parameters to end the command
		case 0x45: data_size=tx_data[4],PC_OPTION_PROG();	break;		//e function instruction
		default:	                                     		break;
	}        
	                                                                                                               
	if((tx_count==0)&&(rx_count==0)){	        //return error code for 0x15
		tx_data[0]=0x15,tx_count=1;
	}			 		
	rx_count=0;Send_out=1;										//report to the host computer
}								 

//Serial port function configuration
void USART1_Configuration(void){
	//uart0Config(19200);
	xTaskCreatePinnedToCore(TX_Task,"TX_Task",5000,(void*) NULL,1,&t1,1);
	xTaskCreatePinnedToCore(RX_Task,"RX_Task",5000,(void*) NULL,2,&t2,1);
}

void uart0Config(int baud){
	//Serial.begin(baud,SERIAL_7E1);
	//Serial.begin(115200);
	//Serial.setRxBufferSize(1024);
	//Serial.setTimeout(500);
}

void RX_Process(){																//Receive serial port data
	static u8 sum,f=1;
	while(Serial.available() > 0){
		rx_data[rx_count] = 0x7f&Serial.read();
		rx_count += 1;
	}

	if(rx_data[0]==0x05){																																		//The host computer makes a communication request
		rx_data[0]=0x00,rx_count=0,tx_data[0]=0x06,tx_count=1,Send_out=1,TX_Process();				//Report to the host computer and return 0 x06 response
	}else if(rx_count>1){																							//Confirm the start of the message
		if(rx_count>141){																					//Read data error greater than 143
			tx_count=0,rx_count=0;
		}

		if((rx_count>1)&&(rx_data[0] != 0x02)){
			rx_count = 0;
		}
		
		if((rx_count>3)&&(rx_data[rx_count-3]==0x03))							//Whether the data transmission is over
		{ 
			sum = check(rx_data);           
			if((rx_data[rx_count-2] == Ascll[sum/0x10])&&(rx_data[rx_count-1] == Ascll[sum%0x10]))//Calculate data and status whether the data is normal
			{
				Process_switch(); 
			}
			else{
				tx_data[0]=0x15,tx_count=1,rx_count=0,Send_out=1,TX_Process();								//Report to the host computer and answer the data exception and return the value 0 x15
			} 
		}else if(rx_data[0] == 0x16){				//- Restart PLC
			ESP.restart();
		}else if(rx_data[0] == 0x17){				//- Get ladder details
			listDir(SPIFFS,"/",0);
			readFile(SPIFFS,Ladder_path);
			writeFile(SPIFFS,Ladder_path,"FF");
			rx_data[0] = 0;
			rx_count = 0;
		}else if(rx_data[0] == 0x18){				//- Clear ladder
			listDir(SPIFFS,"/",0);
			readFile(SPIFFS,Ladder_path);
			deleteFile(SPIFFS,Ladder_path);
			rx_data[0] = 0;
			rx_count = 0;
		}
	}	
}

void RX_Task(void *pvParam){
  while(1){
		RX_Process();
    vTaskDelay(xDelay30ms);   // Delay Task ??? 20 ms
  }  
	vTaskDelete(NULL);  
}

void TX_Process(){
	if(Send_out){
		Serial.write(tx_data,tx_count);
		Send_out = 0;
	}
}

void TX_Task(void *pvParam){
  while(1){
		TX_Process();
    vTaskDelay(xDelay30ms);   // Delay Task ??? 50 ms
  }  
	vTaskDelete(NULL);  
}
