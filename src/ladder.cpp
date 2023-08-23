#include <stdio.h>
#include "plc_io.h"
#include "plc_dialogue.h"
#include "math.h"          									//Math function library
#include "gray.h"          									//Gray code conversion library
#include "plc_conf.h"
#include "tftype_def.h"
#include "tf_modbus_def.h"

//extern uint16_t masterDataPos;
extern unsigned char TxBuffer2[];
extern unsigned char TxCounter2;
//extern void loadDMemory();

extern void RST_T_D_C_M_data(void);	 				//ADD 20151214
extern unsigned char Y0P,Y1P;               //
extern unsigned short Plus_CMP0,Plus_CMP1;	//pulse flag
extern u8  X_DIY;														//filter time
extern u16 PLC_RUN_TIME;										//Scan time
extern void RTC_Set(u16 syear,u8 smon,u8 sday,u8 hour,u8 min,u8 sec);			//time modification program
extern void PLC_IO_Refresh(void);						//io refresh output program
//extern u16 PLC_16BIT[12100];							//plc ram run register
extern void timer_enable(u16 timer_number);	//Here in timer 2, used to start timer processing
extern void timer_disble(u16 timer_number);	//here in1 timer 2
static u8 PLC_ACC_BIT,PLC_MPS_BIT;					//Program execution dedicated (operation stack and branch stack)
static u16 *PLC_Addr;												//plc program pointer   					//- Original - static const u16 *PLC_Addr;
static u16 *PLC_Err;												//plc error step									//- static const u16 *PLC_Err;
static u8 T_number,C_number;								//t&c address buffer register
static u16 T_value;													//t compare cache register
static u16 C_value;													//c compare cache register
static u32 mov_d_addr;											//k?m&y&s&x instruction cache
static u16 *PLC_P_Addr[129];								//Convenient to call the subroutine to get the pointer 					//- static const u16 *PLC_P_Addr[129];
static u16 *p_save[129];										//Save the last execution point when calling the subroutine			//- static const u16 *p_save[129];
u8  Flag_bit=0xff,Transfer_bit,Transfer_bit1;		//In fact, it is the flag bit of k, which reduces the amount of functions and reduces the burden on the CPU.
u16 process[64];														//Save the last subroutine value when calling the subroutine
u32 trade;																	//Act on addition and subtraction, reduce the amount of functions, reduce the burden on the CPU
u16 Transfer=0;															//Multiple transfers and batch transfers reduce the amount of functions and reduce the burden on the CPU
u8 edit_prog;																//Reprogram the cache register
extern u8 Write_Pro_flag;
const unsigned char PLC_BIT_OR[]={0X01,0X02,0X04,0X08,0X10,0X20,0X40,0X80};
const unsigned char PLC_BIT_AND[]={0XFE,0XFD,0XFB,0XF7,0XEF,0XDF,0XBF,0X7F};
float_union FLOAT;
s32_union   u32data,u32data1;
u64_union   u64data,u64data2;
extern u8 PLC_FLASH[];
extern u8 PLC_RAM[];

//extern modebusStack mbStack[];
//extern modebusStack *mbSetStack;
//extern modebusStack *mbGetStack;

bool loadMemSts;

/***************************************************FOR**************************************************/
struct 
{
	const u16 *Addr[7];									//FOR address record
	u16 cycle[7];												//the current number of loops
	u16 count[7];												//number of target loops
	u8  point;													//The number of points pointed to by for
} FOR_CMD;

/***************************************************STL**************************************************/
static u16 PLC_STL_Addr;							//stl instruction address number
static u8  PLC_STL_Status;						//STL instruction current state 0 The whole program has no STL state, program 1 is STL stateful, and 2 is STL stop state
static u8  PLC_STL_CMD;								//stl flag
static u8  PLC_STL_Count;							//count the number of coils
static u16 PLC_STL_Coil[256];					//coil buffer register
/************************************************************************************************************/
static u8 PLC_PL_BIT_TEST(u16 x){return((Step_Address[(x)/8] & PLC_BIT_OR[(x)%8])) ? (1) : (0);}
static u8 PLC_LD_BIT(u16 x){return((MEMORY_8BIT(PLC_RAM+((x)/8)) & PLC_BIT_OR[(x)%8])) ? (1) : (0) ;}

static u8 PLC_LDP_TEST(void)					//Check if it is a rising edge
{ 
	if(PLC_PL_BIT_TEST(PLC_Addr-PLC_START_Address)==OFF)		//Rising edge judgment
	{ 
		   if(PLC_ACC_BIT&0X01)																//judge the current value
	     {
				 PLC_PL_BIT_ON(PLC_Addr-PLC_START_Address);   		//
         return 1;
	     }
			 else return 0;
	 }
	 else
	 {
			if(!(PLC_ACC_BIT&0x01))															//judge the current value
		  PLC_PL_BIT_OFF(PLC_Addr-PLC_START_Address);    			//
			return 0;
	 } 
}

/*********************************************
Function: PLC code error handler
err_id=01: command error (command not recognized)
err_id=02: Command error (command is not supported yet)
err_id=10: data error (unrecognized data type)
err_id=11: data error (data read address exceeded)
err_id=12: data error (index Z address unknown)
err_id=13: data error (index Z address exceeded)
err_id=20: CJ instruction address error
D8061, M8061=PC hardware error
D8063, M8063=Link, communication error
D8064, M8064=Parameter error
D8065, M8065=Syntax error
D8066, M8066=Loop error
D8067, M8067=Operation error
D8068, M8068=Operation error latch
***********************************************/
void PLC_PROG_ERROR(u16 err,u16 err_id)
{
	ERR_ON();
	//Serial2.print("LadderError: ");
	//Serial2.println(err);
	//Serial2.print("Id: ");
	//Serial2.println(err_id);
	//PLC_BIT_ON(err); 																					//Error flag Xiao Xiaosheng shield
	D8012=0;																										//Scan time
	if(D8068==0){
		D8067=err_id;																							//Grammatical errors
		//Serial2.print("Grammar err: ");
		//Serial2.println(D8067);
	}

	if(D8068==0){
		D8068=(PLC_Err-(u16*)FLASH_16BIT(0x5D));									//save the error pc step
		D8069=D8068;
		//Serial2.print("Other err: ");
		//Serial2.println(D8069);
	}	
	ERR_OFF();
}

static void LD(u16 start_addr)																//Start address, add component number value
{ 
	//Serial.printf("Val: %d\n",start_addr);
	 if(PLC_STL_Status == 1)																		//Global step for STL status area
	 {  
		PLC_ACC_BIT<<=1;
		if(PLC_BIT_TEST(start_addr)&&(PLC_BIT_TEST(PLC_STL_Addr)))     
			PLC_ACC_BIT |=0x01;
	 }	
   else
    {  
		//Serial.printf("ACC-before: %X\n",PLC_ACC_BIT);
		PLC_ACC_BIT<<=1;
		if(PLC_BIT_TEST(start_addr)) 
			PLC_ACC_BIT |=0x01;
		
		//Serial.printf("ACC-after: %X\n",PLC_ACC_BIT);
	}		 		
}

static void LDI(u16 start_addr)
{ 
	//Serial.printf("Val: %d\n",start_addr);
	if(PLC_STL_Status == 1)																			//Global step for STL status area
	{   
		PLC_ACC_BIT<<=1;
		if((!(PLC_BIT_TEST(start_addr)))&&(PLC_BIT_TEST(PLC_STL_Addr)))
		PLC_ACC_BIT |=0x01;
	}
	else	
	{		
		//Serial.printf("ACC-before: %X\n",PLC_ACC_BIT);
		PLC_ACC_BIT<<=1;
		if(PLC_BIT_TEST(start_addr));
		else		
		PLC_ACC_BIT |=0x01;
		//Serial.printf("ACC-after: %X\n",PLC_ACC_BIT);
	}
}

void AND(u16 start_addr)
{ 
	if((PLC_BIT_TEST(start_addr))&&(PLC_ACC_BIT&0X01)) 
	PLC_ACC_BIT|=0X01;
	else
	PLC_ACC_BIT&=0XFE; 
}

static void ANI(u16 start_addr)
{ 
	if((!(PLC_BIT_TEST(start_addr)))&&(PLC_ACC_BIT&0X01)) 
		PLC_ACC_BIT|=0X01;
	else
		PLC_ACC_BIT&=0XFE; 
}

static void OR(u16 start_addr)
{ 
	if((PLC_BIT_TEST(start_addr))||(PLC_ACC_BIT&0X01))
	PLC_ACC_BIT|=0X01;
	else
	PLC_ACC_BIT&=0XFE; 
}

static void ORI(u16 start_addr)
{ 
	if((!(PLC_BIT_TEST(start_addr)))||(PLC_ACC_BIT&0X01)) 
	PLC_ACC_BIT|=0X01;
	else
	PLC_ACC_BIT&=0XFE; 
}

void OUT(u16 start_addr)
{	
	if (PLC_STL_CMD == 1)																			//Determine whether to enter the step mode
	{
		if (PLC_STL_Status == 1)																//Does the stl mode open?
		{
				if(start_addr < 0x000A)															//Judgment start step S000-S009
				{
				if((PLC_ACC_BIT&0x01)==0x01)
				{	
					PLC_BIT_OFF(PLC_STL_Addr); 												//OFF			      
					PLC_BIT_ON(start_addr);    												//ON						 
				}
				}
				else 
				{
				if(PLC_BIT_TEST(PLC_STL_Addr))
				{						 
					if((PLC_ACC_BIT&0x01)==0x01)             
					{ 
						PLC_BIT_ON(start_addr);       							//ON
						PLC_STL_Coil[PLC_STL_Count++]=start_addr;		//Record the ON coil address during the step. The next step is established for clearing.
					}       
					else 
						PLC_BIT_OFF(start_addr);      							//OFF 
				}

				}
		}
		else
		{
				if(start_addr < 0x000A)															//Judgment start step S000-S009
				{
					if(PLC_ACC_BIT & 0x01)
					{
						PLC_BIT_ON(start_addr); 												//ON
					}
				}
				else
				{
					if(PLC_ACC_BIT&0x01)
						PLC_BIT_ON(start_addr);         								//ON 
					else 
						PLC_BIT_OFF(start_addr);      									//OFF 
				}
		}
	}
	else
	{
		if(PLC_ACC_BIT&0x01)
			PLC_BIT_ON(start_addr);    														//ON
		else
			PLC_BIT_OFF(start_addr);   														//OFF 
	}
}

static void BIT_SET(u16 start_addr)														//bit setting
{ 
	u8 temp;
	if(PLC_ACC_BIT&0x01)
	{
		if (PLC_STL_Status == 1)																	//is the stl status area
		{
			for(temp=0;temp<=PLC_STL_Count;temp++)
			PLC_BIT_OFF(PLC_STL_Coil[temp]);												//Clear the last on coil state
			
			PLC_BIT_OFF(PLC_STL_Addr); 															//OFF
			PLC_BIT_ON(start_addr);    															//ON
			PLC_STL_Count=0;																				//Clear the last recorded number of on coils
		}
		else PLC_BIT_ON(start_addr);   														//0N
	}
}

static void RST(u16 start_addr)																//reset bit
{ 
  if((PLC_ACC_BIT&0X01)==0X01)
  PLC_BIT_OFF(start_addr);     																//OFF 
}

static void RET(void)
{  
  PLC_STL_Status =0;																					//Exit step mode, let the program enter the ladder diagram
}

void STL(u16 start_addr)																			//step mode
{
	PLC_STL_CMD = 1;																						//Global program enable step flag
	PLC_STL_Status = 1;																					//start step mode
	PLC_STL_Addr = start_addr;																	//Record the step address
	PLC_ACC_BIT<<=1;
	if(PLC_BIT_TEST(PLC_STL_Addr))     
	PLC_ACC_BIT |=0x01;
}

//Xiao Xiaosheng, optimized in 20160926
static void other_function(u8 process_addr)
{
	 switch(process_addr)
   { 
		case 0xF8://block concatenation ANB
    {			
			PLC_ACC_BIT = (PLC_ACC_BIT >> 1)   & ((PLC_ACC_BIT & 0x01)|0xFE);           
			break;  
		}
		case 0xF9://block parallel ORB
    {			
			PLC_ACC_BIT = (PLC_ACC_BIT >> 1)   | (PLC_ACC_BIT & 0x01);                  
			break;  
	  }
		case 0xFA://Push stack MPS
    {			
			PLC_MPS_BIT = (PLC_MPS_BIT << 1)   | (PLC_ACC_BIT & 0x01);                  
			break;  
		}
		case 0xFB://read stack MRD
    {			
			PLC_ACC_BIT = (PLC_ACC_BIT & 0xfe) | (PLC_MPS_BIT & 0x01);                  
			break;	
		}
		case 0xFC://Pop stack MPP
    {			
			PLC_ACC_BIT = (PLC_ACC_BIT & 0xfe) | (PLC_MPS_BIT & 0x01),PLC_MPS_BIT >>= 1;
			break;  
		}
		case 0xFD://negate INV
		{
			PLC_ACC_BIT = (PLC_ACC_BIT & 0xfe) | (~PLC_ACC_BIT & 0x01);                 
			break;
		}
		case 0xFF://negate POP
    {			
			break;  
		}
		default:
		{
			PLC_PROG_ERROR(M8064,02);                                                        
		  break;
		}
   }
}

static void LPS(void)														//m1536~m3071-bit lps instruction function
{ 
	 if(PLC_ACC_BIT&0x01)
	 {
		  if(PLC_PL_BIT_TEST(PLC_Addr-PLC_START_Address)==0)
		  {
			   PLC_PL_BIT_ON(PLC_Addr-PLC_START_Address);
			   PLC_BIT_ON((0x2fff&*PLC_Addr));	 
		  }
			else{PLC_BIT_OFF((0x2fff&*PLC_Addr));}
	 }
	 else{PLC_PL_BIT_OFF(PLC_Addr-PLC_START_Address);}
	 PLC_Addr++;
}

static void LPF(void)														//m1536~m3071-bit lps instruction function
{ 
   if(PLC_ACC_BIT&0x01)
	 {
		if(PLC_PL_BIT_TEST(PLC_Addr-PLC_START_Address)==0)
		{PLC_PL_BIT_ON(PLC_Addr-PLC_START_Address);}
	 }
	 else
	 {
		   if(PLC_PL_BIT_TEST(PLC_Addr-PLC_START_Address)) 
		   {
			   PLC_PL_BIT_OFF(PLC_Addr-PLC_START_Address);	 
		       PLC_BIT_ON((0x2fff&*PLC_Addr));
		   }	 
		   else{PLC_BIT_OFF((0x2fff&*PLC_Addr));} 
	 }
	 PLC_Addr++;
}

static void RESET_T(u8 process_addr)						//timer reset
{  
	if(PLC_ACC_BIT&0x01)													//Is the current value valid?
	{
		PLC_BIT_OFF(0x600+process_addr);						//overflow coil
		PLC_BIT_OFF(0x2600+process_addr);						//enable coil
		PLC_BIT_ON(0x2300+process_addr);						//reset coil
		RAM_16BIT(0x0800+process_addr) = 0;						//actual count
	}
	else
	PLC_BIT_OFF(0x2300+process_addr);							//reset coil
}

static void RESET_C(u8 process_addr)						//timer reset
{ 
	static u16 *p_data;
	if((PLC_ACC_BIT&0x01)==0x01)									//Is the current value valid?
	{ 
		if((process_addr>=0XC8)&&(process_addr<=0XFF))
		{
			p_data = (u16*)PLC_RAM+0x0500+process_addr;			//point to value address
			*p_data=0;																//clear address high bit
			p_data+=1;																//because it is 32 bit
			*p_data=0;																//clear address low bit
			PLC_BIT_OFF(0x00E0+process_addr);					//Point to the overflow coil and clear the overflow coil
		}
		else
		{
			p_data = (u16*)PLC_RAM+0x0500+process_addr;			//point to value address
			*p_data=0;																//clear address
			PLC_BIT_OFF(0x00E0+process_addr);					//Point to the overflow coil and clear the overflow coil
		}
	}
	OUT(0X3700+process_addr);
}

static void RST_T_C(void)												//All t c bit rst instruction function
{  
	switch(*PLC_Addr/0x100)
	{
		case 0x86: RESET_T(*PLC_Addr),PLC_Addr++;break;				//reset t
		case 0x8E: RESET_C(*PLC_Addr),PLC_Addr++;break;				//reset c
	}
}

static void MOV_TO_K_H(u8 i,u32 data,u8 addr)		//Do MOV ?? K?X&Y&S&M address calculation
{  
	u8 LL_BIT;																		//How many bits need to be shifted to the left using the register
	u16 JOB_ADDR;
	int64_t MOV_DATA_64BIT,MOV_DATA_64BIT_BACKUP,MOV_DATA_BACKUP1;				//move 32 bit data
	mov_d_addr|=addr<<8; 
	mov_d_addr+=Transfer;													//In the case of batch transfer and multi-point transfer, there will be data usually 0
	LL_BIT=mov_d_addr%0x20;												//how many bits to move
	JOB_ADDR=(mov_d_addr/0x20)*4;									//start address
	switch(i)							 
	{	//Move the number of bits. The required data is processed well. Move the data to the required number of bits, and then reverse the data for later use.
		case 0x82: MOV_DATA_64BIT_BACKUP=data&0X0000000F,MOV_DATA_64BIT_BACKUP<<=LL_BIT,MOV_DATA_BACKUP1=~(0X0000000F<<LL_BIT); break;//Teleport k1 size
		case 0x84: MOV_DATA_64BIT_BACKUP=data&0X000000FF,MOV_DATA_64BIT_BACKUP<<=LL_BIT,MOV_DATA_BACKUP1=~(0X000000FF<<LL_BIT); break;//Teleport k2 size
		case 0x86: MOV_DATA_64BIT_BACKUP=data&0X00000FFF,MOV_DATA_64BIT_BACKUP<<=LL_BIT,MOV_DATA_BACKUP1=~(0X00000FFF<<LL_BIT); break;//Teleport k3 size
		case 0x88: MOV_DATA_64BIT_BACKUP=data&0X0000FFFF,MOV_DATA_64BIT_BACKUP<<=LL_BIT,MOV_DATA_BACKUP1=~(0X0000FFFF<<LL_BIT); break;//Teleport k4 size
		case 0x8A: MOV_DATA_64BIT_BACKUP=data&0X000FFFFF,MOV_DATA_64BIT_BACKUP<<=LL_BIT,MOV_DATA_BACKUP1=~(0X000FFFFF<<LL_BIT); break;//Teleport k5 size
		case 0x8C: MOV_DATA_64BIT_BACKUP=data&0X00FFFFFF,MOV_DATA_64BIT_BACKUP<<=LL_BIT,MOV_DATA_BACKUP1=~(0X00FFFFFF<<LL_BIT); break;//Teleport k6 size
		case 0x8E: MOV_DATA_64BIT_BACKUP=data&0X0FFFFFFF,MOV_DATA_64BIT_BACKUP<<=LL_BIT,MOV_DATA_BACKUP1=~(0X0FFFFFFF<<LL_BIT); break;//Teleport k7 size
		case 0x90: MOV_DATA_64BIT_BACKUP=data&0XFFFFFFFF,MOV_DATA_64BIT_BACKUP<<=LL_BIT,MOV_DATA_BACKUP1=~(0XFFFFFFFF<<LL_BIT); break;//Send k8 size
		default:   PLC_Addr+=3;                           break;					//encountered an unsupported command
	}                                          
	MOV_DATA_64BIT=MEMORY_64BIT(PLC_RAM+JOB_ADDR);
	MOV_DATA_64BIT&=MOV_DATA_BACKUP1;										//Clear the location to be sent
	MOV_DATA_64BIT|=MOV_DATA_64BIT_BACKUP;							//pass in the desired location
	MEMORY_64BIT(PLC_RAM+JOB_ADDR)=MOV_DATA_64BIT;				//transfer data to destination
}
 
static signed int MOV_K(u8 Addr)								//Calculate k?x&y&s&m
{ 
	static u16 LL_BIT,JOB_ADDR;										//How many bits need to be shifted to the left using the register
	static uint64_t MOV_DATA_64BIT;								//move 64 bit data
	mov_d_addr|=(Addr<<8);                    
	mov_d_addr+=Transfer;													//In the case of batch transfer and multi-point transfer, there will be data usually 0
	LL_BIT=mov_d_addr%0x20;												//how many bits to move
	JOB_ADDR=(mov_d_addr/0x20)*4;									//start address
	MOV_DATA_64BIT=MEMORY_64BIT(PLC_RAM+JOB_ADDR),
	MOV_DATA_64BIT>>=LL_BIT; 									 
	return  (signed int)MOV_DATA_64BIT;
}

//Calculate the high-order address
u16 D_C_T_addr(u8 l_value)
{ 
	static u16 temp; 
	switch(*PLC_Addr/0x100)
	{
		case 0x80: temp=l_value+((*PLC_Addr%0x100)*0x100),temp=0x0700+temp,PLC_Addr++;      break;//greater than or equal to D1000
		case 0x82: temp=l_value+((*PLC_Addr%0x100)*0x100),temp=0x0800+temp,PLC_Addr++;      break;//Calculate the address of T and turn the address into a value
		case 0x84: temp=l_value+((*PLC_Addr%0x100)*0x100),temp=0x0500+temp,PLC_Addr++;      break;//Calculate the address of C and turn the address into a value
		case 0x86: temp=l_value+((*PLC_Addr%0x100)*0x100),temp=0x3000+temp,PLC_Addr++;      break;//Calculate the address of D and turn the address into a value
		case 0x88: temp=l_value+((*PLC_Addr%0x100)*0x100),temp=0x3000+temp,PLC_Addr++; 			break;//greater than or equal to D1000

		//case 0x80: temp=l_value+((*PLC_Addr%0x100)*0x100),temp=0x0700+temp/2,PLC_Addr++;      break;//greater than or equal to D1000
		//case 0x82: temp=l_value+((*PLC_Addr%0x100)*0x100),temp=0x0800+temp/2,PLC_Addr++;      break;//Calculate the address of T and turn the address into a value
		//case 0x84: temp=l_value+((*PLC_Addr%0x100)*0x100),temp=0x0500+temp/2,PLC_Addr++;      break;//Calculate the address of C and turn the address into a value
		//case 0x86: temp=l_value+((*PLC_Addr%0x100)*0x100),temp=0x1000+temp/2,PLC_Addr++;      break;//Calculate the address of D and turn the address into a value
		//case 0x88: temp=l_value+((*PLC_Addr%0x100)*0x100),temp=0x1000+temp/2+1000,PLC_Addr++; break;//greater than or equal to D1000
	}
	return temp;
}

//===================================================== ===================================================== ===
//Function name: static u16 addr_value(void)
//Function description: Calculate PLC address or real number of k
//Input: void
//output: address or data
//global variable:
//Calling the module:
//Author: Xiao Xiaosheng
//Date: June 2, 2015
//Remark:
//-------------------------------------------------------------------------------------------------------
//Modified by:
//date:
//Remarks: Part of the instruction test in the optimization program number test After optimization, the return address k is the data
//-------------------------------------------------------------------------------------------------------
//=======================================================================================================
static u16 addr_value(void)                                        
{  
	static u8 temp;static u16 temp1;
	switch(*PLC_Addr/0x100)
	{
		case 0x84: temp=*PLC_Addr,PLC_Addr++,temp1=*PLC_Addr<<8|temp,PLC_Addr++,Flag_bit=0;break;//Carry out transmission such as K4M0 break;								//Carry out the address of C
		case 0x86: temp=*PLC_Addr,PLC_Addr++;temp1=D_C_T_addr(temp);                       break;//Calculate the address of D, D, D
	}
	return temp1;
}

//===================================================== ===================================================== ===
//Function name: static u32 addr_value_prog(void)
//Function description: Calculate PLC address or real number of k
//Input: void
//output: address or data
//global variable:
//Calling the module:
//Author: Xiao Xiaosheng
//Date: June 23, 2015
//Remark:
//-------------------------------------------------------------------------------------------------------
//Modified by:
//date:
//Remark:
//===================================================== ===================================================== ===
static u32 addr_value_prog(void)                                      
{  
	static u32 temp; 
	u16 Type_F,temp2,Data1,Data2;
	Data1=*PLC_Addr;PLC_Addr++;Data2=*PLC_Addr;
	temp2=Type_F=0;
	Type_F   = (Data1 & 0xff00);
	Type_F  |= (Data2 >> 8);
	
	temp2  = (Data2 << 8);
	temp2 |=mov_d_addr=(u8)Data1;
	
	if(Type_F == 0x8680)      temp=(u32)RAM_D8000_ADDR+temp2,  PLC_Addr++;//Calculate the address of D D8000
	else if(Type_F == 0x8682) temp=(u32)RAM_T_ADDR+temp2,      PLC_Addr++;//Calculate the address of T
	else if(Type_F == 0x8684) temp=(u32)RAM_C_ADDR+temp2,      PLC_Addr++;//Calculate the address of C
	else if(Type_F == 0x8686) temp=(u32)RAM_D_ADDR+temp2,      PLC_Addr++;//Calculate the address of D
	else if(Type_F == 0x8688) temp=(u32)RAM_D1000_ADDR+temp2,  PLC_Addr++;//greater than or equal to D1000
	else if(Type_F == 0x8482) temp=MOV_K(*PLC_Addr)&0X0000000F,Flag_bit=0,PLC_Addr++;//Make teleports like K4M0    
	else if(Type_F == 0x8484) temp=MOV_K(*PLC_Addr)&0X000000FF,Flag_bit=0,PLC_Addr++;//Make teleports like K4M0   
	else if(Type_F == 0x8486) temp=MOV_K(*PLC_Addr)&0X00000FFF,Flag_bit=0,PLC_Addr++;//Make teleports like K4M0    
	else if(Type_F == 0x8488) temp=MOV_K(*PLC_Addr)&0X0000FFFF,Flag_bit=0,PLC_Addr++;//Make teleports like K4M0  
	else if(Type_F == 0x848A) temp=MOV_K(*PLC_Addr)&0X000FFFFF,Flag_bit=0,PLC_Addr++;//Make teleports like K4M0    
	else if(Type_F == 0x848C) temp=MOV_K(*PLC_Addr)&0X00FFFFFF,Flag_bit=0,PLC_Addr++;//Make teleports like K4M0   
	else if(Type_F == 0x848E) temp=MOV_K(*PLC_Addr)&0X0FFFFFFF,Flag_bit=0,PLC_Addr++;//Make teleports like K4M0    
	else if(Type_F == 0x8490) temp=MOV_K(*PLC_Addr),           Flag_bit=0,PLC_Addr++;//Make teleports like K4M0
	return temp;
}

unsigned short V0_V3(u16 temp1)
{
	u8 temp=PLC_v_z_addr(temp1);
	if(temp==0)       return D8029;
	else if(temp==1)  return D8183;
	else if(temp==2)  return D8185;
	else if(temp==3)  return D8187;
	else	return 0;
}

unsigned short V4_V7(u16 temp1)
{
	u8 temp=PLC_v_z_addr(temp1);
	if(temp==0)       return D8189;
  else if(temp==1)  return D8191;
  else if(temp==2)  return D8193;
  else if(temp==3)  return D8195;
  else	return 0;
}

unsigned short Z0_Z3(u16 temp1)
{
	u8 temp=PLC_v_z_addr(temp1);
	if(temp==0)       return D8028;
  else if(temp==1)  return D8182;
  else if(temp==2)  return D8184;
  else if(temp==3)  return D8186;
  else	return 0;
}

unsigned short Z4_Z7(u16 temp1)
{
	u8 temp=PLC_v_z_addr(temp1);
	if(temp==0)       return D8188;
  else if(temp==1)  return D8190;
  else if(temp==2)  return D8192;
  else if(temp==3)  return D8194;
  else	return 0;
}

unsigned int DZ0_Z3(u16 temp1)
{
	u8 temp=PLC_v_z_addr(temp1);
	if(temp==0)       return D8028+D8029*0X10000;
  else if(temp==1)  return D8182+D8183*0X10000;
  else if(temp==2)  return D8184+D8184*0X10000;
  else if(temp==3)  return D8186+D8185*0X10000;
  else	return 0;
}

unsigned short DZ4_Z7(u16 temp1)
{
	u8 temp=PLC_v_z_addr(temp1);
	if(temp==0)       return D8188+D8189*0X10000;
  else if(temp==1)  return D8190+D8191*0X10000;
  else if(temp==2)  return D8192+D8193*0X10000;
  else if(temp==3)  return D8194+D8193*0X10000;
  else	return 0;
}
//===================================================== ===================================================== ===
//Function name: static void target(void)
//Function description: Addition and subtraction "AND", "OR" and "XOR" share assignment functions, such as the results of DMOV, DADD, DSUB and other instructions.
//Input: void
//output: void
//global variable:
//Calling the module:
//Author: Xiao Xiaosheng
//Date: June 23, 2015
//Remark:
//===================================================== ===================================================== ===

static void D_target(void)                                      
{  
	 u16 Type_F,temp2,Data1,Data2;
	 Data1=*PLC_Addr;PLC_Addr++;Data2=*PLC_Addr;
	 temp2=Type_F=0;
	 Type_F   = (Data1 & 0xff00);
	 Type_F  |= (Data2 >> 8);
/************************************/	
	 temp2  = (Data2 << 8);
	 temp2 |=mov_d_addr=(u8)Data1;
/************************************/	
	switch(Type_F)
	{
		case 0x8482: MOV_TO_K_H(Type_F,trade,*PLC_Addr),PLC_Addr++,Transfer_bit=1;break; //Do a transfer like K4M0    
		case 0x8484: MOV_TO_K_H(Type_F,trade,*PLC_Addr),PLC_Addr++,Transfer_bit=1;break; //Do a transfer like K4M0   
		case 0x8486: MOV_TO_K_H(Type_F,trade,*PLC_Addr),PLC_Addr++,Transfer_bit=1;break; //Do a transfer like K4M0   
		case 0x8488: MOV_TO_K_H(Type_F,trade,*PLC_Addr),PLC_Addr++,Transfer_bit=1;break; //Do a transfer like K4M0   
		case 0x848A: MOV_TO_K_H(Type_F,trade,*PLC_Addr),PLC_Addr++,Transfer_bit=1;break; //Do a transfer like K4M0   
		case 0x848C: MOV_TO_K_H(Type_F,trade,*PLC_Addr),PLC_Addr++,Transfer_bit=1;break; //Do a transfer like K4M0  
		case 0x848E: MOV_TO_K_H(Type_F,trade,*PLC_Addr),PLC_Addr++,Transfer_bit=1;break; //Do a transfer like K4M0    
		case 0x8490: MOV_TO_K_H(Type_F,trade,*PLC_Addr),PLC_Addr++,Transfer_bit=1;break; //Do a transfer like K4M0    
		
		case 0x8680: MEMORY_32BIT(RAM_D8000_ADDR+temp2+Transfer)=trade,PLC_Addr++;break; //Calculate the address of D D8000
		case 0x8682: MEMORY_32BIT(RAM_T_ADDR+temp2+Transfer)=trade,PLC_Addr++;break; //Calculate the address of T
		case 0x8684: MEMORY_32BIT(RAM_C_ADDR+temp2+Transfer)=trade,PLC_Addr++;break; //Calculate the address of C
		case 0x8686: MEMORY_32BIT(RAM_D_ADDR+temp2+Transfer)=trade,PLC_Addr++;break; //Calculate the address of D 
		case 0x8688: MEMORY_32BIT(RAM_D1000_ADDR+temp2+Transfer)=trade,PLC_Addr++;break; //Calculate the address of D1000
		/************************************************K1M0?????"Z"*******************************************************************/
		case 0xA482: mov_d_addr=+DZ0_Z3(temp2),MOV_TO_K_H(Type_F,trade,PLC_D_C_T_addr(temp2)/0x100),PLC_Addr++;break;//K1M0??V0-V3	
		case 0xA483: mov_d_addr=+DZ4_Z7(temp2),MOV_TO_K_H(Type_F,trade,PLC_D_C_T_addr(temp2)/0x100),PLC_Addr++;break;//K1M0??V4-V7	
		/************************************************K2M0?????"Z"*******************************************************************/
		case 0xA484: mov_d_addr=+DZ0_Z3(temp2),MOV_TO_K_H(Type_F,trade,PLC_D_C_T_addr(temp2)/0x100),PLC_Addr++;break;//K2M0??V0-V3	
		case 0xA485: mov_d_addr=+DZ4_Z7(temp2),MOV_TO_K_H(Type_F,trade,PLC_D_C_T_addr(temp2)/0x100),PLC_Addr++;break;//K2M0??V4-V7		
		/************************************************K3M0?????"Z"*******************************************************************/
		case 0xA486: mov_d_addr=+DZ0_Z3(temp2),MOV_TO_K_H(Type_F,trade,PLC_D_C_T_addr(temp2)/0x100),PLC_Addr++;break;//K3M0??V0-V3	
		case 0xA487: mov_d_addr=+DZ4_Z7(temp2),MOV_TO_K_H(Type_F,trade,PLC_D_C_T_addr(temp2)/0x100),PLC_Addr++;break;//K3M0??V4-V7		
		/************************************************K4M0?????"Z"*******************************************************************/
		case 0xA488: mov_d_addr=+DZ0_Z3(temp2),MOV_TO_K_H(Type_F,trade,PLC_D_C_T_addr(temp2)/0x100),PLC_Addr++;break;//K4M0??V0-V3
		case 0xA489: mov_d_addr=+DZ4_Z7(temp2),MOV_TO_K_H(Type_F,trade,PLC_D_C_T_addr(temp2)/0x100),PLC_Addr++;break;//K4M0??V4-V7		
		/************************************************K5M0?????"Z"*******************************************************************/
		case 0xA48A: mov_d_addr=+DZ0_Z3(temp2),MOV_TO_K_H(Type_F,trade,PLC_D_C_T_addr(temp2)/0x100),PLC_Addr++;break;//K5M0??V0-V3	
		case 0xA48B: mov_d_addr=+DZ4_Z7(temp2),MOV_TO_K_H(Type_F,trade,PLC_D_C_T_addr(temp2)/0x100),PLC_Addr++;break;//K5M0??V4-V7		
		/************************************************K6M0?????"Z"*******************************************************************/
		case 0xA48C: mov_d_addr=+DZ0_Z3(temp2),MOV_TO_K_H(Type_F,trade,PLC_D_C_T_addr(temp2)/0x100),PLC_Addr++;break;//K6M0??V0-V3	
		case 0xA48D: mov_d_addr=+DZ4_Z7(temp2),MOV_TO_K_H(Type_F,trade,PLC_D_C_T_addr(temp2)/0x100),PLC_Addr++;break;//K6M0??V4-V7		
		/************************************************K7M0?????"Z"*******************************************************************/
		case 0xA48E: mov_d_addr=+DZ0_Z3(temp2),MOV_TO_K_H(Type_F,trade,PLC_D_C_T_addr(temp2)/0x100),PLC_Addr++;break;//K7M0??V0-V3	
		case 0xA48F: mov_d_addr=+DZ4_Z7(temp2),MOV_TO_K_H(Type_F,trade,PLC_D_C_T_addr(temp2)/0x100),PLC_Addr++;break;//K7M0??V4-V7		
		/************************************************K8M0?????"Z"*******************************************************************/
		case 0xA490: mov_d_addr=+DZ0_Z3(temp2),MOV_TO_K_H(Type_F,trade,PLC_D_C_T_addr(temp2)/0x100),PLC_Addr++;break;//k8M0??V0-V4	
		case 0xA491: mov_d_addr=+DZ4_Z7(temp2),MOV_TO_K_H(Type_F,trade,PLC_D_C_T_addr(temp2)/0x100),PLC_Addr++;break;//K8M0??V4-V7			
		/************************************************T?????"Z"*******************************************************************/
		case 0xA682: MEMORY_32BIT(RAM_T_ADDR+PLC_D_C_T_addr(temp2)+DZ0_Z3(temp2)*2)=(u16)trade,PLC_Addr++;break;//???T????????Z0-Z3
		case 0xA683: MEMORY_32BIT(RAM_T_ADDR+PLC_D_C_T_addr(temp2)+DZ4_Z7(temp2)*2)=(u16)trade,PLC_Addr++;break;//???T????????Z4-Z7
		/************************************************C?????"Z"*******************************************************************/
		case 0xA684: MEMORY_32BIT(RAM_C_ADDR+PLC_D_C_T_addr(temp2)+DZ0_Z3(temp2)*2)=(u16)trade,PLC_Addr++;break;//???C????????Z0-Z3
		case 0xA685: MEMORY_32BIT(RAM_C_ADDR+PLC_D_C_T_addr(temp2)+DZ4_Z7(temp2)*2)=(u16)trade,PLC_Addr++;break;//???C????????Z4-Z7
		/************************************************D?????"Z"*******************************************************************/
		case 0xA686: MEMORY_32BIT(RAM_D_ADDR+PLC_D_C_T_addr(temp2)+DZ0_Z3(temp2)*2)=(u16)trade,PLC_Addr++;break;//???D????????Z0-Z3
		case 0xA687: MEMORY_32BIT(RAM_D_ADDR+PLC_D_C_T_addr(temp2)+DZ4_Z7(temp2)*2)=(u16)trade,PLC_Addr++;break;//???D????????Z4-Z7
		case 0xA688: MEMORY_32BIT(RAM_D1000_ADDR+PLC_D_C_T_addr(temp2)+DZ0_Z3(temp2)*2)=(u16)trade,PLC_Addr++;break;//???????D1000??Z0-Z3
		case 0xA689: MEMORY_32BIT(RAM_D1000_ADDR+PLC_D_C_T_addr(temp2)+DZ4_Z7(temp2)*2)=(u16)trade,PLC_Addr++;break;//???????D1000??Z4-Z7
 }
	 PLC_Addr+=2;
}

//===================================================== ===================================================== ===
//Function name: static void target(void)
//Function description: Addition and subtraction "AND", "OR" and "XOR" share assignment functions such as MOV, ADD, SUB and other instructions to pass the result
//Input: void
//output: void
//global variable:
//Calling the module:
//Author: Xiao Xiaosheng
//Date: June 23, 2015
//Remark:
//===================================================== ===================================================== ===
static void target(void)                                      
{  
	 u16 Type_F,temp2,Data1,Data2;
	 Data1=*PLC_Addr;PLC_Addr++;Data2=*PLC_Addr;
	 temp2=Type_F=0;
	 Type_F   = (Data1 & 0xff00);
	 Type_F  |= (Data2 >> 8);
/************************************/	
	 temp2  = (Data2 << 8);
	 temp2 |=mov_d_addr=(u8)Data1;
/************************************/	
	//Serial.printf("Type_F: %X\n",Type_F);
	//Serial.printf("temp2: %X\n",temp2);
	//Serial.printf("transfer: %X\n",Transfer);
	//Serial.printf("trade: %X\n",trade);
	//Serial.printf("Memory D address: %X\n",RAM_D_ADDR+temp2+Transfer);
	//Serial.printf("Memory D base address: %X\n",RAM_D_ADDR);
	//Serial.printf("Memory data base address: %X\n",RAM_ADDR);
	switch(Type_F)
	{
		case  0x8482: MOV_TO_K_H(Type_F,(u16)trade,*PLC_Addr),PLC_Addr++,Transfer_bit=1;break; //Transfer such as K4M0    
		case  0x8484: MOV_TO_K_H(Type_F,(u16)trade,*PLC_Addr),PLC_Addr++,Transfer_bit=1;break; //Transfer such as K4M0    
		case  0x8486: MOV_TO_K_H(Type_F,(u16)trade,*PLC_Addr),PLC_Addr++,Transfer_bit=1;break; //Transfer such as K4M0    
		case  0x8488: MOV_TO_K_H(Type_F,(u16)trade,*PLC_Addr),PLC_Addr++,Transfer_bit=1;break; //Transfer such as K4M0    
		
		case  0x8680: MEMORY_16BIT(RAM_D8000_ADDR+temp2+Transfer)=(u16)trade,PLC_Addr++;break; 	//Calculate the address of D D8000
		case  0x8682: MEMORY_16BIT(RAM_T_ADDR+temp2+Transfer)=(u16)trade,PLC_Addr++;break; 			//Calculate the address of T 
		case  0x8684: MEMORY_16BIT(RAM_C_ADDR+temp2+Transfer)=(u16)trade,PLC_Addr++;break; 			//Calculate the address of C
		case  0x8686: MEMORY_16BIT(RAM_D_ADDR+temp2+Transfer)=(u16)trade,PLC_Addr++;break; 			//Calculate the address of D
		case  0x8688: MEMORY_16BIT(RAM_D1000_ADDR+temp2+Transfer)=(u16)trade,PLC_Addr++;break; 	//greater than or equal to D1000
		/*****************************************************K1M0 register" V"*********************************************************************/
		case  0x9482: mov_d_addr=+V0_V3(temp2),MOV_TO_K_H(Type_F,(u16)trade,PLC_D_C_T_addr(temp2)/0x100),PLC_Addr++;break;//K1M0??V0-V3	
		case  0x9483: mov_d_addr=+V4_V7(temp2),MOV_TO_K_H(Type_F,(u16)trade,PLC_D_C_T_addr(temp2)/0x100),PLC_Addr++;break;//K1M0??V4-V7	
		/*****************************************************K2M0 register" V"*********************************************************************/
		case  0x9484: mov_d_addr=+V0_V3(temp2),MOV_TO_K_H(Type_F,(u16)trade,PLC_D_C_T_addr(temp2)/0x100),PLC_Addr++;break;//K2M0??V0-V3	
		case  0x9485: mov_d_addr=+V4_V7(temp2),MOV_TO_K_H(Type_F,(u16)trade,PLC_D_C_T_addr(temp2)/0x100),PLC_Addr++;break;//K2M0??V4-V7		
		/*****************************************************K3M0 register" V"*********************************************************************/
		case  0x9486: mov_d_addr=+V0_V3(temp2),MOV_TO_K_H(Type_F,(u16)trade,PLC_D_C_T_addr(temp2)/0x100),PLC_Addr++;break;//K3M0??V0-V3	
		case  0x9487: mov_d_addr=+V4_V7(temp2),MOV_TO_K_H(Type_F,(u16)trade,PLC_D_C_T_addr(temp2)/0x100),PLC_Addr++;break;//K3M0??V4-V7		
		/*****************************************************K4M0 Register" V"*********************************************************************/
		case  0x9488: mov_d_addr=+V0_V3(temp2),MOV_TO_K_H(Type_F,(u16)trade,PLC_D_C_T_addr(temp2)/0x100),PLC_Addr++;break;//K4M0??V0-V3	
		case  0x9489: mov_d_addr=+V4_V7(temp2),MOV_TO_K_H(Type_F,(u16)trade,PLC_D_C_T_addr(temp2)/0x100),PLC_Addr++;break;//K4M0??V4-V7						
		/****************************************************T register "V" *****************************************************************/
		case  0x9682: MEMORY_16BIT(RAM_T_ADDR+PLC_D_C_T_addr(temp2)+V0_V3(temp2)*2)=(u16)trade,PLC_Addr++;break;//???T????????V0-V3
		case  0x9683: MEMORY_16BIT(RAM_T_ADDR+PLC_D_C_T_addr(temp2)+V4_V7(temp2)*2)=(u16)trade,PLC_Addr++;break;//???T????????V4-V7
		/****************************************************C register "V" *****************************************************************/
		case  0x9684: MEMORY_16BIT(RAM_C_ADDR+PLC_D_C_T_addr(temp2)+V0_V3(temp2)*2)=(u16)trade,PLC_Addr++;break;//???C????????V0-V3
		case  0x9685: MEMORY_16BIT(RAM_C_ADDR+PLC_D_C_T_addr(temp2)+V4_V7(temp2)*2)=(u16)trade,PLC_Addr++;break;//???C????????V4-V7
		/****************************************************D register "V" *****************************************************************/
		case  0x9686: MEMORY_16BIT(RAM_D_ADDR+PLC_D_C_T_addr(temp2)+V0_V3(temp2)*2)=(u16)trade,PLC_Addr++;break;//???D????????V0-V3
		case  0x9687: MEMORY_16BIT(RAM_D_ADDR+PLC_D_C_T_addr(temp2)+V4_V7(temp2)*2)=(u16)trade,PLC_Addr++;break;//???D????????V4-V7
		case  0x9688: MEMORY_16BIT(RAM_D1000_ADDR+PLC_D_C_T_addr(temp2)+V0_V3(temp2)*2)=(u16)trade,PLC_Addr++;break;//???????D1000??V0-V3
		case  0x9689: MEMORY_16BIT(RAM_D1000_ADDR+PLC_D_C_T_addr(temp2)+V4_V7(temp2)*2)=(u16)trade,PLC_Addr++;break;//???????D1000??V4-V7
		
		/****************************************************k1 m0 register "z" ******************************************************************/
		case  0xA482: mov_d_addr=+Z0_Z3(temp2),MOV_TO_K_H(Type_F,(u16)trade,PLC_D_C_T_addr(temp2)/0x100),PLC_Addr++;break;//K1M0??V0-V3	
		case  0xA483: mov_d_addr=+Z4_Z7(temp2),MOV_TO_K_H(Type_F,(u16)trade,PLC_D_C_T_addr(temp2)/0x100),PLC_Addr++;break;//K1M0??V4-V7	
		/****************************************************K2M0 Register "Z" ******************************************************************/
		case  0xA484: mov_d_addr=+Z0_Z3(temp2),MOV_TO_K_H(Type_F,(u16)trade,PLC_D_C_T_addr(temp2)/0x100),PLC_Addr++;break;//K2M0??V0-V3	
		case  0xA485: mov_d_addr=+Z4_Z7(temp2),MOV_TO_K_H(Type_F,(u16)trade,PLC_D_C_T_addr(temp2)/0x100),PLC_Addr++;break;//K2M0??V4-V7		
		/****************************************************K3M0 Register "Z" ******************************************************************/
		case  0xA486: mov_d_addr=+Z0_Z3(temp2),MOV_TO_K_H(Type_F,(u16)trade,PLC_D_C_T_addr(temp2)/0x100),PLC_Addr++;break;//K3M0??V0-V3	
		case  0xA487: mov_d_addr=+Z4_Z7(temp2),MOV_TO_K_H(Type_F,(u16)trade,PLC_D_C_T_addr(temp2)/0x100),PLC_Addr++;break;//K3M0??V4-V7		
		/****************************************************K4M0 Register "Z" ******************************************************************/
		case  0xA488: mov_d_addr=+Z0_Z3(temp2),MOV_TO_K_H(Type_F,(u16)trade,PLC_D_C_T_addr(temp2)/0x100),PLC_Addr++;break;//K4M0??V0-V3
		case  0xA489: mov_d_addr=+Z4_Z7(temp2),MOV_TO_K_H(Type_F,(u16)trade,PLC_D_C_T_addr(temp2)/0x100),PLC_Addr++;break;//K4M0??V4-V7			
		/****************************************************T Register "Z" ******************************************************************/
		case  0xA682: MEMORY_16BIT(RAM_T_ADDR+PLC_D_C_T_addr(temp2)+Z0_Z3(temp2)*2)=(u16)trade,PLC_Addr++;break;//???C????????Z0-Z3
		case  0xA683: MEMORY_16BIT(RAM_T_ADDR+PLC_D_C_T_addr(temp2)+Z4_Z7(temp2)*2)=(u16)trade,PLC_Addr++;break;//???C????????Z4-Z7
		/****************************************************C Register "Z" ******************************************************************/
		case  0xA684: MEMORY_16BIT(RAM_C_ADDR+PLC_D_C_T_addr(temp2)+Z0_Z3(temp2)*2)=(u16)trade,PLC_Addr++;break;//???C????????Z0-Z3
		case  0xA685: MEMORY_16BIT(RAM_C_ADDR+PLC_D_C_T_addr(temp2)+Z4_Z7(temp2)*2)=(u16)trade,PLC_Addr++;break;//???C????????Z4-Z7
		/****************************************************D Register "Z" ******************************************************************/
		case  0xA686: MEMORY_16BIT(RAM_D_ADDR+PLC_D_C_T_addr(temp2)+Z0_Z3(temp2)*2)=(u16)trade,PLC_Addr++;break;//???D????????Z0-Z3
		case  0xA687: MEMORY_16BIT(RAM_D_ADDR+PLC_D_C_T_addr(temp2)+Z4_Z7(temp2)*2)=(u16)trade,PLC_Addr++;break;//???D????????Z4-Z7
		case  0xA688: MEMORY_16BIT(RAM_D1000_ADDR+PLC_D_C_T_addr(temp2)+Z0_Z3(temp2)*2)=(u16)trade,PLC_Addr++;break;//???????D1000??Z0-Z3
		case  0xA689: MEMORY_16BIT(RAM_D1000_ADDR+PLC_D_C_T_addr(temp2)+Z4_Z7(temp2)*2)=(u16)trade,PLC_Addr++;break;//???????D1000??Z4-Z7
    }
		//Serial.printf("Result: %X\n",MEMORY_16BIT(RAM_D_ADDR+temp2+Transfer));
}

//===================================================== ===================================================== ===
//Function name: static u16 cos_value(void)
//Function description:
//Input: void
//Output: output 16-bit data
//global variable:
//Calling the module:
//Author: Xiao Xiaosheng
//Date: June 19, 2015
//Remark:
//-------------------------------------------------------------------------------------------------------
//Modified by:
//date:
//Remark:
//-------------------------------------------------------------------------------------------------------
//=======================================================================================================
static s16 cos_value()                                      
{  
  static s16 temp; 
	 u16 Type_F,temp2,Data1,Data2;
	 Data1=*PLC_Addr;PLC_Addr++;Data2=*PLC_Addr;
	 temp2=Type_F=0;
	 Type_F   = (Data1 & 0xff00);
	 Type_F  |= (Data2 >> 8);
/************************************/	
	 temp2  = (Data2 << 8);
	 temp2 |=mov_d_addr=(u8)Data1;
/************************************/	
	//Serial.printf("Type_F-cos: %X\n",Type_F);
	//Serial.printf("Temp2: %X\n",temp2);
	//Serial.printf("Transfer: %X\n",Transfer);
	switch(Type_F)
	{
		case  0x8080: temp=temp2,                                      PLC_Addr++;break;  //Calculate the value of K
		case  0x8280: temp=temp2,                                      PLC_Addr++;break;  //Calculate the H value
		case  0x8482: temp=MOV_K(*PLC_Addr)&0X0000000F,Transfer_bit1=1,PLC_Addr++;break;  //Do a transfer like K4M0   
		case  0x8484: temp=MOV_K(*PLC_Addr)&0X000000FF,Transfer_bit1=1,PLC_Addr++;break;  //Do a transfer like K4M0    
		case  0x8486: temp=MOV_K(*PLC_Addr)&0X00000FFF,Transfer_bit1=1,PLC_Addr++;break;  //Do a transfer like K4M0   
		case  0x8488: temp=MOV_K(*PLC_Addr)&0X0000FFFF,Transfer_bit1=1,PLC_Addr++;break;  //Do a transfer like K4M0    	
		case  0x8680: temp=MEMORY_16BIT(RAM_D8000_ADDR+temp2+Transfer),PLC_Addr++;break;  //Calculate the address of D D8000
		case  0x8682: temp=MEMORY_16BIT(RAM_T_ADDR+temp2+Transfer),PLC_Addr++;break;      //Calculate the address of T
		case  0x8684: temp=MEMORY_16BIT(RAM_C_ADDR+temp2+Transfer),PLC_Addr++;break;      //Calculate the address of C
		//case  0x8684: temp=MEMORY_16BIT(0x500+temp2+Transfer),PLC_Addr++;break;      		//Calculate the address of C -- แก้ไขภายหลังนะ
		case  0x8686: temp=MEMORY_16BIT(RAM_D_ADDR+temp2+Transfer),PLC_Addr++;break;      //Calculate the adress of D
		case  0x8688: temp=MEMORY_16BIT(RAM_D1000_ADDR+temp2+Transfer),PLC_Addr++;break;  //greater than or equal to D1000
		 	/************************************************ K register "V" *******************************************************************/
		case  0x9080: temp=temp2+D8029,PLC_Addr++;break;//???K????????V0
		case  0x9081: temp=temp2+D8183,PLC_Addr++;break;//???K????????V1
		case  0x9082: temp=temp2+D8185,PLC_Addr++;break;//???K????????V2
		case  0x9083: temp=temp2+D8187,PLC_Addr++;break;//???K????????V3
		case  0x9084: temp=temp2+D8189,PLC_Addr++;break;//???K????????V4
		case  0x9085: temp=temp2+D8191,PLC_Addr++;break;//???K????????V5
		case  0x9086: temp=temp2+D8193,PLC_Addr++;break;//???K????????V6
		case  0x9087: temp=temp2+D8195,PLC_Addr++;break;//???K????????V7
		case  0x9280: temp=temp2+D8029,PLC_Addr++;break;//???H????????V0
		case  0x9281: temp=temp2+D8183,PLC_Addr++;break;//???H????????V1
		case  0x9282: temp=temp2+D8185,PLC_Addr++;break;//???H????????V2
		case  0x9283: temp=temp2+D8187,PLC_Addr++;break;//???H????????V3
		case  0x9284: temp=temp2+D8189,PLC_Addr++;break;//???H????????V4
		case  0x9285: temp=temp2+D8191,PLC_Addr++;break;//???H????????V5
		case  0x9286: temp=temp2+D8193,PLC_Addr++;break;//???H????????V6
		case  0x9287: temp=temp2+D8195,PLC_Addr++;break;//???H????????V7 	
		/************************************************K1M0?????"V"*******************************************************************/
		case  0x9482: mov_d_addr=+V0_V3(temp2),temp=MOV_K(PLC_D_C_T_addr(temp2)/0x100)&0X0000000F,PLC_Addr++;break;//K1M0??V0-V3	
		case  0x9483: mov_d_addr=+V4_V7(temp2),temp=MOV_K(PLC_D_C_T_addr(temp2)/0x100)&0X0000000F,PLC_Addr++;break;//K1M0??V4-V7	
		/************************************************K2M0?????"V"*******************************************************************/
		case  0x9484: mov_d_addr=+V0_V3(temp2),temp=MOV_K(PLC_D_C_T_addr(temp2)/0x100)&0X000000FF,PLC_Addr++;break;//K2M0??V0-V3	
		case  0x9485: mov_d_addr=+V4_V7(temp2),temp=MOV_K(PLC_D_C_T_addr(temp2)/0x100)&0X000000FF,PLC_Addr++;break;//K2M0??V4-V7		
		/************************************************K3M0?????"V"*******************************************************************/
		case  0x9486: mov_d_addr=+V0_V3(temp2),temp=MOV_K(PLC_D_C_T_addr(temp2)/0x100)&0X00000FFF,PLC_Addr++;break;//K3M0??V0-V3	
		case  0x9487: mov_d_addr=+V4_V7(temp2),temp=MOV_K(PLC_D_C_T_addr(temp2)/0x100)&0X00000FFF,PLC_Addr++;break;//K3M0??V4-V7		
		/************************************************K4M0?????"V"*******************************************************************/
		case  0x9488: mov_d_addr=+V0_V3(temp2),temp=MOV_K(PLC_D_C_T_addr(temp2)/0x100)&0X0000FFFF,PLC_Addr++;break;//K4M0??V0-V3	
		case  0x9489: mov_d_addr=+V4_V7(temp2),temp=MOV_K(PLC_D_C_T_addr(temp2)/0x100)&0X0000FFFF,PLC_Addr++;break;//K4M0??V4-V7						
		/************************************************T?????"V"*******************************************************************/
		case  0x9682: temp=MEMORY_16BIT(RAM_T_ADDR+PLC_D_C_T_addr(temp2)+V0_V3(temp2)*2),PLC_Addr++;break;//???T????????V0-V3
		case  0x9683: temp=MEMORY_16BIT(RAM_T_ADDR+PLC_D_C_T_addr(temp2)+V4_V7(temp2)*2),PLC_Addr++;break;//???T????????V4-V7
		/************************************************C?????"V"*******************************************************************/
		case  0x9684: temp=MEMORY_16BIT(RAM_C_ADDR+PLC_D_C_T_addr(temp2)+V0_V3(temp2)*2),PLC_Addr++;break;//???C????????V0-V3
		case  0x9685: temp=MEMORY_16BIT(RAM_C_ADDR+PLC_D_C_T_addr(temp2)+V4_V7(temp2)*2),PLC_Addr++;break;//???C????????V4-V7
		/************************************************D?????"V"*******************************************************************/
		case  0x9686: temp=MEMORY_16BIT(RAM_D_ADDR+PLC_D_C_T_addr(temp2)+V0_V3(temp2)*2),PLC_Addr++;break;//???D????????V0-V3
		case  0x9687: temp=MEMORY_16BIT(RAM_D_ADDR+PLC_D_C_T_addr(temp2)+V4_V7(temp2)*2),PLC_Addr++;break;//???D????????V4-V7
		case  0x9688: temp=MEMORY_16BIT(RAM_D1000_ADDR+PLC_D_C_T_addr(temp2)+V0_V3(temp2)*2),PLC_Addr++;break;//???????D1000??V0-V3
		case  0x9689: temp=MEMORY_16BIT(RAM_D1000_ADDR+PLC_D_C_T_addr(temp2)+V4_V7(temp2)*2),PLC_Addr++;break;//???????D1000??V4-V7
	
		case  0xA080: temp=temp2+D8028,PLC_Addr++;break;//???K????????Z0
		case  0xA081: temp=temp2+D8182,PLC_Addr++;break;//???K????????Z1
		case  0xA082: temp=temp2+D8184,PLC_Addr++;break;//???K????????Z2
		case  0xA083: temp=temp2+D8186,PLC_Addr++;break;//???K????????Z3
		case  0xA084: temp=temp2+D8188,PLC_Addr++;break;//???K????????Z4
		case  0xA085: temp=temp2+D8190,PLC_Addr++;break;//???K????????Z5
		case  0xA086: temp=temp2+D8192,PLC_Addr++;break;//???K????????Z6
		case  0xA087: temp=temp2+D8194,PLC_Addr++;break;//???K????????Z7 	
		case  0xA280: temp=temp2+D8028,PLC_Addr++;break;//???H????????Z0
		case  0xA281: temp=temp2+D8182,PLC_Addr++;break;//???H????????Z1
		case  0xA282: temp=temp2+D8184,PLC_Addr++;break;//???H????????Z2
		case  0xA283: temp=temp2+D8186,PLC_Addr++;break;//???H????????Z3
		case  0xA284: temp=temp2+D8188,PLC_Addr++;break;//???H????????Z4
		case  0xA285: temp=temp2+D8190,PLC_Addr++;break;//???H????????Z5
		case  0xA286: temp=temp2+D8192,PLC_Addr++;break;//???H????????Z6
		case  0xA287: temp=temp2+D8194,PLC_Addr++;break;//???H????????Z7 	
		/************************************************K1M0?????"Z"*******************************************************************/
		case  0xA482: mov_d_addr=+Z0_Z3(temp2),temp=MOV_K(PLC_D_C_T_addr(temp2)/0x100)&0X0000000F,PLC_Addr++;break;//K1M0??V0-V3	
		case  0xA483: mov_d_addr=+Z4_Z7(temp2),temp=MOV_K(PLC_D_C_T_addr(temp2)/0x100)&0X0000000F,PLC_Addr++;break;//K1M0??V4-V7	
		/************************************************K2M0?????"Z"*******************************************************************/
		case  0xA484: mov_d_addr=+Z0_Z3(temp2),temp=MOV_K(PLC_D_C_T_addr(temp2)/0x100)&0X000000FF,PLC_Addr++;break;//K2M0??V0-V3	
		case  0xA485: mov_d_addr=+Z4_Z7(temp2),temp=MOV_K(PLC_D_C_T_addr(temp2)/0x100)&0X000000FF,PLC_Addr++;break;//K2M0??V4-V7		
		/************************************************K3M0?????"Z"*******************************************************************/
		case  0xA486: mov_d_addr=+Z0_Z3(temp2),temp=MOV_K(PLC_D_C_T_addr(temp2)/0x100)&0X00000FFF,PLC_Addr++;break;//K3M0??V0-V3	
		case  0xA487: mov_d_addr=+Z4_Z7(temp2),temp=MOV_K(PLC_D_C_T_addr(temp2)/0x100)&0X00000FFF,PLC_Addr++;break;//K3M0??V4-V7		
		/************************************************K4M0?????"Z"*******************************************************************/
		case  0xA488: mov_d_addr=+Z0_Z3(temp2),temp=MOV_K(PLC_D_C_T_addr(temp2)/0x100)&0X0000FFFF,PLC_Addr++;break;//K4M0??V0-V3
		case  0xA489: mov_d_addr=+Z4_Z7(temp2),temp=MOV_K(PLC_D_C_T_addr(temp2)/0x100)&0X0000FFFF,PLC_Addr++;break;//K4M0??V4-V7		
		case  0xA682: temp=MEMORY_16BIT(RAM_T_ADDR+PLC_D_C_T_addr(temp2)+Z0_Z3(temp2)*2),PLC_Addr++;break;//???T????????Z0-Z3
		case  0xA683: temp=MEMORY_16BIT(RAM_T_ADDR+PLC_D_C_T_addr(temp2)+Z4_Z7(temp2)*2),PLC_Addr++;break;//???T????????Z4-Z7
		/************************************************C?????"Z"*******************************************************************/
		case  0xA684: temp=MEMORY_16BIT(RAM_C_ADDR+PLC_D_C_T_addr(temp2)+Z0_Z3(temp2)*2),PLC_Addr++;break;//???C????????Z0-Z3
		case  0xA685: temp=MEMORY_16BIT(RAM_C_ADDR+PLC_D_C_T_addr(temp2)+Z4_Z7(temp2)*2),PLC_Addr++;break;//???C????????Z4-Z7
		/************************************************D?????"Z"*******************************************************************/
		case  0xA686: temp=MEMORY_16BIT(RAM_D_ADDR+PLC_D_C_T_addr(temp2)+Z0_Z3(temp2)*2),PLC_Addr++;break;//???D????????Z0-Z3
		case  0xA687: temp=MEMORY_16BIT(RAM_D_ADDR+PLC_D_C_T_addr(temp2)+Z4_Z7(temp2)*2),PLC_Addr++;break;//???D????????Z4-Z7
		case  0xA688: temp=MEMORY_16BIT(RAM_D1000_ADDR+PLC_D_C_T_addr(temp2)+Z0_Z3(temp2)*2),PLC_Addr++;break;//???????D1000??Z0-Z3
		case  0xA689: temp=MEMORY_16BIT(RAM_D1000_ADDR+PLC_D_C_T_addr(temp2)+Z4_Z7(temp2)*2),PLC_Addr++;break;//???????D1000??Z4-Z7
	}	
	//Serial.printf("Temp value: %X\n",temp);
	return temp;
}

//===================================================== ===================================================== ===
//Function name: static u32 cos_u32_value(void)
//Function description:
//Input: void
//Output: output 32-bit data
//global variable:
//Calling the module:
//Author: Xiao Xiaosheng
//Date: June 19, 2015
//Remark:
//-------------------------------------------------------------------------------------------------------
//Modified by:
//date:
//Remark:
//-------------------------------------------------------------------------------------------------------
//=======================================================================================================
#define  D_data  u32data.data 
static s32 cos_u32_value(void)                                      
{   
	 u16 Type_F,temp2,Data1,Data2;
	 unsigned short temp;
	 Data1=*PLC_Addr;
	 PLC_Addr++;
	 Data2=*PLC_Addr;
	 temp2=Type_F=0;
	 Type_F   = (Data1 & 0xff00);
	 Type_F  |= (Data2 >> 8);
/************************************/	
	 temp2  = (Data2 << 8);
	 temp2 |=mov_d_addr=(u8)Data1;
/************************************/
	//Serial.printf("Type_F-cos: %X\n",Type_F);
	//Serial.printf("Temp2: %X\n",temp2);
	//Serial.printf("Transfer: %X\n",Transfer);
	switch(Type_F)
	{		
		case  0x8080: u32data.data1[0]=temp2,PLC_Addr++,u32data.data1[1]=cos_value(),PLC_Addr-=2;break;//???K?
		case  0x8280: u32data.data1[0]=temp2,PLC_Addr++,u32data.data1[1]=cos_value(),PLC_Addr-=2;break;//???H?
		case  0x8482: D_data=MOV_K(*PLC_Addr)&0X0000000F,Transfer_bit1=1,PLC_Addr++;break;//?????? K4M0 ????????    
		case  0x8484: D_data=MOV_K(*PLC_Addr)&0X000000FF,Transfer_bit1=1,PLC_Addr++;break;//?????? K4M0 ????????    
		case  0x8486: D_data=MOV_K(*PLC_Addr)&0X00000FFF,Transfer_bit1=1,PLC_Addr++;break;//?????? K4M0 ????????    
		case  0x8488: D_data=MOV_K(*PLC_Addr)&0X0000FFFF,Transfer_bit1=1,PLC_Addr++;break;//?????? K4M0 ????????    
		case  0x848A: D_data=MOV_K(*PLC_Addr)&0X000FFFFF,Transfer_bit1=1,PLC_Addr++;break;//?????? K4M0 ????????    
		case  0x848C: D_data=MOV_K(*PLC_Addr)&0X00FFFFFF,Transfer_bit1=1,PLC_Addr++;break;//?????? K4M0 ????????    
		case  0x848E: D_data=MOV_K(*PLC_Addr)&0X0FFFFFFF,Transfer_bit1=1,PLC_Addr++;break;//?????? K4M0 ????????    
		case  0x8490: D_data=MOV_K(*PLC_Addr),           Transfer_bit1=1,PLC_Addr++;break;//?????? K4M0 ????????   
			
		case  0x8680: D_data=MEMORY_32BIT(RAM_D8000_ADDR+temp2),PLC_Addr++;break;                 //???D???? D8000
		case  0x8682: D_data=MEMORY_32BIT(RAM_T_ADDR+temp2),PLC_Addr++;break;                 //???T???? 
		case  0x8684: D_data=MEMORY_32BIT(RAM_C_ADDR+temp2),PLC_Addr++;break;                 //???C????
		case  0x8686: D_data=MEMORY_32BIT(RAM_D_ADDR+temp2),PLC_Addr++;break;                 //???D???? 
		case  0x8688: D_data=MEMORY_32BIT(RAM_D1000_ADDR+temp2),PLC_Addr++;break;                 //???????D1000
		case  0xA080: u32data.data1[0]=temp2+D8028,PLC_Addr++,u32data.data1[1]=cos_value()+D8029,PLC_Addr-=2;break;//???K????????Z0
		case  0xA081: u32data.data1[0]=temp2+D8182,PLC_Addr++,u32data.data1[1]=cos_value()+D8183,PLC_Addr-=2;break;//???K????????Z1
		case  0xA082: u32data.data1[0]=temp2+D8184,PLC_Addr++,u32data.data1[1]=cos_value()+D8185,PLC_Addr-=2;break;//???K????????Z2
		case  0xA083: u32data.data1[0]=temp2+D8186,PLC_Addr++,u32data.data1[1]=cos_value()+D8187,PLC_Addr-=2;break;//???K????????Z3
		case  0xA084: u32data.data1[0]=temp2+D8188,PLC_Addr++,u32data.data1[1]=cos_value()+D8189,PLC_Addr-=2;break;//???K????????Z4
		case  0xA085: u32data.data1[0]=temp2+D8190,PLC_Addr++,u32data.data1[1]=cos_value()+D8191,PLC_Addr-=2;break;//???K????????Z5
		case  0xA086: u32data.data1[0]=temp2+D8192,PLC_Addr++,u32data.data1[1]=cos_value()+D8193,PLC_Addr-=2;break;//???K????????Z6
		case  0xA087: u32data.data1[0]=temp2+D8194,PLC_Addr++,u32data.data1[1]=cos_value()+D8195,PLC_Addr-=2;break;//???K????????Z7 
		case  0xA280: u32data.data1[0]=temp2+D8028,PLC_Addr++,u32data.data1[1]=cos_value()+D8029,PLC_Addr-=2;break;//???H????????Z0
		case  0xA281: u32data.data1[0]=temp2+D8182,PLC_Addr++,u32data.data1[1]=cos_value()+D8183,PLC_Addr-=2;break;//???H????????Z1
		case  0xA282: u32data.data1[0]=temp2+D8184,PLC_Addr++,u32data.data1[1]=cos_value()+D8185,PLC_Addr-=2;break;//???H????????Z2
		case  0xA283: u32data.data1[0]=temp2+D8186,PLC_Addr++,u32data.data1[1]=cos_value()+D8187,PLC_Addr-=2;break;//???H????????Z3
		case  0xA284: u32data.data1[0]=temp2+D8188,PLC_Addr++,u32data.data1[1]=cos_value()+D8189,PLC_Addr-=2;break;//???H????????Z4
		case  0xA285: u32data.data1[0]=temp2+D8190,PLC_Addr++,u32data.data1[1]=cos_value()+D8191,PLC_Addr-=2;break;//???H????????Z5
		case  0xA286: u32data.data1[0]=temp2+D8192,PLC_Addr++,u32data.data1[1]=cos_value()+D8193,PLC_Addr-=2;break;//???H????????Z6
		case  0xA287: u32data.data1[0]=temp2+D8194,PLC_Addr++,u32data.data1[1]=cos_value()+D8195,PLC_Addr-=2;break;//???H????????Z7 				
		/************************************************K1M0?????"Z"*******************************************************************/
		case  0xA482: mov_d_addr=+DZ0_Z3(temp2),D_data=MOV_K(PLC_D_C_T_addr(temp2)/0x100)&0X0000000F,PLC_Addr++;break;//K1M0??V0-V3	
		case  0xA483: mov_d_addr=+DZ4_Z7(temp2),D_data=MOV_K(PLC_D_C_T_addr(temp2)/0x100)&0X0000000F,PLC_Addr++;break;//K1M0??V4-V7	
		/************************************************K2M0?????"Z"*******************************************************************/
		case  0xA484: mov_d_addr=+DZ0_Z3(temp2),D_data=MOV_K(PLC_D_C_T_addr(temp2)/0x100)&0X000000FF,PLC_Addr++;break;//K2M0??V0-V3	
		case  0xA485: mov_d_addr=+DZ4_Z7(temp2),D_data=MOV_K(PLC_D_C_T_addr(temp2)/0x100)&0X000000FF,PLC_Addr++;break;//K2M0??V4-V7		
		/************************************************K3M0?????"Z"*******************************************************************/
		case  0xA486: mov_d_addr=+DZ0_Z3(temp2),D_data=MOV_K(PLC_D_C_T_addr(temp2)/0x100)&0X00000FFF,PLC_Addr++;break;//K3M0??V0-V3	
		case  0xA487: mov_d_addr=+DZ4_Z7(temp2),D_data=MOV_K(PLC_D_C_T_addr(temp2)/0x100)&0X00000FFF,PLC_Addr++;break;//K3M0??V4-V7		
		/************************************************K4M0?????"Z"*******************************************************************/
		case  0xA488: mov_d_addr=+DZ0_Z3(temp2),D_data=MOV_K(PLC_D_C_T_addr(temp2)/0x100)&0X0000FFFF,PLC_Addr++;break;//K4M0??V0-V3
		case  0xA489: mov_d_addr=+DZ4_Z7(temp2),D_data=MOV_K(PLC_D_C_T_addr(temp2)/0x100)&0X0000FFFF,PLC_Addr++;break;//K4M0??V4-V7		
		/************************************************K5M0?????"Z"*******************************************************************/
		case  0xA48A: mov_d_addr=+DZ0_Z3(temp2),D_data=MOV_K(PLC_D_C_T_addr(temp2)/0x100)&0X000FFFFF,PLC_Addr++;break;//K1M0??V0-V3	
		case  0xA48B: mov_d_addr=+DZ4_Z7(temp2),D_data=MOV_K(PLC_D_C_T_addr(temp2)/0x100)&0X000FFFFF,PLC_Addr++;break;//K1M0??V4-V7	
		/************************************************K6M0?????"Z"*******************************************************************/
		case  0xA48C: mov_d_addr=+DZ0_Z3(temp2),D_data=MOV_K(PLC_D_C_T_addr(temp2)/0x100)&0X00FFFFFF,PLC_Addr++;break;//K2M0??V0-V3	
		case  0xA48D: mov_d_addr=+DZ4_Z7(temp2),D_data=MOV_K(PLC_D_C_T_addr(temp2)/0x100)&0X00FFFFFF,PLC_Addr++;break;//K2M0??V4-V7		
		/************************************************K7M0?????"Z"*******************************************************************/
		case  0xA48E: mov_d_addr=+DZ0_Z3(temp2),D_data=MOV_K(PLC_D_C_T_addr(temp2)/0x100)&0X0FFFFFFF,PLC_Addr++;break;//K3M0??V0-V3	
		case  0xA48F: mov_d_addr=+DZ4_Z7(temp2),D_data=MOV_K(PLC_D_C_T_addr(temp2)/0x100)&0X0FFFFFFF,PLC_Addr++;break;//K3M0??V4-V7		
		/************************************************K8M0?????"Z"*******************************************************************/
		case  0xA490: mov_d_addr=+DZ0_Z3(temp2),D_data=MOV_K(PLC_D_C_T_addr(temp2)/0x100),PLC_Addr++;break;//K4M0??V0-V3
		case  0xA491: mov_d_addr=+DZ4_Z7(temp2),D_data=MOV_K(PLC_D_C_T_addr(temp2)/0x100),PLC_Addr++;break;//K4M0??V4-V7	
		/************************************************T?????"Z"*******************************************************************/
		case  0xA682: {temp=(PLC_D_C_T_addr(temp2)+DZ0_Z3(temp2)*2);if(temp>=510) PLC_PROG_ERROR(M8067,6706),D_data=0; else D_data=MEMORY_32BIT(RAM_T_ADDR+temp),PLC_Addr++;}break;//???T????????Z0-Z3
		case  0xA683: {temp=(PLC_D_C_T_addr(temp2)+DZ4_Z7(temp2)*2);if(temp>=510) PLC_PROG_ERROR(M8067,6706),D_data=0; else D_data=MEMORY_32BIT(RAM_T_ADDR+temp),PLC_Addr++;}break;//???T????????Z4-Z7
		/************************************************C?????"Z"*******************************************************************/
		case  0xA684: {temp=(PLC_D_C_T_addr(temp2)+DZ0_Z3(temp2)*2);if(temp>=510) PLC_PROG_ERROR(M8067,6706),D_data=0; else D_data=MEMORY_32BIT(RAM_C_ADDR+temp),PLC_Addr++;}break;//???C????????Z0-Z3
		case  0xA685: {temp=(PLC_D_C_T_addr(temp2)+DZ4_Z7(temp2)*2);if(temp>=510) PLC_PROG_ERROR(M8067,6706),D_data=0; else D_data=MEMORY_32BIT(RAM_C_ADDR+temp),PLC_Addr++;}break;//???C????????Z4-Z7
		/************************************************D?????"Z"*******************************************************************/
		case  0xA686: {temp=(PLC_D_C_T_addr(temp2)+DZ0_Z3(temp2)*2);if(temp>=15998) PLC_PROG_ERROR(M8067,6706),D_data=0; else D_data=MEMORY_32BIT(RAM_D_ADDR+temp);PLC_Addr++;}break;//???D????????Z0-Z3
		case  0xA687: {temp=(PLC_D_C_T_addr(temp2)+DZ4_Z7(temp2)*2);if(temp>=15998) PLC_PROG_ERROR(M8067,6706),D_data=0; else D_data=MEMORY_32BIT(RAM_D_ADDR+temp);PLC_Addr++;}break;//???D????????Z4-Z7
		case  0xA688: {temp=(PLC_D_C_T_addr(temp2)+DZ0_Z3(temp2)*2);if(temp>=13998) PLC_PROG_ERROR(M8067,6706),D_data=0; else D_data=MEMORY_32BIT(RAM_D1000_ADDR+temp),PLC_Addr++;}break;//???????D1000??Z0-Z3
		case  0xA689: {temp=(PLC_D_C_T_addr(temp2)+DZ4_Z7(temp2)*2);if(temp>=13998) PLC_PROG_ERROR(M8067,6706),D_data=0; else D_data=MEMORY_32BIT(RAM_D1000_ADDR+temp),PLC_Addr++;}break;//???????D1000??Z4-Z7			
	}
	  PLC_Addr+=2;
			return D_data;
}

//===================================================== ===================================================== ===
//Function name: static float float_value(void)
//Function description:
//Input: void
//output: output float data
//global variable:
//Calling the module:
//Author: Xiao Xiaosheng
//Date: June 19, 2015
//Remark:
//-------------------------------------------------------------------------------------------------------
//Modified by:
//date:
//Remark:
//-------------------------------------------------------------------------------------------------------
//=======================================================================================================
static float float_value(void)                                     
{  
	 u16 Type_F,temp1,temp2,Data1,Data2,Data3,Data4;
	 Data1=*PLC_Addr;
	 PLC_Addr++;
	 Data2=*PLC_Addr;
	 PLC_Addr++;
	 Data3=*PLC_Addr;
	 PLC_Addr++;
	 Data4=*PLC_Addr;
	
	 Type_F   = (Data1 & 0xff00);
	 Type_F  |= (Data2 >> 8);
/************************************/	
	 temp1  = (Data2 << 8);
	 temp1 |=(u8)Data1;
/************************************/
	 temp2  = (Data4 << 8);
	 temp2 |=(u8)Data3;
/************************************/	
	 if(Type_F == 0x8080)      u32data.data1[0]=temp1,u32data.data1[1]=temp2, FLOAT.DATA=(float)u32data.data, PLC_Addr++;//???K?
	 else if(Type_F == 0x8280) u32data.data1[0]=temp1,u32data.data1[1]=temp2, FLOAT.DATA=(float)u32data.data, PLC_Addr++;//???H?            
	 else if(Type_F == 0x8680) FLOAT.DATA=PLC_RAMfolta(RAM_D8000_ADDR+temp1),PLC_Addr++;             //???D???? D8000
	 else if(Type_F == 0x8682) FLOAT.DATA=PLC_RAMfolta(RAM_T_ADDR+temp1),PLC_Addr++;                 //???T???? 
	 else if(Type_F == 0x8684) FLOAT.DATA=PLC_RAMfolta(RAM_C_ADDR+temp1),PLC_Addr++;                 //???C????
	 else if(Type_F == 0x8686) FLOAT.DATA=PLC_RAMfolta(RAM_D_ADDR+temp1),PLC_Addr++;                 //???D???? 
     else if(Type_F == 0x8688) FLOAT.DATA=PLC_RAMfolta(RAM_D1000_ADDR+temp1),PLC_Addr++;             //???????D1000
	 return FLOAT.DATA;
}

static void RST_D(void)                                 
{  
	u8 temp,addr,l_value;
	if(PLC_ACC_BIT&0x01)
	{   
		l_value=*PLC_Addr;PLC_Addr++;addr=*PLC_Addr/0x100;
		if(addr==0x86)
			temp=l_value+((*PLC_Addr%0x100)*0x100),temp=0x0700+temp/2,RAM_16BIT(temp) = 0;
		else if(addr==0x88)
			temp=l_value+((*PLC_Addr%0x100)*0x100),temp=0x1000+temp/2,RAM_16BIT(temp) = 0;
		else if(addr==0x80)
			temp=l_value+((*PLC_Addr%0x100)*0x100),temp=0x1000+temp/2+1000,RAM_16BIT(temp) = 0;
		else {PLC_PROG_ERROR(M8065,6501);}
	}
	else PLC_Addr+=2;
}

//===================================================== ===================================================== ===
//Function name: static void target(void)
//Function description: Addition and subtraction "AND", "OR" and "XOR" share assignment functions such as the results of DEMOV, DEADD, DESUB and other instructions.
//Input: void
//output: void
//global variable:
//Calling the module:
//Author: Xiao Xiaosheng
//Date: June 23, 2015
//Remark:
//===================================================== ===================================================== ===
static void float_target(void)		
{ 
	u16 temp;
	temp=addr_value() ;            
	RAM_16BIT(temp) = FLOAT.DATA1[0];
	RAM_16BIT(temp+1) = FLOAT.DATA1[1];
	PLC_Addr+=2;
}

static void PID(void)
{
	s16 PVn;  // ?????
	s16 SV;   // ?څ???Desired value
	s16 Ts;   // ??????
	s32 Su;
	s16 KP; // P 
	s16 Ti; // I
	s16 KD; // D
	s16 TD; // ???????
	u32 Addr,Addr1; // ??????
	u32 csp;        // PID??????????
	if((PLC_ACC_BIT&0X01)==0X01)        //????????
	{
		SV=RAM_16BIT(addr_value_prog());   // ?څ?
		PVn=RAM_16BIT(addr_value_prog());  // ????????
		Addr=addr_value_prog();            // ?????????????
		Addr1= addr_value_prog();
		Ts=RAM_16BIT(Addr);                // ??????
		KP=RAM_16BIT(Addr+6);  // P
		Ti=RAM_16BIT(Addr+8);  // I
		KD=RAM_16BIT(Addr+10); // D
		TD=RAM_16BIT(Addr+12); // ???????
		csp=Addr+14;           // ?????????????
		 
		RAM_16BIT(csp+14)=KP*((RAM_16BIT(csp)-RAM_16BIT(csp+2))+((Ts/Ti)*RAM_16BIT(csp))+RAM_16BIT(csp+10)); 
		
		RAM_16BIT(csp) = RAM_16BIT(csp+4) - SV;         // ??????????    
		
		RAM_16BIT(csp+4)= RAM_16BIT(Addr+4)*RAM_16BIT(csp+6)+(1-RAM_16BIT(Addr+4))*PVn; 
		
		RAM_16BIT(csp+10)=(TD/(Ts+KD*TD))*(-2*RAM_16BIT(csp+6)+RAM_16BIT(csp+4)+RAM_16BIT(csp+8))+((KD*TD)/(Ts+KD*TD))*RAM_16BIT(csp+12);
		
		
		Su=RAM_16BIT(Addr1)+RAM_16BIT(csp+14);
		if(Su>32766)       RAM_16BIT(Addr1)=32767;
		else if(Su<-32767) RAM_16BIT(Addr1)=-32768;
		else RAM_16BIT(Addr1)= Su;
		RAM_16BIT(csp+12)=RAM_16BIT(csp+10);
		RAM_16BIT(csp+8)=RAM_16BIT(csp+6);
		RAM_16BIT(csp+6)=RAM_16BIT(csp+4);
		RAM_16BIT(csp+2)=RAM_16BIT(csp);
	}
}

static void MOV(void)	          //MOV
{
	if(PLC_ACC_BIT&0X01)         //Determine if the condition is satisfied
	{
		trade=cos_value(),target();  
		//Serial.printf("trade in MOV: %X\n",trade);
	}
	else
	{
		PLC_Addr+=4;		             //The condition is not satisfied to execute the skip program to reduce the cpu overhead?
	}
	
}

static void DMOV(void)	        //?????????????
{  	
   if(PLC_ACC_BIT&0X01)
   trade=cos_u32_value(),D_target(); 
   else	 
   PLC_Addr+=8;		              //??????????????????????��CPU????
}

static void DEMOV(void)	        //?????????????
{  	
   if(PLC_ACC_BIT&0X01)
   trade=float_value(),float_target(); 
   else	 
   PLC_Addr+=8;		              //??????????????????????��CPU????
}

//===================================================== ===================================================== ==
//Function name: static void ZRST(void)
//Function description: ZRST instruction function
//Input: void
//output: void
//global variable:
//Calling the module:
//Author: Xiao Xiaosheng
//Date: June 27, 2015
//Remark:
//===================================================== ===================================================== ===
static void ZRST(void)  
{ 
	u16 temp,temp1;
	if((PLC_ACC_BIT&0X01)==0X01)
	{
		temp=addr_value();
		temp1=addr_value();
		if(Flag_bit==0x00) 
		{
			for(;temp<=temp1;temp++)
			PLC_BIT_OFF(temp);
		} 
		else
		{		 
			for(;temp<=temp1;temp++)
			RAM_16BIT(temp) = 0;
		}
	}
	else PLC_Addr+=4;
}

static void MTR(void)
{
	u16 X,Y,M_Y,K_H,temp=0;
	u8 i,t;	 
	if(PLC_ACC_BIT&0X01)
	{
		X=addr_value();
		Y=addr_value();
		M_Y=addr_value();
		K_H=cos_value();
		for(i=0;i<K_H;i++)
		{
			temp=i*7;
			PLC_BIT_ON(Y+i);
			for(t=0;t<=7;t++)
			(PLC_BIT_TEST(X+t)) ? PLC_BIT_ON(M_Y+temp+t) : PLC_BIT_OFF(M_Y+temp+t);
		}
	}
	else PLC_Addr+=8;
}

static void REFF(void)
{
	 if(PLC_ACC_BIT&0X01)
	 X_DIY=cos_value();
	 else X_DIY=10,PLC_Addr+=2;
}

//===================================================== ===================================================== ==
//Function name: static void DSQR(void)
//Function description: 32-bit square root calculation DSQR
//Input: void
//output: void
//global variable:
//Calling the module:
//Author: Xiao Xiaosheng
//Date: June 27, 2015
//Remark:
//===================================================== ===================================================== ===
static void DSQR(void)
{
   if((PLC_ACC_BIT&0X01)==0X01)
   {
  		trade=(u32)sqrt((double)cos_u32_value());
	    target();	
   } 
}

static void HSCS(void)		//high speed count set
{ 
	s32 temp1,temp2;
	temp1=cos_u32_value();
	temp2=cos_u32_value();
	if((temp2==temp1)&&((PLC_ACC_BIT&0X01)==0X01)) 
	PLC_ACC_BIT|=1;
	else
	PLC_ACC_BIT&=~1;
}

//===================================================== ===================================================== ==
//Function name: static void SQR(void)
//Function description: 16-bit right shift RCR instruction
//Input: void
//output: void
//global variable:
//Calling the module:
//Author: Xiao Xiaosheng
//Date: June 27, 2015
//Remark:
//===================================================== ===================================================== ===
static void SQR(void)
{
   if((PLC_ACC_BIT&0X01)==0X01)
   {
  		trade=(u16)sqrt((double)cos_value());
	    target();	
   } 
}

//===================================================== ===================================================== ==
//Function name: static void DRCR(void)
//Function description: 32-bit right shift RCR instruction
//Input: void
//output: void
//global variable:
//Calling the module:
//Author: Xiao Xiaosheng
//Date: June 27, 2015
//Remark:
//===================================================== ===================================================== ===
static void DRCR(void)	                 
{  	
	u32 temp1,temp2;
	if((PLC_ACC_BIT&0X01)==0X01)
	{
		temp1=cos_u32_value(); 
		temp2=cos_u32_value(); 
		trade=temp1>>temp2;
		PLC_Addr-=8;
		D_target();	
		PLC_Addr+=4;
	}
}

//===================================================== ===================================================== ==
//Function name: static void RCR(void)
//Function description: 16-bit right shift RCR instruction
//Input: void
//output: void
//global variable:
//Calling the module:
//Author: Xiao Xiaosheng
//Date: June 27, 2015
//Remark:
//===================================================== ===================================================== ===
static void RCR(void)	                 
{  	
	u16 temp1,temp2;
	if((PLC_ACC_BIT&0X01)==0X01)
	{
		temp1=cos_value(); 
		temp2=cos_value(); 
		trade=temp1>>temp2;
		PLC_Addr-=4;
		target();	
		PLC_Addr+=2;
	}
	else PLC_Addr+=4;                       //??��???????4??????
}

//===================================================== ===================================================== ==
//Function name: static void DROL(void)
//Function description: 32-bit left shift RCL instruction
//Input: void
//output: void
//global variable:
//Calling the module:
//Author: Xiao Xiaosheng
//Date: June 27, 2015
//Remark:
//===================================================== ===================================================== ===
static void DRCL(void)	                 
{  	
	u32 temp1,temp2;
	if((PLC_ACC_BIT&0X01)==0X01)
	{
		temp1=cos_u32_value(); 
		temp2=cos_u32_value();  
		trade=temp1<<temp2;
		PLC_Addr-=8;       
		D_target();	
		PLC_Addr+=4;
	}
}

//===================================================== ===================================================== ==
//Function name: static void ROL(void)
//Function description: 16-bit left shift RCL instruction
//Input: void
//output: void
//global variable:
//Calling the module:
//Author: Xiao Xiaosheng
//Date: June 27, 2015
//Remark:
//===================================================== ===================================================== ===
static void RCL(void)	                 
{  	
	u16 temp1,temp2;
	if((PLC_ACC_BIT&0X01)==0X01)
	{
		temp1=cos_value(); 
		temp2=cos_value(); 
		trade=temp1<<temp2;
		PLC_Addr-=4;
		target();	
		PLC_Addr+=2;
	}
	else PLC_Addr+=4;                      //??��???????4??????
}

//===================================================== ===================================================== ==
//Function name: static void DROR(void)
//Function description: 32-bit loop right bit ROR instruction
//Input: void
//output: void
//global variable:
//Calling the module:
//Author: Xiao Xiaosheng
//Date: June 27, 2015
//Remark:
//===================================================== ===================================================== ===
static void DROR(void)	                 
{  	
	u32 temp1,temp2;
	if((PLC_ACC_BIT&0X01)==0X01)
	{
		temp1=cos_u32_value(); 
		temp2=cos_u32_value(); 
		trade=ROTATE_RIGHT(temp1,32,temp2);
		PLC_Addr-=8;
		D_target();	
		PLC_Addr+=4;
	}
}

//===================================================== ===================================================== ==
//Function name: static void ROR(void)
//Function description: 16-bit loop right bit ROR instruction
//Input: void
//output: void
//global variable:
//Calling the module:
//Author: Xiao Xiaosheng
//Date: June 27, 2015
//Remark:
//===================================================== ===================================================== ===
static void ROR(void)	                 
{  	
	u16 temp1,temp2;
	if((PLC_ACC_BIT&0X01)==0X01)
	{
		temp1=cos_value(); 
		temp2=cos_value(); 
		trade=ROTATE_RIGHT(temp1,16,temp2);
		PLC_Addr-=4;
		target();	
		PLC_Addr+=2;
	}
	else PLC_Addr+=4;                      //??��???????4??????
}

//===================================================== ===================================================== ==
//Function name: static void DROL(void)
//Function description: 32-bit rotate left shift ROL instruction
//Input: void
//output: void
//global variable:
//Calling the module:
//Author: Xiao Xiaosheng
//Date: June 27, 2015
//Remark:
//===================================================== ===================================================== ===
static void DROL(void)	                 
{  	
	u32 temp1,temp2;
	if((PLC_ACC_BIT&0X01)==0X01)
	{
		temp1=cos_u32_value(); 
		temp2=cos_u32_value();  
		trade=ROTATE_LEFT(temp1,32,temp2);
		PLC_Addr-=8;       
		D_target();	
		PLC_Addr+=4;
	}
}

//===================================================== ===================================================== ==
//Function name: static void ROL(void)
//Function description: 16-bit rotate left shift ROL instruction
//Input: void
//output: void
//global variable:
//Calling the module:
//Author: Xiao Xiaosheng
//Date: June 27, 2015
//Remark:
//===================================================== ===================================================== ===
static void ROL(void)	                 
{  	
	u16 temp1,temp2;
	if((PLC_ACC_BIT&0X01)==0X01)
	{
		temp1=cos_value(); 
		temp2=cos_value(); 
		trade=ROTATE_LEFT(temp1,16,temp2);
		PLC_Addr-=4;
		target();	
		PLC_Addr+=2;
	}
	else PLC_Addr+=4;                      //??��???????4??????
}

//===================================================== ===================================================== ==
//Function name: static void DSWAP(void)
//Function description: 32-bit up and down exchange DSWAP instruction
//Input: void
//output: void
//global variable:
//Calling the module:
//Author: Xiao Xiaosheng
//Date: June 27, 2015
//Remark:
//===================================================== ===================================================== ===
static void DSWAP(void)	                 
{  	
	u32 temp;
	if((PLC_ACC_BIT&0X01)==0X01)
	{
		temp=cos_u32_value(); 
		trade=swap_u32(temp);
		PLC_Addr-=4;
		D_target();	
	}
}

//===================================================== ===================================================== ===
//Function name: static void DGBIN(void)
//Function description: 16-bit up and down exchange DGBIN instruction
//Input: void
//output: void
//global variable:
//Calling the module:
//Author: Xiao Xiaosheng
//Date: August 4, 2015
//Remark:
//===================================================== ===================================================== ===
static void DGBIN(void)	                 
{  	
	signed int temp;
	if((PLC_ACC_BIT&0X01)==0X01)
	{
		temp=cos_value();		  
		trade=GtoB(temp);
		D_target();		 
	}
	else PLC_Addr+=8;                      //??��???????8??????
}

//===================================================== ===================================================== ===
//Function name: static void GBIN(void)
//Function description: 16-bit up and down exchange GBIN instruction
//Input: void
//output: void
//global variable:
//Calling the module:
//Author: Xiao Xiaosheng
//Date: June 27, 2015
//Remark:
//===================================================== ===================================================== ===
static void GBIN(void)	                 
{  	
	signed short int temp;
	if((PLC_ACC_BIT&0X01)==0X01)
	{
		temp=cos_value();		  
		trade=(u16)GtoB((unsigned int)temp);
		target();		 
	}
	else PLC_Addr+=4;                      //??��???????4??????
}

//===================================================== ===================================================== ===
//Function name: static void DGRY(void)
//Function description: 16-bit up and down exchange DGRY instruction
//Input: void
//output: void
//global variable:
//Calling the module:
//Author: Xiao Xiaosheng
//Date: August 4, 2015
//Remark:
//===================================================== ===================================================== ===
static void DGRY(void)	                 
{  	
	signed int temp;
	if((PLC_ACC_BIT&0X01)==0X01)
	{
		temp=cos_value();		  
		trade=BtoG(temp);
		D_target();		 
	}
	else PLC_Addr+=8;                      //??��???????8??????
}

//===================================================== ===================================================== ===
//Function name: static void GRY(void)
//Function description: 16-bit up and down exchange GRY instruction
//Input: void
//output: void
//global variable:
//Calling the module:
//Author: Xiao Xiaosheng
//Date: June 27, 2015
//Remark:
//===================================================== ===================================================== ===
static void GRY(void)	                 
{  	
	signed short int temp;
	if((PLC_ACC_BIT&0X01)==0X01)
	{
		temp=cos_value();		  
		trade=(u16)BtoG((unsigned int)temp);
		target();		 
	}
	else PLC_Addr+=4;                      //??��???????4??????
}

//===================================================== ===================================================== ===
//Function name: static void SWAP(void)
//Function description: 16-bit up and down exchange SWAP instruction
//Input: void
//output: void
//global variable:
//Calling the module:
//Author: Xiao Xiaosheng
//Date: June 27, 2015
//Remark:
//===================================================== ===================================================== ===
static void SWAP(void)	                 
{  	
	signed short int temp;
	if((PLC_ACC_BIT&0X01)==0X01)
	{
		temp=cos_value();	  
		trade=swap_u16(temp);
		PLC_Addr-=2;
		target();		 
	}
	else PLC_Addr+=4;                      //??��???????4??????
}

//=======================================================================================================
// ????????:  static void SFTR(void)	 
// ?????????? SFTR???
// ????:  void      
// ????:  void     
// ????????:  
// ???????: 
// ??????:  ����??
// ?????:  2015??8??9??
// ??  ?:  
//=======================================================================================================
//static void SFTR(void)	                 
//{  	
//signed short int addr1,addr2,temp1,temp2,temp5,temp6,i;
//	u8 temp3,temp4;
//   if(PLC_ACC_BIT&0X01)
//   {
//		  addr1=addr_value();
//		  Flag_bit=0xff;
//		 
//		  addr2=addr_value();
//		  Flag_bit=0xff;
//		 
//		  temp1=cos_value();
//		  temp5=cos_value();
//		  temp2=temp5+temp4;
//		  temp6=temp1-temp5;
//  		for(i=0;i<temp6;i++)
//		  { 
//				(PLC_BIT_TEST(addr2+i)) ? PLC_BIT_ON(addr2+i) : PLC_BIT_OFF(addr2+i);
//			}
//			for(;i<temp1;i++)
//		  { 
//				(PLC_BIT_TEST(addr1+i)) ? PLC_BIT_ON(addr2+i) : PLC_BIT_OFF(addr2+i);
//			}
//	 }
//	 else PLC_Addr+=8;                      //??��???????4??????
//}

//===================================================== ===================================================== ===
//Function name: static void XCH(void)
//Function description: 16-bit exchange transfer XCH command
//Input: void
//output: void
//global variable:
//Calling the module:
//Author: Xiao Xiaosheng
//Date: June 27, 2015
//Remark:
//===================================================== ===================================================== ===
static void XCH(void)	                 
{  	
	signed short int temp1,temp2;
	if((PLC_ACC_BIT&0X01)==0X01)
	{
		temp1=cos_value();
		temp2=cos_value();
		PLC_Addr-=4;
		trade=temp2;
		D_target();
		trade=temp1;
		D_target();
	}
	else PLC_Addr+=4;                      //??��???????4??????
}

//===================================================== ===================================================== ===
//Function name: static void DFMOV(void)
//Function description: 32-bit exchange transfer DXCH instruction
//Input: void
//output: void
//global variable:
//Calling the module:
//Author: Xiao Xiaosheng
//Date: June 27, 2015
//Remark:
//===================================================== ===================================================== ===
static void DXCH(void)	                 
{  	
	signed int temp1,temp2;
	if((PLC_ACC_BIT&0X01)==0X01)
	{
		temp1=cos_u32_value();
		temp2=cos_u32_value();
		PLC_Addr-=8;
		trade=temp2;
		D_target();
		trade=temp1;
		D_target();
	}
}

//===================================================== ===================================================== ===
//Function name: static void DFMOV(void)
//Function description: 32-bit multicast DFMOV instruction
//Input: void
//output: void
//global variable:
//Calling the module:
//Author: Xiao Xiaosheng
//Date: June 27, 2015
//Remark:
//===================================================== ===================================================== ===
static void DFMOV(void)	               
{  	
	signed short int temp,i;
	if((PLC_ACC_BIT&0X01)==0X01)
	{
		trade=cos_u32_value();            //??????????
		D_target();                       //?????????????
		temp=cos_u32_value();             //            <<<-------------|
		PLC_Addr-=4;                      //PLC_Addr-=4????????????? |<<-----|
		for(i=1;i<temp;i++)               //                                    |
		{                                 //                                    |
			if(Transfer_bit==1)Transfer=i*32;//                                  |
			else Transfer=i*4;             //                                    |
			PLC_Addr-=4;D_target();        //PLC_Addr-=4??????????????��???	  |
		}                                 //                                    |
		PLC_Addr+=2;Transfer=0;           //PLC_Addr+=2????????????????????---|
	}
}

//===================================================== ===================================================== ===
//Function name: static void FMOV(void)
//Function description: 16-bit multicast FMOV instruction
//Input: void
//output: void
//global variable:
//Calling the module:
//Author: Xiao Xiaosheng
//Date: June 27, 2015
//Remark:
//===================================================== ===================================================== ===
static void FMOV(void)	             
{  	
	signed short int temp,i;
	if((PLC_ACC_BIT&0X01)==0X01)
	{
		trade=cos_value();                //??????????
		target();                         //?????????????
		temp=cos_value();                 //            <<<-------------|
		PLC_Addr-=2;                      //PLC_Addr-=2????????????? |<<-----|
		for(i=1;i<temp;i++)               //                                    |
		{                                 //                                    |
			if(Transfer_bit==1)Transfer=i*16;//                                  |
			else Transfer=i*2;             //                                    |
			PLC_Addr-=2;target();          //PLC_Addr-=2??????????????��???	  |
		}                                 //                                    |
		PLC_Addr+=2;Transfer=0;           //PLC_Addr+=2????????????????????---|
	}
	else PLC_Addr+=6;                    //??��???????6??????
}

//===================================================== ===================================================== ===
//Function name: static void FMOV(void)
//Function description: 16-bit batch transfer BMOV instruction
//Input: void
//output: void
//global variable:
//Calling the module:
//Author: Xiao Xiaosheng
//Date: June 27, 2015
//Remark:
//===================================================== ===================================================== ===
static void BMOV(void)	                 
{  	
	signed short int temp,i;
	if((PLC_ACC_BIT&0X01)==0X01)
		{
		trade=cos_value();                //??????????
		target();                         //?????????????
		temp=cos_value();
		PLC_Addr-=2;                        //<<<---------------------------------|
		for(i=1;i<temp;i++)                 //                                    |
		{                                   //                                    |
			if(Transfer_bit1==1)Transfer=i*16;//                                    |
			else Transfer=i*2;               //                                    |
			PLC_Addr-=4;                     //                 
			trade=cos_value();               //??????????
			if(Transfer_bit==1)Transfer=i*16;
			else Transfer=i*2;
			target();                   //?????????
		}                                   //                                      |
		PLC_Addr+=2;Transfer=0;             //PLC_Addr+=2????????????????????-----|
	}
	else PLC_Addr+=6;                      //??????????????6??????
}

static void DCML(void)	                 //32��???????
{  	
	s32 temp1;
	if((PLC_ACC_BIT&0X01)==0X01)
	{
		temp1=cos_u32_value();
		trade=~temp1;
		D_target();
	}
	else PLC_Addr+=8;                     //????8??????
}

static void CML(void)	                    //???????
{  	
	signed short int temp1;
	if((PLC_ACC_BIT&0X01)==0X01)
	{
		temp1=cos_value();
		trade=~temp1;
		target();
	}
	else PLC_Addr+=4;                   //????4??????
}

u16 bcd[4]={0x1,0x10,0x100,0x1000};
static void SMOV(void)	          //16��?????????
{ 
	u16 temp1,temp2,temp3,temp4,temp5,temp6;
	if((PLC_ACC_BIT&0X01)==0X01)
	{
		temp1=cos_value();
		temp2=cos_value();
		temp3=cos_value();
		temp4=addr_value();
		temp5=cos_value();
		temp1%=bcd[temp2];
		for(temp6=0;temp6<temp3;temp6++)
		{ 
			temp2--;temp5--;
			RAM_16BIT(temp4) |= (temp1/bcd[temp2])*bcd[temp5];
			if((temp2==1)&&(temp5==1))RAM_16BIT(temp4) |= temp1%0x10*bcd[temp5], temp6=temp3+1;
			else temp1%=bcd[temp2];
		}
	}
	else PLC_Addr+=10;              //????10??????
}

//===================================================== ===================================================== ==
//Function name: static void TZCP(void)
//Function description: TZCP instruction function
//Input: void
//output: void
//global variable:
//Calling the module:
//Author: Xiao Xiaosheng
//Date: July 26, 2015
//Remark:
//===================================================== ===================================================== ===
static void TZCP(void)
{
	u16 h,min,s,temp,temp1,temp3,temp4,h1,min1,s1;
	if((PLC_ACC_BIT&0X01)==0X01)
	{	
		temp3=addr_value();               //S1 ????????????
		h = RAM_16BIT(temp3);
		min = RAM_16BIT(temp3+1);
		s=RAM_16BIT(temp3+2);
		
		temp4=addr_value();               //S2 ????????????
		h1=RAM_16BIT(temp4);
		min1=RAM_16BIT(temp4+1);
		s1=RAM_16BIT(temp4+2);
		
		temp=addr_value();                //S3 ???????
		
		temp1=addr_value();Flag_bit=0XFF;
		PLC_BIT_OFF(temp1);PLC_BIT_OFF(temp1+1);PLC_BIT_OFF(temp1+2);  
		if((h>=RAM_16BIT(temp))&&(min>=RAM_16BIT(temp+1))&&(s>RAM_16BIT(temp+2)))
		{PLC_BIT_ON(temp1);}
		else if(((h<=RAM_16BIT(temp))&&(min<=RAM_16BIT(temp+1))&&(s<=RAM_16BIT(temp+1)))&&((h1>=RAM_16BIT(temp))&&(min1>=RAM_16BIT(temp+1))&&(s1>=RAM_16BIT(temp+2))))
		{PLC_BIT_ON(temp1+1);}
		else if((h1<=RAM_16BIT(temp))&&(min1<=RAM_16BIT(temp+1))&&(s1<RAM_16BIT(temp+2)))
		{PLC_BIT_ON(temp1+2);}
	}
}

static void EZCP(void)	          //16��?????????
{ 
	float temp1,temp2,temp3;u32 temp4;
	if((PLC_ACC_BIT&0X01)==0X01)
	{
		temp1=float_value();
		temp2=float_value();
		temp3=float_value();
		temp4=addr_value();Flag_bit=0XFF;
		PLC_BIT_OFF(temp4);PLC_BIT_OFF(temp4+1);PLC_BIT_OFF(temp4+2);
		if(temp1>temp3)       PLC_BIT_ON(temp4+0); 
		else if((temp1<=temp3)&&(temp3<=temp2)) PLC_BIT_ON(temp4+1);
		else if(temp2<temp3)  PLC_BIT_ON(temp4+2); 
	}
	else PLC_Addr+=16;              //????16??????
}

static void DZCP(void)	          //16��?????????
{ 
	s32 temp1,temp2,temp3,temp4;
	if((PLC_ACC_BIT&0X01)==0X01)
	{
		temp1=cos_u32_value();
		temp2=cos_u32_value();
		temp3=cos_u32_value();
		temp4=addr_value();Flag_bit=0XFF;
		PLC_BIT_OFF(temp4);PLC_BIT_OFF(temp4+1);PLC_BIT_OFF(temp4+2);
		if(temp1>temp3)       PLC_BIT_ON(temp4); 
		else if((temp1<=temp3)&&(temp3<=temp2)) PLC_BIT_ON(temp4+1);
		else if(temp2<temp3)  PLC_BIT_ON(temp4+2); 
	}
	else PLC_Addr+=16;              //????16??????
}

static void ZCP(void)	            //16��?????????
{ 
	signed short int temp1,temp2,temp3,temp4;
	if((PLC_ACC_BIT&0X01)==0X01)
	{
		temp1=cos_value();
		temp2=cos_value();
		temp3=cos_value();
		temp4=addr_value();Flag_bit=0XFF;
		PLC_BIT_OFF(temp4);PLC_BIT_OFF(temp4+1);PLC_BIT_OFF(temp4+2);
		if(temp1>temp3)       PLC_BIT_ON(temp4); 
		else if((temp1<=temp3)&&(temp3<=temp2)) PLC_BIT_ON(temp4+1);
		else if(temp2<temp3)  PLC_BIT_ON(temp4+2); 
	}
	else PLC_Addr+=8;              //????8??????
}

//===================================================== ===================================================== ==
//Function name: static void TCMP(void)
//Function description: TCMP instruction function
//Input: void
//output: void
//global variable:
//Calling the module:
//Author: Xiao Xiaosheng
//Date: July 26, 2015
//Remark:
//===================================================== ===================================================== ===
static void TCMP(void)
{   
	u16 h,min,s,temp,temp1;
	if((PLC_ACC_BIT&0X01)==0X01)
	{	
		h=cos_value();min=cos_value();s=cos_value();
		temp=addr_value();
		temp1=addr_value();Flag_bit=0XFF;
		PLC_BIT_OFF(temp1);PLC_BIT_OFF(temp1+1);PLC_BIT_OFF(temp1+2);  
		if((h>=RAM_16BIT(temp))&&(min>=RAM_16BIT(temp+1))&&(s>RAM_16BIT(temp+2)))
		{PLC_BIT_ON(temp1);}
		else if((h==RAM_16BIT(temp))&&(min==RAM_16BIT(temp+1))&&(s==RAM_16BIT(temp+2)))
		{PLC_BIT_ON(temp1+1);}
		else if((h<=RAM_16BIT(temp))&&(min<=RAM_16BIT(temp+1))&&(s<RAM_16BIT(temp+2)))
		{PLC_BIT_ON(temp1+2);}
	}
	else PLC_Addr+=10;
}

//===================================================== ===================================================== ==
//Function name: static void ECMP(void)
//Function description: ECMP command function
//Input: void
//output: void
//global variable:
//Calling the module:
//Author: Xiao Xiaosheng
//Date: July 26, 2015
//Remark:
//===================================================== ===================================================== ===
static void ECMP(void)	          //?????????????
{ 
	signed short int temp3;
	static float temp1,temp2;
	if((PLC_ACC_BIT&0X01)==0X01)
	{	
		temp1=float_value();
		temp2=float_value();
		temp3=addr_value();Flag_bit=0XFF;
		PLC_BIT_OFF(temp3);PLC_BIT_OFF(temp3+1);PLC_BIT_OFF(temp3+2);
		if(temp1>temp2)       PLC_BIT_ON(temp3); 
		else if(temp1==temp2) PLC_BIT_ON(temp3+1);
		else if(temp1<temp2)  PLC_BIT_ON(temp3+2); 
	}
	else PLC_Addr+=12;              //????12??????
}

//===================================================== ===================================================== ==
//Function name: static void DCMP(void)
//Function description: DCMP instruction function
//Input: void
//output: void
//global variable:
//Calling the module:
//Author: Xiao Xiaosheng
//Date: July 26, 2015
//Remark:
//===================================================== ===================================================== ===
static void DCMP(void)	          //32��?????????
{ 
	signed short int temp3;
	static int temp1,temp2;
	if(PLC_ACC_BIT&0X01)
	{	
		temp1=cos_u32_value();
		temp2=cos_u32_value();
		temp3=addr_value();Flag_bit=0XFF;
		PLC_BIT_OFF(temp3);PLC_BIT_OFF(temp3+1);PLC_BIT_OFF(temp3+2);
		if(temp1>temp2)       PLC_BIT_ON(temp3+0); 
		else if(temp1==temp2) PLC_BIT_ON(temp3+1);
		else if(temp1<temp2)  PLC_BIT_ON(temp3+2); 
	}
	else PLC_Addr+=12;              //????12??????
}

static void DCMPP(void)	  
{ 
	signed short int temp3;
	static int temp1,temp2;
	if(PLC_LDP_TEST())        //???????��?
	{ 
		temp1=cos_u32_value();
		temp2=cos_u32_value();
		temp3=addr_value();Flag_bit=0XFF;
		PLC_BIT_OFF(temp3);PLC_BIT_OFF(temp3+1);PLC_BIT_OFF(temp3+2);
		if(temp1>temp2)       PLC_BIT_ON(temp3+0); 
		else if(temp1==temp2) PLC_BIT_ON(temp3+1);
		else if(temp1<temp2)  PLC_BIT_ON(temp3+2);  
	}
	else
	PLC_Addr+=12;		         //??????????????????????��CPU????
}

//===================================================== ===================================================== ==
//Function name: static void CMP(void)
//Function description: CMP instruction function
//Input: void
//output: void
//global variable:
//Calling the module:
//Author: Xiao Xiaosheng
//Date: July 26, 2015
//Remark:
//===================================================== ===================================================== ===
static void CMP(void)	          //16��?????????
{ 
	signed short int temp1,temp2,temp3;
	if((PLC_ACC_BIT&0X01)==0X01)
	{
		temp1=cos_value();
		temp2=cos_value();
		temp3=addr_value();Flag_bit=0XFF;
		PLC_BIT_OFF(temp3);PLC_BIT_OFF(temp3+1);PLC_BIT_OFF(temp3+2);
		if(temp1>temp2)       PLC_BIT_ON(temp3); 
		else if(temp1==temp2) PLC_BIT_ON(temp3+1);
		else if(temp1<temp2)  PLC_BIT_ON(temp3+2); 	 
	}
	else PLC_Addr+=6;              //????6??????
}

//===================================================== ===================================================== ==
//Function name: static void CMP_P(void)
//Function description: CMPP instruction function
//Input: void
//output: void
//global variable:
//Calling the module:
//Author: Xiao Xiaosheng
//Date: August 26, 2015
//Remark:
//===================================================== ===================================================== ===
static void CMPP(void)	  
{ 
	signed short int temp1,temp2,temp3;
	if(PLC_LDP_TEST())                       //???????��?
	{ 
		temp1=cos_value();
		temp2=cos_value();
		temp3=addr_value();Flag_bit=0XFF;
		PLC_BIT_OFF(temp3);PLC_BIT_OFF(temp3+1);PLC_BIT_OFF(temp3+2);
		if(temp1>temp2)       PLC_BIT_ON(temp3); 
		else if(temp1==temp2) PLC_BIT_ON(temp3+1);
		else if(temp1<temp2)  PLC_BIT_ON(temp3+2); 	 
	}
	else
	PLC_Addr+=6;		                          //??????????????????????��CPU????
}

static void DINC(void)	         //32��??????? ??1???
{ 
	if((PLC_ACC_BIT&0X01)==0X01)
	{
		trade=(u32)cos_u32_value()+1;
		PLC_Addr-=4;
		D_target();
	}
	else PLC_Addr+=4;              //????4??????
}

static void DINC_P(void)	  //CALLP
{ 
	if(PLC_PL_BIT_TEST(PLC_Addr-PLC_START_Address)==OFF)//???????��?
	{ 
		if(PLC_ACC_BIT&0X01)			                      //?????��?
		{
			PLC_PL_BIT_ON(PLC_Addr-PLC_START_Address);
			trade=(u32)cos_u32_value()+1;
			PLC_Addr-=4;
			trade++;
			D_target();
		}
		else PLC_Addr+=4;
	}
	else
	{
		if(!((PLC_ACC_BIT&0x01)==0x01))						 //?????��?
		PLC_PL_BIT_OFF(PLC_Addr-PLC_START_Address);//
		PLC_Addr+=4;		                           //??????????????????????��CPU????
	} 
}

static void INC(void)	            //??????? ??1???
{ 
	if((PLC_ACC_BIT&0X01)==0X01)
	{
		trade=cos_value();
		PLC_Addr-=2;
		trade++;
		target();
	}
	else PLC_Addr+=2;              //????2??????
}

static void INCP(void)	  //INCP
{ 
	if(PLC_LDP_TEST())//???????��?
	{ 
		trade=cos_value();
		PLC_Addr-=2;
		trade++;
		target();
	}
	else
	{
		PLC_Addr+=2;		                            //??????????????????????��CPU????
	} 
}

static void DDEC(void)                             //32��??????? ??1???
{
	if((PLC_ACC_BIT&0X01)==0X01)
	{
		trade=cos_u32_value()-1;
		PLC_Addr-=4;
		D_target();
	}
	else PLC_Addr+=4;              //????4??????
}

static void DECV(void)             //??????? ??1???
{
	if(PLC_ACC_BIT&0X01)
	{
		trade=cos_value()-1;
		PLC_Addr-=2;
		target();
	}
	else PLC_Addr+=2;              //????2??????
}

static void DECP(void)	          //INCP
{ 
	if(PLC_LDP_TEST())              //???????��?
	{ 
		trade=cos_value()-1;
		PLC_Addr-=2;
		target();
	}
	else
	PLC_Addr+=2;		                           //??????????????????????��CPU????
}

static void DNEG(void)                             //32��????????????
{
	if((PLC_ACC_BIT&0X01)==0X01)
	{
		trade=0-cos_u32_value();
		PLC_Addr-=4;
		D_target();		 
	}
	else PLC_Addr+=4;              //????4??????
}

static void NEG(void)                             //????????????
{
	if((PLC_ACC_BIT&0X01)==0X01)
	{
		trade=0-cos_value();
		PLC_Addr-=2;
		target();
	}
	else PLC_Addr+=2;              //????2??????
}

static void DWAND(void)	                          //?????????
{ 
	u32 temp1,temp2;
	if((PLC_ACC_BIT&0X01)==0X01)
	{
		temp1=cos_u32_value();
		temp2=cos_u32_value();;
		trade=temp1&temp2;
		D_target();
	}
	else PLC_Addr+=12;              //????12??????
}

static void WAND(void)	                          //?????????
{ 
	signed short int temp1,temp2;
	if((PLC_ACC_BIT&0X01)==0X01)
	{
		temp1=cos_value();
		temp2=cos_value();
		trade=temp1&temp2;
		target();
	}
	else PLC_Addr+=6;              //????6??????
}

static void DWOR(void)	                           //?????????
{  
	u32 temp1,temp2;
	if((PLC_ACC_BIT&0X01)==0X01)
	{
		temp1=cos_u32_value();
		temp2=cos_u32_value();;
		trade=temp1|temp2;
		D_target();
	}
	else PLC_Addr+=12;              //????12??????
}

static void WOR(void)	                           //?????????
{  
	signed short int temp1,temp2;
	if((PLC_ACC_BIT&0X01)==0X01)
	{
		temp1=cos_value();
		temp2=cos_value();
		trade=temp1|temp2;
		target();
	}
	else PLC_Addr+=6;              //????6??????
}

static void DWXOR(void)	                          //???????????
{ 
	u32 temp1,temp2;
	if((PLC_ACC_BIT&0X01)==0X01)
	{
		temp1=cos_u32_value();
		temp2=cos_u32_value();
		trade=temp1^temp2;
		D_target();
	}
	else PLC_Addr+=12;              //????12??????
}

static void WXOR(void)	                          //???????????
{ 
	if((PLC_ACC_BIT&0X01)==0X01)
	{
		trade=cos_value()^cos_value();
		target();
	}
	else PLC_Addr+=6;              //????6??????
}

//======================================================================================================
// ????????: static void TADD(void)
// ??????????TADD?????
// ????:  void      
// ????:  void     
// ????????:  
// ???????: 
// ??????:  ����??
// ?????:  2015??7??26??
// ??  ?:  
//=======================================================================================================
static void TADD(void)
{  
	u16 temp,temp1,temp2;
	if(PLC_ACC_BIT&0X01)
	{ 
		temp1=addr_value();              
		temp2=addr_value();  
		temp=addr_value();
		RAM_16BIT(temp) = RAM_16BIT(temp1) + RAM_16BIT(temp2);
		RAM_16BIT(temp+1) = RAM_16BIT(temp1+1) + RAM_16BIT(temp2+1);
		RAM_16BIT(temp+2) = RAM_16BIT(temp1+2) + RAM_16BIT(temp2+2);
	}
}

void MEAN(void)
{
	u16 temp,temp2;uint64_t data;u32 temp1;
	if(PLC_ACC_BIT&0X01)
	{ 
		temp1=addr_value_prog();             
		PLC_Addr+=2;        //��????????  
		temp2=cos_value();
		if(Flag_bit==0xff)  //?????K4M0?????????
		{
			for(temp=0;temp<temp2;temp++)
			{data+=RAM_16BIT(temp1+temp*2);}
			PLC_Addr-=4;        //????????
		}
		else
		{  
			data=(u16)temp1;
			PLC_Addr-=4;   //????
			for(temp=1;temp<temp2;temp++)
			{ 
				PLC_Addr-=2;
				Transfer=temp*16;
				data+=addr_value_prog();
			}
			Flag_bit=0xff;
		}
		trade=data/temp2;
		target();
		PLC_Addr+=2;        
	}
	else PLC_Addr+=6;
}

//=======================================================================================================
// ????????:  static void ADD(void)	 
// ?????????? 16��???????? ADD???  ???
// ????:  void      
// ????:  void     
// ????????:  
// ???????: 
// ??????:  ����??
// ?????:  2015??8??4??
// ??  ?:  
//=======================================================================================================
static void ADD(void)	   
{ 
	if(PLC_ACC_BIT&0X01)
	{
		trade=cos_value()+cos_value();
		target();
	}
	else PLC_Addr+=6;              //????6??????
}

//=======================================================================================================
// ????????:  static void ALT(void)	 
// ?????????? 16��???????? ALT???
// ????:  void      
// ????:  void     
// ????????:  
// ???????: 
// ??????:  ����??
// ?????:  2015??8??4??
// ??  ?:  
//=======================================================================================================
static void ALT(void)	                 
{  	
	signed int temp;
	if(PLC_ACC_BIT&0X01)
	{
		temp=addr_value(); 
		if(PLC_BIT_TEST(temp))
		PLC_BIT_OFF(temp);
		else
		PLC_BIT_ON(temp);
	}
	else PLC_Addr+=2;                      //??��???????2??????
}

//======================================================================================================
// ????????: static void TRD(void)
// ??????????TRD?????
// ????:  void      
// ????:  void     
// ????????:  
// ???????: 
// ??????:  ����??
// ?????:  2015??7??26??
// ??  ?:  
//=======================================================================================================
static void TRD(void)
{
	u16 temp;
	if(PLC_ACC_BIT&0X01)
	{
		temp=addr_value();
		RAM_16BIT(temp) = RAM_16BIT(0x0E12);
		RAM_16BIT(temp+1) = RAM_16BIT(0x0E11);
		RAM_16BIT(temp+2) = RAM_16BIT(0x0E10);
		RAM_16BIT(temp+3) = RAM_16BIT(0x0E0F);
		RAM_16BIT(temp+4) = RAM_16BIT(0x0E0E);
		RAM_16BIT(temp+5) = RAM_16BIT(0x0E0D);
		RAM_16BIT(temp+6) = RAM_16BIT(0x0E13);
		//????????????????????
	}
	else PLC_Addr+=2;              //????2??????
}

//======================================================================================================
// ????????: static void TWR(void)
// ??????????TWR?????
// ????:  void      
// ????:  void     
// ????????:  
// ???????: 
// ??????:  ����??
// ?????:  2015??7??26??
// ??  ?:  
//=======================================================================================================
static void TWR(void)
{
	u16 temp;
	if(PLC_ACC_BIT&0X01)
	{
		temp=addr_value();
		//RTC_Set(PLC_16BIT[temp],PLC_16BIT[temp+1],PLC_16BIT[temp+2],PLC_16BIT[temp+3],PLC_16BIT[temp+4],PLC_16BIT[temp+5]);
		//��?????????????
	}
	else PLC_Addr+=2;              //????2??????
}

//======================================================================================================
// ????????: static void TSUB(void)
// ??????????TSUB?????
// ????:  void      
// ????:  void     
// ????????:  
// ???????: 
// ??????:  ����??
// ?????:  2015??7??26??
// ??  ?:  
//=======================================================================================================
static void TSUB(void)
{
	u16 temp,temp1,temp2;
	if(PLC_ACC_BIT&0X01)
	{
		temp1=addr_value();  
		temp2=addr_value();  
		temp=addr_value();
		RAM_16BIT(temp) = RAM_16BIT(temp1) - RAM_16BIT(temp2);
		RAM_16BIT(temp+1) = RAM_16BIT(temp1+1) - RAM_16BIT(temp2+1);
		RAM_16BIT(temp+2) = RAM_16BIT(temp1+2) - RAM_16BIT(temp2+2);
		//?????????
	}
}

static void SUB(void)	   //????
{ 
	if(PLC_ACC_BIT&0X01)
	{
		PLC_Err=PLC_Addr;
		trade=cos_value()-cos_value();
		target();
	} 
	else PLC_Addr+=6;              //????6??????
}

static void DBCD(void)	            //?????????DBCD
{ 
	signed int can1,add1,add2,add3,add4,buffer1,buffer2,buffer3,buffer4;
	if(PLC_ACC_BIT&0X01)
	{
		PLC_Err=PLC_Addr;
		can1=cos_u32_value();
		add1=can1%10;
		add2=can1/10;
		add2=add2%10;
		add3=can1/100;
		add3=add3%10;
		add4=can1/1000;
		add4=add4%10;  
		
		buffer1=can1/10000;
		buffer1=buffer1%10;
		
		buffer2=can1/100000;
		buffer2=buffer2%10;
		
		buffer3=can1/1000000;
		buffer3=buffer3%10;
		
		buffer4=can1/10000000;
		buffer4=buffer4%10;
		
		trade=buffer4*16*256*65536+buffer3*256*65536+buffer2*16*65536+buffer1*65536+add4*16*256+add3*256+add2*16+add1;
		D_target();
	}
	else PLC_Addr+=8;              //????4??????
}

static void BCD(void)	            //?????????BCD
{ 
	signed short Ia, Ic;
	if((PLC_ACC_BIT&0X01)==0X01)
	{ 
		PLC_Err=PLC_Addr;
		Ic = cos_value();
		Ia   = (Ic / 1000) << 12;
		Ic  %= 1000;
		Ia  |= (Ic / 100 ) << 8;
		Ic  %= 100;
		Ia  |= (Ic / 10 ) << 4;
		Ic  %= 10;
		Ia  |=  Ic;
		trade=Ia;
		target();
	}
	else PLC_Addr+=4;              //????4??????
}

static void DBIN(void)	         //?????????DBIN
{ 
	signed int can1,add1,add2,add3,add4,buffer1,buffer2,buffer3,buffer4;
	if(PLC_ACC_BIT&0X01)
	{  
		PLC_Err=PLC_Addr;
		can1=cos_u32_value();
		add1=can1%16;
		add2=can1/16;
		add2=add2%16;
		add3=can1/256;
		add3=add3%16;
		add4=can1/(16*256);
		add4=add4%16;
		
		can1=can1/65536;
		buffer1=can1%16;
		buffer2=can1/16;
		buffer2=buffer2%16;
		buffer3=can1/256;
		buffer3=buffer3%16;
		buffer4=can1/(16*256);
		buffer4=buffer4%16;
		
		trade=buffer4*10000000+buffer3*1000000+buffer2*100000+buffer1*10000+add4*1000+add3*100+add2*10+add1;
		
		D_target();
	}
	else PLC_Addr+=8;              //????4??????
}

static void BINV(void)	            //?????????BIN
{ 
	signed short Ia, Ic;
	if((PLC_ACC_BIT&0X01)==0X01)
	{	
		PLC_Err=PLC_Addr;
		Ic = cos_value();
		Ia   = ((Ic >> 12) & 0x0f) * 1000;
		Ia  += ((Ic >> 8 ) & 0x0f) * 100;
		Ia  += ((Ic >> 4 ) & 0x0f) * 10;
		Ia  +=   Ic        & 0x0f;
		trade=Ia;
		target();
	}
	else PLC_Addr+=4;              //????4??????
}
 
static void MUL(void)	 //???
{ 
	signed int temp1,temp2;u32 temp3;
	if((PLC_ACC_BIT&0X01)==0X01)
	{
		PLC_Err=PLC_Addr;
		temp1=cos_value();
		temp2=cos_value();
		temp3=addr_value_prog(); 
		RAM_32BIT(temp3)=temp1*temp2;
	}
	else PLC_Addr+=6;              //????6??????
}
 
static void DIV(void)	 //????
{  
	signed short int temp1,temp2,temp3;
	if((PLC_ACC_BIT&0X01)==0X01)
	{
		PLC_Err=PLC_Addr;
		temp1=cos_value();
		temp2=cos_value();
		temp3=addr_value();

		RAM_16BIT(temp3) = temp1/temp2;
		RAM_16BIT(temp3+2) = temp1%temp2;
	}
	else PLC_Addr+=6;              //????6??????
}

static void DADD(void)
{
	if((PLC_ACC_BIT&0X01)==0X01)
	{
		PLC_Err=PLC_Addr;
		trade=cos_u32_value()+cos_u32_value();
		D_target();
	}
	else PLC_Addr+=12;              //????12??????
}

static void DSUB(void)
{
	if((PLC_ACC_BIT&0X01)==0X01)
	{  
		PLC_Err=PLC_Addr;
		trade=cos_u32_value()-cos_u32_value();
		D_target();
	}
	else PLC_Addr+=12;              //????12??????
}

static void DMUL(void)    
{	 
	signed short int temp;
	if(PLC_ACC_BIT&0X01)
	{
		u64data.data=(int64_t)(cos_u32_value()*cos_u32_value()); //��????64��?????
		temp=addr_value(); PLC_Addr+=2;               
		RAM_16BIT(temp+0*2) = u64data.data1[0];         //????
		RAM_16BIT(temp+1*2) = u64data.data1[1]; 
		RAM_16BIT(temp+2*2) = u64data.data1[2]; 
		RAM_16BIT(temp+3*2) = u64data.data1[3];	
	}
	else PLC_Addr+=12;              //????12??????
}

static void DDIV(void)
{
	signed short int temp1,temp2,temp3;
	if(PLC_ACC_BIT&0X01)
	{
		temp1 = cos_u32_value();             
		temp2 = cos_u32_value();		
		
		u32data.data=temp1/temp2;                   //??
		u32data1.data=temp1%temp2;                  //????
		
		temp3=addr_value() ;PLC_Addr+=2;                         
		RAM_16BIT(temp3+0*2) = u32data.data1[0];         //????
		RAM_16BIT(temp3+1*2) = u32data.data1[1]; 
		RAM_16BIT(temp3+2*2) = u32data1.data1[0]; 
		RAM_16BIT(temp3+3*2) = u32data1.data1[1];	
	}
	else PLC_Addr+=12;              //????12??????
}

static void DFLT(void)	                 //?????????
{  	
	signed int temp1;
	if(PLC_ACC_BIT&0X01)
	{
		temp1=cos_u32_value();
		trade1=(float)temp1;					
		float_target();
	}
	else PLC_Addr+=8;              //????8??????
}

//======================================================================================================
// ????????:  static void DESQR(void)
// ?????????? ???????????????
// ????:  void      
// ????:  void     
// ????????:  
// ???????: 
// ??????:  ����??
// ?????:  2014??6??27??
// ??  ?:  
//=======================================================================================================
static void DINT(void)
{
	if(PLC_ACC_BIT&0X01)
	{
		trade=(u32)float_value();
		D_target();
	} 
	else PLC_Addr+=8;              //????8??????
}

//======================================================================================================
// ????????:  static void INT(void)
// ?????????? ???????????????
// ????:  void      
// ????:  void     
// ????????:  
// ???????: 
// ??????:  ����??
// ?????:  2014??6??27??
// ??  ?:  
//=======================================================================================================
static void INT(void)
{
	if(PLC_ACC_BIT&0X01)
	{
		trade=(u16)float_value();
		PLC_Addr-=2; 
		target();
	} 
	else PLC_Addr+=4;              //????4??????
}

//======================================================================================================
// ????????:  static void FLT(void)
// ?????????? ????????? FLT
// ????:  void      
// ????:  void     
// ????????:  
// ???????: 
// ??????:  ����??
// ?????:  2014??6??27??
// ??  ?:  
//=======================================================================================================
static void FLT(void)	                
{  	
	if(PLC_ACC_BIT&0X01)
	{
		trade1=(float)cos_value();			
		float_target();PLC_Addr-=2; 
	}
	else PLC_Addr+=4;              //????4??????
}

//======================================================================================================
// ????????:  static void DTAN(void)
// ?????????? ???????? DTAN
// ????:  void      
// ????:  void     
// ????????:  
// ???????: 
// ??????:  ����??
// ?????:  2014??6??27??
// ??  ?:  
//=======================================================================================================
static void DTAN(void)
{
	if(PLC_ACC_BIT&0X01)
	{
		trade1=(float)tan((double)float_value());
		float_target();
	} 
	else PLC_Addr+=8;              //????8??????
}

//======================================================================================================
// ????????:  static void DCOS(void)
// ?????????? ???????? DCOS
// ????:  void      
// ????:  void     
// ????????:  
// ???????: 
// ??????:  ����??
// ?????:  2014??6??27??
// ??  ?:  
//=======================================================================================================
static void DCOS(void)
{
	if(PLC_ACC_BIT&0X01)
	{
		trade1=(float)cos((double)float_value());
		float_target();
	} 
	else PLC_Addr+=8;              //????8??????
}

//======================================================================================================
// ????????:  static void DSIN(void)
// ?????????? ????????  DSIN
// ????:  void      
// ????:  void     
// ????????:  
// ???????: 
// ??????:  ����??
// ?????:  2014??6??27??
// ??  ?:  
//=======================================================================================================
static void DSIN(void)
{
	if(PLC_ACC_BIT&0X01)
	{
		trade1=(float)sin((double)float_value());
		float_target();
	} 
	else PLC_Addr+=8;              //????8??????
}

//======================================================================================================
// ????????:  static void DESQR(void)
// ?????????? ?????????? DESQR
// ????:  void      
// ????:  void     
// ????????:  
// ???????: 
// ??????:  ����??
// ?????:  2014??6??27??
// ??  ?:  
//=======================================================================================================
static void DESQR(void)
{
	if(PLC_ACC_BIT&0X01)
	{
		trade1=(float)sqrt((double)float_value());
		float_target();
	} 
	else PLC_Addr+=8;              //????8??????
}

//======================================================================================================
// ????????:  static void DEADD(void)	
// ?????????? ??????????? ???DEADD
// ????:  void      
// ????:  void     
// ????????:  
// ???????: 
// ??????:  ����??
// ?????:  2014??6??27??
// ??  ?:  
//=======================================================================================================
static void DEADD(void)	 
{  
	if(PLC_ACC_BIT&0X01)
	{
		trade1=float_value()+ float_value(); 						
		float_target();
	}
	else PLC_Addr+=12;              //????12??????
}

static void DESUB(void)	           //???????????? 
{  
	float temp1,temp2;
	if(PLC_ACC_BIT&0X01)
	{
		temp1 = float_value();              
		temp2 = float_value(); 
		
		trade1=temp1-temp2;					
		float_target();
	}
	else PLC_Addr+=12;              //????12??????
}

static void DEDIV(void)	           //???????????? 
{  
	float temp1,temp2;
	if((PLC_ACC_BIT&0X01)==0X01)
	{
		temp1 = float_value();              
		temp2 = float_value(); 
		
		trade1=temp1/temp2;					
		float_target();
	}
	else PLC_Addr+=12;              //????12??????
}

static void DEMUL(void)	                //???????????
{  
	if((PLC_ACC_BIT&0X01)==0X01)
	{
		trade1=float_value()*float_value();					
		float_target();
	}
	else PLC_Addr+=12;              //????12??????
}

u16 DE[16]={0x0001,0x0003,0x0007,0x000F,0X001F,0X003F,0X007F,0X00FF,0X01FF,0X03FF,0X07FF,0X0FFF,0X1FFF,0X3FFF,0X7FFF,0XFFFF};	

void DECO()
{
  u8 i,t=1;
	int temp1=cos_value();      //?????????
	int temp2=addr_value();     
	int n=cos_value();          //???????????  
	temp1&=DE[n-1];             //????
  for(i=0;i<n;i++)            //????2??n?��?
	{t*=2;}		 
//	if()
  RAM_16BIT(temp2) = 1<<(temp1-1);	
}

void PLSY(void)
{  
	/*
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	static  u8 addr;
	long sys;
	signed short temp,PUS_TOTAL=0;	
	if(PLC_ACC_BIT&0X01)									//Determine if the condition is satisfied
	{ 
		temp=cos_value();										//pulse frequency
		PUS_TOTAL=cos_value();							//pulse count
		addr=*PLC_Addr;											//output address
		PLC_Addr+=2;
		if(PLC_16BIT[0x078C] >= PUS_TOTAL)PLC_BIT_ON(M8029);
		
		if((addr==0x00)&&(!(PLC_BIT_TEST(M8145)))&&(Y0P==0))           //Y0
		{
			PLC_BIT_ON(0x500);           			//Y00 ON 
			PLC_BIT_OFF(M8029);          			//M8029
		// PLC_RAM32(0X20005F1C)=0;     		//D8140
			Y0P=1;
			Plus_CMP0=PUS_TOTAL; 
			sys=72000000/((71+1)*(temp*2))-1;
		//Io port initialization
			GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;      //A7 
			GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
			GPIO_InitStructure.GPIO_Mode =  GPIO_Mode_Out_PP;
			GPIO_Init(GPIOC, &GPIO_InitStructure);
			
			RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3 , ENABLE);
			TIM_DeInit(TIM3);
			TIM_TimeBaseStructure.TIM_Period=sys;											//auto-reload register period value (count value)
		//Generate an update or interrupt after accumulating TIM_Period frequencies
			TIM_TimeBaseStructure.TIM_Prescaler= (72 - 1);						//Clock prescaler 72M/72
			TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1;			//sample frequency division
			TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up;	//count up mode
			TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
			TIM_ClearFlag(TIM3, TIM_FLAG_Update);											//clear overflow interrupt flag
			TIM_ITConfig(TIM3,TIM_IT_Update,ENABLE);
			TIM_Cmd(TIM3, ENABLE);//start the clock
			RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3 , ENABLE);			//first close and wait for use
			PLC_BIT_ON(M8147);                                        // M8147 
		}	
	}
	else  
	{  
		PLC_Addr+=4;
		addr=*PLC_Addr;										//output address
		PLC_Addr+=2;
		if(addr==0x00)                		//Y0
		{
			if(Y0P==1)
			{ 
				TIM_Cmd(TIM5, DISABLE); 
				TIM_Cmd(TIM3, DISABLE);
				TIM_ClearFlag(TIM3, TIM_FLAG_Update);
				TIM_ITConfig(TIM3, TIM_IT_Update, DISABLE);
							
// 				GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;  //A2
// 				GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
// 				GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
// 				GPIO_Init(GPIOC, &GPIO_InitStructure);						
				Y0P=0;
				PLC_BIT_OFF(0x500);    			//Y00 OFF
				PLC_BIT_OFF(M8147);    			//M8147
			}
		}
		
	}	*/
}

static void Damount(void)	 //32��?????????
{  
	s32 temp1,temp2;
	temp1=cos_u32_value();
	temp2=cos_u32_value();
	if(PLC_STL_Status == 1)  //?STL????
	{ 
		PLC_ACC_BIT<<=1;		
		if((temp1==temp2)&&(PLC_BIT_TEST(PLC_STL_Addr)))      
		PLC_ACC_BIT|=1;
	}	
	else
	{ 
		PLC_ACC_BIT<<=1;
		if(temp1==temp2)     //?????��?
		PLC_ACC_BIT|=1;
	}
}

static void amount(void)	 //16��?????????
{ 
	signed short int temp1,temp2;
	temp1=cos_value();
	temp2=cos_value();
	if(PLC_STL_Status == 1)     //?STL????
	{  
		PLC_ACC_BIT<<=1;		
		if((temp1==temp2)&&(PLC_BIT_TEST(PLC_STL_Addr)))     
		PLC_ACC_BIT|=1;
	}	
	else
	{ 
		PLC_ACC_BIT<<=1;
		if(temp1==temp2) 					//?????��?
		PLC_ACC_BIT|=1;
	}
}

static void amount_OR()
{ 
	signed short int temp1,temp2;
	temp1=cos_value();
	temp2=cos_value();
	if((temp1==temp2)||(PLC_ACC_BIT&0X01))  //?????��?
	PLC_ACC_BIT|=0X01;
	else
	PLC_ACC_BIT&=0xFE; 

}

static void Damount_OR()
{ 
	s32 temp1,temp2;
	temp1=cos_u32_value();
	temp2=cos_u32_value();
	if((temp1==temp2)||(PLC_ACC_BIT&0X01))  //?????��?
	PLC_ACC_BIT|=0X01;
	else
	PLC_ACC_BIT&=0xFE; 
}

static void Damount_and(void)	 //32��AND??????
{ 
	s32 temp1,temp2;
	temp1=cos_u32_value();
	temp2=cos_u32_value();
	if((temp1==temp2)&&((PLC_ACC_BIT&0X01)==0X01)) 
	PLC_ACC_BIT|=1;
	else
	PLC_ACC_BIT&=~1;
}

static void amount_and(void)	 //16��AND??????
{ 
	signed short int temp1,temp2;
	temp1=cos_value();
	temp2=cos_value();
	if((temp1==temp2)&&((PLC_ACC_BIT&0X01)==0X01)) 
	PLC_ACC_BIT|=1;
	else
	PLC_ACC_BIT&=~1;
}

static void Dbig(void)		    //32��?????????
{ 
	s32 temp1,temp2;
	temp1=cos_u32_value();
	temp2=cos_u32_value();
	if(PLC_STL_Status == 1)                       //?STL????
	{  
		PLC_ACC_BIT<<=1;		
		if((temp1>temp2)&&(PLC_BIT_TEST(PLC_STL_Addr)))    
		PLC_ACC_BIT|=1;
	}	
	else
	{ 
		PLC_ACC_BIT<<=1;
		if(temp1>temp2) 						               //?????��?  ?��????"<"?????">"
		PLC_ACC_BIT|=1;
	}
}

static void big(void)		     //16��?????????
{ 
	signed short int temp1,temp2;
	temp1=cos_value();
	temp2=cos_value();

	//Serial.printf("temp1: %X\n",temp1);
	//Serial.printf("temp2: %X\n",temp2);

	if(PLC_STL_Status == 1)                       //?STL????
	{  
		PLC_ACC_BIT<<=1;		
		if((temp1>temp2)&&(PLC_BIT_TEST(PLC_STL_Addr)))      
		PLC_ACC_BIT|=1;
	}	
	else
	{ 
		PLC_ACC_BIT<<=1;
		if(temp1>temp2) 						               //?????��? ?��????"<"?????">"
		PLC_ACC_BIT|=1;
	}
}

static void big_OR()
{ 
	signed short int temp1,temp2;
	temp1=cos_value();
	temp2=cos_value();
	if((temp1>temp2)||(PLC_ACC_BIT&0X01))    
	PLC_ACC_BIT|=0X01;
	else
	PLC_ACC_BIT&=0xFE; 
}

static void Dbig_OR()
{ 
	s32 temp1,temp2;
	temp1=cos_u32_value();
	temp2=cos_u32_value();
	if((temp1>=temp2)||(PLC_ACC_BIT&0X01))    
	PLC_ACC_BIT|=0X01;
	else
	PLC_ACC_BIT&=0xFE; 
}

static void Dbig_and(void)		//32��AND?????????
{ 
	s32 temp1,temp2;
	temp1=cos_u32_value();
	temp2=cos_u32_value();
	if((temp1>temp2)&&((PLC_ACC_BIT&0X01)==0X01)) 
	PLC_ACC_BIT|=1;
	else
	PLC_ACC_BIT&=~1;
}

static void big_and(void)		//16��AND?????????
{ 
	signed short int temp1,temp2;
	temp1=cos_value();
	temp2=cos_value();
	if((temp1>temp2)&&((PLC_ACC_BIT&0X01)==0X01)) 
	PLC_ACC_BIT|=1;
	else
	PLC_ACC_BIT&=~1;
}

static void Dless(void)	     //32�˧�???????
{ 
	s32 temp1,temp2;
	temp1=cos_u32_value();
	temp2=cos_u32_value();
	if(PLC_STL_Status == 1)                       //?STL????
	{  
		PLC_ACC_BIT<<=1;		
		if((temp1<temp2)&&(PLC_BIT_TEST(PLC_STL_Addr)))      
		PLC_ACC_BIT|=1;
	}	
	else
	{ 
		PLC_ACC_BIT<<=1;
		if(temp1<temp2) 						               //?????��?
		PLC_ACC_BIT|=1;
	}
}

static void less(void)	     //��????
{ 
	signed short int temp1,temp2;
	temp1=cos_value();
	temp2=cos_value();
	if(PLC_STL_Status == 1)                       //?STL????
	{  
		PLC_ACC_BIT<<=1;		
		if((temp1<temp2)&&(PLC_BIT_TEST(PLC_STL_Addr)))    
		PLC_ACC_BIT|=1;
	}	
	else
	{ 
		PLC_ACC_BIT<<=1;
		if(temp1<temp2) 						               //?????��?
		PLC_ACC_BIT|=1;
	}
}

static void less_OR()
{ 
	signed short int temp1,temp2;
	temp1=cos_value();
	temp2=cos_value();
	if((temp1<temp2)||(PLC_ACC_BIT&0X01))    
	PLC_ACC_BIT|=0X01;
	else
	PLC_ACC_BIT&=0xFE; 
}

static void Dless_OR()
{ 
	s32 temp1,temp2;
	temp1=cos_u32_value();
	temp2=cos_u32_value();
	if((temp1<temp2)||(PLC_ACC_BIT&0X01))    
	PLC_ACC_BIT|=0X01;
	else
	PLC_ACC_BIT&=0xFE; 
}

static void Dless_and(void)	   //32��AND��????
{ 
	s32 temp1,temp2;
	temp1=cos_u32_value();
	temp2=cos_u32_value();
	if((temp1<temp2)&&((PLC_ACC_BIT&0X01)==0X01)) 
	PLC_ACC_BIT|=1;
	else
	PLC_ACC_BIT&=~1;
}

static void less_and(void)	   //16��AND��????
{ 
	signed short int temp1,temp2;
	temp1=cos_value();
	temp2=cos_value();
	if((temp1<temp2)&&((PLC_ACC_BIT&0X01)==0X01)) 
	PLC_ACC_BIT|=1;
	else
	PLC_ACC_BIT&=~1;
}

static void Dless_amount(void)	     //32�˧�??????????
{ 
	s32 temp1,temp2;
	temp1=cos_u32_value();
	temp2=cos_u32_value();
	if(PLC_STL_Status == 1)            //?STL????
	{  
		PLC_ACC_BIT<<=1;		
		if((temp1<=temp2)&&(PLC_BIT_TEST(PLC_STL_Addr)))     
		PLC_ACC_BIT|=1;
	}	
	else
	{ 
		PLC_ACC_BIT<<=1;
		if(temp1<=temp2) 						    //?????��?
		PLC_ACC_BIT|=1;
	}
}	

static void less_amount(void)	      //16�˧�??????????
{ 
	signed short int temp1,temp2;
	temp1=cos_value();
	temp2=cos_value();
	if(PLC_STL_Status == 1)          //?STL????
	{  
		PLC_ACC_BIT<<=1;		
		if((temp1<=temp2)&&(PLC_BIT_TEST(PLC_STL_Addr)))     
		PLC_ACC_BIT|=1;
	}	
	else
	{ 
		PLC_ACC_BIT<<=1;
		if(temp1<=temp2) 						               //?????��?
		PLC_ACC_BIT|=1;
	}
}	

static void less_amount_OR()
{ 
	signed short int temp1,temp2;
	temp1=cos_value();
	temp2=cos_value();
	if((temp1<=temp2)||(PLC_ACC_BIT&0X01))    
	PLC_ACC_BIT|=0X01;
	else
	PLC_ACC_BIT&=0xFE; 
}

static void Dless_amount_OR()
{ 
	s32 temp1,temp2;
	temp1=cos_u32_value();
	temp2=cos_u32_value();
	if((temp1<=temp2)||(PLC_ACC_BIT&0X01))    
	PLC_ACC_BIT|=0X01;
	else
	PLC_ACC_BIT&=0xFE; 
}

static void Dless_amount_and(void)	   //32��AND��???????
{ 
	s32 temp1,temp2;
	temp1=cos_u32_value();
	temp2=cos_u32_value();
	if((temp1<=temp2)&&((PLC_ACC_BIT&0X01)==0X01)) 
	PLC_ACC_BIT|=1;
	else
	PLC_ACC_BIT&=~1;
}

static void less_amount_and(void)	     //16��AND��???????
{ 
	signed short int temp1,temp2;
	temp1=cos_value();
	temp2=cos_value();
	if((temp1<=temp2)&&(PLC_ACC_BIT&0X01)) 
	PLC_ACC_BIT|=0X01;
	else
	PLC_ACC_BIT&=0XFE;
}

static void Dbig_amount(void)	     //32��????????????
{ 
	s32 temp1,temp2;
	temp1=cos_u32_value();
	temp2=cos_u32_value();
	if(PLC_STL_Status == 1)            //?STL????
	{  
		PLC_ACC_BIT<<=1;		
		if((temp1>=temp2)&&(PLC_BIT_TEST(PLC_STL_Addr)))     
		PLC_ACC_BIT|=1;
	}	
	else
	{ 
		PLC_ACC_BIT<<=1;
		if(temp1>=temp2) 						               //?????��?
		PLC_ACC_BIT|=1;
	}
}

static void big_amount(void)	     //16��????????????
{ 
	signed short int temp1,temp2;
	temp1=cos_value();
	temp2=cos_value();
	if(PLC_STL_Status == 1)            //?STL????
	{  
		PLC_ACC_BIT<<=1;		
		if((temp1>=temp2)&&(PLC_BIT_TEST(PLC_STL_Addr)))     
		PLC_ACC_BIT|=1;
	}	
	else
	{ 
		PLC_ACC_BIT<<=1;
		if(temp1>=temp2) 						               //?????��?
		PLC_ACC_BIT|=1;
	}
}

static void big_amount_OR()
{ 
	signed short int temp1,temp2;
	temp1=cos_value();
	temp2=cos_value();
	if((temp1>=temp2)||(PLC_ACC_BIT&0X01))    
	PLC_ACC_BIT|=0X01;
	else
	PLC_ACC_BIT&=0xFE; 
}

static void Dbig_amount_OR()
{ 
	s32 temp1,temp2;
	temp1=cos_u32_value();
	temp2=cos_u32_value();
	if((temp1>=temp2)||(PLC_ACC_BIT&0X01))    
	PLC_ACC_BIT|=0X01;
	else
	PLC_ACC_BIT&=0xFE; 
}

static void Dbig_amount_and(void)	   //32��AND?????????
{ 
	s32 temp1,temp2;
	temp1=cos_u32_value();
	temp2=cos_u32_value();
	if((temp1>=temp2)&&((PLC_ACC_BIT&0X01)==0X01)) 
	PLC_ACC_BIT|=1;
	else
	PLC_ACC_BIT&=~1;
}

static void big_amount_and(void)	   //16��AND?????????
{ 
	signed short int temp1,temp2;
	temp1=cos_value();
	temp2=cos_value();
	if((temp1>=temp2)&&(PLC_ACC_BIT&0X01)) 
	PLC_ACC_BIT|=1;
	else
	PLC_ACC_BIT&=~1;
}

static void Dno_amount(void)	   //32��???????????
{ 
	s32 temp1,temp2;
	temp1=cos_u32_value();
	temp2=cos_u32_value();
	if(PLC_STL_Status == 1)            //?STL????
	{  
		PLC_ACC_BIT<<=1;		
		if((temp1!=temp2)&&(PLC_BIT_TEST(PLC_STL_Addr)))     
		PLC_ACC_BIT|=1;
	}	
	else
	{ 
		PLC_ACC_BIT<<=1;
		if(temp1!=temp2) 					 //?????��?
		PLC_ACC_BIT|=1;
	}
}

static void no_amount(void)	    //16��???????????
{ 
	signed short int temp1,temp2;
	temp1=cos_value();
	temp2=cos_value();
	if(PLC_STL_Status == 1)            //?STL????
	{  
		PLC_ACC_BIT<<=1;		
		if((temp1!=temp2)&&(PLC_BIT_TEST(PLC_STL_Addr)))     
		PLC_ACC_BIT|=1;
	}	
	else
	{ 
		PLC_ACC_BIT<<=1;
		if(temp1!=temp2) 						               //?????��?
		PLC_ACC_BIT|=1;
	}
}

static void no_amount_OR()
{ 
	signed short int temp1,temp2;
	temp1=cos_value();
	temp2=cos_value();
	if((temp1!=temp2)||(PLC_ACC_BIT&0X01))     
	PLC_ACC_BIT|=0X01;
	else
	PLC_ACC_BIT&=0xFE; 
}

static void Dno_amount_OR()
{ 
	s32 temp1,temp2;
	temp1=cos_u32_value();
	temp2=cos_u32_value();
	if((temp1!=temp2)||(PLC_ACC_BIT&0X01))     
	PLC_ACC_BIT|=0X01;
	else
	PLC_ACC_BIT&=0xFE; 
}

static void Dno_amount_and(void)	   //32��AND???????????
{ 
	s32 temp1,temp2;
	temp1=cos_u32_value();
	temp2=cos_u32_value();
	if((temp1!=temp2)&&((PLC_ACC_BIT&0X01)==0X01)) 
	PLC_ACC_BIT|=1;
	else
	PLC_ACC_BIT&=~1;
}

static void no_amount_and(void)	   //16��AND???????????
{ 
	signed short int temp1,temp2;
	temp1=cos_value();
	temp2=cos_value();
	if((temp1!=temp2)&&((PLC_ACC_BIT&0X01)==0X01)) 
	PLC_ACC_BIT|=1;
	else
	PLC_ACC_BIT&=~1;
}

static void LDP(void)	                        //LDP
{
	if(PLC_PL_BIT_TEST(PLC_Addr-PLC_START_Address))	
	{ 
		PLC_ACC_BIT<<=1;
		if(!(PLC_LD_BIT(0X2fff&*PLC_Addr)))							                    //?????��?
		PLC_PL_BIT_OFF(PLC_Addr-PLC_START_Address);		     
	} 
	else  							                                                     //???????��?
	{ 
		if(PLC_STL_Status == 1)                                             //?STL????
		{  
			PLC_ACC_BIT<<=1;
			if((PLC_LD_BIT(0X2fff&*PLC_Addr))&&(PLC_BIT_TEST(PLC_STL_Addr)))  //?????��? 				              
			PLC_ACC_BIT|=0x01,PLC_PL_BIT_ON(PLC_Addr-PLC_START_Address);      //
		}	
		else
		{ 
			PLC_ACC_BIT<<=1;
			if(PLC_LD_BIT(0X2fff&*PLC_Addr))							                   //?????��?
			PLC_ACC_BIT|=0x01,PLC_PL_BIT_ON(PLC_Addr-PLC_START_Address);      //
		}
	} 
	PLC_Addr++;	    
}

static void LDF(void)	 //LDF
{ 
	if(PLC_PL_BIT_TEST(PLC_Addr-PLC_START_Address))	 //???????��?
	{  
		if(PLC_STL_Status == 1)                       //?STL???? 
		{  
			PLC_ACC_BIT<<=1;
			if((!(PLC_LD_BIT(0X2fff&*PLC_Addr)))&&(PLC_BIT_TEST(PLC_STL_Addr)))//?????��? 				              
			PLC_ACC_BIT|=0x01,PLC_PL_BIT_ON(PLC_Addr-PLC_START_Address);
		}	
		else
		{ 
			PLC_ACC_BIT<<=1;
			if(!(PLC_LD_BIT(0X2fff&*PLC_Addr)))                     //?????��?
			PLC_ACC_BIT|=1,PLC_PL_BIT_OFF(PLC_Addr-PLC_START_Address);//
		}		         
	}
	else
	{  
		PLC_ACC_BIT<<=1,PLC_ACC_BIT&=0XFE;        //?????????????
		if(PLC_LD_BIT(0X2fff&*PLC_Addr))        //?????��?
		PLC_PL_BIT_ON(PLC_Addr-PLC_START_Address);//
	}  
	PLC_Addr++;
}

static void ANDP(void)	 //ANDP
{ 
	u8  logic;
	if(PLC_PL_BIT_TEST(PLC_Addr-PLC_START_Address))       //???????????????????
	{ 
		logic=0;
		if(!(PLC_LD_BIT(0X2fff&*PLC_Addr)))   					   //?????��?
		PLC_PL_BIT_OFF(PLC_Addr-PLC_START_Address);         //
	}  
	else	 							                                   //???????��?
	{ 
		if(PLC_LD_BIT(0X2fff&*PLC_Addr))	                 //?????��?
		logic=1,PLC_PL_BIT_ON(PLC_Addr-PLC_START_Address);  //?????????
		else
		logic=0;		                                         //???????????
	}
	if((PLC_ACC_BIT&0x01)&&(logic==1))
	PLC_ACC_BIT|=0X01;
	else
	PLC_ACC_BIT&=0XFE;
	PLC_Addr++;	    
}

static void ANDF(void)	 //ANDF
{ 
	u8  logic;
	if(PLC_PL_BIT_TEST(PLC_Addr-PLC_START_Address))			  //???????��?
	{ 
		if(!(PLC_LD_BIT(0X2fff&*PLC_Addr)))                //?????��?
		logic=1,PLC_PL_BIT_OFF(PLC_Addr-PLC_START_Address);  //
		else
		logic=0;		 //
	}
	else
	{
		logic=0;
		if(PLC_LD_BIT(0X2fff&*PLC_Addr))                   //?????��?
		PLC_PL_BIT_ON(PLC_Addr-PLC_START_Address);		        //
	}  
	if((PLC_ACC_BIT&0x01)&&(logic==1))
	PLC_ACC_BIT|=0X01;
	else
	PLC_ACC_BIT&=0XFE;
	PLC_Addr++;
} 

static void ORP(void)	 //ORP
{ 
	u8  logic;
	if(PLC_PL_BIT_TEST(PLC_Addr-PLC_START_Address))							  
	{ 
		logic=0;                                           //
		if(!(PLC_LD_BIT(0X2fff&*PLC_Addr)))							   //?????��?
		PLC_PL_BIT_OFF(PLC_Addr-PLC_START_Address);		       //
	} 
	else                                                   //???????��?
	{ 
		if(PLC_LD_BIT(0X2fff&*PLC_Addr))							     //?????��?
		logic=1,PLC_PL_BIT_ON(PLC_Addr-PLC_START_Address); //
		else
		logic=0;		 //
	}	
	
	if(((PLC_ACC_BIT&0x01)==0x01)||(logic==1))
	PLC_ACC_BIT|=0x01;
	else
	PLC_ACC_BIT&=0XFE;
	PLC_Addr++;	    
}

static void ORF(void)	 //ORF
{ 
	u8  logic;
	if(PLC_PL_BIT_TEST(PLC_Addr-PLC_START_Address))							                            //???????��?
	{ 
		if(!(PLC_LD_BIT(0X2fff&*PLC_Addr)))						    //?????��?
		logic=1,PLC_PL_BIT_OFF(PLC_Addr-PLC_START_Address);//
		else
		logic=0;		 //
	}
	else
	{  
		logic=0;
		if(PLC_LD_BIT(0X2fff&*PLC_Addr))							   //?????��?
		PLC_PL_BIT_ON(PLC_Addr-PLC_START_Address);	//
	}  
	if(((PLC_ACC_BIT&0x01)==0x01)||(logic==1))
	PLC_ACC_BIT|=1;
	else
	PLC_ACC_BIT&=~1;
	PLC_Addr++;
}

static void CJ_EX(u8 value)  //????????
{ 
	PLC_Addr++;
	if((*PLC_Addr&0xff00)==0x8000)
	{PLC_Addr=PLC_P_Addr[value/2],PLC_Addr++;}//???��
}

static void CJ(void)
{ 
	if(PLC_ACC_BIT&0X01)
	{
		if((*PLC_Addr&0xff00)==0x8800) CJ_EX(*PLC_Addr); 
	}
	else PLC_Addr+=2;
}
 
static void CJP(void)	  //CJP
{ 
	if(PLC_LDP_TEST())    //???????��?
	{if((*PLC_Addr&0xff00)==0x8800) CJ_EX(*PLC_Addr);}
	else
	PLC_Addr+=2;		      //??????????????????????��CPU????
}
 
static void SRET(void)
{ 
	u8 temp;
	PLC_ACC_BIT=process[0];	    //???????????????
	PLC_Addr=p_save[0];	  	    //???????????????????��??
	for(temp=62;temp>0;temp--)
	{
		process[temp]=process[temp+1];    //data mov down
		p_save[temp]=p_save[temp+1]; 
	}      
}

static void P_MOV(void)
{ 
	u8 temp;
	for(temp=62;temp>0;temp--)
	{
		process[temp+1]=process[temp];    //???? MOV up
		p_save[temp+1]=p_save[temp]; 
	}
	process[0]=PLC_ACC_BIT;	               //???????????????
	p_save[0]=PLC_Addr;				             //???????????????????��??
}

static void CALL_EX(u8 value)
{ 
	PLC_Addr++;
	if((*PLC_Addr&0xff00)==0x8000)
	{P_MOV(),PLC_Addr=PLC_P_Addr[value/2];}//	?????????????????????P?????????
}

static void CALL(void)
{ 
	if(PLC_ACC_BIT&0X01)
	{ 
		if((*PLC_Addr&0xff00)==0x8800) 
		{CALL_EX(*PLC_Addr);}
	}
	else PLC_Addr+=2;
}

static void CALLP(void)	  //CALLP
{ 
	if(PLC_LDP_TEST())       //???????��?
	{if((*PLC_Addr&0xff00)==0x8800)CALL_EX(*PLC_Addr);}
	else
	PLC_Addr+=2;		        //??????????????????????��CPU???? 
}

void expand_SET(void)
{
  BIT_SET(0X2FFF&*PLC_Addr);PLC_Addr++;
}
	
void expand_RST(void)
{
  RST(0X2FFF&*PLC_Addr);PLC_Addr++;
}
	
void expand_OUT(void)
{
  OUT(0X2FFF&*PLC_Addr);PLC_Addr++;
}
	
void expand_LD(void)
{
  LD(0X2FFF&*PLC_Addr);PLC_Addr++;
}
	
void expand_LDI(void)
{
  LDI(0x2FFF&*PLC_Addr);PLC_Addr++;
}
	
void expand_AND(void)
{
  AND(0x2FFF&*PLC_Addr);PLC_Addr++;
}
	
void expand_ANI(void)
{
  ANI(0x2FFF&*PLC_Addr);PLC_Addr++;
}
	
void expand_OR(void)
{
  OR(0x2FFF&*PLC_Addr);PLC_Addr++;
}
	
void expand_ORI(void)
{
  ORI(0x2FFF&*PLC_Addr);PLC_Addr++;
}	

static void enable_T_K(void)
{ 
	static u16 *p_data;
	T_value=*PLC_Addr%0x100;                //Assign the lower 8-bit value
	PLC_Addr++;
	T_value+=(*PLC_Addr%0x100)*0x100;       //Assign high 8-bit value
	p_data=(u16*)PLC_RAM+0x0900+T_number;   //The pointer points to the value address of T comparison
	*p_data=T_value;                        //Assign to address
	timer_enable(T_number);
	OUT(0x1600+(u8)T_number);
}

static void enable_T_D(void)
{ 
	u16 *p_data;
	//RAM_16BIT(0x0900+T_number*2) = RAM_16BIT(0x1000+T_value*2);
	p_data = (u16*)PLC_RAM+0x0900+T_number;
	*p_data = RAM_16BIT(0x3000+T_value*2);
	timer_enable(T_number);
	OUT(0X1600+(u8)T_number);
}

static void disable_T(void)
{
	timer_disble(T_number);
	OUT(0X1600+(u8)T_number);	 //disable T coil	
	OUT(0x0600+(u8)T_number);	 //reset T over coil
}

static void T_given_value_K(void)	      //
{
	if(PLC_ACC_BIT&0X01)  //
	enable_T_K();
	else
	PLC_Addr++,disable_T(); 
}

static void T_given_value_D(void)	      //
{ 
	T_value=(*PLC_Addr%0x100)/2;
	PLC_Addr++;
	switch(*PLC_Addr/0x100) 
	{ 
		case 0x86: T_value+=(*PLC_Addr%0x100)*0x80;        break;
		case 0x88: T_value+=(*PLC_Addr%0x100)*0x80+1000;   break; 
	}
	if((PLC_ACC_BIT&0X01)==0X01)  //is it effective
		enable_T_D();
	else
		disable_T();
}

static void operation_T(void)
{ 
	T_number=*PLC_Addr;       //Send the number of the operation timer into
	PLC_Addr++;				        //The next function is k assignment or d assignment
	switch(*PLC_Addr/0x100) 
	{ 
		case 0x80: T_given_value_K();  break;  //Perform K assignment operation
		case 0x86: T_given_value_D();  break;  //Perform D assignment operation
	}	
}

static void enable_C_K(void)	           //Assign with constant K
{
	u16 temp_bit,*p_C_enable_coil;u32 C;
	C_value=*PLC_Addr%0x100;                //Assign the lower 8-bit value
	PLC_Addr++;
	C_value+=(*PLC_Addr%0x100)*0x100;       //Assign high 8-bit value
	if(C_number>=0xC8)                      //Judge whether it is a register above C200
	{  
		PLC_Addr++;
		C_value+=(*PLC_Addr%0x100)*0x10000;  //Assign the lower 8-bit value
		PLC_Addr++;
		C_value+=(*PLC_Addr%0x100)*0x1000000;//Assign the high 8-bit value
		C=0x20001C00+(C_number-0xC8)*4; 
		temp_bit=1<<(C_number%0x10);
		if(RAM_32BIT(C)<C_value)             //Compare the current value of C with the target value
		{
			p_C_enable_coil = (u16*)PLC_RAM+0x0270+(C_number/0X10);//compare enable coil
			if(!((*p_C_enable_coil&temp_bit)==temp_bit))
			RAM_32BIT(C)+=1;
		}
		if(RAM_32BIT(C)<C_value)              //compare overflow value
			PLC_BIT_OFF(0x0E00+C_number);		 
		else
			PLC_BIT_ON(0x0E00+C_number);
	}
	else
	{
		//static u16 *p_data;
		//p_data = (u16*)PLC_RAM+0x2000+C_number*2;
		temp_bit=1<<(C_number%0x10);
		if(RAM_16BIT(0x2000+C_number*2)<C_value)             
		{
			p_C_enable_coil = (u16*)PLC_RAM+0x0270+(C_number/0x10);  //compare enable coil 
			if(!((*p_C_enable_coil&temp_bit)==temp_bit))
				RAM_16BIT(0x2000+C_number*2)+=1;
		}
		if(RAM_16BIT(0x2000+C_number*2)<C_value)                   //compare overflow value
			PLC_BIT_OFF(0x0E00+C_number);		 
		else
			PLC_BIT_ON(0x0E00+C_number);
	}
	OUT(0x2700+(u8)C_number);
}

static void enable_C_D(void)	    	//Assign with register D
{
	static u16 *p_data;
	u16 temp_bit,*p_C_enable_coil;u32 C;
	C_value = RAM_16BIT(0x3000+C_value*2);	//8606 >> C_value = 3
	//Serial2.printf("C-D: %X\n",C_value);		//- print C_value
	if(C_number>=0xC8)               //Judge whether it is a register above C200
	{  		 
		C_value+=RAM_16BIT(0x3000+C_value*2+1)*0x10000;
		C=0x20001C00+(C_number-0xC8)*4; 
		temp_bit=1<<(C_number%0x10);
		if(RAM_32BIT(C)<C_value)      //Compare the current value of c with the target value
		{
			p_C_enable_coil = (u16*)PLC_RAM+0x0270+(C_number/0x10);   //compare enable coil
			if(!((*p_C_enable_coil&temp_bit)==temp_bit))
				RAM_32BIT(C)+=1;
		}
		if(*p_data<C_value)            //compare overflow value
			PLC_BIT_OFF(0x0E00+C_number);		 
		else
			PLC_BIT_ON(0x0E00+C_number);
		PLC_Addr+=2;                  
	}
	else
	{
		//p_data = (u16*)PLC_RAM+0x2000+C_number*2;
		//p_data = (u16*)PLC_RAM+0x2000+C_number;
		temp_bit=1<<(C_number%0x10);
		if(RAM_16BIT(0x2000+C_number*2)<C_value)    				//Compare the current value of C with the target value
		{
			p_C_enable_coil = (u16*)PLC_RAM+0x0270+(C_number/0x10);   //compare enable coil
			if(!((*p_C_enable_coil&temp_bit)==temp_bit)){
				RAM_16BIT(0x2000+C_number*2)+=1;
			}	
		}
		if(RAM_16BIT(0x2000+C_number*2)<C_value)                   //compare overflow value
		//if(*p_data<=C_value)  				//compare overflow value
			PLC_BIT_OFF(0x0E00+C_number);		 
		else
			PLC_BIT_ON(0x0E00+C_number);
	}
	OUT(0x2700+(u8)C_number);
}
 
static void disable_C_K(void)
{	
	u32 C;static u16 *p_data;
	C_value=*PLC_Addr%0x100;           //Assign the lower 8-bit value
	PLC_Addr++;
	C_value+=(*PLC_Addr%0x100)*0x100;  //Assign high 8-bit value
	if(C_number>=0xC8)                 //Judge whether it is a register above C200
	{  
		PLC_Addr++;
		C_value=(*PLC_Addr%0x100)*0x10000;   //Assign the lower 8-bit value
		PLC_Addr++;
		C_value+=(*PLC_Addr%0x100)*0x1000000;//Assign the high 8-bit value
		C=0x20001C00+(C_number-0xC8)*4; 
		if(RAM_32BIT(C)<C_value)              //Compare the current value of C with the target value
		PLC_BIT_OFF(0x0E00+C_number);		 
		else
		PLC_BIT_ON(0x0E00+C_number);        
	}
	else{
		//p_data = (u16*)PLC_RAM+0x0500+C_number;

		if(RAM_16BIT(0x2000+C_number*2)<C_value)                 //????????
		PLC_BIT_OFF(0x0E00+C_number);		 
		else
		PLC_BIT_ON(0x0E00+C_number);       
	}
	OUT(0X2700+(u8)C_number);
}

static void disable_C_D(void)	     	//Close the counter C
{ 
	u32 C;static u16 *p_data;  
	if(C_number>=0xC8)               //Judge whether it is a register above C200
	{  
		C_value = RAM_16BIT(0x3000+C_value*2);
		C_value += RAM_16BIT(0x3000+C_value*2+1)*0x10000;
		C=0x20001C00+(C_number-0xC8)*4; 
		if(RAM_32BIT(C)<C_value)      //compare overflow value
		PLC_BIT_OFF(0x0E00+C_number);		 
		else
		PLC_BIT_ON(0x0E00+C_number);
		PLC_Addr+=2;                  
	}
	else
	{
		C_value = RAM_16BIT(0x3000+C_value*2);
		p_data = (u16*)PLC_RAM+0x0500+C_number;
		if(RAM_16BIT(0x2000+C_number*2)<C_value)            //compare overflow value
		PLC_BIT_OFF(0x0E00+C_number);
		else
		PLC_BIT_ON(0x0E00+C_number);
	}	
	OUT(0x2700+(u8)C_number);
}

static void C_given_value_K(void)	  	//The program uses k to set the value
{
	if((PLC_ACC_BIT&0X01)==0X01)       	//
	enable_C_K();				                //start the counter
	else
	disable_C_K(); 
}

static void C_given_value_D(void)	  //The program uses d to set the value
{  
	C_value=(*PLC_Addr%0x100)/2;
	PLC_Addr++;
	switch(*PLC_Addr/0x100) 
	{ 
		case 0x86: C_value+=(*PLC_Addr%0x100)*0x80;        break;
		case 0x88: C_value+=(*PLC_Addr%0x100)*0x80+1000;   break; 
	}
	if(PLC_ACC_BIT&0x01)      //
		enable_C_D();
	else
		disable_C_D();
}
 
static void operation_C()
{
	C_number=*PLC_Addr;       //Send the number of the operation counter into
	PLC_Addr++;				        //The next function is K assignment or D assignment
	switch(*PLC_Addr/0x100) 
	{
		case 0x80: C_given_value_K();break;  //Perform K assignment operation
		case 0x86: C_given_value_D();break;  //Perform D assignment operation
	}	
}

static void TFROM(){														//- Function FROM by Tom 20/11/2022
	if(PLC_ACC_BIT&0x01){
		/*
		uint16_t _slaveID,_function,_dataAddress,_dataCount;
	
		_slaveID 	= *PLC_Addr%0x100;								//- Get Slave ID
		PLC_Addr++;
		_function = *PLC_Addr%0x100;								//- Get Function code
		PLC_Addr++;

		_dataAddress 	= *PLC_Addr%0x100;						//- Get low byte
		PLC_Addr++;
		_dataAddress |= (*PLC_Addr%0x100)*256;			//- Get high byte
		PLC_Addr++;

		masterDataPos = *PLC_Addr%0x100;						//- Store data position low byte
		PLC_Addr++;
		masterDataPos |= (*PLC_Addr%0x100)*256;			//- Store data position high byte
		PLC_Addr++;

		_dataCount  = *PLC_Addr%0x100;							//- Data count low byte
		PLC_Addr++;
		_dataCount  |= (*PLC_Addr%0x100)*256;				//- Data count high byte
		PLC_Addr++;

		(*mbSetStack).slave 		= _slaveID;
		(*mbSetStack).func  		= _function;
		(*mbSetStack).data[0]	= _dataAddress/0x100;
		(*mbSetStack).data[1]	= _dataAddress%0x100;
		(*mbSetStack).count[0]	= _dataCount/0x100;
		(*mbSetStack).count[1]	= _dataCount%0x100;
		(*mbSetStack).dataPos		=	masterDataPos;

		mbSetStack++;
		*/
	}
}

static void TTO(){															//- Function TO by Tom 23/11/2022
	if(PLC_ACC_BIT&0x01){
		/*
		uint16_t _slaveID,_function,_dataAddress,_dataCount;
	
		_slaveID 	= *PLC_Addr%0x100;								//- Get Slave ID
		PLC_Addr++;
		_function = *PLC_Addr%0x100;								//- Get Function code
		PLC_Addr++;

		_dataAddress 	= *PLC_Addr%0x100;						//- Get low byte
		PLC_Addr++;
		_dataAddress |= (*PLC_Addr%0x100)*256;			//- Get high byte
		PLC_Addr++;

		masterDataPos = *PLC_Addr%0x100;						//- Store data position low byte
		PLC_Addr++;
		masterDataPos |= (*PLC_Addr%0x100)*256;			//- Store data position high byte
		PLC_Addr++;

		_dataCount  = *PLC_Addr%0x100;							//- Data count low byte
		PLC_Addr++;
		_dataCount  |= (*PLC_Addr%0x100)*256;				//- Data count high byte
		PLC_Addr++;

		(*mbSetStack).slave 		= _slaveID;
		(*mbSetStack).func  		= _function;
		(*mbSetStack).data[0]		= _dataAddress/0x100;
		(*mbSetStack).data[1]		= _dataAddress%0x100;
		(*mbSetStack).count[0]	= _dataCount/0x100;
		(*mbSetStack).count[1]	= _dataCount%0x100;
		(*mbSetStack).dataPos		=	masterDataPos;

		mbSetStack++;*/
	}
}

static void FNC_AppInstruct(void) 
 { 
	 switch(*PLC_Addr) 
	{
		case 0x0002: PLC_Addr++,expand_OUT();              break;  //Instructions above m1535
		case 0x0003: PLC_Addr++,expand_SET();              break;  //Instructions above m1535
		case 0x0004: PLC_Addr++,expand_RST();              break;  //Instructions above m1535
		
		case 0x0005: PLC_Addr++,expand_OUT();              break;  //
		case 0x0006: PLC_Addr++,expand_SET();              break;  //
		case 0x0007: PLC_Addr++,expand_RST();              break;  //
		case 0x0008: PLC_Addr++,LPS();                     break;  //
		case 0x0009: PLC_Addr++,LPF();                     break;  //
		case 0x000C: PLC_Addr++,RST_T_C();                 break;  //Execute RST C&T
		case 0x000D: PLC_Addr++,RST_D();                   break;  //Execute D register reset
		
		case 0x0010: PLC_Addr++,CJ();                      break;  //CJ  
		case 0x1010: PLC_Addr++,CJP();                     break;  //CJP  
		case 0x0012: PLC_Addr++,CALL();                    break;  //CALL
		case 0x1012: PLC_Addr++,CALLP();                   break;  //CALLP
		case 0x0014: PLC_Addr++,SRET();                    break;  //SRET			
		case 0x001C: PLC_Addr=PLC_Addr;                    break;  //FEND
		
//	case 0X0020: PLC_Addr++,FOR();                     break;  //����?? 20160929??????FOR???
//  case 0X0022: PLC_Addr++,FOR_NEXT();                break;  //����?? 20160929??????FOR_NEST ???????
		case 0X0024: PLC_Addr++,CMP();                     break;  //16��?????????
		case 0X1024: PLC_Addr++,CMPP();                    break;  //16��??????????????
		case 0X0025: PLC_Addr++,DCMP();                    break;  //32��?????????
		case 0X1025: PLC_Addr++,DCMPP();                   break;  //32��??????????????
		case 0X0026: PLC_Addr++,ZCP();                     break;  //16��??????????????
		case 0X0027: PLC_Addr++,DZCP();                    break;  //32��??????????????
		case 0x0028: PLC_Addr++,MOV();		                 break;  //???16bit???????
		case 0X0029: PLC_Addr++,DMOV();                    break;  //DMOV 
		case 0X002A: PLC_Addr++,SMOV();                    break;  //SMOV 	
		case 0X002C: PLC_Addr++,CML();                     break;  //CML??????
		case 0X002D: PLC_Addr++,DCML();                    break;  //DCML??????
		case 0X002E: PLC_Addr++,BMOV();                    break;  //????????
		case 0X0030: PLC_Addr++,FMOV();                    break;  //??????
		case 0X0031: PLC_Addr++,DFMOV();                   break;  //32��??????
		case 0X0032: PLC_Addr++,XCH();                     break;  //????????
		case 0X0033: PLC_Addr++,DXCH();                    break;  //32��????????
		case 0X0034: PLC_Addr++,BCD();                     break;  //?????????BCD
		case 0X0035: PLC_Addr++,DBCD();                    break;  //?????????DBCD
		case 0X0036: PLC_Addr++,BINV();                     break;  //?????????BIN
		case 0X0037: PLC_Addr++,DBIN();                    break;  //?????????DBIN
		
		case 0X0038: PLC_Addr++,ADD();					           break;  //??????
		case 0x0039: PLC_Addr++,DADD();                    break;  //DADD???????			
		case 0X003A: PLC_Addr++,SUB();					           break;  //???????
		case 0x003B: PLC_Addr++,DSUB();                    break;  //DSUB????????			
		case 0x003C: PLC_Addr++,MUL();                     break;  //MUL ??????
		case 0x003D: PLC_Addr++,DMUL();                    break;  //DMUL???????			
		case 0x003E: PLC_Addr++,DIV();                     break;  //DIV ??????
		case 0x003F: PLC_Addr++,DDIV();                    break;  //DDIV????????			
		case 0x0040: PLC_Addr++,INC();                     break;  //16��?????????1???
		case 0x1040: PLC_Addr++,INCP();                   break;  //16��???????????????1???
		case 0x0041: PLC_Addr++,DINC();                    break;  //32��?????????1???
		case 0x1041: PLC_Addr++,DINC_P();                  break;  //32��???????????????1???
		case 0x0042: PLC_Addr++,DECV();                     break;  //16��?????????1???
		case 0x1042: PLC_Addr++,DECP();                   break;  //16��???????????????1???
		case 0x0043: PLC_Addr++,DDEC();                    break;  //32��?????????1???
		case 0x0044: PLC_Addr++,WAND();	                   break;  //????????????
		case 0x0045: PLC_Addr++,DWAND();	                 break;  //32��????????????
		case 0x0046: PLC_Addr++,WOR();                     break;  //????????????
		case 0x0047: PLC_Addr++,DWOR();                    break;  //32��????????????
		case 0x0048: PLC_Addr++,WXOR();                    break;  //??????????????
		case 0x0049: PLC_Addr++,DWXOR();                   break;  //32��??????????????
		case 0x004A: PLC_Addr++,NEG();                     break;  //????????????
		case 0x004B: PLC_Addr++,DNEG();                    break;  //32��????????????			
		case 0x004C: PLC_Addr++,ROR();                     break;  //ROR
		case 0x004D: PLC_Addr++,DROR();                    break;  //DROR
		case 0x004E: PLC_Addr++,ROL();                     break;  //ROL
		case 0x004F: PLC_Addr++,DROL();                    break;  //DROL
		case 0x0050: PLC_Addr++,RCR();                     break;  //RCR
		case 0x0051: PLC_Addr++,DRCR();                    break;  //DRCR
		case 0x0052: PLC_Addr++,RCL();                     break;  //RCL
		case 0x0053: PLC_Addr++,DRCL();                    break;  //DRCL
			
//	case 0x0054: PLC_Addr++,SFTR();                    break;  //SFTR
		
		case 0x0060: PLC_Addr++,ZRST();                    break; 
		case 0x0062: PLC_Addr++,DECO();                    break;  //?????????    
		case 0x006A: PLC_Addr++,MEAN();                    break;	 //MEAN???????????		
		case 0x0070: PLC_Addr++,SQR();	                   break;  //SQR16��????????			
		case 0x0071: PLC_Addr++,DSQR();	                   break;  //SQR32��????????
		case 0x0072: PLC_Addr++,FLT();	                   break;  //16��?????????
		case 0x0073: PLC_Addr++,DFLT();	                   break;  //32��?????????	
		case 0x0076: PLC_Addr++,REFF();	                   break;  //REFF	
		case 0x0078: PLC_Addr++,MTR();	                   break;  //MTR
    case 0x007A: PLC_Addr++,HSCS();		                 break;  //?????????��  20160709
		
// 	case 0x0084: PLC_Addr++,PWM();                     break;  //PWM????
		case 0x0082: PLC_Addr++,PLSY();                    break;  //????????????
		case 0x0094: PLC_Addr++,ALT();	                   break;  //ALT
		case 0x00AC: PLC_Addr++,TFROM();									 break;						//- From function by Tom
		case 0x00AE: PLC_Addr++,TTO();										 break;						//- To function by Tom
//	case 0x00B4: PLC_Addr++,ASCI();	                   break;  //ASCI
		case 0x00C0: PLC_Addr++,PID();	                   break;  //PID
		case 0x00ED: PLC_Addr++,ECMP();	                   break;  //ECMP
		case 0x00EE: PLC_Addr++,EZCP();	                   break;  //EZCP
		
    case 0x00F1: PLC_Addr++,DEMOV();                   break;  //
// 	case 0x00FD: PLC_Addr++,DEBCD();	                 break;  //DEBCD

		case 0x0101: PLC_Addr++,DEADD();                   break;  //???????????
		case 0x0103: PLC_Addr++,DESUB();	                 break;  //????????????
		case 0x0107: PLC_Addr++,DEDIV();	                 break;  //???????????
		case 0x0105: PLC_Addr++,DEMUL();                   break;  //????????????
		case 0x010F: PLC_Addr++,DESQR();                   break;  //DESQR??????
		case 0x0112: PLC_Addr++,INT();                     break;  //INT
		case 0x0113: PLC_Addr++,DINT();                    break;  //DINT
		case 0x0115: PLC_Addr++,DSIN();	                   break;  //DSIN
		case 0x0117: PLC_Addr++,DCOS();	                   break;  //DCOS
		case 0x0119: PLC_Addr++,DTAN();	                   break;  //DTAN			

		case 0x0136: PLC_Addr++,SWAP();                    break;  //SWAP
		case 0x0137: PLC_Addr++,DSWAP();                   break;  //DSWAP
		
		case 0x0150: PLC_Addr++,TCMP();	                   break;  //TCMP
		case 0x0152: PLC_Addr++,TZCP();	                   break;  //TZCP
		case 0x0154: PLC_Addr++,TADD();	                   break;  //TADD
		case 0x0156: PLC_Addr++,TSUB();	                   break;  //TSUB		
		case 0x015C: PLC_Addr++,TRD();	                   break;  //TRD	
		case 0x015E: PLC_Addr++,TWR();	                   break;  //TWR	
		case 0x0164: PLC_Addr++,GRY();	                   break;  //GRY
		case 0x0165: PLC_Addr++,DGRY();	                   break;  //DGRY
		case 0x0166: PLC_Addr++,GBIN();	                   break;  //GBIN
		case 0x0167: PLC_Addr++,DGBIN();	                 break;  //DGBIN
		
		case 0x01C2: PLC_Addr++,expand_LD();               break;  //M1535????????
		case 0x01C3: PLC_Addr++,expand_LDI();              break;  //
		case 0x01C4: PLC_Addr++,expand_AND();              break;  //
		case 0x01C5: PLC_Addr++,expand_ANI();              break;  //
		case 0x01C6: PLC_Addr++,expand_OR();               break;  //
		case 0x01C7: PLC_Addr++,expand_ORI();              break;  //
		
		case 0x01CA: PLC_Addr++,LDP();			               break;  //?????????????
		case 0x01CB: PLC_Addr++,LDF();			               break;  //?????????????
		case 0x01CC: PLC_Addr++,ANDP();			               break;  //?????????????
		case 0x01CD: PLC_Addr++,ANDF();			               break;  //?????????????
		case 0x01CE: PLC_Addr++,ORP();			               break;  //?????????????
		case 0x01CF: PLC_Addr++,ORF();			               break;  //?????????????
		
		case 0X01D0: PLC_Addr++,amount();                  break;  //LD 16��??????
		case 0X01D1: PLC_Addr++,Damount();                 break;  //LD 32��??????
		case 0X01D2: PLC_Addr++,big();                     break;  //LD 16��??????
		case 0X01D3: PLC_Addr++,Dbig();                    break;  //LD 32��??????
		case 0X01D4: PLC_Addr++,less();                    break;  //LD 16�˧�????
		case 0X01D5: PLC_Addr++,Dless();                   break;  //LD 32�˧�????
		case 0X01D8: PLC_Addr++,no_amount();	             break;  //LD 16��???????????
		case 0X01D9: PLC_Addr++,Dno_amount();	             break;  //LD 32��???????????
		case 0X01DA: PLC_Addr++,less_amount();             break;  //LD 16�˧�???????
		case 0X01DB: PLC_Addr++,Dless_amount();            break;  //LD 32�˧�???????
		case 0X01DC: PLC_Addr++,big_amount();              break;  //LD 16��?????????
		case 0X01DD: PLC_Addr++,Dbig_amount();             break;  //LD 32��?????????
		
		case 0X01E0: PLC_Addr++,amount_and();              break;  //LD AND 16��??????
		case 0X01E1: PLC_Addr++,Damount_and();             break;  //LD AND 32��??????
		case 0X01E2: PLC_Addr++,big_and();                 break;  //LD AND 16��??????
		case 0X01E3: PLC_Addr++,Dbig_and();                break;  //LD AND 32��??????
		case 0X01E4: PLC_Addr++,less_and();                break;  //LD AND 16�˧�????
		case 0X01E5: PLC_Addr++,Dless_and();               break;  //LD AND 32�˧�????
		case 0X01E8: PLC_Addr++,no_amount_and(); 	         break;  //LD 16��???????????
		case 0X01E9: PLC_Addr++,Dno_amount_and(); 	       break;  //LD 32��???????????
		case 0X01EA: PLC_Addr++,less_amount_and();         break;  //LD AND 16�˧�???????
		case 0X01EB: PLC_Addr++,Dless_amount_and();        break;  //LD AND 32�˧�???????
		case 0X01EC: PLC_Addr++,big_amount_and();          break;  //LD AND 16��?????????
		case 0X01ED: PLC_Addr++,Dbig_amount_and();         break;  //LD AND 32��?????????
		
		case 0X01F0: PLC_Addr++,amount_OR();               break;  //LD OR 16��??????
		case 0X01F1: PLC_Addr++,Damount_OR();              break;  //LD OR 32��??????
		case 0X01F2: PLC_Addr++,big_OR();                  break;  //LD OR 16��??????
		case 0X01F3: PLC_Addr++,Dbig_OR();                 break;  //LD OR 32��??????
		case 0X01F4: PLC_Addr++,less_OR();                 break;  //LD OR 16�˧�????
		case 0X01F5: PLC_Addr++,Dless_OR();                break;  //LD OR 32�˧�????
		case 0X01F8: PLC_Addr++,no_amount_OR(); 	         break;  //LD 16��???????????
		case 0X01F9: PLC_Addr++,Dno_amount_OR(); 	         break;  //LD 32��???????????
		case 0X01FA: PLC_Addr++,less_amount_OR();          break;  //LD OR 16�˧�???????
		case 0X01FB: PLC_Addr++,Dless_amount_OR();         break;  //LD OR 32�˧�???????
		case 0X01FC: PLC_Addr++,big_amount_OR();           break;  //LD OR 16��?????????
		case 0X01FD: PLC_Addr++,Dbig_amount_OR();          break;  //LD OR 32��?????????					
		
		case 0x000F: PLC_Addr=PLC_Addr;                    break;  //???????END??????????????
		case 0XF7FF: PLC_Addr++,RET();                     break;  //RET
		
		default:PLC_PROG_ERROR(M8065,02); PLC_Addr++;      break;  //??????????????
	}
}
 
void find_p(void)//Find the address where P is located
{  
	u16 temp;
	PLC_Addr=PLC_START_Address;
	uint16_t loop,tick;
	for(temp=0;temp<3999;temp++)//16000 steps in total			//- 15999 By tom 6-9-2022
	{ 
		if((*PLC_Addr&0xFF00)==0xB000){
			PLC_P_Addr[*PLC_Addr%0x100]=PLC_Addr;
			tick = loop;
		}
		PLC_Addr++;
		loop++;
	}
}

void RST_Y(void){
	PLC_RAM[160] = PLC_RAM[161] = PLC_RAM[162] = PLC_RAM[163] = 0;
}

u16 find_toend(void)//Find the address where P is located
{  
	u16 temp;
	PLC_Addr=PLC_START_Address-1;
	temp=0;	
	do{
			PLC_Addr++; 
			temp++;
		}while((!(*PLC_Addr==0x000f))&&(temp<3998)); 	//- 15998 - By tom 6-9-2022 
	return temp; 
}

//Xiao Xiaosheng, optimized in 20160929
void PLC_ProInstructParse(void){ 
	static u8  puls,run_flag; 
	uint16_t PLCloop=0;
  if(PLC_RUN){																			//Do you need to run the program
		if(RAM_8BIT(0x01E0)==0x09&&!loadMemSts){				//Load retentive data every time mode change to RUN
			//loadDMemory();
			loadMemSts = true;
		}else if(RAM_8BIT(0x01E0)!=0x09&&loadMemSts){
			loadMemSts = false;
		}

    if(run_flag == 1){
	    run_flag = 0;
			RAM_8BIT(0x01E0) = 0x09;
	  }
		
  	if(RAM_8BIT(0x01E0)==0x09){											//Do you need to run the program
			PLC_BIT_ON(M8000);								//Run to force m80000 to on
			PLC_BIT_OFF(M8001);								//Run to force m80001 to off
			//TOGGLE_RUN();
			RUN_ON();
			//statusRUN(true,5);
			//STOP_WAR;
			
			if(edit_prog==0x00){							//Determine whether there is program editing, if you edit the program once, you must recalculate the address where p is located
				find_p();
				edit_prog=1;
				if(find_toend()>3998){  				//- 15998 - By tom 6-9-2022
					RAM_8BIT(0x01E0) = 0x09;
					goto all_end;  
				}
			}
		
			if(puls==0x00){										//The initialization pulse is used 8002 8003
				PLC_BIT_ON(M8002);
				PLC_BIT_OFF(M8003);
				//loadDMemory();
			}

			if(Write_Pro_flag == 0){
				PLC_IO_Refresh();								//refresh y output
			} 

			PLC_Addr = PLC_START_Address;			//plc to start address
			//mbSetStack = mbStack;
			
			do{
				//Serial2.printf("Ins: %X\n",*PLC_Addr/0x100);
				//delay(1500);
				switch(*PLC_Addr/0x100)					//Get the upper 8 bits of data
				{ 
					case 0x06: operation_T(),PLC_Addr++;                    break;  //operation all timer
					case 0x0E: operation_C(),PLC_Addr++;                    break;  //			
				//Operate all functions of the S-bit element
					case 0x20: LD(0X0FFF&*PLC_Addr),PLC_Addr++;             break;  //
					case 0x30: LDI(0X0FFF&*PLC_Addr),PLC_Addr++;            break;  //
					case 0x40: AND(0X0FFF&*PLC_Addr),PLC_Addr++;            break;  //
					case 0x50: ANI(0X0FFF&*PLC_Addr),PLC_Addr++;            break;  //
					case 0x60: OR(0X0FFF&*PLC_Addr),PLC_Addr++;             break;  //
					case 0x70: ORI(0X0FFF&*PLC_Addr),PLC_Addr++;            break;	//
				//Operate all functions of the S-bit element
					case 0x21: LD(0X0FFF&*PLC_Addr),PLC_Addr++;             break;  //
					case 0x31: LDI(0X0FFF&*PLC_Addr),PLC_Addr++;            break;  //
					case 0x41: AND(0X0FFF&*PLC_Addr),PLC_Addr++;            break;  //
					case 0x51: ANI(0X0FFF&*PLC_Addr),PLC_Addr++;            break;  //
					case 0x61: OR(0X0FFF&*PLC_Addr),PLC_Addr++;             break;  //
					case 0x71: ORI(0X0FFF&*PLC_Addr),PLC_Addr++;            break;//
				//Operate all functions of the S-bit element
					case 0x22: LD(0X0FFF&*PLC_Addr),PLC_Addr++;             break;  //
					case 0x32: LDI(0X0FFF&*PLC_Addr),PLC_Addr++;            break;  //
					case 0x42: AND(0X0FFF&*PLC_Addr),PLC_Addr++;            break;  //
					case 0x52: ANI(0X0FFF&*PLC_Addr),PLC_Addr++;            break;  //
					case 0x62: OR(0X0FFF&*PLC_Addr),PLC_Addr++;             break;  //
					case 0x72: ORI(0X0FFF&*PLC_Addr),PLC_Addr++;            break;//
				//Operate all functions of the S-bit element
					case 0x23: LD(0X0FFF&*PLC_Addr),PLC_Addr++;             break;  //
					case 0x33: LDI(0X0FFF&*PLC_Addr),PLC_Addr++;            break;  //
					case 0x43: AND(0X0FFF&*PLC_Addr),PLC_Addr++;            break;  //
					case 0x53: ANI(0X0FFF&*PLC_Addr),PLC_Addr++;            break;  //
					case 0x63: OR(0X0FFF&*PLC_Addr),PLC_Addr++;             break;  //
					case 0x73: ORI(0X0FFF&*PLC_Addr),PLC_Addr++;            break;//
				//All functions that operate on X-bit components
					case 0x24: LD(0X0FFF&*PLC_Addr),PLC_Addr++;             break;  //
					case 0x34: LDI(0X0FFF&*PLC_Addr),PLC_Addr++;            break;  //
					case 0x44: AND(0X0FFF&*PLC_Addr),PLC_Addr++;            break;  //
					case 0x54: ANI(0X0FFF&*PLC_Addr),PLC_Addr++;            break;  //
					case 0x64: OR(0X0FFF&*PLC_Addr),PLC_Addr++;             break;  //
					case 0x74: ORI(0X0FFF&*PLC_Addr),PLC_Addr++;            break;//
				//All functions that operate on Y-bit components
					case 0x25: LD(0X0FFF&*PLC_Addr),PLC_Addr++;             break;  //
					case 0x35: LDI(0X0FFF&*PLC_Addr),PLC_Addr++;            break;  //
					case 0x45: AND(0X0FFF&*PLC_Addr),PLC_Addr++;            break;
					case 0x55: ANI(0X0FFF&*PLC_Addr),PLC_Addr++;            break;
					case 0x65: OR(0X0FFF&*PLC_Addr),PLC_Addr++;             break;  //
					case 0x75: ORI(0X0FFF&*PLC_Addr),PLC_Addr++;            break;  //
					case 0XC5: OUT(0X0FFF&*PLC_Addr),PLC_Addr++;			      break;  //
					case 0XD5: BIT_SET(0X0FFF&*PLC_Addr),PLC_Addr++;	      break;  //
					case 0XE5: RST(0X0FFF&*PLC_Addr),PLC_Addr++;			      break;//
				//All functions that operate on T-bit components
					case 0x26: LD(0X0FFF&*PLC_Addr),PLC_Addr++;             break;  //
					case 0x36: LDI(0X0FFF&*PLC_Addr),PLC_Addr++;            break;  //
					case 0x46: AND(0X0FFF&*PLC_Addr),PLC_Addr++;            break;
					case 0x56: ANI(0X0FFF&*PLC_Addr),PLC_Addr++;            break;
					case 0x66: OR(0X0FFF&*PLC_Addr),PLC_Addr++;             break;  //
					case 0x76: ORI(0X0FFF&*PLC_Addr),PLC_Addr++;            break;  //
					case 0XC6: OUT(0X0FFF&*PLC_Addr),PLC_Addr++;			      break;//
				//All functions that operate on T-bit components
					case 0x27: LD(0X0FFF&*PLC_Addr),PLC_Addr++;             break;  //
					case 0x37: LDI(0X0FFF&*PLC_Addr),PLC_Addr++;            break;  //
					case 0x47: AND(0X0FFF&*PLC_Addr),PLC_Addr++;            break;
					case 0x57: ANI(0X0FFF&*PLC_Addr),PLC_Addr++;            break;
					case 0x67: OR(0X0FFF&*PLC_Addr),PLC_Addr++;             break;  //
					case 0x77: ORI(0X0FFF&*PLC_Addr),PLC_Addr++;            break;  //
					case 0XC7: OUT(0X0FFF&*PLC_Addr),PLC_Addr++;			      break;//
				//Operate all functions of M0_255 bit element
					case 0x28: LD(0X0FFF&*PLC_Addr),PLC_Addr++;             break;  //
					case 0x38: LDI(0X0FFF&*PLC_Addr),PLC_Addr++;            break;  //
					case 0x48: AND(0X0FFF&*PLC_Addr),PLC_Addr++;            break;
					case 0x58: ANI(0X0FFF&*PLC_Addr),PLC_Addr++;            break;
					case 0x68: OR(0X0FFF&*PLC_Addr),PLC_Addr++;             break;  //
					case 0x78: ORI(0X0FFF&*PLC_Addr),PLC_Addr++;            break;  //
					case 0XC8: OUT(0X0FFF&*PLC_Addr),PLC_Addr++;			      break;  //
					case 0XD8: BIT_SET(0X0FFF&*PLC_Addr),PLC_Addr++;	      break;  //
					case 0XE8: RST(0X0FFF&*PLC_Addr),PLC_Addr++;			      break;//
				//Operate all functions of M256_511 bit components
					case 0x29: LD(0X0FFF&*PLC_Addr),PLC_Addr++;             break;  //
					case 0x39: LDI(0X0FFF&*PLC_Addr),PLC_Addr++;            break;  //
					case 0x49: AND(0X0FFF&*PLC_Addr),PLC_Addr++;            break;
					case 0x59: ANI(0X0FFF&*PLC_Addr),PLC_Addr++;            break;
					case 0x69: OR(0X0FFF&*PLC_Addr),PLC_Addr++;             break;  //
					case 0x79: ORI(0X0FFF&*PLC_Addr),PLC_Addr++;            break;  //
					case 0XC9: OUT(0X0FFF&*PLC_Addr),PLC_Addr++;			      break;  //
					case 0XD9: BIT_SET(0X0FFF&*PLC_Addr),PLC_Addr++;	      break;  //
					case 0XE9: RST(0X0FFF&*PLC_Addr),PLC_Addr++;			      break;//
				//Operate all functions of M512_767 bit components
					case 0x2A: LD(0X0FFF&*PLC_Addr),PLC_Addr++;             break;  //
					case 0x3A: LDI(0X0FFF&*PLC_Addr),PLC_Addr++;            break;  //
					case 0x4A: AND(0X0FFF&*PLC_Addr),PLC_Addr++;            break;
					case 0x5A: ANI(0X0FFF&*PLC_Addr),PLC_Addr++;            break;
					case 0x6A: OR(0X0FFF&*PLC_Addr),PLC_Addr++;             break;  //
					case 0x7A: ORI(0X0FFF&*PLC_Addr),PLC_Addr++;            break;  //
					case 0XCA: OUT(0X0FFF&*PLC_Addr),PLC_Addr++;			      break;  //
					case 0XDA: BIT_SET(0X0FFF&*PLC_Addr),PLC_Addr++;	      break;  //
					case 0XEA: RST(0X0FFF&*PLC_Addr),PLC_Addr++;			      break;//
				//Operate all functions of M768_1023 bit components
					case 0x2B: LD(0X0FFF&*PLC_Addr),PLC_Addr++;             break;  //
					case 0x3B: LDI(0X0FFF&*PLC_Addr),PLC_Addr++;            break;  //
					case 0x4B: AND(0X0FFF&*PLC_Addr),PLC_Addr++;            break;
					case 0x5B: ANI(0X0FFF&*PLC_Addr),PLC_Addr++;            break;
					case 0x6B: OR(0X0FFF&*PLC_Addr),PLC_Addr++;             break;  //
					case 0x7B: ORI(0X0FFF&*PLC_Addr),PLC_Addr++;            break;  //
					case 0XCB: OUT(0X0FFF&*PLC_Addr),PLC_Addr++;			      break;  //
					case 0XDB: BIT_SET(0X0FFF&*PLC_Addr),PLC_Addr++;	      break;  //
					case 0XEB: RST(0X0FFF&*PLC_Addr),PLC_Addr++;			      break;//
				//Operate all functions of the M1024_1279 bit element
					case 0x2C: LD(0X0FFF&*PLC_Addr),PLC_Addr++;             break;  //
					case 0x3C: LDI(0X0FFF&*PLC_Addr),PLC_Addr++;            break;  //
					case 0x4C: AND(0X0FFF&*PLC_Addr),PLC_Addr++;            break;
					case 0x5C: ANI(0X0FFF&*PLC_Addr),PLC_Addr++;            break;
					case 0x6C: OR(0X0FFF&*PLC_Addr),PLC_Addr++;             break;  //
					case 0x7C: ORI(0X0FFF&*PLC_Addr),PLC_Addr++;            break;  //
					case 0XCC: OUT(0X0FFF&*PLC_Addr),PLC_Addr++;			      break;  //
					case 0XDC: BIT_SET(0X0FFF&*PLC_Addr),PLC_Addr++;	      break;  //
					case 0XEC: RST(0X0FFF&*PLC_Addr),PLC_Addr++;			      break;//
				//Operate all functions of the M1280_1535 bit element
					case 0x2D: LD(0X0FFF&*PLC_Addr),PLC_Addr++;             break;  //
					case 0x3D: LDI(0X0FFF&*PLC_Addr),PLC_Addr++;            break;  //
					case 0x4D: AND(0X0FFF&*PLC_Addr),PLC_Addr++;            break;
					case 0x5D: ANI(0X0FFF&*PLC_Addr),PLC_Addr++;            break;
					case 0x6D: OR(0X0FFF&*PLC_Addr),PLC_Addr++;             break;  //
					case 0x7D: ORI(0X0FFF&*PLC_Addr),PLC_Addr++;            break;  //
					case 0XCD: OUT(0X0FFF&*PLC_Addr),PLC_Addr++;			      break;  //
					case 0XDD: BIT_SET(0X0FFF&*PLC_Addr),PLC_Addr++;	      break;  //
					case 0XED: RST(0X0FFF&*PLC_Addr),PLC_Addr++;			      break;//
				//Operate all functions of C0-C255 bit components
					case 0x2E: LD(0X0FFF&*PLC_Addr),PLC_Addr++;             break;  //
					case 0x3E: LDI(0X0FFF&*PLC_Addr),PLC_Addr++;            break;  //
					case 0x4E: AND(0X0FFF&*PLC_Addr),PLC_Addr++;            break;
					case 0x5E: ANI(0X0FFF&*PLC_Addr),PLC_Addr++;            break;
					case 0x6E: OR(0X0FFF&*PLC_Addr),PLC_Addr++;             break;  //
					case 0x7E: ORI(0X0FFF&*PLC_Addr),PLC_Addr++;            break;  //
				//m8000-m8255
					case 0x2F: LD(0X0FFF&*PLC_Addr),PLC_Addr++;             break;  //
					case 0x3F: LDI(0X0FFF&*PLC_Addr),PLC_Addr++;            break;  //
					case 0x4F: AND(0X0FFF&*PLC_Addr),PLC_Addr++;            break;
					case 0x5F: ANI(0X0FFF&*PLC_Addr),PLC_Addr++;            break;
					case 0x6F: OR(0X0FFF&*PLC_Addr),PLC_Addr++;             break;  //
					case 0x7F: ORI(0X0FFF&*PLC_Addr),PLC_Addr++;            break;  //
					case 0XCF: OUT(0X0FFF&*PLC_Addr),PLC_Addr++;			      break;  //
					case 0XDF: BIT_SET(0X0FFF&*PLC_Addr),PLC_Addr++;	      break;  //
					case 0XEF: RST(0X0FFF&*PLC_Addr),PLC_Addr++;			      break;//
				//***********************STL stepping mode***************************
					case 0xF0: STL(0X0FFF&*PLC_Addr),PLC_Addr++;            break;  //S
					case 0xF1: STL(0X0FFF&*PLC_Addr),PLC_Addr++;            break;  //
					case 0xF2: STL(0X0FFF&*PLC_Addr),PLC_Addr++;            break;  //
					case 0xF3: STL(0X0FFF&*PLC_Addr),PLC_Addr++;            break;  //
					
				///////////////////////////////////////////////////
				//Basic logic instruction two, such as ANB, ORB, MPP, MRD, MPS, INV, etc.
					case 0XFF: 
					{
						other_function(*PLC_Addr);
						PLC_Addr++;	      
						break;   //MPP,MPS
					}
					case 0xB0://pointer p identifier
					{
						PLC_Addr++;                                  
						break;  
					}
					case 0x00://Encounter 0 x001 c is fend, 0 x000 f is end command
					{
						if(((*PLC_Addr%0x100)==0x1C)||((*PLC_Addr%0x100)==0x0F)){
							goto all_end;
						}
					}
					
				//////////////////////////////////////////////////////////////
				// Xiao Xiaosheng, 20160929 Notes, Application Instructions				
				//////////////////////////////////////////////////////////////					
					default://When encountering an unsupported command, you need to execute the command with 16bit command here
					{
						//Serial.printf("Curr-inst-addr: %X \n",PLC_Addr);  
						//Serial.printf("Curr-inst: %X \n",*PLC_Addr); 
						FNC_AppInstruct();                         
						break; 
					}
				}
				PLCloop++;
			}while(1);
			
			all_end: 
			D8010=D8011=D8012=PLC_RUN_TIME;				//keep scan time
			PLC_RUN_TIME=0;												//clear scan time
			puls=0x01;		 
			PLC_BIT_OFF(M8002),PLC_BIT_ON(M8003); //The initialization pulse is used 8002 8003
	 	}
		else{ 
			PLC_BIT_OFF(M8000);	              //No operation forces M80000 to OFF
			PLC_BIT_ON(M8001);	              //Not running force M80001 to on
			D8012=0; 
			edit_prog=0;	                    //used in programming
			puls=0; 		                      //The initialization pulse is used 8002 8003
			RUN_OFF();                        	//- Turn off the running light, if you switch from running state to stop state, you need to clear the Y output
			//statusRUN(true,1500);

			if(Write_Pro_flag == 0)
			{
				RST_Y(); 
				PLC_IO_Refresh();               //refresh Y output  
				RST_T_D_C_M_data();
				//loadDMemory();
			}  
			PLC_STL_CMD = PLC_STL_Status = 0;	  //Step in last program
		}
	}
	else{
		RST_Y();														//If you switch from running to stopped, you need to clear the y output
		PLC_BIT_OFF(M8000);									//No running forces m80000 to off
		PLC_BIT_ON(M8001);	               	//Not running force M80001 to on
		D8012=0; 
		edit_prog=0;	                     	//used in programming
		puls=0; 													 	//The initialization pulse is used 8002 8003		                                          
		RUN_OFF();                         		//Turn off the running lights
		Write_Pro_flag = 0;
		PLC_IO_Refresh();

		PLC_STL_CMD = PLC_STL_Status = 0;	
		if(run_flag == 0){
			run_flag = 1;
			PLC_RUN_TIME=0;                      
			RST_T_D_C_M_data();
			//loadDMemory();
		}
	}
	RAM_16BIT(0x0E01) = 0x000;		         						//set version number
} 
