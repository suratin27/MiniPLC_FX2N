/************************************************************
Library name: PLC_Dialogue
Function description: PLC macro processing
Author: Biography
Date: July 22, 2015
Version: 1.1
************************************************************/
#ifndef __PLC_Dialogue_H__
#define __PLC_Dialogue_H__
//#include "stm32f10x.h"
#include "tftype_def.h"

#ifdef __cplusplus
extern "C"  {
#endif
extern const unsigned char PLC_BIT_OR[];
extern const unsigned char PLC_BIT_AND[];
//extern u8  PLC_8BIT[4];								//plc ram running data
extern u8  Step_Address[2000];				//ΪPLS OR PLF use
#ifdef __cplusplus
}
#endif

//#define PLC_MEM					0x3FFB7EC8		//- Flash Memory to be use to Store Ladder Command
//#define PLC_RAM_ADDR    PLC_MEM		//0x8006000 - The starting address must be 0X800000 and start with an integer multiple of 0X1000, because block erase is required when writing PLC programs later
//#define RAM							0x3FFB2EC8
#define RAM_ADDR        PLC_RAM						//PLC RAM start address
#define RAM_T_ADDR      PLC_RAM+0x1000		//PLC T start address
#define RAM_C_ADDR      PLC_RAM+0x2000		//PLC C start address
#define RAM_D_ADDR      PLC_RAM+0x3000 		//PLC D start address
#define RAM_D1000_ADDR  PLC_RAM+0x37D0		//PLC D1000 start address
//#define RAM_D8000_ADDR  PLC_RAM+0x1E00		//PLC D8000 start address
#define RAM_D8000_ADDR  PLC_RAM+0x0E00		//PLC D8000 start address

#define PLC_START_Address   (u16*)PLC_FLASH+0x2E		//0x800605C - Plc program start address
//#define PLC_START_Address   ((u16*)(PLC_FLASH+0x5C))		//0x800605C - Plc program start address

#define ROTATE_LEFT(x, s, n)        ((x) << (n)) | ((x) >> ((s) - (n)))					//Circular shift left x is the data s is the number of data bits n is the number of shifts
#define ROTATE_RIGHT(x, s, n)       ((x) >> (n)) | ((x) << ((s) - (n)))					//Rotate right x is the data s is the number of data bits n is the number of shifts

#define swap_u16(x)       	((x) >> (8)) | ((x) << (8))										//transfer up and down
#define swap_u32(x)       	((x) >> (16))|((x) << (16))										//transfer up and down

#define	PLC_D_C_T_addr(x)  	((x) % (0x4000))						//v,z used when

#define	PLC_v_z_addr(x)    	((x) / (0x4000))						//v,z used when

#define PLC_RAMfolta(x)   	(*(float*)(x))						//(*(float*)(u32)(x)) -	Word mode R/W RAM R ROM

#define trade1         			FLOAT.DATA								//Use when floating point output

#define ON  1

#define	OFF 0

typedef union
{
	struct{
		u8 bit0			:1;
		u8 bit1			:1;
		u8 bit2			:1;
		u8 bit3			:1;
		u8 bit4			:1;
		u8 bit5			:1;
		u8 bit6			:1;
		u8 bit7			:1;
		u8 bit10		:1;
		u8 bit11		:1;
		u8 bit12		:1;
		u8 bit13		:1;
		u8 bit14		:1;
		u8 bit15		:1;	
		u8 bit16		:1;
		u8 bit17		:1;
	}bits;												//can be addressed by bit field
  u16 bytes;										//can be addressed by byte
}bit_byte;											//define a new variable type that can be addressed both by bit field and by byte

/*
*16-bit integer community
*/
/*
typedef union
{
  u8  PLC_8BIT[24200];
	u16 PLC_16BIT[12100];
} union_16BIT;
*/
/*
*Floating point community
*/
typedef union                                
{
  float DATA;
	u16   DATA1[2];
	u32   bata;
} float_union;

/*
*32-bit integer community
*/
typedef union
{
  s32 data;
	s16 data1[2];
} s32_union;

/*
*64-bit integer community
*/
typedef union
{
  int64_t data;
	u16 data1[4];
} u64_union;


/*--------------------------------------------------------------------
																By TF
--------------------------------------------------------------------*/
typedef union{
	struct{
		u8 bit0			:1;
		u8 bit1			:1;
		u8 bit2			:1;
		u8 bit3			:1;
		u8 bit4			:1;
		u8 bit5			:1;
		u8 bit6			:1;
		u8 bit7			:1;
		u8 bit10		:1;
		u8 bit11		:1;
		u8 bit12		:1;
		u8 bit13		:1;
		u8 bit14		:1;
		u8 bit15		:1;	
	}bit;
	u8 byte[2];
	u16 bytes;
}PLC_data;

typedef union{
	struct{
		u8 bit0			:1;
		u8 bit1			:1;
		u8 bit2			:1;
		u8 bit3			:1;
		u8 bit4			:1;
		u8 bit5			:1;
		u8 bit6			:1;
		u8 bit7			:1;
		u8 bit10		:1;
		u8 bit11		:1;
		u8 bit12		:1;
		u8 bit13		:1;
		u8 bit14		:1;
		u8 bit15		:1;	
	}bit;
	u8 byte[2];
	u16 bytes;
}IO;

extern u8 PLC_FLASH[];
extern u8 PLC_RAM[];
/*
#define RAM_ADDR        PLC_RAM						//0x3FFB2EC8		//0x20001000 - plc ram start address
#define RAM_T_ADDR      PLC_RAM+0x1000		//0x3FFB3EC8		//0x20002000 - PLC T start address
#define RAM_C_ADDR      PLC_RAM+0x2000		//0x3FFB24C8		//0x20001A00 - PLC C start address
#define RAM_D_ADDR      PLC_RAM+0x3000 		//0x3FFB4EC8		//0x20003000 - PLC D start address
#define RAM_D1000_ADDR  PLC_RAM+0x37D0		//0x3FFB5698		//0x200037D0 - PLC D1000 start address
#define RAM_D8000_ADDR  PLC_RAM+0x0E00		//0x3FFB3CC8		//0x20001E00 - PLC D8000 start address
*/
#define MEMORY_8BIT(x)			(*((u8*)(x)))
#define MEMORY_16BIT(x)			(*((u16*)(x)))
#define MEMORY_32BIT(x)			(*((u32*)(x)))
#define MEMORY_64BIT(x)			(*((u64*)(x)))
#define MEMORY_32BITF(x)		(*((float*)(x)))
#define FLASH_8BIT(x)				MEMORY_8BIT(PLC_FLASH+x)
#define FLASH_16BIT(x)			MEMORY_16BIT(PLC_FLASH+x)
#define RAM_8BIT(x)					MEMORY_8BIT(PLC_RAM+x)
#define RAM_16BIT(x)				MEMORY_16BIT(PLC_RAM+x)
#define RAM_32BIT(x)				MEMORY_32BIT(PLC_RAM+x)
#define RAM_64BIT(x)				MEMORY_64BIT(PLC_RAM+x)
#define RAM_32BITF(x)				MEMORY_32BITF(PLC_RAM+x)

#define PLC_BIT_TEST(x)	    (MEMORY_8BIT(PLC_RAM+((x)/8))	& PLC_BIT_OR[(x)%8])
#define PLC_BIT_ON(x)		  	(MEMORY_8BIT(PLC_RAM+((x)/8))	|=PLC_BIT_OR[(x)%8])
#define PLC_BIT_OFF(x)			(MEMORY_8BIT(PLC_RAM+((x)/8))	&=PLC_BIT_AND[(x)%8])
#define PLC_PL_BIT_ON(x)	 	(Step_Address[(x)/8] |=PLC_BIT_OR[(x)%8])						//Write the status value to 1 and use the rising and falling edges
#define PLC_PL_BIT_OFF(x)	 	(Step_Address[(x)/8] &=PLC_BIT_AND[(x)%8])

#endif
