#include "GPIO.h"
#include "LCD.h"
#include "LCD_CFG.h"

#include <stdint.h>

static void Delay(unsigned int DelayVal);
static void SendCmd(unsigned char CmdId);
static void SendData(unsigned char Data);

#if(DATA_MODE == DATA_MODE_8)
static const unsigned int DataPinsIdx[8] = 
{ 
	DB0_GROUP_IDX, DB1_GROUP_IDX, DB2_GROUP_IDX, DB3_GROUP_IDX, DB4_GROUP_IDX, DB5_GROUP_IDX, DB6_GROUP_IDX, DB7_GROUP_IDX
};
#else
static const unsigned int DataPinsIdx[4] = 
{
	DB4_GROUP_IDX, DB5_GROUP_IDX, DB6_GROUP_IDX, DB7_GROUP_IDX
};
#endif
static int InitState = 0;

void LCD_Init(void)
{
	Delay(15000);
	#if DATA_MODE == DATA_MODE_4
	SendCmd(0x33);
	SendCmd(0x32);
	SendCmd(0x28);
	#else
	SendCmd(0x38);
	#endif
	#if CURSOR_MODE == CURSOR_OFF
	SendCmd(0x0C);
	#else
	SendCmd(0x0E);
	#endif
	SendCmd(0x01);
	InitState = 1;
}
LCD_ChkType LCD_DispChar(char Data)
{
	LCD_ChkType RetVar;
	if(InitState == 1)
	{
		SendData(Data);
		RetVar = LCD_OK;
	}
	else
	{
		RetVar = LCD_NOK;
	}
	return RetVar;
}
LCD_ChkType LCD_SetCursPos(int x,int y)
{
	LCD_ChkType RetVar;
	if(y == 0 && InitState == 1)
	{
		SendCmd(0x80 + x);
		RetVar = LCD_OK;
	}
	else if(InitState == 1)
	{
		SendCmd(0xC0 + x);
		RetVar = LCD_OK;
	}
	else
	{
		RetVar = LCD_NOK;
	}
	return RetVar;
}
LCD_ChkType LCD_Clear(void)
{
	LCD_ChkType RetVar;
	if(InitState == 1)
	{
		SendCmd(0x01);
		RetVar = LCD_OK;
	}
	else
	{
		RetVar = LCD_NOK;
	}
	return RetVar;
}
LCD_ChkType LCD_DispString(uint8_t* StrPtr)
{
	LCD_ChkType RetVar;
	if(InitState == 1)
	{
		while((*StrPtr) != '\0')
		{
			SendData(*StrPtr);
			StrPtr ++;
		}
		RetVar = LCD_OK;	
	}
	else
	{
		RetVar = LCD_NOK;
	}
	return RetVar;
}

static void SendCmd(unsigned char CmdId)
{
	int i;
	unsigned char GpioData,Cmd1;
	GPIO_Write(RS_GROUP_IDX,0x00);
	GPIO_Write(RW_GROUP_IDX,0x00);
	#if(DATA_MODE == DATA_MODE_8)
	for(i = 0; i < 8; i++)
	{
       GpioData = CmdId & 0x01;
       GpioData = (~GpioData)+ 1;
       CmdId = CmdId >> 1;
       GPIO_Write(DataPinsIdx[i],GpioData);
	}
    GPIO_Write(EN_GROUP_IDX,0xff);
    Delay(100);
    GPIO_Write(EN_GROUP_IDX,0x00);
	#else
	Cmd1 = (CmdId & 0xF0) >> 4;
	for(i = 0; i < 4; i++)
	{
		GpioData = Cmd1 & 0x01;
		GpioData = (~GpioData) + 1;
		Cmd1 = Cmd1 >> 1;
		GPIO_Write(DataPinsIdx[i],GpioData);
	}
	GPIO_Write(EN_GROUP_IDX,0xff);
	Delay(100);
	GPIO_Write(EN_GROUP_IDX,0x00);
	Cmd1 = (CmdId & 0x0f);
	for(i = 0; i < 4; i++)
	{
		GpioData = Cmd1 & 0x01;
		GpioData = (~GpioData) + 1;
		Cmd1 = Cmd1 >> 1;
		GPIO_Write(DataPinsIdx[i],GpioData);
	}
	GPIO_Write(EN_GROUP_IDX,0xff);
	Delay(100);
	GPIO_Write(EN_GROUP_IDX,0x00);
	#endif
	Delay(2000);
}
static void SendData(unsigned char DataId)
{
	int i;
	unsigned char GpioData,Cmd1;
	GPIO_Write(RS_GROUP_IDX,0xff);
	GPIO_Write(RW_GROUP_IDX,0x00);
	#if(DATA_MODE == DATA_MODE_8)
	for(i = 0; i < 8; i++)
	{
		GpioData = DataId & 0x01;
		GpioData = (~GpioData)+ 1;
		DataId = DataId >> 1;
		GPIO_Write(DataPinsIdx[i],GpioData);
	}
	GPIO_Write(EN_GROUP_IDX,0xff);
	Delay(100);
	GPIO_Write(EN_GROUP_IDX,0x00);
	#else
	Cmd1 = (DataId & 0xF0) >> 4;
	for(i = 0; i < 4; i++)
	{
		GpioData = Cmd1 & 0x01;
		GpioData = (~GpioData) + 1;
		Cmd1 = Cmd1 >> 1;
		GPIO_Write(DataPinsIdx[i],GpioData);
	}
	GPIO_Write(EN_GROUP_IDX,0xff);
	Delay(100);
	GPIO_Write(EN_GROUP_IDX,0x00);
	Cmd1 = (DataId & 0x0f);
	for(i = 0; i < 4; i++)
	{
		GpioData = Cmd1 & 0x01;
		GpioData = (~GpioData) + 1;
		Cmd1 = Cmd1 >> 1;
		GPIO_Write(DataPinsIdx[i],GpioData);
	}
	GPIO_Write(EN_GROUP_IDX,0xff);
	Delay(100);
	GPIO_Write(EN_GROUP_IDX,0x00);
	#endif
	Delay(10);
}
static void Delay(unsigned int DelayVal)
{
	volatile long int i;
	for(i = 0; i < (DelayVal*4); i++);
}
