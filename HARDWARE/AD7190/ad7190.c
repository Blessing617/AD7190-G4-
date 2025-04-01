#include "ad7190.h"

#define V_Ref 3		//参考电压

#define AD7190_Write 				0
#define AD7190_Read 				1

//寄存器选择
#define ComState_register  		(0)<<3
#define Mode_register  		 	(1)<<3
#define Config_register  	 	(2)<<3
#define Data_register  	 	 	(3)<<3
#define ID_register  	 	 	(4)<<3
#define GPOCON_register  	 	(5)<<3
#define Disorders_register 		(6)<<3
#define FullScale_register 		(7)<<3

//模式寄存器
#define MODE_ADC_OneByOne 		(0)<<21
#define MODE_ADC_OnlyOne 		(1)<<21
#define MODE_ADC_Free 			(2)<<21
#define MODE_ADC_SavePower 		(3)<<21
#define MODE_ADC_AdjustZero 	(4)<<21
#define MODE_ADC_AdjustFull 	(5)<<21
#define MODE_ADC_SysAdjustZero 	(6)<<21
#define MODE_ADC_SysAdjustFull 	(7)<<21
#define MODE_MCLK_OUTosc		(0)<<18
#define MODE_MCLK_OUTclo		(1)<<18
#define MODE_MCLK_IN			(2)<<18
#define MODE_MCLK_INcloOut		(3)<<18
#define MODE_SINC3				(1)<<15
#define MODE_ENPAR				(1)<<13
#define MODE_Single				(1)<<11
#define MODE_REJ60				(1)<<10
//#define MODE_Filter_Speed		0
//配置寄存器
#define Config_Chop_EN			(1)<<23
#define Config_REFSEL_IO		(1)<<20
#define Config_Burn_EN			(1)<<7
#define Config_REFDET_EN		(1)<<6
#define Config_BUF_EN			(1)<<4
#define Config_UB_EN			(1)<<3

#define Config_Ch0_A1A2			(1)<<8
#define Config_Ch1_A3A4			(1)<<9
#define Config_Ch2_temp			(1)<<10
#define Config_Ch3_A2A2			(1)<<11
#define Config_Ch4_A1AC			(1)<<12
#define Config_Ch5_A2AC			(1)<<13
#define Config_Ch6_A3AC			(1)<<14
#define Config_Ch7_A4AC			(1)<<15

#define Config_ADC_Gain_1		0
#define Config_ADC_Gain_8		3
#define Config_ADC_Gain_16		4
#define Config_ADC_Gain_32		5
#define Config_ADC_Gain_64		6
#define Config_ADC_Gain_128		7


typedef struct {
uint32_t ADC_Mode;
uint32_t Return_state;
uint32_t ADC_SCLK;
uint32_t SINC3_EN;
uint32_t ENPAR;
uint32_t Single_EN;
uint32_t REJ60_EN;
uint32_t Filter;
}AD7190_MODE_SET;
typedef struct {
uint32_t Config_Channel;
uint32_t Config_ADC_Gain;	
uint32_t Config_Chop;			//斩波使能
uint32_t Config_REFSEL;
uint32_t Config_Burn;
uint32_t Config_REFDET;
uint32_t Config_BUF;
uint32_t Config_UB;
}AD7190_Config_SET;

uint8_t 	ADC_Channel,text;
uint8_t 	shuju;
uint32_t 	ADC_Gain;

void AD7190_Connect_Set(uint8_t Write_EN,uint8_t WR,uint8_t dat);

uint8_t WaitDataRDY(void)
{
	uint8_t buf;
	while(1)
	{
		
		AD7190_Connect_Set(0,AD7190_Read,ComState_register);
		ReadFromAD7190(1,&buf);
		
		if((buf&0x40))
		{			
			delay_ms(10);
			return 0;
		}else if(!(buf&0x80))
		{
			return 1;
		}
		delay_ms(3);
	}
}

void WriteToAD7190(unsigned char count, unsigned char *buf)
{
	unsigned	char	ValueToWrite = 0;
    unsigned	char	i = 0;
	unsigned	char	j = 0;
		
	AD7190_SCK(1);
	delay_us(1);
	AD7190_CS(1);
	delay_us(1);
	AD7190_CS(0);
	delay_us(1);

	for(i=count;i>0;i--)
 	{
	 	ValueToWrite = *(buf + i - 1);
		for(j=0; j<8; j++)
		{
			AD7190_SCK(0);
			if(0x80 == (ValueToWrite & 0x80))
			{
				AD7190_DIN(1);	  //Send one to SDO pin
			}
			else
			{
				AD7190_DIN(0);	  //Send zero to SDO pin
			}
			delay_us(1);
			AD7190_SCK(1);
			delay_us(1);
			ValueToWrite <<= 1;	//Rotate data
		}
	}
	AD7190_CS(1);
}


void ReadFromAD7190(unsigned char count, unsigned char *buf)
{
	unsigned	char	i = 0;
	unsigned	char	j = 0;
	unsigned	char  	RotateData = 0;

	AD7190_SCK(1);
	delay_us(1);
	AD7190_CS(1);
	delay_us(1);
	AD7190_CS(0);
	delay_us(1);

	for(j=count; j>0; j--)
	{
		for(i=0; i<8; i++)
		{
		    AD7190_SCK(0);
			RotateData <<= 1;		//Rotate data
			delay_us(1);
			RotateData |= AD7190_DOUT();			//Read SDI of AD7190
			AD7190_SCK(1);	
			delay_us(1);
		}
		*(buf + j - 1)= RotateData;
		RotateData=0;
	}	 
	AD7190_CS(1);
} 

unsigned long GET_AD7190(void)
{
	uint32_t  	DAC_Dat=0;
	uint8_t 	buf[3];
	
	ReadFromAD7190(3,buf);
	
		AD7190_Connect_Set(0,AD7190_Read,ComState_register);
		ReadFromAD7190(1,&text);	
		ADC_Channel=text&0x07;

	DAC_Dat=buf[2]*256*256+buf[1]*256+buf[0];
	
	return DAC_Dat;
} 

unsigned long GET_AD7190_1(void)
{
	uint32_t DAC_Dat=0;
	uint8_t buf[4];
	
	ReadFromAD7190(4,buf);
	DAC_Dat=buf[3]*256*256*256+buf[2]*256*256+buf[1]*256+buf[0];
	
	return DAC_Dat;
}

void AD7190_Connect_Set(uint8_t Write_EN,uint8_t WR,uint8_t dat)
{
	uint8_t buf=dat;
	
	if(Write_EN)	buf=1<<7;
	if(WR)			buf=1<<6;
	WriteToAD7190(1,&buf);
}

void AD7190_Mode_Set(AD7190_MODE_SET *Mode)
{
	uint32_t buf=Mode->ADC_Mode;
	uint8_t Write_dat[3],Dat;
	buf|=Mode->ADC_SCLK;
	buf|=Mode->Filter;
	if(Mode->Return_state)buf|=(1<<20);
	if(Mode->SINC3_EN)buf|=MODE_SINC3;
	if(Mode->ENPAR)buf|=MODE_ENPAR;
	if(Mode->Single_EN)buf|=MODE_Single;
	if(Mode->REJ60_EN)buf|=MODE_REJ60;
	Write_dat[2]=buf>>16;
	Write_dat[1]=buf>>8;
	Write_dat[0]=buf;
	Dat=Mode_register;
	WriteToAD7190(1,&Dat);
	WriteToAD7190(3,Write_dat);
}


void AD7190_Config_Set(AD7190_Config_SET *Mode)
{
	uint32_t buf=Mode->Config_Channel|Mode->Config_ADC_Gain;
	uint8_t Write_dat[3],Dat;
	ADC_Gain=pow(2,Mode->Config_ADC_Gain);
	if(Mode->Config_Chop)buf|=Config_Chop_EN;
	if(Mode->Config_REFSEL)buf|=Config_REFSEL_IO;
	if(Mode->Config_Burn)buf|=Config_Burn_EN;
	if(Mode->Config_REFDET)buf|=Config_REFDET_EN;
	if(Mode->Config_BUF)buf|=Config_BUF_EN;
	if(Mode->Config_UB)buf|=Config_UB_EN;

	Write_dat[2]=buf>>16;
	Write_dat[1]=buf>>8;
	Write_dat[0]=buf;
	Dat=Config_register;
	WriteToAD7190(1,&Dat);
	WriteToAD7190(3,Write_dat);
}

long int DisordersData = 0;
long int DisordersTemp = 0;

void AD7190_Mode_Init(void)
{
	uint8_t buf[5];
	AD7190_MODE_SET 		Mode;
	AD7190_Config_SET   	Config;
	
//	Config.Config_Channel		=		Config_Ch2_temp;
//	Config.Config_Channel		=		Config_Ch1_A3A4|Config_Ch0_A1A2;//Config_Ch4_A1AC
//	Config.Config_Channel		=		Config_Ch0_A1A2;
	Config.Config_Channel		=		Config_Ch4_A1AC|Config_Ch5_A2AC|Config_Ch6_A3AC|Config_Ch7_A4AC;
	
	Config.Config_ADC_Gain		=		Config_ADC_Gain_1;
	Config.Config_Chop			=		0;
	Config.Config_REFSEL		=		0;
	Config.Config_Burn			=		0;
	Config.Config_REFDET		=		1;
	Config.Config_BUF			=		0;
	Config.Config_UB			=		1;
	
	Mode.ADC_Mode			=	MODE_ADC_AdjustZero;
	Mode.Return_state		=	0;
	Mode.ADC_SCLK			=	MODE_MCLK_IN;
	Mode.SINC3_EN			=	0;
	Mode.ENPAR				=	0;
	Mode.Single_EN			=	0;
	Mode.REJ60_EN			=	0;
	Mode.Filter				=	1;
	
	
	AD7190_Config_Set(&Config);
	WaitDataRDY();
	AD7190_Mode_Set(&Mode);
	WaitDataRDY();
	Mode.ADC_Mode			=	MODE_ADC_AdjustFull;
	AD7190_Mode_Set(&Mode);
	WaitDataRDY();
	Mode.ADC_Mode			=	MODE_ADC_OneByOne;
	Mode.Filter				=	10;
	AD7190_Mode_Set(&Mode);
	
	AD7190_Connect_Set(0,AD7190_Read,Disorders_register);
	ReadFromAD7190(3,&buf[0]);
	DisordersData = (((uint32_t)buf[0])<<16)|(((uint32_t)buf[1])<<8)|(((uint32_t)buf[2]));
	DisordersTemp = (DisordersData&0X007FFFFF)^0X007FFFFF;
//	DisordersTemp = (DisordersData&0XFF7FFFFF)^0XFF800000+1;
	DisordersData = (DisordersData>=0X00800000)?(-(DisordersTemp+1)):(DisordersData);	
}

float ADC_Votage(void)
{
	uint8_t buf[3];
	uint32_t tempn;
	float Vo;
	WaitDataRDY();

 	buf[0] = 0x58;
	WriteToAD7190(1,buf);		//write communication register 0x58 to control the progress to read data register
	
	tempn=GET_AD7190();
	Vo=tempn*V_Ref/(float)(16777216*ADC_Gain);
	
//	tempn=tempn>>8;		//把滤波后的24bit数据转换为16bit。
	return Vo;

}

uint32_t ADC_Num(void)
{
	uint8_t buf[3];
	uint32_t tempn;
	float Vo;
	WaitDataRDY();

 	buf[0] = 0x58;
	WriteToAD7190(1,buf);		//write communication register 0x58 to control the progress to read data register
	
	tempn=GET_AD7190();
	
	return tempn;
}

uint8_t AdcGroupRead(long int adcResult[4])
{
	uint32_t datatemp;
	datatemp =ADC_Num();
	
	if(datatemp)
	{
		if(ADC_Channel==4)
		{
			adcResult[0] = datatemp;
		}
		else if(ADC_Channel==5)
		{
			adcResult[1] = datatemp;
		}
		else if(ADC_Channel==6)
		{
			adcResult[2] = datatemp;
		}
		else if(ADC_Channel==7)
		{
			adcResult[3] = datatemp;//DisordersData;//datatemp;
		}
		return 1;
	}
	return 0;
}

void AD7190_Init(void)
{
	uint8_t buf[3];
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  AD7190_CS_CLKEN();
  AD7190_SCK_CLKEN();
  AD7190_DIN_CLKEN();
  AD7190_DOUT_CLKEN();

  AD7190_CS(1);
  GPIO_InitStruct.Pin = AD7190_CS_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(AD7190_CS_GPIO, &GPIO_InitStruct);

  AD7190_SCK(1);
  GPIO_InitStruct.Pin = AD7190_SCK_PIN;
  HAL_GPIO_Init(AD7190_SCK_GPIO, &GPIO_InitStruct);

  AD7190_DIN(1);
  GPIO_InitStruct.Pin = AD7190_DIN_PIN;
  HAL_GPIO_Init(AD7190_DIN_GPIO, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = AD7190_DOUT_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(AD7190_DOUT_GPIO, &GPIO_InitStruct);
  
	buf[0] = 0xff;
	WriteToAD7190(1,buf);
	buf[0] = 0xff;
	WriteToAD7190(1,buf);
	buf[0] = 0xff;
	WriteToAD7190(1,buf);
	buf[0] = 0xff;
	WriteToAD7190(1,buf);
	buf[0] = 0xff;
	WriteToAD7190(1,buf);
	AD7190_Mode_Init();
//  AD7190_refV = refV;

//  AD7190_Reset();
//  return AD7190_GetID();
}



