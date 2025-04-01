#include "weight_manage.h"

u8 AdcGroupRead(long int adcResult[4])
{
	u32 datatemp;
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
u8 Use_Ch = 0;
WeightCon weight_ch0, weight_ch1;
u16 NoReadCount = 0;
u16 OnlyReadCh0Count = 0;
u16 OnlyReadCh1Count = 0;
u8 AveWeightDataGet(void)
{
	u32 datatemp;
	char dis_buf[20];
	u8 buf[3];
	float price;
	
	long int gram;
	u32 ave, base;
	datatemp =ADC_Num();
	if(datatemp)
	{
		
		if(ADC_Channel==4)
		{
			NoReadCount = 0;
			OnlyReadCh0Count++;
			OnlyReadCh1Count = 0;
			InsertSort(datatemp, &weight_ch0.wei_ave, weight_ch0.weightline, WEILINE_LEN);
		}
		else if(ADC_Channel==5)
		{
			NoReadCount = 0;
			OnlyReadCh1Count++;
			OnlyReadCh0Count = 0;
			InsertSort(datatemp, &weight_ch1.wei_ave, weight_ch1.weightline, WEILINE_LEN);
		}
		else ////数据扰动后差错检测
		{
			NoReadCount++;
////			if((NoReadCount > 10)||(OnlyReadCh0Count > 10)||(OnlyReadCh1Count > 10))
////			{
////				NoReadCount = 0;
////				buf[0] = 0xff;	WriteToAD7190(1,buf);
////				buf[0] = 0xff;	WriteToAD7190(1,buf);
////				buf[0] = 0xff;	WriteToAD7190(1,buf);
////				buf[0] = 0xff;	WriteToAD7190(1,buf);
////				buf[0] = 0xff;	WriteToAD7190(1,buf);
////				AD7190_Init();
////			}
		}
		

////		if(Task_ID == 0)
////		{
//			LCD_Show_CEStr(0,0,"总价:");
			
//			if(weight_ch0.wei_ave < weight_ch0.weight_base)
//				weight_ch0.wei_ave = weight_ch0.weight_base;
//			if(weight_ch1.wei_ave < weight_ch1.weight_base)
//				weight_ch1.wei_ave = weight_ch1.weight_base;
			weight_ch0.gram = ((long int)weight_ch0.wei_ave - (long int)weight_ch0.weight_base)/weight_ch0.per_gram;
			weight_ch1.gram = ((long int)weight_ch1.wei_ave - (long int)weight_ch1.weight_base)/weight_ch1.per_gram;
			
			if(weight_ch0.gram <= 0)	weight_ch0.gram = 0;
			if(weight_ch0.gram > 99000)	weight_ch0.gram = 0;
			if(weight_ch0.gram > 30000)	weight_ch0.gram = 30000;
			
			if(weight_ch1.gram <= 0)	weight_ch1.gram = 0;
			if(weight_ch1.gram > 99000)	weight_ch1.gram = 0;
			if(weight_ch1.gram > 30000)	weight_ch1.gram = 30000;
			
			if(Use_Ch == 0)
			{
				price = weight_ch0.gram/1000*weight_ch0.Unit_Price;
				gram = weight_ch0.gram;
			}
			else
			{
				price = weight_ch1.gram/1000*weight_ch1.Unit_Price;
				gram = weight_ch1.gram;
			}
			
//			sprintf(dis_buf, "%4d.%02d元    ", (u32)(price*100)%100000,(u32)(price*100)%100);
//			LCD_Show_CEStr(38,0, (u8*)dis_buf);
//			
//			LCD_Show_CEStr(0,4,"重量:");
//			sprintf(dis_buf, "%2d.%03dkg   ", (u32)gram/1000,(u32)gram%1000);
//			LCD_Show_CEStr(38,4, (u8*)dis_buf);
//			_fresh = 1;
////		}
//		if(Task_ID == 0)
//		{
////			LCD_Clear();
//			LCD_Show_CEStr(0,0,"总价:");
////			sprintf(dis_buf, "%4d.%02d元    ", (u32)(price*100)%100000,(u32)(price*100)%100);
//			sprintf(dis_buf, "%6.02f元    ", price);
//			LCD_Show_CEStr(38,0, (u8*)dis_buf);
//			
//			LCD_Show_CEStr(0,4,"重量:");
//			sprintf(dis_buf, "%2d.%03dkg   ", (u32)gram/1000,(u32)gram%1000);
//			LCD_Show_CEStr(38,4, (u8*)dis_buf);
//			_fresh = 1;
//		}
//		else if(Task_ID == 1)
//		{
//			LCD_Clear();
//			LCD_Show_CEStr(0,4,"重量:");
//			sprintf(dis_buf, "%2d.%03dkg   ", (u32)gram/1000,(u32)gram%1000);
//			LCD_Show_CEStr(38,4, (u8*)dis_buf);
//			_fresh = 1;
//		}
//		else
//		{
//			LCD_Clear();
//			LCD_Show_CEStr(0,0,"总价:");
//			sprintf(dis_buf, "%4d.%02d元    ", (u32)(price*100)%100000,(u32)(price*100)%100);
//			LCD_Show_CEStr(38,0, (u8*)dis_buf);
//			
//			LCD_Show_CEStr(0,4,"重量:");
//			sprintf(dis_buf, "%2d.%03dkg   ", (u32)gram/1000,(u32)gram%1000);
//			LCD_Show_CEStr(38,4, (u8*)dis_buf);
////			_fresh = 1;
//			LCD_Clear();
//		}
		return 1;
	}
	return 0;
}
/*
函数名：InsertSort
功能：带奇异值去除的均值滤波
输入：pushdata->待滤波数据源，dst->存放采集数据的数组，len->数组长度
输出：avedata->均值
作者：康威电子
更新日期:	2015/12/29
*/
#define WEILINE_LEN 48
#define CUTHEAD_LEN 4
#define CUTLAST_LEN 4
#define USEDATA_LEN (WEILINE_LEN - CUTHEAD_LEN - CUTLAST_LEN)
u32 weightline[WEILINE_LEN];
u32 wei_ave;
void InsertSort(u32 pushdata, u32* avedata, u32 dst[], u8 len)
{
	u16 i, j;
	u32 buf, sum = 0;
	u32 datasbuf[WEILINE_LEN];
	///insert
	for(i = 0; i < (len-1); i++)
	{
		dst[i] = dst[i+1];
	}
	dst[len-1] = pushdata;
	///sort
	for(i = 0 ; i < len; i++)
		datasbuf[i] = dst[i];
	for(i = len-1; i > 0; i--)
	{
		for(j = 0; j < i; j++)
		{
			if(datasbuf[j] < datasbuf[j + 1])
			{
				buf = datasbuf[j+1];
				datasbuf[j+1] = datasbuf[j];
				datasbuf[j] = buf;
			}
		}
	}
	///get ave
	for(i = CUTHEAD_LEN; i < (len- CUTLAST_LEN); i++)
	{
		sum += datasbuf[i];
	}
	*avedata = sum/USEDATA_LEN;
}
#define OneCHInfoLen	8
void SaveGramInfo(u8 ch, u32 baseweight, u32 per_gram)
{
	u8 savebuf[10];
//	u32 baseweight, per_gram;
//	if(ch == 0)
//	{
//		baseweight	= weight_ch0.weight_base;
//		per_gram		= weight_ch0.per_gram;
//	}
//	else
//	{
//		baseweight	= weight_ch1.weight_base;
//		per_gram		= weight_ch1.per_gram;
//	}
	
	savebuf[0] = ((per_gram>>24)&0X000000FF);
	savebuf[1] = ((per_gram>>16)&0X000000FF);
	savebuf[2] = ((per_gram>>8)&0X000000FF);
	savebuf[3] = ((per_gram)&0X000000FF);
	
	savebuf[4] = ((baseweight>>24)&0X000000FF);
	savebuf[5] = ((baseweight>>16)&0X000000FF);
	savebuf[6] = ((baseweight>>8)&0X000000FF);
	savebuf[7] = ((baseweight)&0X000000FF);
				
	STMFLASH_Write(FLASH_SAVE_ADDR+ch*OneCHInfoLen,(u16*)savebuf,8);
}

void ReadGramInfo(u8 ch)
{
	u8 readflashbuf[10];
	if(ch == 0)
	{
		STMFLASH_Read(FLASH_SAVE_ADDR+ch*OneCHInfoLen,(u16*)readflashbuf,8);
		weight_ch0.per_gram = (((u32)readflashbuf[0])<<24)+(((u32)readflashbuf[1])<<16)+(((u32)readflashbuf[2])<<8)+(((u32)readflashbuf[3]));
		weight_ch0.weight_base = (((u32)readflashbuf[4])<<24)+(((u32)readflashbuf[5])<<16)+(((u32)readflashbuf[6])<<8)+(((u32)readflashbuf[7]));
	}
	else if(ch == 1)
	{
		STMFLASH_Read(FLASH_SAVE_ADDR+ch*OneCHInfoLen,(u16*)readflashbuf,8);
		weight_ch1.per_gram = (((u32)readflashbuf[0])<<24)+(((u32)readflashbuf[1])<<16)+(((u32)readflashbuf[2])<<8)+(((u32)readflashbuf[3]));
		weight_ch1.weight_base = (((u32)readflashbuf[4])<<24)+(((u32)readflashbuf[5])<<16)+(((u32)readflashbuf[6])<<8)+(((u32)readflashbuf[7]));
	}
}


