#ifndef __weight_manage_H
#define __weight_manage_H

#include "sys.h"
#include "AD7190.h"
#include "lcd.h"
#include "stmflash.h"
#include "task_manage.h"
//#include "stmflash.h"

typedef struct{
#define WEILINE_LEN 48
#define CUTHEAD_LEN 4
#define CUTLAST_LEN 4
#define USEDATA_LEN (WEILINE_LEN - CUTHEAD_LEN - CUTLAST_LEN)
	
u32 weightline[WEILINE_LEN];
u32 wei_ave;
long int per_gram;
u32 weight_base;
float gram;
float Unit_Price;
} WeightCon;
extern WeightCon weight_ch0, weight_ch1;
extern u8 _fresh;
extern u8 Task_ID;
extern u8 Use_Ch;

u8 AdcGroupRead(long int adcResult[4]);
u8 AveWeightDataGet(void);
void InsertSort(u32 pushdata, u32* avedata, u32 dst[], u8 len);

void SaveGramInfo(u8 ch, u32 baseweight, u32 per_gram);
void ReadGramInfo(u8 ch);

#endif
