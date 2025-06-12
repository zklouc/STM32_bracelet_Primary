#ifndef __DS3231_H
#define __DS3231_H

//读取数据的地址，不怎么变化，选择枚举
typedef enum
{
	SECONDS=0x00,
	MINUTES,
	HOURS,
	DAY,			//星期 1-7
	DATE,			//日 1-31
	MONTH,
	YEAR,
}DS3231_Addr;

typedef struct
{
	uint8_t sec;  
	uint8_t min;
	uint8_t hour;
	uint8_t day;  //星期
	uint8_t date; //日
	uint8_t month;
	uint8_t year;	
}DS3231_TimeType;

void DS3231_Init(void);
void DS3231_WriteReg(uint8_t RegAddress, uint8_t Data);
void DS3231_Write_Time(DS3231_TimeType Time);
void DS3231_Read_Time(DS3231_TimeType *DS3231_Time);

#endif
