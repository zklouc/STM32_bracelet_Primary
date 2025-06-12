#include "stm32f10x.h"                  // Device header
#include "DS3231.h"
#include "MyI2C.h"

#define DS3231_ADDRESS		0xD0		//DS3231的I2C从机地址


/**
  * 函    数：DS3231写寄存器
  * 参    数：RegAddress 寄存器地址，范围：参考DS3231手册的寄存器描述
  * 参    数：Data 要写入寄存器的数据，范围：0x00~0xFF
  * 返 回 值：无
  */
void DS3231_WriteReg(uint8_t RegAddress, uint8_t Data)
{
	MyI2C_Start();						//I2C起始
	MyI2C_SendByte(DS3231_ADDRESS);	//发送从机地址，读写位为0，表示即将写入
	MyI2C_ReceiveAck();					//接收应答
	MyI2C_SendByte(RegAddress);			//发送寄存器地址
	MyI2C_ReceiveAck();					//接收应答
	MyI2C_SendByte(Data);				//发送要写入寄存器的数据，最高位为1
	MyI2C_ReceiveAck();					//接收应答
	MyI2C_Stop();						//I2C终止
}

/**
  * 函    数：DS3231读寄存器
  * 参    数：RegAddress 寄存器地址，范围：参考DS3231手册的寄存器描述
  * 返 回 值：读取寄存器的数据，范围：0x00~0xFF
  */
uint8_t DS3231_ReadReg(uint8_t RegAddress)
{
	uint8_t Data;
	
	MyI2C_Start();						//I2C起始
	MyI2C_SendByte(DS3231_ADDRESS);	//发送从机地址，读写位为0，表示即将写入
	MyI2C_ReceiveAck();					//接收应答
	MyI2C_SendByte(RegAddress);			//发送寄存器地址
	MyI2C_ReceiveAck();					//接收应答
	
	MyI2C_Start();						//I2C重复起始
	MyI2C_SendByte(DS3231_ADDRESS | 0x01);	//发送从机地址，读写位为1，表示即将读取
	MyI2C_ReceiveAck();					//接收应答
	Data = MyI2C_ReceiveByte();			//接收指定寄存器的数据
	MyI2C_SendAck(1);					//发送应答，给从机非应答，终止从机的数据输出
	MyI2C_Stop();						//I2C终止
	
	return Data;
}
void DS3231_WriteRegS(uint8_t RegAddress,uint8_t *Data,uint8_t Count)
{
	uint8_t i;
	MyI2C_Start();						//I2C起始
	MyI2C_SendByte(DS3231_ADDRESS);	//发送从机地址，读写位为0，表示即将写入
	MyI2C_ReceiveAck();					//接收应答
	MyI2C_SendByte(RegAddress);			//发送寄存器地址
	MyI2C_ReceiveAck();					//接收应答
	for(i=0;i<Count;i++)
	{
		MyI2C_SendByte(Data[i]);				//发送要写入寄存器的数据，最高位为1
	  MyI2C_ReceiveAck();					//接收应答
	}
	MyI2C_Stop();						//I2C终止
}

void DS3231_ReadRegS(uint8_t RegAddress,uint8_t *Data,uint8_t Count)
{
	uint8_t i;
	MyI2C_Start();						//I2C起始
	MyI2C_SendByte(DS3231_ADDRESS);	//发送从机地址，读写位为0，表示即将写入
	MyI2C_ReceiveAck();					//接收应答
	MyI2C_SendByte(RegAddress);			//发送寄存器地址
	MyI2C_ReceiveAck();					//接收应答
	
	MyI2C_Start();						//I2C重复起始
	MyI2C_SendByte(DS3231_ADDRESS | 0x01);	//发送从机地址，读写位为1，表示即将读取
	MyI2C_ReceiveAck();					//接收应答
	for(i=0;i<Count;i++)
	{
		if(i==Count-1)
		{
			Data[i] = MyI2C_ReceiveByte();			//接收指定寄存器的数据
			MyI2C_SendAck(1);					//发送应答，给从机非应答，终止从机的数据输出
		}
		else
		{
			Data[i] = MyI2C_ReceiveByte();			//接收指定寄存器的数据
			MyI2C_SendAck(0);					//发送应答，给从机应答
		}
	}	
	MyI2C_Stop();						//I2C终止
}


/**
  * 函    数：DS3231初始化
  * 参    数：无
  * 返 回 值：无
  */
void DS3231_Init(void)
{
	MyI2C_Init();
}

/**
 * @brief     8位8421BCD码转为十进制码
 * @param     8421BCD码
 * @retval    十进制码  
 */
uint8_t Bcd_Dec(uint8_t Bcd_Num)
{
	return ((Bcd_Num>>4)*10+(Bcd_Num & 0x0f));
}


/**
 * @brief     十进制码转为8421BCD码
 * @param     十进制码
 * @retval    8421BCD码  
 */
uint8_t Dec_Bcd(uint8_t Dec_Num)
{
	return (((Dec_Num/10)<<4) | (Dec_Num % 10));
}

//设置时间
/**
 * @brief     设置时间
 * @param     时间结构体
 * @retval      
 */
void DS3231_Write_Time(DS3231_TimeType Time)
{
	uint8_t bcd_data[7];
	bcd_data[6]=Dec_Bcd(Time.year);
	bcd_data[5]=Dec_Bcd(Time.month);
	bcd_data[4]=Dec_Bcd(Time.date);
	bcd_data[3]=Dec_Bcd(Time.day);
	bcd_data[2]=Dec_Bcd(Time.hour);
	bcd_data[1]=Dec_Bcd(Time.min);
	bcd_data[0]=Dec_Bcd(Time.sec);
	
	DS3231_WriteRegS(SECONDS, bcd_data,7);
}

//  //初始化ds3231的时间数据
//  DS3231_TimeType currentTime;
//  currentTime.year = 25;    
//  currentTime.month = 02;    
//  currentTime.day = 07;
//  currentTime.date = 16;   	
//  currentTime.hour = 22;   
//  currentTime.min = 33;     
//  currentTime.sec = 00;      
//  DS3231_Write_Time(currentTime);

/**
 * @brief     读取时间
 * @param     时间结构体指针
 * @retval      
 */
void DS3231_Read_Time(DS3231_TimeType *DS3231_Time)
{
	uint8_t bcd_data[7];
	DS3231_ReadRegS(SECONDS,bcd_data,sizeof(bcd_data));
	
	DS3231_Time->sec  = Bcd_Dec(bcd_data[0]);               // 秒
	DS3231_Time->min  = Bcd_Dec(bcd_data[1]);               // 分钟
	DS3231_Time->hour = Bcd_Dec(bcd_data[2] & 0x3F);        // 小时（24小时制，去掉最高两位）
	DS3231_Time->day  = Bcd_Dec(bcd_data[3]);               // 星期几
	DS3231_Time->date = Bcd_Dec(bcd_data[4]);               // 日期
	DS3231_Time->month= Bcd_Dec(bcd_data[5] & 0x1F);        // 月份（去掉世纪位）
	DS3231_Time->year = Bcd_Dec(bcd_data[6]);               // 年份

}
