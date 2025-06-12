#include "stm32f10x.h"                  // Device header
#include "Delay.h"
#include "OLED.h"
#include "Serial.h"
#include "DS3231.h"
#include <string.h>
#include "ADXL345.h"
#include "Max30102.h"

//实时时间
DS3231_TimeType DS3231_R_Time;	//日历结构体
int Last_Date;  //记录上一秒，用于更新
//步数
uint8_t Active_axis=1;  //1-X,2-Y,3-Z 默认初始为x轴
Peak_Value Peak = {{0}, {0}, {0}};//ADXL345样本集中最大和最小值及差值
Slid_Reg Slid = {{0}, {0}};//移位寄存器高频滤波
Aver_Acc Cur_Sample;//ADCL345当前样本
uint8_t Step_Count=0;//步数
//心率血氧
uint32_t buffer_length=500; //缓冲区长度为500，可存储以100sps运行的5秒样本; 数据长度（红外光数据与红光数据数据长度相等）
uint32_t ir_buffer[500]; 	 //IR LED   红外光数据，用于计算血氧
uint32_t red_buffer[500];  //Red LED	红光数据，用于计算心率曲线以及计算心率
int32_t n_sp02=0; 				//SPO2值
int8_t ch_spo2_valid;   //用于显示SP02计算是否有效的指示符
int32_t n_heart_rate=0;   //心率值
int8_t  ch_hr_valid;    //用于显示心率计算是否有效的指示符
int index_count;
uint32_t un_min, un_max;  //存储红光数据的最小值和最大值，用于归一化处理
uint8_t dis_hr=0,dis_spo2=0;//用于显示的心率和血氧饱和度值

uint8_t UART_Star=0;
int main(void)
{
	/*模块初始化*/
	OLED_Init();			//OLED初始化
	Serial_Init();//连接蓝牙
	DS3231_Init();//实时时钟
	ADXL345_Init();
	MAX30102_Init();
	OLED_Clear();
//  //初始化ds3231的时间数据
//  DS3231_TimeType currentTime;
//  currentTime.year = 25;    
//  currentTime.month = 05;    
//  currentTime.day = 01;
//  currentTime.date = 26;   	
//  currentTime.hour = 14;   
//  currentTime.min = 41;     
//  currentTime.sec = 00;      
//  DS3231_Write_Time(currentTime);
	//显示“心率：”
	OLED_ShowChinese(0,14,"心率");
	OLED_ShowChar(34,14,':',OLED_8X16	);
	OLED_ShowString(70,14,"BMP",OLED_8X16	);
	
	//显示“血氧：”
	OLED_ShowChinese(0,31,"血氧");
	OLED_ShowChar(34,31,':',OLED_8X16	);
	OLED_ShowChar(70,31,'%',OLED_8X16	);
	
	OLED_ShowChinese(0,48,"步数");
	OLED_ShowChar(34,48,':',OLED_8X16	);
	OLED_Update();
	//读取前500个样本，并确定信号范围
	for(index_count=0;index_count<buffer_length;index_count++)
	{	
		  while(MAX30102_INT()==1);
  		MAX30102_FIFO_ReadWord(&red_buffer[index_count],&ir_buffer[index_count]);//获取红光和红外光的数据  
	}
 	//计算前500个样本（前5秒的样本）后的心率和血氧饱和度
  maxim_heart_rate_and_oxygen_saturation(ir_buffer, buffer_length, red_buffer, &n_sp02, &ch_spo2_valid, &n_heart_rate, &ch_hr_valid); 
	OLED_ShowNum(44, 14,n_heart_rate,3,OLED_8X16);OLED_ShowNum(44,31,n_sp02,2,OLED_8X16);dis_hr=0;
	while (1)
	{
		  DS3231_Read_Time(&DS3231_R_Time);
			if(Last_Date!=DS3231_R_Time.sec)//一秒更新一次数据显示
			{
				OLED_Printf(0, 0, OLED_6X8, "%02d-%02d-%02d", DS3231_R_Time.year, DS3231_R_Time.month, DS3231_R_Time.date); //显示日期	
				OLED_Printf(60, 0, OLED_6X8, "%02d:%02d:%02d", DS3231_R_Time.hour, DS3231_R_Time.min, DS3231_R_Time.sec); //显示时间		
				OLED_DrawLine(0,10,127,10);
				Last_Date=DS3231_R_Time.sec;
				if (UART_Star == 1)		//	串口发送数据
				{
					Serial_Printf( "HeartRate:%03d,Blood Oxygen:%02d%,Steps:%05d",n_heart_rate, n_sp02,Step_Count);	
				}
				else if(UART_Star==2) //串口停止发送数据
				{
					UART_Star = 0;
				}
			}
			Step_Count=Detect_Step(&Peak, &Slid, &Cur_Sample,&Active_axis);//监测步数，显示步数
			OLED_ShowNum(44, 48, Step_Count, 5, OLED_8X16);
//			OLED_ShowNum(90, 48, Active_axis, 1, OLED_8X16);//显示最活跃轴
			OLED_Update();
			//判断手指是否放上去
      for(index_count=0;index_count<30;index_count++)//获取40组数据，手指放上去时，新监测到的数据才不为0，获取多组，保证获取的是新获取的数据，
			{
					while(MAX30102_INT()==1);
					MAX30102_FIFO_ReadWord(&red_buffer[index_count],&ir_buffer[index_count]);//获取红光和红外光的数据，
			}	
			if((red_buffer[index_count-1]!=0)&&(ir_buffer[index_count-1]!=0))//index_count=40,目前只独立0-39共40个数据
			{
					for(index_count=100;index_count<buffer_length;index_count++)
					{
							red_buffer[index_count-100]=red_buffer[index_count];	//将100-500缓存数据移位到0-400
							ir_buffer[index_count-100] =ir_buffer[index_count];		//将100-500缓存数据移位到0-400
					}
					for(index_count=400;index_count<buffer_length;index_count++)
					{
							while(MAX30102_INT()==1);
							MAX30102_FIFO_ReadWord(&red_buffer[index_count],&ir_buffer[index_count]);//获取红光和红外光的数据
					}
					maxim_heart_rate_and_oxygen_saturation(ir_buffer, buffer_length, red_buffer, &n_sp02, &ch_spo2_valid, &n_heart_rate, &ch_hr_valid);
					if(ch_hr_valid == 1 && n_heart_rate<120)
					{
					n_heart_rate-=20;	//做补偿
					OLED_ShowNum(44, 14,n_heart_rate,3,OLED_8X16);OLED_ShowNum(44,31,n_sp02,2,OLED_8X16);dis_hr=n_heart_rate;}
					else if((n_heart_rate>120)&&(ch_hr_valid== 1)&&dis_hr>0)
					{
						n_heart_rate=dis_hr;
						OLED_ShowNum(44, 14,n_heart_rate,3,OLED_8X16);
						OLED_ShowNum(44,31,n_sp02,2,OLED_8X16);dis_hr=n_heart_rate;
					}
					else
					{
						n_heart_rate=0;
						n_sp02=0;
						OLED_ShowNum(44, 14,n_heart_rate,3,OLED_8X16);OLED_ShowNum(44,31,n_sp02,2,OLED_8X16);dis_hr=0;
					}
			}
			else
			{
				n_heart_rate=0;
				n_sp02=0;
				OLED_ShowNum(44, 14,n_heart_rate,3,OLED_8X16);OLED_ShowNum(44,31,n_sp02,2,OLED_8X16);dis_hr=0;
				Delay_ms(150);			//延时100ms，手动增加一些转换的间隔时间
			}
			OLED_Update();
			
	}
}

 

