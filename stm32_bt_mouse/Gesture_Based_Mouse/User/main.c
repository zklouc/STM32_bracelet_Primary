#include "stm32f10x.h"                  // Device header
#include "Delay.h"
#include "Serial.h"
#include "AD.h"
#include "LED.h"
#include "Key.h"
#include "Encoder.h"
#include "MPU6050.h"
#include <stdint.h>
#include "Cursor_Movement.h"

int16_t ax,ay,az;							//���ٶȣ������ǽ��ٶ�
float BAT_Value;							//����ֵ
uint8_t bat_flag=0;           //��������־
uint8_t Key_Num=0;            //��������
uint8_t Key_SendFlag;         //�������ͱ�־

int main(void)
{
	AD_Init();//adc������ʼ��
	Key_Init();
	LED_Init();
	BAT_Value=((float)AD_GetValue()*3.3/4095/2.1*100);//��ص���
	Bat_LED_Show(BAT_Value);//��Դ��
	Encoder_Init();//��ת��������ʼ��
	Serial_Init();//���ڳ�ʼ��
	MPU6050_Init();//MPU6050��ʼ��
	Cursor_Init();//���ٶ�ȥ����ƫ
	TIM4_Init();//������ⶨʱ��
	TIM1_Init();//�Ӷ�����ģʽ���ظ�������ʱ��
	while (1)
	{ 
		MPU6050_GetAcc(&ax,&ay,&az);//��ü��ٶ�
		ProcessSensorData(ax, ay);
		Serial_TxPacket[0]=Key_Num;//����
		Serial_TxPacket[1]= (-mouse_x) ;//���
		Serial_TxPacket[2]=(mouse_y) ;
		Serial_TxPacket[3]=Encoder_Get();//����		
		Serial_SendPacket();
		if(Serial_TxPacket[0]!=0)
		Key_SendFlag++;//�������ͱ�־�����ڰ�����Ҫ�෢������
		if(Key_SendFlag==3)  {Key_SendFlag=0;Key_Num=0;}	
    Serial_TxPacket[0]=0;
		Serial_TxPacket[1]=0;
		Serial_TxPacket[2]=0; 
		Serial_TxPacket[3]=0;
		if(bat_flag==20)
		{
			BAT_Value=((float)AD_GetValue()*3.3/4095/2.1*100);//��ص���
			//3.3��adc�����������̵�ѹ��4095��adc���������̣�2.1��3.7v﮵������4.2v����ѹ��*100ת��Ϊ�ٷֱ�
			Bat_LED_Show(BAT_Value);//��Դ��
			bat_flag=0;
		}
		Delay_ms(7);
	}
}

//3���ж�һ��
void TIM4_IRQHandler(void)
{
	if (TIM_GetITStatus(TIM4, TIM_IT_Update) == SET)
	{
		TIM_ClearITPendingBit(TIM4, TIM_IT_Update);
		bat_flag++;
		
	}
}


