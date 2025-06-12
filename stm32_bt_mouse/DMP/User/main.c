#include "stm32f10x.h"                  // Device header
#include "Delay.h"
#include "Serial.h"
#include "AD.h"
#include "LED.h"
#include "Key.h"
#include "Encoder.h"
#include "stdio.h"
#include "MPU6050.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
#include <stdlib.h>  // �ṩabs()��������
#include <math.h>    // �ṩfabsf()��������


//��ֵ�޷�
#define CONSTRAIN(value, min, max)  ((value) < (min) ? (min) : ((value) > (max) ? (max) : (value)))

float Pitch,Roll,Yaw;								//������Ĭ�ϸ���ֵһ���������ǣ�ƫ����
int16_t ax,ay,az,gx,gy,gz;							//���ٶȣ������ǽ��ٶ�

uint8_t Buf[4];//hidͨ�����ֽ�
float BAT_Value;//����ֵ
uint8_t bat_flag=0;//��������־
uint8_t Key_Num=0;//����ֵ
uint8_t Key_SendFlag;//�������ͱ�־
 
volatile uint8_t Mode_Flag=0;//ģʽֵ
uint8_t key_delay_falg=0;//�ڶ�ģʽ�İ����������󴥵���ʱ��־

float angel_pitch,angel_roll,angel_yaw; //��̬������
uint8_t i;
static uint8_t TimeOUT;//���Ի�ȡ��̬�����ݵ�������
int8_t mouse_x = 0, mouse_y = 0;          // ���չ���ƶ���

void AirMouse_Update(float roll, float yaw,int8_t *xdata,int8_t *ydata)  ;
void TIM1_Init(void);

int main(void)
{
	AD_Init();//adc�����ɼ���ʼ��
	Key_Init();//������ʼ��
	LED_Init();//led�Ƴ�ʼ��
	BAT_Value=((float)AD_GetValue()*3.3/4095/2.1*100);//��ص���
	Bat_LED_Show(BAT_Value);//��Դ��
	Encoder_Init();//��ת��������ʼ��
	Serial_Init();//���ڳ�ʼ��
	MPU6050_Init();//mpu6050��ʼ��
	MPU6050_DMP_Init();//dmp���ʼ��
	TIM4_Init();//��ʱ��4��ʼ��
	TIM1_Init();
	while (1)
	{ 
		 while(MPU6050_DMP_Get_Data(&angel_pitch, &angel_roll, &angel_yaw));
//		while(MPU6050_DMP_Get_Data(&angel_pitch, &angel_roll, &angel_yaw)&&TimeOUT<3)//��ȡ��̬�����ݣ��������ֵ��0����ٴγ��Զ�ȡ
//		{
//			MPU6050_DMP_Get_Data(&angel_pitch, &angel_roll, &angel_yaw);//�ٴγ��Ի�ȡ��̬������
//			TimeOUT++;//��������
//		}
//		TimeOUT%=3;//�������
		AirMouse_Update(angel_roll, angel_yaw, &mouse_x,&mouse_y);
		if(Mode_Flag==0){
			Serial_TxPacket[1]= (-mouse_x) ;//���
		  Serial_TxPacket[2]=(mouse_y) ;
		}
		else if(key_delay_falg==0){
			if(mouse_x>8)
			{
				Key_Num|=0x01;//���
				key_delay_falg=1;
				TIM_SetCounter(TIM1,0);//�������
		    TIM_ITConfig(TIM1, TIM_IT_Update, ENABLE);					//����TIM1�ĸ����ж�
			}
			else if(mouse_x<(-8)){
				Key_Num|=0x02;//�Ҽ�
				key_delay_falg=1;
				TIM_SetCounter(TIM1,0);//�������
		    TIM_ITConfig(TIM1, TIM_IT_Update, ENABLE);					//����TIM1�ĸ����ж�
			}
		}	
		Serial_TxPacket[0]=Key_Num;//����
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
		Delay_ms(5);
	}
}

//3���ж�һ��
//��ʱ��4�жϺ���
void TIM4_IRQHandler(void)
{
	if (TIM_GetITStatus(TIM4, TIM_IT_Update) == SET)
	{
		TIM_ClearITPendingBit(TIM4, TIM_IT_Update);
		bat_flag++;
		
	}
}

//��ʱ��1��ʼ��
void TIM1_Init(void)
{
	/*����ʱ��*/
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);			//����TIM1��ʱ��
	
	/*ʱ����Ԫ��ʼ��*/
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;				//����ṹ�����
	TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;		//ʱ�ӷ�Ƶ��ѡ�񲻷�Ƶ���˲������������˲���ʱ�ӣ���Ӱ��ʱ����Ԫ����
	TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;	//������ģʽ��ѡ�����ϼ���
	TIM_TimeBaseInitStructure.TIM_Period = 5000 - 1;				//�������ڣ���ARR��ֵ ʵ��0.5S��ʱ�ж�
	TIM_TimeBaseInitStructure.TIM_Prescaler = 7200 - 1;				//Ԥ��Ƶ������PSC��ֵ
	TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;			//�ظ����������߼���ʱ���Ż��õ�
	TIM_TimeBaseInit(TIM1, &TIM_TimeBaseInitStructure);				//���ṹ���������TIM_TimeBaseInit������TIM1��ʱ����Ԫ	
	
	/*�ж��������*/
	TIM_ClearFlag(TIM1, TIM_FLAG_Update);						//�����ʱ�����±�־λ
																//TIM_TimeBaseInit����ĩβ���ֶ������˸����¼�
																//��������˱�־λ�������жϺ󣬻����̽���һ���ж�
																//�������������⣬������˱�־λҲ��
	TIM_ITConfig(TIM1, TIM_IT_Update, ENABLE);		
	/*NVIC�жϷ���*/
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);				//����NVICΪ����2
																//����ռ���ȼ���Χ��0~3����Ӧ���ȼ���Χ��0~3
																//�˷������������������н������һ��
																//���ж���жϣ����԰Ѵ˴������main�����ڣ�whileѭ��֮ǰ
																//�����ö�����÷���Ĵ��룬���ִ�е����ûḲ����ִ�е�����
	
	/*NVIC����*/
	NVIC_InitTypeDef NVIC_InitStructure;						//����ṹ�����
	NVIC_InitStructure.NVIC_IRQChannel = TIM1_UP_IRQn;				//ѡ������NVIC��TIM1��
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;				//ָ��NVIC��·ʹ��
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;	//ָ��NVIC��·����ռ���ȼ�Ϊ2
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;			//ָ��NVIC��·����Ӧ���ȼ�Ϊ1
	NVIC_Init(&NVIC_InitStructure);								//���ṹ���������NVIC_Init������NVIC����
	
	/*TIMʹ��*/
	TIM_Cmd(TIM1, ENABLE);			//ʹ��TIM1����ʱ����ʼ����
	TIM_ITConfig(TIM1, TIM_IT_Update, DISABLE);					//�ر�TIM1�ĸ����жϣ����ⲿ�жϴ���ʱ�Ž���
}


//��ʱ��1�жϺ���
void TIM1_UP_IRQHandler(void)
{
	if (TIM_GetITStatus(TIM1, TIM_IT_Update) == SET)
	{
		TIM_ClearITPendingBit(TIM1, TIM_IT_Update);
		key_delay_falg=0;
		TIM_ITConfig(TIM1, TIM_IT_Update, DISABLE);	
		
	}
}



/**
 * @brief     ������Ǻ�ƫ��������ת��Ϊ������xy���ƶ�����
 * @param     ����Ǻ�ƫ����
 * @param     �洢xy���������� 
 * @retval    ��
 */
void AirMouse_Update(float roll, float yaw,int8_t *xdata,int8_t *ydata) 
{
    // ��������
    const float CALIBRATION_THRESHOLD = 0.8f; // ������Ư
    const float MAX_DELTA_ANGLE = 15.0f;     // ���仯�Ƕ�
    const float SENSITIVITY = 10.0f;          // ������ϵ��
    const uint8_t DEAD_ZONE = 3;              //�����Ƕ�

    // ����Ƕȱ仯��
    static float last_roll = 0, last_yaw = 0;
    float delta_roll = roll - last_roll;
    float delta_yaw = yaw - last_yaw;
    last_roll = roll;
    last_yaw = yaw;
    
    // ��̬��ƯУ׼
    if(fabs(delta_roll) < CALIBRATION_THRESHOLD && 
       fabs(delta_yaw) < CALIBRATION_THRESHOLD) {
        delta_roll = delta_yaw = 0;
    }
    
    // �ٶ�����
    delta_roll = CONSTRAIN(delta_roll, -MAX_DELTA_ANGLE, MAX_DELTA_ANGLE);
    delta_yaw = CONSTRAIN(delta_yaw, -MAX_DELTA_ANGLE, MAX_DELTA_ANGLE);
    
    // ת��Ϊ�з���HID����ֵ��-1�Ǳ�֤ѡ���������ƶ�����һ��
    int8_t x_move = (int8_t)(delta_yaw * SENSITIVITY);
    int8_t y_move = (int8_t)(delta_roll * SENSITIVITY)*(-1); 
		
		// ����������С�����ֵ�����ݻᱻ���㣬��С��abs������ȡ����ֵ����
    if(abs(x_move) < DEAD_ZONE) x_move = 0;
    if(abs(y_move) < DEAD_ZONE) y_move = 0;

     *xdata=(x_move & 0xFF);
     *ydata=(y_move & 0xFF);
 
}


