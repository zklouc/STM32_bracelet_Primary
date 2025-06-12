#include "stm32f10x.h"                  // Device header
#include "Delay.h"

//��ʱ��ʼ��
void TIM2_Init(void)
{
	/*����ʱ��*/
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);			//����TIM2��ʱ��
	
	/*ʱ����Ԫ��ʼ��*/
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;				//����ṹ�����
	TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;		//ʱ�ӷ�Ƶ��ѡ�񲻷�Ƶ���˲������������˲���ʱ�ӣ���Ӱ��ʱ����Ԫ����
	TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;	//������ģʽ��ѡ�����ϼ���
	TIM_TimeBaseInitStructure.TIM_Period = 2000 - 1;				//�������ڣ���ARR��ֵ ʵ��20ms��ʱ�ж�
	TIM_TimeBaseInitStructure.TIM_Prescaler = 720 - 1;				//Ԥ��Ƶ������PSC��ֵ
	TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;			//�ظ����������߼���ʱ���Ż��õ�
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseInitStructure);				//���ṹ���������TIM_TimeBaseInit������TIM2��ʱ����Ԫ	
	
	/*�ж��������*/
	TIM_ClearFlag(TIM2, TIM_FLAG_Update);						//�����ʱ�����±�־λ
																//TIM_TimeBaseInit����ĩβ���ֶ������˸����¼�
																//��������˱�־λ�������жϺ󣬻����̽���һ���ж�
																//�������������⣬������˱�־λҲ��
	TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);		
	/*NVIC�жϷ���*/
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);				//����NVICΪ����2
																//����ռ���ȼ���Χ��0~3����Ӧ���ȼ���Χ��0~3
																//�˷������������������н������һ��
																//���ж���жϣ����԰Ѵ˴������main�����ڣ�whileѭ��֮ǰ
																//�����ö�����÷���Ĵ��룬���ִ�е����ûḲ����ִ�е�����
	
	/*NVIC����*/
	NVIC_InitTypeDef NVIC_InitStructure;						//����ṹ�����
	NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;				//ѡ������NVIC��TIM2��
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;				//ָ��NVIC��·ʹ��
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;	//ָ��NVIC��·����ռ���ȼ�Ϊ2
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;			//ָ��NVIC��·����Ӧ���ȼ�Ϊ1
	NVIC_Init(&NVIC_InitStructure);								//���ṹ���������NVIC_Init������NVIC����
	
	/*TIMʹ��*/
	TIM_Cmd(TIM2, ENABLE);			//ʹ��TIM2����ʱ����ʼ����
	TIM_ITConfig(TIM2, TIM_IT_Update, DISABLE);					//�ر�TIM2�ĸ����жϣ����ⲿ�жϴ���ʱ�Ž���
}

//����������ó�ʼ��
void Key_Init(void)
{
	TIM2_Init();
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);   //����GPIOA��ʱ��
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);		//����AFIO��ʱ�ӣ��ⲿ�жϱ��뿪��AFIO��ʱ��
	//��ʼ��GPIO����
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1 | GPIO_Pin_2| GPIO_Pin_3;//�����м�
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	/*AFIOѡ���ж�����*/
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource1);//���ⲿ�жϵ�1����ӳ�䵽GPIOA����ѡ��PA1Ϊ�ⲿ�ж�����
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource2);//���ⲿ�жϵ�2����ӳ�䵽GPIOA����ѡ��PA2Ϊ�ⲿ�ж�����
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource3);//���ⲿ�жϵ�3����ӳ�䵽GPIOA����ѡ��PA3Ϊ�ⲿ�ж�����
	
  /*EXTI��ʼ��*/
	EXTI_InitTypeDef EXTI_InitStructure;														//����ṹ�����
	EXTI_InitStructure.EXTI_Line=EXTI_Line1|EXTI_Line2|EXTI_Line3;	//ѡ�������ⲿ�жϵ�0���ߺ�1����
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;												//ָ���ⲿ�ж���ʹ��
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;							//ָ���ⲿ�ж���Ϊ�ж�ģʽ
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;					//ָ���ⲿ�ж���Ϊ�½��ش������������³����½���
	EXTI_Init(&EXTI_InitStructure);																	//���ṹ���������EXTI_Init������EXTI���� 
	
	/*NVIC����*/
	NVIC_InitTypeDef NVIC_InitStructure;										 	//����ṹ�����
	NVIC_InitStructure.NVIC_IRQChannel = EXTI1_IRQn;			   	//ѡ������NVIC��EXTI1��
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;					 	//ָ��NVIC��·ʹ��
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;	//ָ��NVIC��·����ռ���ȼ�Ϊ1
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;			  //ָ��NVIC��·����Ӧ���ȼ�Ϊ0
	NVIC_Init(&NVIC_InitStructure);														//���ṹ���������NVIC_Init������NVIC����
	
	NVIC_InitStructure.NVIC_IRQChannel = EXTI2_IRQn;			 	  //ѡ������NVIC��EXTI2��
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;					 	//ָ��NVIC��·ʹ��
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;	//ָ��NVIC��·����ռ���ȼ�Ϊ1
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;			  //ָ��NVIC��·����Ӧ���ȼ�Ϊ2
	NVIC_Init(&NVIC_InitStructure);														//���ṹ���������NVIC_Init������NVIC����

	NVIC_InitStructure.NVIC_IRQChannel = EXTI3_IRQn;			 	  //ѡ������NVIC��EXTI3��
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;					 	//ָ��NVIC��·ʹ��
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;	//ָ��NVIC��·����ռ���ȼ�Ϊ1
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;			  //ָ��NVIC��·����Ӧ���ȼ�Ϊ3
	NVIC_Init(&NVIC_InitStructure);														//���ṹ���������NVIC_Init������NVIC����
	
}

//�ⲿ�жϺ���
void EXTI1_IRQHandler(void)
{
	if (EXTI_GetITStatus(EXTI_Line1) == SET)		//�ж��Ƿ����ⲿ�ж�1���ߴ������ж�
	{
		TIM_SetCounter(TIM2,0);//�������
		TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);					//����TIM2�ĸ����ж�
		EXTI_ClearITPendingBit(EXTI_Line1);			//����ⲿ�ж�1���ߵ��жϱ�־λ
													//�жϱ�־λ�������
													//�����жϽ��������ϵش�����������������
	}
}
//�ⲿ�жϺ���
void EXTI2_IRQHandler(void)
{
	if (EXTI_GetITStatus(EXTI_Line2) == SET)		//�ж��Ƿ����ⲿ�ж�2���ߴ������ж�
	{
		TIM_SetCounter(TIM2,0);//�������
		TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);	
		EXTI_ClearITPendingBit(EXTI_Line2);		
	}
}
//�ⲿ�жϺ���
void EXTI3_IRQHandler(void)
{
	if (EXTI_GetITStatus(EXTI_Line3) == SET)		//�ж��Ƿ����ⲿ�ж�3���ߴ������ж�
	{
		TIM_SetCounter(TIM2,0);//�������
		TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);	
		EXTI_ClearITPendingBit(EXTI_Line3);		
	}
}

extern uint8_t Key_Num;
extern uint8_t Mode_Flag;

void TIM2_IRQHandler(void)
{
	if (TIM_GetITStatus(TIM2, TIM_IT_Update) == SET)
	{
		TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
		if(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_1)==RESET)//�������
		{
			Key_Num|=0x01;
		}
		else if(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_3)==RESET)//�Ҽ�����
		{
			Key_Num|=0x02;
		}
		else if(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_2)==RESET)//�м�����
		{
			//Key_Num|=0x04;
			Mode_Flag=~Mode_Flag;
		}
		TIM_ITConfig(TIM2, TIM_IT_Update, DISABLE);	
		
	}
}


