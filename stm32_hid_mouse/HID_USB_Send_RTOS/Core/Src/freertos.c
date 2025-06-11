/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"
#include "tim.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "adc.h"
#include "NRF24L01.h"
#include "stdio.h"
#include "MPU6050.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
#include <stdlib.h>  // �ṩabs()��������
#include <math.h>    // �ṩfabsf()��������
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
//��ֵ�޷�
#define CONSTRAIN(value, min, max)  ((value) < (min) ? (min) : ((value) > (max) ? (max) : (value)))
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
//???
static TaskHandle_t SendData_TaskHandle;
static TaskHandle_t MPU6050_TaskHandle ;
static TaskHandle_t Battery_TaskHandle ;
static TaskHandle_t UART_TaskHandle;
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
uint16_t ADC_Value1;	
uint8_t Buf[32];
uint8_t Key_SendFlag;
float angel_pitch,angel_roll,angel_yaw; 	
/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
void SendData_Task(void * params);
void MPU6050_Task(void * params);
void Battery_Task(void * params);
void UART_Task(void * params);
void AirMouse_Update(float pitch, float roll, uint8_t *Buf_Data) ;
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
  NRF24L01_Init();
	if(MPU_Init())  
		printf("MPU6050_Init Error.\r\n");			
	else 
		printf("MPU6050_Init Success.\r\n");
  mpu_dmp_init();	
  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  //defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
//  xTaskCreate(SendData_Task,"SendData_Task", 128, NULL, osPriorityNormal+5, &SendData_TaskHandle);//����������������
  xTaskCreate(MPU6050_Task, "MPU6050_Task", 128, NULL, osPriorityNormal+4, &MPU6050_TaskHandle);//������̬��������
	xTaskCreate(UART_Task, "UART_Task", 128, NULL, osPriorityNormal+3, &UART_TaskHandle);
//  xTaskCreate(Battery_Task, "Battery_Task", 128, NULL, osPriorityNormal+2, &Battery_TaskHandle);//������Դ��������
	
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

//ͨ�Ż��ź�����������ر�
void UART_Task(void * params)
{
	while(1)
	{
		printf("pitch:%.1f,roll:%.1f,yaw:%.1f",angel_pitch, angel_roll, angel_yaw);
		vTaskDelay(1000);//10s����د�K
	}
	
}





uint8_t Encoder_GetCount()
{		
		int16_t count = (int16_t)__HAL_TIM_GET_COUNTER(&htim3);// ��ȡ��ǰ����������ֵ	
		__HAL_TIM_SET_COUNTER(&htim3, 0);// �������ֵ
		uint8_t enc_count = (uint8_t)(count & 0xFF);  // ȡ��8λ
	  return enc_count;
}

void SendData_Task(void * params)
{
	uint8_t i;
	while(1)
	{
		Buf[0]=Key_Number;//���밴������
		Buf[3]=Encoder_GetCount();//�������������
		Send(Buf);//����2.4Gͨ������
 //	printf("Buf: %3d, %3d, %3d, %3d.\r\n",Buf[0],Buf[1],Buf[2],Buf[3]);//���ڵ���
    Key_SendFlag++;//�������ͱ�־�����ڰ�����Ҫ�෢������
		if(Key_SendFlag==3)  //�������ݶ෢�ͼ���
    {Key_SendFlag=0;Key_Number=0;}	
		for (i = 0; i < 5; i++) Buf[i] = 0;
		vTaskDelay(10);
	}	
}


void MPU6050_Task(void * params)
{
	static uint8_t TimeOUT;
	while(1)
	{
		while(mpu_dmp_get_data(&angel_pitch, &angel_roll, &angel_yaw)&&TimeOUT<3)//��ȡ��̬�����ݣ��������ֵ��0����ٴγ��Զ�ȡ
		{
			mpu_dmp_get_data(&angel_pitch, &angel_roll, &angel_yaw);
			TimeOUT++;//��������
		}
		TimeOUT=0;//�������
		AirMouse_Update(angel_roll, angel_yaw, Buf);//������Ǻ�ƫ��������ת��Ϊ������xy���ƶ�����
		vTaskDelay(10);//��ʱ10ms
	}
}


//ͨ�Ż��ź�����������ر�
void Battery_Task(void * params)
{
	float Bat_Value;
	while(1)
	{
		//�������
		HAL_ADC_Start(&hadc1);
		HAL_ADC_PollForConversion(&hadc1,HAL_MAX_DELAY);//�ȴ�ADCת�����
		ADC_Value1=(uint16_t)HAL_ADC_GetValue(&hadc1);//ͨ��ADC������ȡ����ֵ
		//3.7v﮵�ص�����Ϊ4.2v,�����˷�ѹ��adc������12λ��Ӧ111111111111��������=4095,ADC�����ı�׼��ѹ3.3V
		Bat_Value=((float)ADC_Value1*3.3/4095)/2.1 *100;//��adc��������ת��λʵ�ʵ����ٷֱ�
		HAL_GPIO_WritePin(LED_1_GPIO_Port,LED_1_Pin, (Bat_Value<=20)? GPIO_PIN_RESET: GPIO_PIN_SET);
		HAL_GPIO_WritePin(LED_2_GPIO_Port,LED_2_Pin, (Bat_Value<=20)? GPIO_PIN_SET: GPIO_PIN_RESET);
		vTaskDelay(10000);//10s����һ��
//		vTaskDelete(NULL);//������ɱ
	}
	
}

/**
 * @brief     ������Ǻ�ƫ��������ת��Ϊ������xy���ƶ�����
 * @param     ����Ǻ�ƫ����
 * @param     �洢xy���������� 
 * @retval    ��
 */
void AirMouse_Update(float roll, float yaw,uint8_t *Buf_Data) 
{
    // ��������
    const float CALIBRATION_THRESHOLD = 0.8f; // ������Ư
    const float MAX_DELTA_ANGLE = 15.0f;     // ���仯�Ƕ�
    const float SENSITIVITY = 10.0f;          // ������ϵ��
    const uint8_t DEAD_ZONE = 5;              //�����Ƕ�

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
    int8_t x_move = (int8_t)(delta_yaw * SENSITIVITY)*(-1);
    int8_t y_move = (int8_t)(delta_roll * SENSITIVITY)*(-1); 
		
		// ����������С�����ֵ�����ݻᱻ���㣬��С��abs������ȡ����ֵ����
    if(abs(x_move) < DEAD_ZONE) x_move = 0;
    if(abs(y_move) < DEAD_ZONE) y_move = 0;

     Buf_Data[1]=(uint8_t)(x_move & 0xFF);
     Buf_Data[2]=(uint8_t)(y_move & 0xFF);
 
}
/* USER CODE END Application */

