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
#include <stdlib.h>  // 提供abs()函数声明
#include <math.h>    // 提供fabsf()函数声明
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
//数值限幅
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
//  xTaskCreate(SendData_Task,"SendData_Task", 128, NULL, osPriorityNormal+5, &SendData_TaskHandle);//创建发送数据任务
  xTaskCreate(MPU6050_Task, "MPU6050_Task", 128, NULL, osPriorityNormal+4, &MPU6050_TaskHandle);//创建姿态解算任务
	xTaskCreate(UART_Task, "UART_Task", 128, NULL, osPriorityNormal+3, &UART_TaskHandle);
//  xTaskCreate(Battery_Task, "Battery_Task", 128, NULL, osPriorityNormal+2, &Battery_TaskHandle);//创建电源管理任务
	
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

//通信或信号量启动而后关闭
void UART_Task(void * params)
{
	while(1)
	{
		printf("pitch:%.1f,roll:%.1f,yaw:%.1f",angel_pitch, angel_roll, angel_yaw);
		vTaskDelay(1000);//10s调用丿欿
	}
	
}





uint8_t Encoder_GetCount()
{		
		int16_t count = (int16_t)__HAL_TIM_GET_COUNTER(&htim3);// 读取当前编码器计数值	
		__HAL_TIM_SET_COUNTER(&htim3, 0);// 清零计数值
		uint8_t enc_count = (uint8_t)(count & 0xFF);  // 取低8位
	  return enc_count;
}

void SendData_Task(void * params)
{
	uint8_t i;
	while(1)
	{
		Buf[0]=Key_Number;//存入按键数据
		Buf[3]=Encoder_GetCount();//存入编码器数据
		Send(Buf);//发生2.4G通信数据
 //	printf("Buf: %3d, %3d, %3d, %3d.\r\n",Buf[0],Buf[1],Buf[2],Buf[3]);//串口调试
    Key_SendFlag++;//按键发送标志，对于按键需要多发生几次
		if(Key_SendFlag==3)  //按键数据多发送几次
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
		while(mpu_dmp_get_data(&angel_pitch, &angel_roll, &angel_yaw)&&TimeOUT<3)//获取姿态角数据，如果返回值非0则会再次尝试读取
		{
			mpu_dmp_get_data(&angel_pitch, &angel_roll, &angel_yaw);
			TimeOUT++;//尝试三次
		}
		TimeOUT=0;//清除计数
		AirMouse_Update(angel_roll, angel_yaw, Buf);//将横滚角和偏航角数据转换为鼠标光标的xy轴移动数据
		vTaskDelay(10);//延时10ms
	}
}


//通信或信号量启动而后关闭
void Battery_Task(void * params)
{
	float Bat_Value;
	while(1)
	{
		//电量监测
		HAL_ADC_Start(&hadc1);
		HAL_ADC_PollForConversion(&hadc1,HAL_MAX_DELAY);//等待ADC转换完成
		ADC_Value1=(uint16_t)HAL_ADC_GetValue(&hadc1);//通过ADC采样获取采样值
		//3.7v锂电池的满电为4.2v,进行了分压，adc采样是12位对应111111111111，满量程=4095,ADC采样的标准电压3.3V
		Bat_Value=((float)ADC_Value1*3.3/4095)/2.1 *100;//将adc采样数据转换位实际电量百分比
		HAL_GPIO_WritePin(LED_1_GPIO_Port,LED_1_Pin, (Bat_Value<=20)? GPIO_PIN_RESET: GPIO_PIN_SET);
		HAL_GPIO_WritePin(LED_2_GPIO_Port,LED_2_Pin, (Bat_Value<=20)? GPIO_PIN_SET: GPIO_PIN_RESET);
		vTaskDelay(10000);//10s调用一次
//		vTaskDelete(NULL);//任务自杀
	}
	
}

/**
 * @brief     将横滚角和偏航角数据转换为鼠标光标的xy轴移动数据
 * @param     横滚角和偏航角
 * @param     存储xy轴数据数组 
 * @retval    无
 */
void AirMouse_Update(float roll, float yaw,uint8_t *Buf_Data) 
{
    // 参数定义
    const float CALIBRATION_THRESHOLD = 0.8f; // 限制零漂
    const float MAX_DELTA_ANGLE = 15.0f;     // 最大变化角度
    const float SENSITIVITY = 10.0f;          // 灵敏度系数
    const uint8_t DEAD_ZONE = 5;              //死区角度

    // 计算角度变化量
    static float last_roll = 0, last_yaw = 0;
    float delta_roll = roll - last_roll;
    float delta_yaw = yaw - last_yaw;
    last_roll = roll;
    last_yaw = yaw;
    
    // 动态零漂校准
    if(fabs(delta_roll) < CALIBRATION_THRESHOLD && 
       fabs(delta_yaw) < CALIBRATION_THRESHOLD) {
        delta_roll = delta_yaw = 0;
    }
    
    // 速度限制
    delta_roll = CONSTRAIN(delta_roll, -MAX_DELTA_ANGLE, MAX_DELTA_ANGLE);
    delta_yaw = CONSTRAIN(delta_yaw, -MAX_DELTA_ANGLE, MAX_DELTA_ANGLE);
    
    // 转换为有符号HID报告值，-1是保证选择方向和鼠标移动方向一致
    int8_t x_move = (int8_t)(delta_yaw * SENSITIVITY)*(-1);
    int8_t y_move = (int8_t)(delta_roll * SENSITIVITY)*(-1); 
		
		// 死区处理，即小于这个值的数据会被清零，减小误差，abs函数是取绝对值函数
    if(abs(x_move) < DEAD_ZONE) x_move = 0;
    if(abs(y_move) < DEAD_ZONE) y_move = 0;

     Buf_Data[1]=(uint8_t)(x_move & 0xFF);
     Buf_Data[2]=(uint8_t)(y_move & 0xFF);
 
}
/* USER CODE END Application */

