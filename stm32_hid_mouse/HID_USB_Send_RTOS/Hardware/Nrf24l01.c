#include "stm32f1xx_hal.h"  // HAL 库头文件
#include "NRF24L01.h"
#include "gpio.h"
#include "spi.h"

/***************************************************************
以下为NRF24L01接发送数据的模板

发送的数据格式如下

[第一位，第二位，第三位，第四位.....，第三十三位]

其中，第一位也就是Buf[0]位接收到的数据长度
第二位到第三十三位一个32个字节的数据位接受到的数据；

例子如下，5代表后面跟着5个数据
{5, 0x11, 0x22, 0x33, 0x44, 0x55};

****************************************************************/
//发送端的接收地址和接收端的发送地址，需要一致
uint8_t T_ADDR[5]={0xF0,0xF0,0xF0,0xF0,0xF0};
uint8_t R_ADDR[5]={0xF0,0xF0,0xF0,0xF0,0xF0};

//gpio操作函数


void W_CSN(uint8_t Value)
{
    HAL_GPIO_WritePin(CSN_GPIO_Port, CSN_Pin, Value ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void W_CE(uint8_t Value)
{
    HAL_GPIO_WritePin(CE_GPIO_Port, CE_Pin, Value ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

uint8_t R_IRQ(void)
{
    return HAL_GPIO_ReadPin(IRQ_GPIO_Port, IRQ_Pin);
}


uint8_t SPI_SwapByte(uint8_t Byte)
{
    uint8_t ByteReceive = 0x00;//返回的数据	  
    HAL_SPI_TransmitReceive(&hspi2, &Byte, &ByteReceive, 1, 1000);//读取单个字节，延时时间为1000ms
    return ByteReceive;
}

//写寄存器
void W_Reg(uint8_t Reg, uint8_t Value)
{
    W_CSN(0);  // 选中NRF24L01
    SPI_SwapByte(Reg);  // 写寄存器地址
    SPI_SwapByte(Value);  // 写寄存器值
    W_CSN(1);  // 停止选中NRF24L01
}

uint8_t R_Reg(uint8_t Reg)
{
    uint8_t Value;
    W_CSN(0);  // 选中NRF24L01
    SPI_SwapByte(Reg);  // 写寄存器地址
    Value = SPI_SwapByte(NOP);  // 读寄存器值
    W_CSN(1);  // 停止选中NRF24L01
    return Value;
}

//写数组
void W_Buf(uint8_t Reg, uint8_t* Buf, uint8_t Len)
{
    W_CSN(0);  // 选中NRF24L01
    SPI_SwapByte(Reg);  // 写寄存器地址
	  // &hspi2是SPI句柄，Buf是指向要发送的数据数组的指针，Len是要发送的数据长度，1000是超时时间（毫秒）
	  HAL_SPI_Transmit(&hspi2, Buf, Len, 1000);
    W_CSN(1);  // 停止选中NRF24L01
}

void R_Buf(uint8_t Reg, uint8_t* Buf, uint8_t Len)
{
    W_CSN(0);  // 选中NRF24L01
    SPI_SwapByte(Reg);  // 写寄存器地址
	  HAL_SPI_Receive(&hspi2, Buf, Len, 1000);
    W_CSN(1);  // 停止选中NRF24L01
}



void NRF24L01_Init(void)
{

    // 配置寄存器
    W_CE(0);
    W_Buf(W_REGISTER + TX_ADDR, T_ADDR, 5);  // 配置发送地址
    W_Buf(W_REGISTER + RX_ADDR_P0, R_ADDR, 5);  // 配置接收通道0
    W_Reg(W_REGISTER + CONFIG, 0x0F);  // 配置成接收模式
    W_Reg(W_REGISTER + EN_AA, 0x01);  // 通道0开启自动应答
    W_Reg(W_REGISTER + RF_CH, 0x00);  // 配置通信频率2.4G
    W_Reg(W_REGISTER + RX_PW_P0, 32);  // 配置接收通道0接收的数据宽度32字节
    W_Reg(W_REGISTER + EN_RXADDR, 0x01);  // 接收通道0使能
    W_Reg(W_REGISTER + SETUP_RETR, 0x1A);  // 配置580us重发时间间隔, 重发10次
    W_Reg(FLUSH_RX, NOP);

    W_CE(1);
}


void Receive(uint8_t* Buf)
{
    uint8_t Status;
	  uint16_t i;
    Status = R_Reg(R_REGISTER + STATUS);
    if (Status & RX_OK)
    {
        R_Buf(R_RX_PAYLOAD, Buf, 32);
        W_Reg(FLUSH_RX, NOP);
        W_Reg(W_REGISTER + STATUS, Status);  // 清除中断标志
			  for (i = 0; i < 10; i++);//适当延时等待
    }
}

uint8_t Send(uint8_t* Buf)
{
    uint8_t Status;
    W_Buf(W_TX_PAYLOAD, Buf, 32);  // 写入发送缓冲区
    W_CE(0);
    W_Reg(W_REGISTER + CONFIG, 0x0E);  // 配置为发送模式
    W_CE(1);

    while (R_IRQ() == 1);  // 等待中断
    Status = R_Reg(R_REGISTER + STATUS);

    if (Status & MAX_TX)  // 达到最大重发次数
    {
        W_Reg(FLUSH_TX, NOP);  // 清空发送缓冲区
        W_Reg(W_REGISTER + STATUS, Status);  // 清除中断标志
        return MAX_TX;
    }
    if (Status & TX_OK)  // 发送成功
    {
        W_Reg(W_REGISTER + STATUS, Status);  // 清除中断标志
        return TX_OK;
		}
		return NOP; // 返回未知状态
		
}

	
	
