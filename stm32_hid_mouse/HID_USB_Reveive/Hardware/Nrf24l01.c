#include "stm32f1xx_hal.h"  // HAL ��ͷ�ļ�
#include "NRF24L01.h"
#include "gpio.h"
#include "spi.h"

/***************************************************************
����ΪNRF24L01�ӷ������ݵ�ģ��

���͵����ݸ�ʽ����

[��һλ���ڶ�λ������λ������λ.....������ʮ��λ]

���У���һλҲ����Buf[0]λ���յ������ݳ���
�ڶ�λ������ʮ��λһ��32���ֽڵ�����λ���ܵ������ݣ�

�������£�5����������5������
{5, 0x11, 0x22, 0x33, 0x44, 0x55};

****************************************************************/
//���Ͷ˵Ľ��յ�ַ�ͽ��ն˵ķ��͵�ַ����Ҫһ��
uint8_t T_ADDR[5]={0xF0,0xF0,0xF0,0xF0,0xF0};
uint8_t R_ADDR[5]={0xF0,0xF0,0xF0,0xF0,0xF0};

//gpio��������


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
    uint8_t ByteReceive = 0x00;//���ص�����	  
    HAL_SPI_TransmitReceive(&hspi2, &Byte, &ByteReceive, 1, 1000);//��ȡ�����ֽڣ���ʱʱ��Ϊ1000ms
    return ByteReceive;
}

//д�Ĵ���
void W_Reg(uint8_t Reg, uint8_t Value)
{
    W_CSN(0);  // ѡ��NRF24L01
    SPI_SwapByte(Reg);  // д�Ĵ�����ַ
    SPI_SwapByte(Value);  // д�Ĵ���ֵ
    W_CSN(1);  // ֹͣѡ��NRF24L01
}

uint8_t R_Reg(uint8_t Reg)
{
    uint8_t Value;
    W_CSN(0);  // ѡ��NRF24L01
    SPI_SwapByte(Reg);  // д�Ĵ�����ַ
    Value = SPI_SwapByte(NOP);  // ���Ĵ���ֵ
    W_CSN(1);  // ֹͣѡ��NRF24L01
    return Value;
}

//д����
void W_Buf(uint8_t Reg, uint8_t* Buf, uint8_t Len)
{
    W_CSN(0);  // ѡ��NRF24L01
    SPI_SwapByte(Reg);  // д�Ĵ�����ַ
	  // &hspi2��SPI�����Buf��ָ��Ҫ���͵����������ָ�룬Len��Ҫ���͵����ݳ��ȣ�1000�ǳ�ʱʱ�䣨���룩
	  HAL_SPI_Transmit(&hspi2, Buf, Len, 1000);
    W_CSN(1);  // ֹͣѡ��NRF24L01
}

void R_Buf(uint8_t Reg, uint8_t* Buf, uint8_t Len)
{
    W_CSN(0);  // ѡ��NRF24L01
    SPI_SwapByte(Reg);  // д�Ĵ�����ַ
	  HAL_SPI_Receive(&hspi2, Buf, Len, 1000);
    W_CSN(1);  // ֹͣѡ��NRF24L01
}



/*
 *==============================================================================
 *�������ƣ�Med_Nrf24l01_Connect_Check
 *�������ܣ����NRF24L01������״̬
 *�����������
 *����ֵ��0������������1�������쳣
 *��  ע����
 *==============================================================================
*/
uint8_t Med_Nrf24l01_Connect_Check (void)
{
	uint8_t buf[5] = {0XA5,0XA5,0XA5,0XA5,0XA5};
	uint8_t i;
	//24L01�����SPIʱ��Ƶ��10MHz	 
	W_Buf(W_REGISTER + TX_ADDR,buf,5);   // д��5���ֽڵĵ�ַ.	
	R_Buf(TX_ADDR,buf,5);   // ����д��ĵ�ַ
	
	for(i = 0;i < 5;i ++)
	{
		if(buf[i] != 0XA5)
		{
			break;
		}
	}
	if(i != 5)
	{
		return 1;   // ���24L01����
	}
	return 0;   // ��⵽24L01
}


//��ʼ��
void NRF24L01_Init(void)
{
	MX_SPI2_Init();
	//NRF24L01��ʼ�����üĴ���
	W_CE(0);
	W_CSN(1);//��ѡ��NRF24L01
    HAL_Delay(10);
	W_Buf(W_REGISTER+TX_ADDR, T_ADDR, 5);//���÷��͵�ַ
	W_Buf(W_REGISTER+RX_ADDR_P0, R_ADDR, 5);//���ý���ͨ��0
	W_Reg(W_REGISTER+CONFIG,0x0F);//���óɽ���ģʽ
	W_Reg(W_REGISTER+EN_AA,0x01);//ͨ��0�����Զ�Ӧ��
	W_Reg(W_REGISTER+RF_CH,0x00);//����ͨ��Ƶ��2.4G
	W_Reg(W_REGISTER+RX_PW_P0,32);//���ý���ͨ��0���յ����ݿ��32�ֽ�
	W_Reg(W_REGISTER+EN_RXADDR,0x01);//����ͨ��0ʹ��
	W_Reg(W_REGISTER+SETUP_RETR,0x1A);//����580us�ط�ʱ����,�ط�10��
	W_Reg(FLUSH_RX,NOP);
	
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
        W_Reg(W_REGISTER + STATUS, Status);  // ����жϱ�־
		  	for(i=0;i<150*72;i++);//��ʱ150us
			//  for (i = 0; i < 10; i++);//�ʵ���ʱ�ȴ�
    }
}

uint8_t Send(uint8_t* Buf)
{
    uint8_t Status;
    W_Buf(W_TX_PAYLOAD, Buf, 32);  // д�뷢�ͻ�����
    W_CE(0);
    W_Reg(W_REGISTER + CONFIG, 0x0E);  // ����Ϊ����ģʽ
    W_CE(1);

    while (R_IRQ() == 1);  // �ȴ��ж�
    Status = R_Reg(R_REGISTER + STATUS);

    if (Status & MAX_TX)  // �ﵽ����ط�����
    {
        W_Reg(FLUSH_TX, NOP);  // ��շ��ͻ�����
        W_Reg(W_REGISTER + STATUS, Status);  // ����жϱ�־
        return MAX_TX;
    }
    if (Status & TX_OK)  // ���ͳɹ�
    {
        W_Reg(W_REGISTER + STATUS, Status);  // ����жϱ�־
        return TX_OK;
		}
		return NOP; // ����δ֪״̬
		
}

	
	
