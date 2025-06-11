#ifndef __OLED_H
#define __OLED_H
#include "i2c.h"
#include "OLED_Data.h"
#include <stdint.h>

/*FontSize参数取值*/
/*此参数值不仅用于判断，而且用于计算横向字符偏移，默认值为字体像素宽度*/
#define OLED_8X16				8
#define OLED_6X8				6
#define OLED_12X24			12

/*IsFilled参数数值*/
#define OLED_UNFILLED			0
#define OLED_FILLED				1



/*初始化函数*/
void OLED_Init(void);

/*更新函数*/
void OLED_Update(void);
void OLED_UpdateArea(uint8_t X, uint8_t Y, uint8_t Width, uint8_t Height);

/*显存控制函数*/
void OLED_Clear(void);
void OLED_ClearArea(uint8_t X, uint8_t Y, uint8_t Width, uint8_t Height);
void OLED_Reverse(void);
void OLED_ReverseArea(uint8_t X, uint8_t Y, uint8_t Width, uint8_t Height);

/*显示函数*/
void OLED_ShowChar(uint8_t X, uint8_t Y, char Char, uint8_t FontSize);
void OLED_ShowString(uint8_t X, uint8_t Y, char *String, uint8_t FontSize);
void OLED_ShowNum(uint8_t X, uint8_t Y, uint32_t Number, uint8_t Length, uint8_t FontSize);
void OLED_ShowSignedNum(uint8_t X, uint8_t Y, int32_t Number, uint8_t Length, uint8_t FontSize);
void OLED_ShowHexNum(uint8_t X, uint8_t Y, uint32_t Number, uint8_t Length, uint8_t FontSize);
void OLED_ShowBinNum(uint8_t X, uint8_t Y, uint32_t Number, uint8_t Length, uint8_t FontSize);
void OLED_ShowFloatNum(uint8_t X, uint8_t Y, double Number, uint8_t IntLength, uint8_t FraLength, uint8_t FontSize);
void OLED_ShowChinese(uint8_t X, uint8_t Y, char *Chinese);
void OLED_ShowImage(uint8_t X, uint8_t Y, uint8_t Width, uint8_t Height, const uint8_t *Image);
//void OLED_Printf(uint8_t X, uint8_t Y, uint8_t FontSize, char *format, ...);
void OLED_DrawPoint(int16_t X, int16_t Y);
void OLED_DrawLine(int16_t X0, int16_t Y0, int16_t X1, int16_t Y1);
void OLED_DrawRectangle(int16_t X, int16_t Y, uint8_t Width, uint8_t Height, uint8_t IsFilled);

#endif

