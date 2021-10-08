#ifndef __TOUCH_7846_H
#define __TOUCH_7846_H

#ifdef __cplusplus
 extern "C" {
#endif

//#include "misc.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_conf.h"
#include "stm32f4xx.h"
#include "stm32f4xx_it.h"
#include "stm32f4xx_hal_spi.h"
//#include "stm32f4xx_exti.h"
#include "stm32f4xx_hal_gpio.h"
#include "math.h"


#include "GUI.h"

//mel
 void pfSendCmd(unsigned char Data);
 unsigned short int pfGetResult();
 char pfGetBusy();
 void pfSetCS(char OnOff);
 char pfGetPENIRQ();

GUI_PID_STATE pstate; //for stemwin

typedef struct
{
	unsigned short int X0;
	unsigned short int Y0;
	unsigned short int X;
	unsigned short int Y;
	unsigned char  Key_Sta;
	unsigned char  Key_LSta;
    unsigned char  noise;
    unsigned short inttime;
	float xfac;
	float yfac;
	short xoff;
	short yoff;
}Pen_Holder;



#define TOUCH_CS_PORT	 TOUCH_CS_GPIO_Port
//#define TOUCH_CS_PIN	 GPIO_Pin_7

#define T_CS()    GPIO_ResetBits(TOUCH_CS_PORT, TOUCH_CS_PIN);
#define T_DCS()   GPIO_SetBits(TOUCH_CS_PORT, TOUCH_CS_PIN);

#define CMD_RDY 0X90
#define CMD_RDX	0XD0


#define PEN  GPIOE->IDR&(1<<8) //
#define NPEN !(0x0080&PEN)  //!PEN


//#define sFLASH_CS_PIN                        GPIO_PIN_11
//#define sFLASH_CS_GPIO_PORT                  GPIOB
//#define sFLASH_CS_GPIO_CLK                   RCC_AHB1Periph_GPIOB
//
//#define sFLASH_CS_LOW()       HAL_GPIO_WritePin(sFLASH_CS_GPIO_PORT, sFLASH_CS_PIN, GPIO_PIN_RESET)
//#define sFLASH_CS_HIGH()      HAL_GPIO_WritePin(sFLASH_CS_GPIO_PORT, sFLASH_CS_PIN, GPIO_PIN_SET)

unsigned char SPI_RW_Byte(unsigned char num);
void SPI_Delay(unsigned int DelayCnt);
unsigned short int TPReadX(void);
unsigned short int TPReadY(void);
unsigned short int read_once(void);
unsigned char Read_Ads7846(void);

void EXTI9_5_IRQHandler(void);
void NVIC_TOUCHConfiguration(void);
void touch_init(void);

void Convert_Pos(void);
void Touch_Adjust(void);
void Convert_Pos(void);

#define Key_Down 0x01
#define Key_Up   0x00

#ifdef __cplusplus
}
#endif

#endif

















