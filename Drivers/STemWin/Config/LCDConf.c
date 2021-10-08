/*********************************************************************
*                SEGGER Microcontroller GmbH & Co. KG                *
*        Solutions for real time microcontroller applications        *
**********************************************************************
*                                                                    *
*        (c) 1996 - 2017  SEGGER Microcontroller GmbH & Co. KG       *
*                                                                    *
*        Internet: www.segger.com    Support:  support@segger.com    *
*                                                                    *
**********************************************************************

** emWin V5.44 - Graphical user interface for embedded applications **
All  Intellectual Property rights  in the Software belongs to  SEGGER.
emWin is protected by  international copyright laws.  Knowledge of the
source code may not be used to write a similar product.  This file may
only be used in accordance with the following terms:

The  software has  been licensed  to STMicroelectronics International
N.V. a Dutch company with a Swiss branch and its headquarters in Plan-
les-Ouates, Geneva, 39 Chemin du Champ des Filles, Switzerland for the
purposes of creating libraries for ARM Cortex-M-based 32-bit microcon_
troller products commercialized by Licensee only, sublicensed and dis_
tributed under the terms and conditions of the End User License Agree_
ment supplied by STMicroelectronics International N.V.
Full source code is available at: www.segger.com

We appreciate your understanding and fairness.
----------------------------------------------------------------------
File        : LCDConf_FlexColor_Template.c
Purpose     : Display controller configuration (single layer)
---------------------------END-OF-HEADER------------------------------
*/

/**
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2018 STMicroelectronics. 
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license SLA0044,
  * the "License"; You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *                      http://www.st.com/SLA0044
  *
  ******************************************************************************
  */

#include "GUI.h"
#include "GUIDRV_FlexColor.h"
#include "main.h"
#include "LCDConf.h"
#include "GUITDRV_ADS7846.h"

GUITDRV_ADS7846_CONFIG pConfig;


extern SPI_HandleTypeDef hspi1;

/*********************************************************************
*
*       Layer configuration (to be modified)
*
**********************************************************************
*/

#ifdef ILI9341
//
// Physical display size
//
#define XSIZE_PHYS  240 // To be adapted to x-screen size
#define YSIZE_PHYS  320 // To be adapted to y-screen size

/* Color conversion */
#define COLOR_CONVERSION    GUICC_565  //ili9341

#elif defined ILI9488
//
// Physical display size
//
#define XSIZE_PHYS  320 // To be adapted to x-screen size
#define YSIZE_PHYS  480 // To be adapted to y-screen size

/* Color conversion */
#define COLOR_CONVERSION    GUICC_888	//ili9488

#elif defined ST7735
//
// Physical display size
//
#define XSIZE_PHYS  128 // To be adapted to x-screen size
#define YSIZE_PHYS  160 // To be adapted to y-screen size

/* Color conversion */
#define COLOR_CONVERSION    GUICC_M565  //ili9341

#else
	#error Define LCDTYPE in Preprocessor Define Symbols
#endif

/* Display driver */
//#define DISPLAY_DRIVER GUIDRV_COMPACT_COLOR_16 //EmWin
#define DISPLAY_DRIVER GUIDRV_FLEXCOLOR			 //STemWin

// Buffers / VScreens
//
#define NUM_BUFFERS   1
#define NUM_VSCREENS  1

//
// Display orientation
//

#define DISPLAY_ORIENTATION  GUI_SWAP_XY | GUI_MIRROR_Y
//
// Touch screen
//
#define USE_TOUCH   1
//
// 	Touch screen calibration
//	calibrate screen

//ILI9341
//#define TSLEFT 137
//#define TSRIGHT 1909
//#define TSTOP 161
//#define TSBOTTOM 1850

#ifdef ILI9341			//0,0 in top left not bottom right
#define TOUCH_X_MIN 137 //y landscape
#define TOUCH_X_MAX 1909
#define TOUCH_Y_MIN 161 //x in landscape
#define TOUCH_Y_MAX 1850


//ILI9488
//#define TSLEFT 85
//#define TSRIGHT 1935
//#define TSTOP 130
//#define TSBOTTOM 1935

#elif defined ILI9488			 //0,0 in top left not bottom right
#define TOUCH_X_MIN 1935 //y landscape
#define TOUCH_X_MAX 85
#define TOUCH_Y_MIN 1935 //x in landscape
#define TOUCH_Y_MAX 130

#else					//generic
#define TOUCH_X_MIN 1935 //y landscape
#define TOUCH_X_MAX 85
#define TOUCH_Y_MIN 1935 //x in landscape
#define TOUCH_Y_MAX 130
#endif

/*********************************************************************
*
*       Configuration checking
*
**********************************************************************
*/
#ifndef   VXSIZE_PHYS
  #define VXSIZE_PHYS XSIZE_PHYS
#endif
#ifndef   VYSIZE_PHYS
  #define VYSIZE_PHYS YSIZE_PHYS
#endif



#ifndef   XSIZE_PHYS
  #error Physical X size of display is not defined!
#endif
#ifndef   YSIZE_PHYS
  #error Physical Y size of display is not defined!
#endif
#ifndef   GUICC_565
  #error Color conversion not defined!
#endif
#ifndef   GUIDRV_FLEXCOLOR
  #error No display driver defined!
#endif

#ifndef   NUM_VSCREENS
  #define NUM_VSCREENS 1
#else
  #if (NUM_VSCREENS <= 0)
    #error At least one screeen needs to be defined!
  #endif
#endif
#if (NUM_VSCREENS > 1) && (NUM_BUFFERS > 1)
  #error Virtual screens and multiple buffers are not allowed!
#endif

#ifndef   LCD_SWAP_XY
  #define LCD_SWAP_XY 0
#endif
#if LCD_SWAP_XY
  #define LCD_XSIZE YSIZE_PHYS
  #define LCD_YSIZE XSIZE_PHYS
#else
  #define LCD_XSIZE XSIZE_PHYS
  #define LCD_YSIZE YSIZE_PHYS
#endif
#define LCD_VXSIZE LCD_XSIZE
#define LCD_VYSIZE LCD_YSIZE


#ifndef   DISPLAY_ORIENTATION
  #define DISPLAY_ORIENTATION  0
#endif

#if ((DISPLAY_ORIENTATION & GUI_SWAP_XY) != 0)
#define LANDSCAPE   1
#else
#define LANDSCAPE   0
#endif

#if (LANDSCAPE == 1)
#define WIDTH       YSIZE_PHYS  /* Screen Width (in pixels)         */
#define HEIGHT      XSIZE_PHYS  /* Screen Hight (in pixels)         */
#else
#define WIDTH       XSIZE_PHYS  /* Screen Width (in pixels)         */
#define HEIGHT      YSIZE_PHYS  /* Screen Hight (in pixels)         */
#endif

#if ((DISPLAY_ORIENTATION & GUI_SWAP_XY) != 0)
  #if ((DISPLAY_ORIENTATION & GUI_MIRROR_X) != 0)
    #define TOUCH_TOP    TOUCH_X_MAX
    #define TOUCH_BOTTOM TOUCH_X_MIN
  #else
    #define TOUCH_TOP    TOUCH_X_MIN
    #define TOUCH_BOTTOM TOUCH_X_MAX
  #endif
  #if ((DISPLAY_ORIENTATION & GUI_MIRROR_Y) != 0)
    #define TOUCH_LEFT   TOUCH_Y_MAX
    #define TOUCH_RIGHT  TOUCH_Y_MIN
  #else
    #define TOUCH_LEFT   TOUCH_Y_MIN
    #define TOUCH_RIGHT  TOUCH_Y_MAX
  #endif
#else
  #if ((DISPLAY_ORIENTATION & GUI_MIRROR_X) != 0)
    #define TOUCH_LEFT   TOUCH_X_MAX
    #define TOUCH_RIGHT  TOUCH_X_MIN
  #else
    #define TOUCH_LEFT   TOUCH_X_MIN
    #define TOUCH_RIGHT  TOUCH_X_MAX
  #endif
  #if ((DISPLAY_ORIENTATION & GUI_MIRROR_Y) != 0)
    #define TOUCH_TOP    TOUCH_Y_MAX
    #define TOUCH_BOTTOM TOUCH_Y_MIN
  #else
    #define TOUCH_TOP    TOUCH_Y_MIN
    #define TOUCH_BOTTOM TOUCH_Y_MAX
  #endif
#endif

/*********************************************************************
*
*       Local functions
*
**********************************************************************
*/
/********************************************************************
*
*       LcdWriteReg
*
* Function description:
*   Sets display register
*/
void LcdWriteReg(U8 Command) {

	HAL_GPIO_WritePin(TFT_DC_GPIO_Port, TFT_DC_Pin, GPIO_PIN_RESET);	//command mode

	HAL_SPI_Transmit(&hspi1, &Command, 1, 100);

}

/********************************************************************
*
*       LcdWriteData
*
* Function description:
*   Writes a value to a display register
*/
void LcdWriteData(U8 Data) {

	HAL_GPIO_WritePin(TFT_DC_GPIO_Port, TFT_DC_Pin, GPIO_PIN_SET);	//data mode

	HAL_SPI_Transmit(&hspi1, &Data, 1, 100);

}

/********************************************************************
*
*       LcdWriteDataMultiple
*
* Function description:
*   Writes multiple values to a display register.
*/
void LcdWriteDataMultiple(U8 * pData, int NumItems) {

		HAL_GPIO_WritePin(TFT_DC_GPIO_Port, TFT_DC_Pin, GPIO_PIN_SET);	//data mode

		HAL_SPI_Transmit(&hspi1, pData, NumItems, 100);

}

/********************************************************************
*
*       LcdReadData
*
* Function description:
*   Reads multiple values from a display register.
*/
U8 LcdReadData() {

	  U8 tempData = 0;

	  HAL_GPIO_WritePin(TFT_DC_GPIO_Port, TFT_DC_Pin, GPIO_PIN_SET);	//data mode

	  HAL_SPI_Receive(&hspi1, &tempData, 1, 100);

	  return tempData;

}

/********************************************************************
*
*       LcdReadDataMultiple
*
* Function description:
*   Reads multiple values from a display register.
*/
void LcdReadDataMultiple(U8 * pData, int NumItems) {

	  HAL_GPIO_WritePin(TFT_DC_GPIO_Port, TFT_DC_Pin, GPIO_PIN_SET);	//data mode

	  HAL_SPI_Receive(&hspi1, pData, NumItems, 100);

}

/*********************************************************************
*
*       Public functions
*
**********************************************************************
*/
/*********************************************************************
*
*       LCD_X_Config
*
* Function description:
*   Called during the initialization process in order to set up the
*   display driver configuration.
*
*/
void LCD_X_Config(void) {
  GUI_DEVICE * pDevice;
  CONFIG_FLEXCOLOR Config = {0};
  GUI_PORT_API PortAPI = {0};
  //
  // Set display driver and color conversion
  //
  pDevice = GUI_DEVICE_CreateAndLink(GUIDRV_FLEXCOLOR, COLOR_CONVERSION, 0, 0);
  //
  // Display driver configuration, required for Lin-driver
  //
  LCD_SetSizeEx (0, XSIZE_PHYS , YSIZE_PHYS);
  LCD_SetVSizeEx(0, VXSIZE_PHYS, VYSIZE_PHYS);
  //
  // Orientation
  //
  Config.Orientation = DISPLAY_ORIENTATION; //GUI_SWAP_XY | GUI_MIRROR_Y;

  //mel
  //Config.NumDummyReads = 1;  //needed for ili9341 shield

  GUIDRV_FlexColor_Config(pDevice, &Config);


  //
  // Set controller and operation mode
  //
  PortAPI.pfWrite8_A0  = LcdWriteReg;
  PortAPI.pfWrite8_A1  = LcdWriteData;
  PortAPI.pfWriteM8_A1 = LcdWriteDataMultiple;
  PortAPI.pfRead8_A1   = LcdReadData;
  PortAPI.pfReadM8_A1  = LcdReadDataMultiple;



#ifdef ILI9488
  GUIDRV_FlexColor_SetFunc(pDevice, &PortAPI, GUIDRV_FLEXCOLOR_F66709, GUIDRV_FLEXCOLOR_M24C0B8);

#else //ILI9341
  GUIDRV_FlexColor_SetFunc(pDevice, &PortAPI, GUIDRV_FLEXCOLOR_F66709, GUIDRV_FLEXCOLOR_M16C0B8);
#endif
  //
  // Display driver configuration
  //
 if (LCD_GetSwapXY()) {
    LCD_SetSizeEx (0, YSIZE_PHYS, XSIZE_PHYS);
    LCD_SetVSizeEx(0, YSIZE_PHYS * NUM_VSCREENS, XSIZE_PHYS);
  } else {
    LCD_SetSizeEx (0, XSIZE_PHYS, YSIZE_PHYS);
    LCD_SetVSizeEx(0, XSIZE_PHYS, YSIZE_PHYS * NUM_VSCREENS);
  }


#if (USE_TOUCH == 1)

#ifndef SHIELD
 	 pConfig.Orientation = GUI_SWAP_XY;
#else
	 pConfig.Orientation = GUI_SWAP_XY | GUI_MIRROR_Y;		//shield XMAX on left
#endif
	 // - Orientation: Orientation of the touch screen if not the same as the physical orientation.
	 //                 A or-combination of the defines GUI_SWAP_XY, GUI_MIRROR_X and GUI_MIRROR_Y
	 //                 can be used.
	 pConfig.xLog0 = 0; 				// - xLog0      : Logical pixel value of horizontal reference point 0. Typically 0.
	 pConfig.xLog1 = XSIZE_PHYS -1; 	// - xLog1      : Logical pixel value of horizontal reference point 1. Typically horizontal screen resolution -1.
	 pConfig.xPhys0 = TOUCH_LEFT;		// - xPhys0     : Physical AD value of horizontal reference point 0.
	 pConfig.xPhys1 = TOUCH_RIGHT;		// - xPhys1     : Physical AD value of horizontal reference point 1.
	 pConfig.yLog0 = 0;					// - yLog0      : Logical pixel value of vertical reference point 0. Typically 0.
	 pConfig.yLog1 = YSIZE_PHYS - 1;	// - yLog1      : Logical pixel value of vertical reference point 1. Typically vertical screen resolution -1.
	 pConfig.yPhys0 = TOUCH_TOP;		// - yPhys0     : Physical AD value of vertical reference point 0.
	 pConfig.xPhys1 = TOUCH_BOTTOM;		// - yPhys1     : Physical AD value of vertical reference point 1.

	    // Set orientation of touch screen
	    //
	    GUI_TOUCH_SetOrientation(pConfig.Orientation);

	    //
	    // Calibrate touch screen
	    //
	    GUI_TOUCH_Calibrate(GUI_COORD_X, 0, WIDTH  - 1, TOUCH_LEFT, TOUCH_RIGHT);
	    GUI_TOUCH_Calibrate(GUI_COORD_Y, 0, HEIGHT - 1, TOUCH_TOP,  TOUCH_BOTTOM);

#endif
}

/*********************************************************************
*
*       LCD_X_DisplayDriver
*
* Function description:
*   This function is called by the display driver for several purposes.
*   To support the according task the routine needs to be adapted to
*   the display controller. Please note that the commands marked with
*   'optional' are not cogently required and should only be adapted if
*   the display controller supports these features.
*
* Parameter:
*   LayerIndex - Index of layer to be configured
*   Cmd        - Please refer to the details in the switch statement below
*   pData      - Pointer to a LCD_X_DATA structure
*
* Return Value:
*   < -1 - Error
*     -1 - Command not handled
*      0 - Ok
*/
int LCD_X_DisplayDriver(unsigned LayerIndex, unsigned Cmd, void * pData) {
  int r;
  (void) LayerIndex;
  (void) pData;
  
  switch (Cmd) {
  case LCD_X_INITCONTROLLER: {
    //
    // Called during the initialization process in order to set up the
    // display controller and put it into operation. If the display
    // controller is not initialized by any external routine this needs
    // to be adapted by the customer...
    //
    // ...

#ifdef ILI9341
	 InitLCD_ILI9341();
#elif defined ILI9481
	 InitLCD_ILI9481();
#elif defined ILI9488
	 InitLCD_ILI9488();
#else
	 InitLCD_ILI9341(); //anything else
#endif

    return 0;
  }
  default:
    r = -1;
  }
  return r;
}

//ILI9341 specific functions

void InitLCD_ILI9341(void) {

	HAL_GPIO_WritePin(TFT_RESET_GPIO_Port, TFT_RESET_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(TFT_DC_GPIO_Port, TFT_DC_Pin, GPIO_PIN_SET);
	//HAL_GPIO_WritePin(LCD_CS_GPIO_Port, LCD_CS_Pin, GPIO_PIN_SET); 		//disable CS
	HAL_GPIO_WritePin(TFT_CS_GPIO_Port, TFT_CS_Pin, GPIO_PIN_RESET); 		//enable CS


	/* Force reset */
	  HAL_GPIO_WritePin(TFT_RESET_GPIO_Port, TFT_RESET_Pin, GPIO_PIN_SET);
	  HAL_Delay(30);
	  HAL_GPIO_WritePin(TFT_RESET_GPIO_Port, TFT_RESET_Pin, GPIO_PIN_RESET);
	  HAL_Delay(10);
	  HAL_GPIO_WritePin(TFT_RESET_GPIO_Port, TFT_RESET_Pin, GPIO_PIN_SET);


	/* Delay for RST response */
	HAL_Delay(200);

	/* Software reset */
	LcdWriteReg(ILI9341_RESET);
	HAL_Delay(100);

	LcdWriteReg(ILI9341_POWERA);
	LcdWriteData(0x39);
	LcdWriteData(0x2C);
	LcdWriteData(0x00);
	LcdWriteData(0x34);
	LcdWriteData(0x02);
	LcdWriteReg(ILI9341_POWERB);
	LcdWriteData(0x00);
	LcdWriteData(0xC1);
	LcdWriteData(0x30);
	LcdWriteReg(ILI9341_DTCA);
	LcdWriteData(0x85);
	LcdWriteData(0x00);
	LcdWriteData(0x78);
	LcdWriteReg(ILI9341_DTCB);
	LcdWriteData(0x00);
	LcdWriteData(0x00);
	LcdWriteReg(ILI9341_POWER_SEQ);
	LcdWriteData(0x64);
	LcdWriteData(0x03);
	LcdWriteData(0x12);
	LcdWriteData(0x81);
	LcdWriteReg(ILI9341_PRC);
	LcdWriteData(0x20);
	LcdWriteReg(ILI9341_POWER1); //4.6V
	LcdWriteData(0x23);
	LcdWriteReg(ILI9341_POWER2);
	LcdWriteData(0x10);
	LcdWriteReg(ILI9341_VCOM1);
	LcdWriteData(0x3E);		//vcomh 4.250V
	LcdWriteData(0x28);		//vcoml -1.500V
	LcdWriteReg(ILI9341_VCOM2);
	LcdWriteData(0x86);
	LcdWriteReg(ILI9341_MAC); //memory access control
	LcdWriteData(0x48);	//48
	LcdWriteReg(ILI9341_PIXEL_FORMAT);
	LcdWriteData(0x55);	//rgb  16bits/pixel mcu 16bits/pixel
	LcdWriteReg(ILI9341_FRC); //Frame Rate Cntrol
	LcdWriteData(0x00); //diva freq == fosc
	LcdWriteData(0x18);	//79Hz refresh
	LcdWriteReg(ILI9341_DFC);//display function control
	LcdWriteData(0x08);
	LcdWriteData(0xA2); //normally white, gs - g1-g320, ss - s720-s1
	LcdWriteData(0x27);
	LcdWriteReg(ILI9341_3GAMMA_EN);
	LcdWriteData(0x00);
	LcdWriteReg(ILI9341_COLUMN_ADDR);
	LcdWriteData(0x00);
	LcdWriteData(0x00);
	LcdWriteData(0x00);
	LcdWriteData(0xEF);
	LcdWriteReg(ILI9341_PAGE_ADDR);
	LcdWriteData(0x00);
	LcdWriteData(0x00);
	LcdWriteData(0x01);
	LcdWriteData(0x3F);
	LcdWriteReg(ILI9341_GAMMA);
	LcdWriteData(0x01);
	LcdWriteReg(ILI9341_PGAMMA);
	LcdWriteData(0x0F);
	LcdWriteData(0x31);
	LcdWriteData(0x2B);
	LcdWriteData(0x0C);
	LcdWriteData(0x0E);
	LcdWriteData(0x08);
	LcdWriteData(0x4E);
	LcdWriteData(0xF1);
	LcdWriteData(0x37);
	LcdWriteData(0x07);
	LcdWriteData(0x10);
	LcdWriteData(0x03);
	LcdWriteData(0x0E);
	LcdWriteData(0x09);
	LcdWriteData(0x00);
	LcdWriteReg(ILI9341_NGAMMA);
	LcdWriteData(0x00);
	LcdWriteData(0x0E);
	LcdWriteData(0x14);
	LcdWriteData(0x03);
	LcdWriteData(0x11);
	LcdWriteData(0x07);
	LcdWriteData(0x31);
	LcdWriteData(0xC1);
	LcdWriteData(0x48);
	LcdWriteData(0x08);
	LcdWriteData(0x0F);
	LcdWriteData(0x0C);
	LcdWriteData(0x31);
	LcdWriteData(0x36);
	LcdWriteData(0x0F);
	LcdWriteReg(ILI9341_SLEEP_OUT);

	HAL_Delay(100);

	LcdWriteReg(ILI9341_DISPLAY_ON);
	LcdWriteReg(ILI9341_GRAM);
}


void InitLCD_ILI9481(void)
{
	  /* Reset */

	HAL_GPIO_WritePin(TFT_RESET_GPIO_Port, TFT_RESET_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(TFT_DC_GPIO_Port, TFT_DC_Pin, GPIO_PIN_SET);
	//HAL_GPIO_WritePin(LCD_CS_GPIO_Port, LCD_CS_Pin, GPIO_PIN_SET); 		//disable CS
	HAL_GPIO_WritePin(TFT_CS_GPIO_Port, TFT_CS_Pin, GPIO_PIN_RESET); 		//enable CS


	/* Force reset */
	  HAL_GPIO_WritePin(TFT_RESET_GPIO_Port, TFT_RESET_Pin, GPIO_PIN_SET);
	  HAL_Delay(30);
	  HAL_GPIO_WritePin(TFT_RESET_GPIO_Port, TFT_RESET_Pin, GPIO_PIN_RESET);
	  HAL_Delay(10);
	  HAL_GPIO_WritePin(TFT_RESET_GPIO_Port, TFT_RESET_Pin, GPIO_PIN_SET);


	/* Delay for RST response */
	HAL_Delay(200);

    LcdWriteReg(0x11);		// exit sleep mode, 0x0010 to enter
 //   LcdWriteData(0x00);
    HAL_Delay(50);

    LcdWriteReg(0x13);		// enter the normal display mode


    LcdWriteReg(0xd0);		// set the power parameters follow three parameters
    LcdWriteData(0x07);		//1.0xVci 07
    LcdWriteData(0x42);		// 40
    LcdWriteData(0x18);		//. 1C

    LcdWriteReg(0xd1);     	// Vcom subsequent
    LcdWriteData(0x00);		//00 18 1d
    LcdWriteData(0x07);
    LcdWriteData(0x10);

    LcdWriteReg(0xd2);		// set the parameters of the power in the normal mode
    LcdWriteData(0x01);
    LcdWriteData(0x11);

    LcdWriteReg(0xc0);		// Setting Panel Driving
    LcdWriteData(0x10);		// Set the scan mode 00
    LcdWriteData(0x3b);		// set the number of lines 480 lines 3B
    LcdWriteData(0x00);
    LcdWriteData(0x02);		// 5frames 02
    LcdWriteData(0x11);

    LcdWriteReg(0xc1);		// Setting the Timing
    LcdWriteData(0x10);
    LcdWriteData(0x0b);
    LcdWriteData(0x88);



    LcdWriteReg(0xC5);		// the Frame Rate Control Inversion
    LcdWriteData(0x01);

    LcdWriteReg(0xC8);		// gamma correction, the subsequent 12 parameters
    LcdWriteData(0x00);
    LcdWriteData(0x32);
    LcdWriteData(0x36);
    LcdWriteData(0x45);
    LcdWriteData(0x06);
    LcdWriteData(0x16);
    LcdWriteData(0x37);
    LcdWriteData(0x75);
    LcdWriteData(0x77);
    LcdWriteData(0x54);
    LcdWriteData(0x0f);
    LcdWriteData(0x00);

//    LcdWriteReg(0x36);
//    LcdWriteData(0x0a);//0a no effect


    LcdWriteReg(0x3a);		//Set_pixel_format
    LcdWriteData(0x55);		//16bit/pixel (65,536 colors)55


    LcdWriteReg(0x2a);
    LcdWriteData(0x00);
    LcdWriteData(0x00);
    LcdWriteData(0x01);
    LcdWriteData(0x3f);

    LcdWriteReg(0x2b);
    LcdWriteData(0x00);
    LcdWriteData(0x00);
    LcdWriteData(0x01);
    LcdWriteData(0xdf);


    HAL_Delay(50);

    LcdWriteReg(0x29);		//Set_display_on
    LcdWriteReg(0x2c);		//start writing data
    HAL_Delay(25);

}

void InitLCD_ILI9488(void)
{
	HAL_GPIO_WritePin(TFT_RESET_GPIO_Port, TFT_RESET_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(TFT_DC_GPIO_Port, TFT_DC_Pin, GPIO_PIN_SET);
	//HAL_GPIO_WritePin(LCD_CS_GPIO_Port, LCD_CS_Pin, GPIO_PIN_SET); 		//disable CS
	HAL_GPIO_WritePin(TFT_CS_GPIO_Port, TFT_CS_Pin, GPIO_PIN_RESET); 		//enable CS


	/* Force reset */
	  HAL_GPIO_WritePin(TFT_RESET_GPIO_Port, TFT_RESET_Pin, GPIO_PIN_SET);
	  HAL_Delay(30);
	  HAL_GPIO_WritePin(TFT_RESET_GPIO_Port, TFT_RESET_Pin, GPIO_PIN_RESET);
	  HAL_Delay(10);
	  HAL_GPIO_WritePin(TFT_RESET_GPIO_Port, TFT_RESET_Pin, GPIO_PIN_SET);


	/* Delay for RST response */
	HAL_Delay(200);


	LcdWriteReg(0xE0); //Positive Gamma Control

	LcdWriteData(0x00);
	LcdWriteData(0x03);
	LcdWriteData(0x09);
	LcdWriteData(0x08);
	LcdWriteData(0x16);
	LcdWriteData(0x0a);
	LcdWriteData(0x3f);
	LcdWriteData(0x78);
	LcdWriteData(0x4c);
	LcdWriteData(0x09);
	LcdWriteData(0x0a);
	LcdWriteData(0x08);
	LcdWriteData(0x16);
	LcdWriteData(0x1a);
	LcdWriteData(0x0f);


	LcdWriteReg(0xE1); //Negative Gamma Control

	LcdWriteData(0x00);
	LcdWriteData(0x16);
	LcdWriteData(0x19);
	LcdWriteData(0x03);
	LcdWriteData(0x0f);
	LcdWriteData(0x05);
	LcdWriteData(0x32);
	LcdWriteData(0x45);
	LcdWriteData(0x46);
	LcdWriteData(0x04);
	LcdWriteData(0x0e);
	LcdWriteData(0x0d);
	LcdWriteData(0x35);
	LcdWriteData(0x37);
	LcdWriteData(0x0f);

	LcdWriteReg(0xc0); //Power Control 1
	LcdWriteData(0x17);//Vreg1out
	LcdWriteData(0x15);//Verg2out

	LcdWriteReg(0xc1); //Power Control 2
	LcdWriteData(0x41);//VGH, VGL

	LcdWriteReg(0xc5); //Power Control 3
	LcdWriteData(0x00);
	LcdWriteData(0x12);//Vcom
	LcdWriteData(0x80);

	LcdWriteReg(0x36); //memory access
	LcdWriteData(0x08);

	LcdWriteReg(0x3a); // Interface Pixel Format
	LcdWriteData(0x56);//11-8bitcm 55-16 bit, 66 18bit ,77 24bit color mode

	LcdWriteReg(0xb0); //interface mode control
	LcdWriteData(0x00);//or 0x80 sdo not used


	LcdWriteReg(0xb1); //frame rate
	LcdWriteData(0xc0);//75Hz was a0 - 60Hz
	LcdWriteData(0x11);//16 clocks per line

	LcdWriteReg(0xb4); //display inversion control
	LcdWriteData(0x02);//2 dot

	LcdWriteReg(0xb6); //display function control RGB/MCU
	LcdWriteData(0x02);//MCU
	LcdWriteData(0x22);//source gate scan direction
	LcdWriteData(0x3b);

//	LcdWriteReg(0x0c); //dbi
//	LcdWriteData(0x00);//
//	LcdWriteData(0x66);//55-16 bit, 66-18bit, 77-24bit

	LcdWriteReg(0xe9); //set image function
	LcdWriteData(0x00);//disable 24bit data

	LcdWriteReg(0xf7); //adjust control
	LcdWriteData(0xa9);
	LcdWriteData(0x51);
	LcdWriteData(0x2c);
	LcdWriteData(0x82); //d7 stream, loose

	LcdWriteReg(ILI9488_SLPOUT); //exit sleep

	HAL_Delay(100);

	LcdWriteReg(ILI9488_DISPON); //display on


//	//software reset
//	LcdWriteReg(0x01);
//	HAL_Delay(100);
//
//	//sleep out
//	LcdWriteReg(0x11);
//	HAL_Delay(100);
//
//	//memory access control
//	LcdWriteReg(0x36);
//	LcdWriteData(0xe8);
//	HAL_Delay(100);
//
//	//set dbi
//	LcdWriteReg(0x3a);
//	LcdWriteData(0x11);//06
//	HAL_Delay(100);
//
//
//	//normal mode (partial mode off)
//	LcdWriteReg(0x13);
//	HAL_Delay(100);
//
//	//display on
//	LcdWriteReg(0x29);
//	HAL_Delay(100);
//
//	//set cursor
//	LcdWriteReg(0x2a);
//
//	//set startx
//	LcdWriteData(0x00);
//	LcdWriteData(0x00);
//
//	//set endx
//	LcdWriteData(0x01);
//	LcdWriteData(0xdf);
//	LcdWriteReg(0x00);
//
//	LcdWriteReg(0x2b);
//
//	//set starty
//	LcdWriteData(0x00);
//	LcdWriteData(0x00);
//
//	//set endy
//	LcdWriteData(0x01);
//	LcdWriteData(0x3f);
//	LcdWriteReg(0x00);
//
//	HAL_Delay(100);
//
//	//set brightness
//	LcdWriteReg(0x51);
//	LcdWriteData(0x0f);
//	HAL_Delay(100);
//
//	//set brightness control
//	LcdWriteReg(0x53);
//	LcdWriteData(0x2c);
//	HAL_Delay(100);
//
//	//set framerate
//	LcdWriteReg(0xb1);
//	LcdWriteData(0xb0);
//	LcdWriteData(0x11);
//	HAL_Delay(50);
//
//	//HAL_GPIO_WritePin(TFT_CS_GPIO_Port, TFT_CS_Pin, GPIO_PIN_SET);
//	HAL_Delay(50);

}

/*************************** End of file ****************************/

