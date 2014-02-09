/**
  ******************************************************************************
  * @file    DCMI/DCMI_CameraExample/camera_api.h 
  * @author  MCD Application Team
  * @version V1.3.0
  * @date    13-November-2013
  * @brief   Header for camera_api.c module
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2013 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */
  
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __CAMERA_API_H
#define __CAMERA_API_H

/* Includes ------------------------------------------------------------------*/
//#include "lcd_log.h"
#include "main.h"

/* Exported constants --------------------------------------------------------*/
#define DCMI_DR_ADDRESS       0x50050028
#define FSMC_LCD_ADDRESS      0x68000002
#define NOKEY                 0
#define SEL                   1
#define UP                    2
#define DOWN                  3

/* Exported types ------------------------------------------------------------*/
/* Camera devices enumeration */
typedef enum   
{
  OV9655_CAMERA            =   0x00,	 /* Use OV9655 Camera */
 // OV2640_CAMERA            =   0x01      /* Use OV2640 Camera */
	OV5640_CAMERA            =   0x01 ,     /* Use OV5640 Camera */
}Camera_TypeDef;




/* Image Sizes enumeration */
typedef enum   
{
 // BMP_QQVGA             =   0x00,	    /* BMP Image QQVGA 160x120 Size */
	
  //BMP_QVGA              =   0x01,     /* BMP Image QVGA 320x240 Size */
  RAW_320x240             =   0x01,	
	RAW_640x480             =   0x02,	
	RAW_1280_720            =   0x03,	
	RAW_1920_1080           =   0x04,	
  RAW_5MP_QSXGA_2592_1944 =   0x05,	
  JPEG_1280_720           =   0x06,	
  JPEG_1920_1080          =   0x07,	
  JPEG_5MP_2592_1944      =   0x08,	
}ImageFormat_TypeDef;

/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
//uint8_t Demo_Init(void);  vvvv
//void Display_Menu(uint8_t ForematIndex, uint8_t MaxForematIndex); vvvv
void Camera_Config(void);
void OV2640_SpecialEffects(uint8_t index);
void Delay(uint32_t nTime);
void TimingDelay_Decrement(void);
//void Demo_LCD_Clear(void); vvvvv

#endif /* __CAMERA_API_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
