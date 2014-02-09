/**
  ******************************************************************************
  * @file    DCMI/DCMI_CameraExample/camera_api.c 
  * @author  MCD Application Team
  * @version V1.3.0
  * @date    13-November-2013
  * @brief   This file contains the routinue needed to configure OV9655/OV2640 
  *          Camera modules.
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

/* Includes ------------------------------------------------------------------*/

#include "stm32f4xx.h"
#include "camera_api.h"
#include "dcmi_ov5640.h"
#include "stm32f4xx_dcmi.h"
#include "stm32f4xx_rcc.h"

/** @addtogroup STM32F4xx_StdPeriph_Examples
  * @{
  */

/** @addtogroup DCMI_CameraExample
  * @{
  */ 

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Image Formats */
const uint8_t *ImageForematArray[] =
{
  (uint8_t*)"BMP QQVGA Format    ",
  (uint8_t*)"BMP QVGA Format     ",
};
static __IO uint32_t TimingDelay;
__IO uint32_t PressedKey = 0;
uint8_t ValueMin = 0, ValueMax = 0;
Camera_TypeDef Camera;
ImageFormat_TypeDef ImageFormat;

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/


/**
  * @brief  Configures OV5640 Camera module
  * @param  ImageBuffer: Pointer to the camera configuration structure
  * @retval None
  */
void Camera_Config(void)
{
   if(Camera == OV5640_CAMERA)
  {
    switch (ImageFormat)
    {
      case RAW_320x240:
      {
        /* Configure the OV5640 camera and set the RAW_320x240 */
				OV5640_HW_Init();
				OV5640_Init(RAW_320x240);
				OV5640_RAW_320_240();

        break;
      }
      case RAW_640x480:
      {
        /* Configure the OV2640 camera and set the QQVGA mode */
				OV5640_HW_Init();
				OV5640_Init(RAW_640x480);
        OV5640_RAW_640_480();
        break;
      }
			case RAW_1280_720:
      {
        /* Configure the OV2640 camera and set the RAW_1280_720 */
				OV5640_HW_Init();
				OV5640_Init(RAW_1280_720);
        OV5640_RAW_1280_720();
        break;
      }
			
			case RAW_1920_1080:
      {
        /* Configure the OV2640 camera and set the QQVGA mode */
				OV5640_HW_Init();
				OV5640_Init(RAW_1920_1080);
        OV5640_RAW_1920_1080();
        break;
      }
			
			case RAW_5MP_QSXGA_2592_1944:
      {
        /* Configure the OV2640 camera and set the QQVGA mode */
				OV5640_HW_Init();
				OV5640_Init(RAW_5MP_QSXGA_2592_1944);
        OV5640_RAW_5MP_QSXGA_2592_1944();
        break;
      }
			
			case JPEG_1280_720:
      {
        /* Configure the OV2640 camera and set the QQVGA mode */
				OV5640_HW_Init();
				OV5640_Init(JPEG_1280_720);
        OV5640_JPEG_1280_720();
        break;
      }
			
			case JPEG_1920_1080:
      {
        /* Configure the OV2640 camera and set the QQVGA mode */
				OV5640_HW_Init();
				OV5640_Init(JPEG_1920_1080);
        OV5640_JPEG_1920_1080();
        break;
      }
			case JPEG_5MP_2592_1944:
      {
        /* Configure the OV2640 camera and set the QQVGA mode */
				OV5640_HW_Init();
				OV5640_Init(JPEG_5MP_2592_1944);
        OV5640_JPEG_5MP_2592_1944();
        break;
      }
					
      default:
      {
        /* Configure the OV2640 camera and set the QQVGA mode */
			  OV5640_HW_Init();
				OV5640_Init(RAW_640x480);
        OV5640_RAW_640_480();
        break; 
      }
    }
  }
}

#if 0
/**
  * @brief  OV2640 camera special effects.
* @param  index: 
  * @retval None
  */
void OV2640_SpecialEffects(uint8_t index)
{
  switch (index)
  {
    case 1:
    {
 //     LCD_DisplayStringLine(LINE(16), (uint8_t*)" Antique               ");
    OV2640_ColorEffectsConfig(0x40, 0xa6);/* Antique */ 
      break;
    }
    case 2:
    {
 //     LCD_DisplayStringLine(LINE(16), (uint8_t*)" Bluish                ");
      OV2640_ColorEffectsConfig(0xa0, 0x40);/* Bluish */
      break;
    }
    case 3:
    {
  //    LCD_DisplayStringLine(LINE(16), (uint8_t*)" Greenish              ");
      OV2640_ColorEffectsConfig(0x40, 0x40);/* Greenish */
      break;
    }
    case 4:
    {
  //    LCD_DisplayStringLine(LINE(16), (uint8_t*)" Reddish               ");
      OV2640_ColorEffectsConfig(0x40, 0xc0);/* Reddish */
      break;
    }
    case 5:
    {
   //   LCD_DisplayStringLine(LINE(16), (uint8_t*)" Black & White         ");
      OV2640_BandWConfig(0x18);/* Black & White */
      break;
    }
    case 6:
    {
 //     LCD_DisplayStringLine(LINE(16), (uint8_t*)" Negative              ");
      OV2640_BandWConfig(0x40);/* Negative */
      break;
    }
    case 7:
    {
 //     LCD_DisplayStringLine(LINE(16), (uint8_t*)" Black & White negative");
      OV2640_BandWConfig(0x58);/* B&W negative */
      break;
    }
    case 8:
    {
   //   LCD_DisplayStringLine(LINE(16), (uint8_t*)" Normal                ");
      OV2640_BandWConfig(0x00);/* Normal */
      break;
    }
    default:
      break;
  }
}

#endif

/**
  * @brief  Inserts a delay time.
  * @param  nTime: specifies the delay time length, in milliseconds
  * @retval None
  */
void Delay(uint32_t nTime)
{
  TimingDelay = nTime;

  while(TimingDelay != 0);

}

/**
  * @brief  Decrements the TimingDelay variable.
  * @param  None
  * @retval None
  */
void TimingDelay_Decrement(void)
{
  if (TimingDelay != 0x00)
  { 
    TimingDelay--;
  }
}


  
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
