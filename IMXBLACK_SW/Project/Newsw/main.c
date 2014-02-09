/**
  ******************************************************************************
  * @file    USART/USART_Printf/main.c 
  * @author  MCD Application Team
  * @version V1.3.0
  * @date    13-November-2013
  * @brief   Main program body
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
#include "stm32f4xx.h"                  // Device header
#include "main.h"
#include "stm32f4xx_usart.h"   //added by vv
#include "stm32f4xx_sdio.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_tim.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_fsmc.h"
#include "fsmc_nand.h"
#include "stm32f4xx_rcc.h"
#include "dcmi_ov5640.h"
#include "camera_api.h"
#include "dcmi_ov9655.h"
#include "stm324xg_eval_fsmc_sram.h"
#include "serial_receive.h"
#include "stm32f4xx_i2c.h"
#include "stm32f4xx_dcmi.h"


/** @addtogroup STM32F4xx_StdPeriph_Examples
  * @{
  */

/** @addtogroup USART_Printf
  * @{
  */ 

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/

// char RX_BUFF,RX_BUFF1[255];
// uint8_t rxbuff_pos=0; //Rx buffer POsition
 char buff_dat[255];   //DATA buffer
 char rxbuff_data[128];  //RX Buffer Data

#if 1   //VV SRAM

/** @addtogroup FSMC_SRAM
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define BUFFER_SIZE         ((uint32_t)0x0100)
#define WRITE_READ_ADDR     ((uint32_t)0x0800)

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Read/Write Buffers */

uint16_t aTxBuffer[BUFFER_SIZE];
uint16_t aRxBuffer[BUFFER_SIZE];

/* Status variables */
__IO uint32_t uwWriteReadStatus_SRAM = 0;

/* Counter index */
uint32_t uwIndex = 0;

/* Private function prototypes -----------------------------------------------*/
static void Fill_Buffer(uint16_t *pBuffer, uint32_t uwBufferLenght, uint16_t uhOffset);

/* Private variables ---------------------------------------------------------*/
I2C_InitTypeDef  I2C_InitStructure;
DMA_InitTypeDef  DMA_InitStructure;
uint8_t aTxBuffer1[] = "I2C DMA Example: Communication between two I2C using DMA";
uint8_t aRxBuffer1 [256];
__IO uint32_t TimeOut = 0x0;
RCC_ClocksTypeDef RCC_Clocks;

uint8_t HEADER_ADDRESS_Write = (((SLAVE_ADDRESS & 0xFF00) >> 7) | 0xF0);
uint8_t HEADER_ADDRESS_Read = (((SLAVE_ADDRESS & 0xFF00) >> 7) | 0xF1);
#define countof(a)   (sizeof(a) / sizeof(*(a)))  

static void I2C_Config(void);
/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Main program
  * @param  None
  * @retval None
  */


#endif   //VV SRAM
 
#if 1   //VV TIMER
TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
TIM_OCInitTypeDef  TIM_OCInitStructure;
__IO uint16_t CCR1_Val = 40961;
__IO uint16_t CCR2_Val = 27309;
__IO uint16_t CCR3_Val = 13654;
__IO uint16_t CCR4_Val = 6826;
uint16_t PrescalerValue = 0;

/* Private function prototypes -----------------------------------------------*/
void TIM_Config(void);
//void TIMER_START(void);
void TIMER_START(uint16_t numOfMilleseconds);
#endif  //VV TIMER



#if 1   //VV Camera
RCC_ClocksTypeDef RCC_Clocks;
OV9655_IDTypeDef  OV9655_Camera_ID;
OV2640_IDTypeDef  OV2640_Camera_ID;
OV5640_IDTypeDef OV5640_Camera_ID;
__IO uint16_t  uhADCVal = 0;
uint8_t        abuffer[40];
#define Camera_5MP 0x01
extern Camera_TypeDef       Camera;
extern ImageFormat_TypeDef  ImageFormat;
extern __IO uint8_t         ValueMax;
extern const uint8_t *      ImageForematArray[];

static void ADC_Config(void);

#endif  //VV Camera


	FSMC_NANDInitTypeDef* FSMC_NAND;
 	FSMC_NORSRAMInitTypeDef* FSMC_NORSRAM;


static void USART_Config(void);

#ifdef __GNUC__
  /* With GCC/RAISONANCE, small printf (option LD Linker->Libraries->Small printf
     set to 'Yes') calls __io_putchar() */
  #define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
  #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */
 

	
/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Main program
  * @param  None
  * @retval None
  */
	

int main(void)
{
  /*!< At this stage the microcontroller clock setting is already configured, 
       this is done through SystemInit() function which is called from startup
       files (startup_stm32f40_41xxx.s/startup_stm32f427_437xx.s/startup_stm32f429_439xx.s)
       before to branch to application main. 
       To reconfigure the default setting of SystemInit() function, refer to
       system_stm32f4xx.c file.
     */     

	uint8_t rxbuff_pos=0,rxbuff_pos1=0; //Rx buffer POsition
	char RX_BUFF,RX_BUFF1[255];
	int i,j;
	
  /* USART configuration */
  USART_Config();     
  /* Output a message on Hyperterminal using printf function */
  printf("\n\rUSART Printf Example: retarget the C library printf function to the USART\n\r");


	#if 1   //VV SRAM
	
  /* Initialize the SRAM memory */
  SRAM_Init();
  
  /* Fill the buffer to send */
 // Fill_Buffer(aTxBuffer, BUFFER_SIZE, 0x250F);
  
  /* Write data to the SRAM memory */
  SRAM_WriteBuffer(aTxBuffer, WRITE_READ_ADDR, BUFFER_SIZE);  
    
  /* Read back data from the SRAM memory */
  SRAM_ReadBuffer(aRxBuffer, WRITE_READ_ADDR, BUFFER_SIZE); 
   
  /* Check the SRAM memory content correctness */   
  for (uwIndex = 0; (uwIndex < BUFFER_SIZE) && (uwWriteReadStatus_SRAM == 0); uwIndex++)
  {
    if (aRxBuffer[uwIndex] != aTxBuffer[uwIndex])
    {
      uwWriteReadStatus_SRAM++;
    }
  }
 
	#endif  //VV SRAM

#if defined NAND_MT29F1G08
	
//	 FSMC_NANDStructInit(FSMC_NAND);
//	 FSMC_NANDInit(FSMC_NAND);
	 FSMC_MT29F1_NAND_Init();
//	FSMC_NAND_ReadSmallPage(uint8_t *pBuffer, NAND_ADDRESS Address, uint32_t NumPageToRead);
	//FSMC_NAND_WriteSmallPage(uint8_t *pBuffer, NAND_ADDRESS Address, uint32_t NumPageToWrite);
//FSMC_NAND_ReadID(NAND_IDTypeDef* NAND_ID);	
#endif	
#if 1  //VV TIMER
 /* TIM Configuration */
  TIM_Config();
//   TIMER_START();
#endif   //VV TIMER
	
	 I2C_Config();
	
/*************CAMERA OV5640**************************/	
	 /* SysTick end of count event each 10ms */
  RCC_GetClocksFreq(&RCC_Clocks);
  SysTick_Config(RCC_Clocks.HCLK_Frequency / 100);
  /* ADC configuration */
// ADC_Config();  //vv
	
	OV5640_HW_Init();
	OV5640_ReadID(&OV5640_Camera_ID);
	Camera=Camera_5MP;
	ImageFormat= RAW_320x240 ;
	DCMI_0V5670_PWDN_Init();//PWR DOWN and RESET
	Camera_Config();  //Inside Hw init Format init 
 // OV5640_Init(BMP_QQVGA);
 // OV5640_JPEGConfig(JPEG_1280_720);
	DMA_Cmd(DMA2_Stream1, ENABLE); 
  DCMI_Cmd(ENABLE);
/*************CAMERA OV5640**************************/
	
//UART4 PORT OPEN For Waiting  the camera Capture start/stop commands	
//  USART_Cmd(UART4, ENABLE);
//vv	USART_ReceiveData(UART4);
//vv	USART_DMACmd(UART4,USART_DMAReq_Tx, ENABLE);     //DMA Request TX
//	USART_DMACmd(UART4,USART_DMAReq_Rx, ENABLE);     //DMA Request TX

	while(1)
	{
	 RX_BUFF=USART_ReceiveData(UART4);
//	 EXTI_GetITStatus((EXTI_Line2) != RESET);
	 Serialcongig();	
// TIMER_START(50); //50 ms
	}
	

}


/**
  * @brief  Configures the USART Peripheral.
  * @param  None
  * @retval None
  */
static void USART_Config(void)
{
  USART_InitTypeDef USART_InitStructure;
  
  /* USARTx configured as follows:
        - BaudRate = 115200 baud  
        - Word Length = 8 Bits
        - One Stop Bit
        - No parity
        - Hardware flow control disabled (RTS and CTS signals)
        - Receive and transmit enabled
  */
  USART_InitStructure.USART_BaudRate = 115200;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

  //STM_EVAL_COMInit(COM1, &USART_InitStructure);  //vvcmded
	USART_Init(UART4, &USART_InitStructure);  //UART4
}

/**
  * @brief  Retargets the C library printf function to the USART.
  * @param  None
  * @retval None
  */
PUTCHAR_PROTOTYPE
{
  /* Place your implementation of fputc here */
  /* e.g. write a character to the USART */
 // USART_SendData(EVAL_COM1, (uint8_t) ch);  //VV cmd
	USART_SendData(UART4, (uint8_t) ch);
  /* Loop until the end of transmission */
 // while (USART_GetFlagStatus(EVAL_COM1, USART_FLAG_TC) == RESET)  //VV cmd
		 while (USART_GetFlagStatus(UART4, USART_FLAG_TC) == RESET)
  {}

  return ch;
}



/**
  * @brief  Fills buffer with user predefined 16 bit length data.
  * @param  pBuffer: pointer on the buffer to fill
  * @param  uwBufferLenght: size of the buffer to fill
  * @param  uhOffset: first value to fill on the buffer
  * @retval None
  */
static void Fill_Buffer(uint16_t *pBuffer, uint32_t uwBufferLenght, uint16_t uhOffset)
{
  uint32_t tmpIndex = 0;

  /* Put in global buffer different values */
  for (tmpIndex = 0; tmpIndex < uwBufferLenght; tmpIndex++ )
  {
    pBuffer[tmpIndex] = tmpIndex + uhOffset;
  }
}



void TIM_Config(void)
{
  NVIC_InitTypeDef NVIC_InitStructure;

  /* TIM3 clock enable */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

  /* Enable the TIM3 gloabal Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0; //Value 0-15//
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;  //Value 0-15//
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

}

//void TIMER_START()
void TIMER_START(uint16_t numOfMilleseconds)
{

  /* Compute the prescaler value */
  PrescalerValue = (uint16_t) ((SystemCoreClock / 2) / 6000000) - 1;

	/*
  // Time base configuration //
 // TIM_TimeBaseStructure.TIM_Period = 65535;
  TIM_TimeBaseStructure.TIM_Period = 50000;  //5s vv
 // TIM_TimeBaseStructure.TIM_Prescaler = 0;
	TIM_TimeBaseStructure.TIM_Prescaler = 84-1; //5s vv
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
*/
		
	// Time base configuration //
TIM_TimeBaseStructure.TIM_Period = (500*numOfMilleseconds) - 1; // 1 MHz down to 1 KHz (1 ms)
//TIM_TimeBaseStructure.TIM_Prescaler = 84 - 1; // 24 MHz Clock down to 1 MHz (adjust per your clock)
TIM_TimeBaseStructure.TIM_Prescaler = 168-1; //168MHz Clock should be down to 1MHz
TIM_TimeBaseStructure.TIM_ClockDivision = 0;
TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
	
  
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

	//TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
  TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

/* TIM IT enable */
  TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
  TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);

  /* TIM3 enable counter */
  TIM_Cmd(TIM3, ENABLE);

}

#if 0

void TIM2_IRQHandler(void)
{
if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET)
{
TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
GPIO_ToggleBits(GPIOA, GPIO_Pin_5);
TIM2->ARR = (500*10) - 1; /* Update the timer for 10 ms */
}
}

#endif

static void ADC_Config(void)
{
  ADC_InitTypeDef ADC_InitStructure;
  ADC_CommonInitTypeDef ADC_CommonInitStructure;
  GPIO_InitTypeDef GPIO_InitStructure;

  /* Enable ADC3 clock */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC3, ENABLE);

  /* GPIOF clock enable */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF, ENABLE); 

  /* Configure ADC Channel7 as analog */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9; 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
  GPIO_Init(GPIOF, &GPIO_InitStructure);

  /* ADC Common Init */
  ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;
  ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div6;
  ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;
  ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles; 
  ADC_CommonInit(&ADC_CommonInitStructure); 

  /* ADC3 Configuration ------------------------------------------------------*/
  ADC_StructInit(&ADC_InitStructure);
  ADC_InitStructure.ADC_Resolution = ADC_Resolution_8b;
  ADC_InitStructure.ADC_ScanConvMode = DISABLE;
  ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
  ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None; 
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
  ADC_InitStructure.ADC_NbrOfConversion = 1;
  ADC_Init(ADC3, &ADC_InitStructure);

  /* ADC3 Regular Channel Config */
  ADC_RegularChannelConfig(ADC3, ADC_Channel_7, 1, ADC_SampleTime_56Cycles);

  /* Enable ADC3 */
  ADC_Cmd(ADC3, ENABLE);

  /* ADC3 regular Software Start Conv */ 
  ADC_SoftwareStartConv(ADC3);
}



#if defined CAMERA_OV5640
void camera_ov5640_init()
{
  /* SysTick end of count event each 10ms */
  RCC_GetClocksFreq(&RCC_Clocks);
  SysTick_Config(RCC_Clocks.HCLK_Frequency / 100);

  /* ADC configuration */
  ADC_Config();

  /* Initializes the DCMI interface (I2C and GPIO) used to configure the camera */
  OV5640_HW_Init();

  /* Read the OV9655/OV2640 Manufacturer identifier */
  OV2640_ReadID(&OV2640_Camera_ID);

  if(OV2640_Camera_ID.PIDH  == 0x26)
  {
    Camera = OV2640_CAMERA;
    sprintf((char*)abuffer, "OV2640 Camera ID 0x%x", OV2640_Camera_ID.PIDH);
    ValueMax = 2;
  }
 
  /* Initialize demo */
  ImageFormat = (ImageFormat_TypeDef)Demo_Init();

  Camera_Config();

  sprintf((char*)abuffer, " Image selected: %s", ImageForematArray[ImageFormat]);
 

  /* Enable DMA2 stream 1 and DCMI interface then start image capture */
  DMA_Cmd(DMA2_Stream1, ENABLE); 
  DCMI_Cmd(ENABLE); 

  /* Insert 100ms delay: wait 100ms */
  Delay(200); 

  DCMI_CaptureCmd(ENABLE); 

 
  if(ImageFormat == BMP_QQVGA)
  {
    /* LCD Display window */
    LCD_SetDisplayWindow(179, 239, 120, 160);
    LCD_WriteReg(LCD_REG_3, 0x1038);
    LCD_WriteRAM_Prepare(); 
  }
  else if(ImageFormat == BMP_QVGA)
  {
    /* LCD Display window */
    LCD_SetDisplayWindow(239, 319, 240, 320);
    LCD_WriteReg(LCD_REG_3, 0x1038);
    LCD_WriteRAM_Prepare(); 
  }

  while(1)
  {
 
   /* Insert 100ms delay */
    Delay(10);
    /* Get the last ADC3 conversion result data */
    uhADCVal = ADC_GetConversionValue(ADC3);
 
    if(Camera == OV2640_CAMERA)
    {
      OV2640_BrightnessConfig(uhADCVal/2);
    }
  }


}

#endif

/**
  * @brief  Enables the I2C Clock and configures the different GPIO ports.
  * @param  None
  * @retval None
  */
static void I2C_Config(void)
{
  GPIO_InitTypeDef  GPIO_InitStructure;
    
  /* RCC Configuration */
  /*I2C Peripheral clock enable */
  RCC_APB1PeriphClockCmd(I2Cx_CLK, ENABLE);
  
  /*SDA GPIO clock enable */
  RCC_AHB1PeriphClockCmd(I2Cx_SDA_GPIO_CLK, ENABLE);
  
  /*SCL GPIO clock enable */
  RCC_AHB1PeriphClockCmd(I2Cx_SCL_GPIO_CLK, ENABLE);
  
  /* Reset I2Cx IP */
  RCC_APB1PeriphResetCmd(I2Cx_CLK, ENABLE);
  
  /* Release reset signal of I2Cx IP */
  RCC_APB1PeriphResetCmd(I2Cx_CLK, DISABLE);
  
  /* Enable the DMA clock */
  RCC_AHB1PeriphClockCmd(DMAx_CLK, ENABLE);
  
  /* GPIO Configuration */
  /*Configure I2C SCL pin */
  GPIO_InitStructure.GPIO_Pin = I2Cx_SCL_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
  GPIO_Init(I2Cx_SCL_GPIO_PORT, &GPIO_InitStructure);
  
  /*Configure I2C SDA pin */
  GPIO_InitStructure.GPIO_Pin = I2Cx_SDA_PIN;
  GPIO_Init(I2Cx_SDA_GPIO_PORT, &GPIO_InitStructure);
    
  /* Connect PXx to I2C_SCL */
  GPIO_PinAFConfig(I2Cx_SCL_GPIO_PORT, I2Cx_SCL_SOURCE, I2Cx_SCL_AF);
  
  /* Connect PXx to I2C_SDA */
  GPIO_PinAFConfig(I2Cx_SDA_GPIO_PORT, I2Cx_SDA_SOURCE, I2Cx_SDA_AF);

#if 0  //DMA

  /* DMA Configuration */
  /* Clear any pending flag on Tx Stream  */
  DMA_ClearFlag(I2Cx_DMA_STREAM_TX, I2Cx_TX_DMA_TCFLAG | I2Cx_TX_DMA_FEIFLAG | I2Cx_TX_DMA_DMEIFLAG | \
                                       I2Cx_TX_DMA_TEIFLAG | I2Cx_TX_DMA_HTIFLAG);
                                       
  /* Clear any pending flag on Rx Stream  */
  DMA_ClearFlag(I2Cx_DMA_STREAM_RX, I2Cx_RX_DMA_TCFLAG | I2Cx_RX_DMA_FEIFLAG | I2Cx_RX_DMA_DMEIFLAG | \
                                       I2Cx_RX_DMA_TEIFLAG | I2Cx_RX_DMA_HTIFLAG);
  
  /* Disable the I2C Tx DMA stream */
  DMA_Cmd(I2Cx_DMA_STREAM_TX, DISABLE);
  /* Configure the DMA stream for the I2C peripheral TX direction */
  DMA_DeInit(I2Cx_DMA_STREAM_TX);
  
  /* Disable the I2C Rx DMA stream */
  DMA_Cmd(I2Cx_DMA_STREAM_RX, DISABLE);
  /* Configure the DMA stream for the I2C peripheral RX direction */
  DMA_DeInit(I2Cx_DMA_STREAM_RX);
  
  /* Initialize the DMA_Channel member */
  DMA_InitStructure.DMA_Channel = I2Cx_DMA_CHANNEL;
  
  /* Initialize the DMA_PeripheralBaseAddr member */
  DMA_InitStructure.DMA_PeripheralBaseAddr = I2Cx_DR_ADDRESS;
  
  /* Initialize the DMA_PeripheralInc member */         
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  
  /* Initialize the DMA_MemoryInc member */
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
  
  /* Initialize the DMA_PeripheralDataSize member */
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
  
  /* Initialize the DMA_MemoryDataSize member */
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
  
  /* Initialize the DMA_Mode member */
  DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
  
  /* Initialize the DMA_Priority member */
  DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
  
  /* Initialize the DMA_FIFOMode member */
  DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Enable;
  
  /* Initialize the DMA_FIFOThreshold member */
  DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
  
  /* Initialize the DMA_MemoryBurst member */
  DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
  
  /* Initialize the DMA_PeripheralBurst member */
  DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
  
  /* Init DMA for Reception */
   /* Initialize the DMA_DIR member */
   DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
   /* Initialize the DMA_Memory0BaseAddr member */
   DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)aRxBuffer1;
   /* Initialize the DMA_BufferSize member */
   DMA_InitStructure.DMA_BufferSize = RXBUFFERSIZE;
   DMA_DeInit(I2Cx_DMA_STREAM_RX);
   DMA_Init(I2Cx_DMA_STREAM_RX, &DMA_InitStructure);
  
  /* Init DMA for Transmission */
   /* Initialize the DMA_DIR member */
   DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;
   /* Initialize the DMA_Memory0BaseAddr member */
   DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)aTxBuffer1;
   /* Initialize the DMA_BufferSize member */
   DMA_InitStructure.DMA_BufferSize = TXBUFFERSIZE;
   DMA_DeInit(I2Cx_DMA_STREAM_TX);
   DMA_Init(I2Cx_DMA_STREAM_TX, &DMA_InitStructure);
   
   /* Configure I2C Filters */
   I2C_AnalogFilterCmd(I2Cx,ENABLE);
   I2C_DigitalFilterConfig(I2Cx,0x0F);
   
  /* I2C ENABLE */
  //I2C_Cmd(I2Cx, ENABLE);
	#endif  //DMA
}




#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif





/**************************END OF FILE****/
