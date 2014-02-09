/******************** (C) COPYRIGHT 2010 STMicroelectronics ********************
* File Name          : fsmc_nand.h
* Author             : MCD Application Team
* Version            : V3.2.1
* Date               : 07/05/2010
* Description        : Header for fsmc_nand.c file.
********************************************************************************
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __FSMC_NAND_H
#define __FSMC_NAND_H

/* Includes ------------------------------------------------------------------*/
//#include "stm32f10x.h"
//#include "stm32f10x_lib.h"

#include "stm32f4xx.h"                  
#include "stm32f4xx_fsmc.h"
#include "stm32f4xx_rcc.h"

//#define FSMC_Bank_NAND     FSMC_Bank2_NAND
#define Bank_NAND_ADDR     Bank2_NAND_ADDR 
#define Bank2_NAND_ADDR    ((uint32_t)0x70000000)


#if 1  //STM32F4xx
#define FSMC_Bank_NAND   FSMC_Bank2_NAND //FSMC_Bank3_NAND
/* PORT D */
#define NAND_CLE        GPIO_Pin_11         /* FSMC */
#define NAND_CLE_s      GPIO_PinSource11    /* FSMC */
#define NAND_ALE        GPIO_Pin_12         /* FSMC */
#define NAND_ALE_s      GPIO_PinSource12    /* FSMC */
#define NAND_D0         GPIO_Pin_14         /* FSMC */
#define NAND_D0_s       GPIO_PinSource14    /* FSMC */
#define NAND_D1         GPIO_Pin_15         /* FSMC */
#define NAND_D1_s       GPIO_PinSource15    /* FSMC */
#define NAND_D2         GPIO_Pin_0          /* FSMC */
#define NAND_D2_s       GPIO_PinSource0     /* FSMC */
#define NAND_D3         GPIO_Pin_1          /* FSMC */
#define NAND_D3_s       GPIO_PinSource1     /* FSMC */
#define NAND_NOE        GPIO_Pin_4          /* FSMC */
#define NAND_NOE_s      GPIO_PinSource4     /* FSMC */
#define NAND_NWE        GPIO_Pin_5          /* FSMC */
#define NAND_NWE_s      GPIO_PinSource5     /* FSMC */
#define NAND_NWAIT      GPIO_Pin_6          /* FSMC */
#define NAND_NWAIT_s    GPIO_PinSource6     /* FSMC */
//CHIP SELECT
#define NAND_NCE2       GPIO_Pin_7          /* FSMC when BANK 2 is used */
#define NAND_NCE2_s     GPIO_PinSource7     /* FSMC when BANK 2 is used */

//#define NAND_NCE2   GPIO_Pin_7  /* FSMC when BANK 2 is used */
/* PORT E */
#define NAND_D4         GPIO_Pin_7          /* FSMC */
#define NAND_D4_s       GPIO_PinSource7     /* FSMC */
#define NAND_D5         GPIO_Pin_8          /* FSMC */
#define NAND_D5_s       GPIO_PinSource8     /* FSMC */
#define NAND_D6         GPIO_Pin_9          /* FSMC */
#define NAND_D6_s       GPIO_PinSource9     /* FSMC */
#define NAND_D7         GPIO_Pin_10         /* FSMC */
#define NAND_D7_s       GPIO_PinSource10    /* FSMC */
/* PORT G */
#define NAND_NCE3       GPIO_Pin_9          /* FSMC when BANK 3 is used */
#define NAND_NCE3_s     GPIO_PinSource9     /* FSMC when BANK 3 is used */

#endif //STM32F4xx











/* Exported types ------------------------------------------------------------*/
typedef struct
{
  uint8_t Maker_ID;
  uint8_t Device_ID;
  uint8_t Third_ID;
  uint8_t Fourth_ID;
}NAND_IDTypeDef;

typedef struct 
{
  uint16_t Zone;
  uint16_t Block;
  uint16_t Page;
} NAND_ADDRESS;

/* Exported constants --------------------------------------------------------*/
/* NAND Area definition  for STM3210E-EVAL Board RevD */
#define CMD_AREA                   (uint32_t)(1<<16)  /* A16 = CLE  high */
#define ADDR_AREA                  (uint32_t)(1<<17)  /* A17 = ALE high */

#define DATA_AREA                  ((uint32_t)0x00000000)

/* FSMC NAND memory command */
#define	NAND_CMD_AREA_A            ((uint8_t)0x00)
#define	NAND_CMD_AREA_B            ((uint8_t)0x01)
#define NAND_CMD_AREA_C            ((uint8_t)0x50)
#define NAND_CMD_AREA_TRUE1        ((uint8_t)0x30)

#define NAND_CMD_WRITE0            ((uint8_t)0x80)
#define NAND_CMD_WRITE_TRUE1       ((uint8_t)0x10)
	
#define NAND_CMD_ERASE0            ((uint8_t)0x60)
#define NAND_CMD_ERASE1            ((uint8_t)0xD0)

#define NAND_CMD_READID            ((uint8_t)0x90)
#define NAND_CMD_STATUS            ((uint8_t)0x70)
#define NAND_CMD_LOCK_STATUS       ((uint8_t)0x7A)
#define NAND_CMD_RESET             ((uint8_t)0xFF)

/* NAND memory status */
#define NAND_VALID_ADDRESS         ((uint32_t)0x00000100)
#define NAND_INVALID_ADDRESS       ((uint32_t)0x00000200)
#define NAND_TIMEOUT_ERROR         ((uint32_t)0x00000400)
#define NAND_BUSY                  ((uint32_t)0x00000000)
#define NAND_ERROR                 ((uint32_t)0x00000001)
#define NAND_READY                 ((uint32_t)0x00000040)

/* FSMC NAND memory parameters */
#define NAND_PAGE_SIZE             ((uint16_t)0x0200) /* 512 bytes per page w/o Spare Area */
#define NAND_BLOCK_SIZE            ((uint16_t)0x0020) /* 32x512 bytes pages per block */
#define NAND_ZONE_SIZE             ((uint16_t)0x0400) /* 1024 Block per zone */
#define NAND_SPARE_AREA_SIZE       ((uint16_t)0x0010) /* last 16 bytes as spare area */
#define NAND_MAX_ZONE              ((uint16_t)0x0004) /* 4 zones of 1024 block */

/* FSMC NAND memory address computation */
#define ADDR_1st_CYCLE(ADDR)       (uint8_t)((ADDR)& 0xFF)               /* 1st addressing cycle */
#define ADDR_2nd_CYCLE(ADDR)       (uint8_t)(((ADDR)& 0xFF00) >> 8)      /* 2nd addressing cycle */
#define ADDR_3rd_CYCLE(ADDR)       (uint8_t)(((ADDR)& 0xFF0000) >> 16)   /* 3rd addressing cycle */
#define ADDR_4th_CYCLE(ADDR)       (uint8_t)(((ADDR)& 0xFF000000) >> 24) /* 4th addressing cycle */

/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
//void FSMC_NAND_Init(void);
void FSMC_MT29F1_NAND_Init(void);
void FSMC_NAND_ReadID(NAND_IDTypeDef* NAND_ID);
uint32_t FSMC_NAND_WriteSmallPage(uint8_t *pBuffer, NAND_ADDRESS Address, uint32_t NumPageToWrite);
uint32_t FSMC_NAND_ReadSmallPage (uint8_t *pBuffer, NAND_ADDRESS Address, uint32_t NumPageToRead);
uint32_t FSMC_NAND_WriteSpareArea(uint8_t *pBuffer, NAND_ADDRESS Address, uint32_t NumSpareAreaTowrite);
uint32_t FSMC_NAND_ReadSpareArea(uint8_t *pBuffer, NAND_ADDRESS Address, uint32_t NumSpareAreaToRead);
uint32_t FSMC_NAND_EraseBlock(NAND_ADDRESS Address);
uint32_t FSMC_NAND_Reset(void);
uint32_t FSMC_NAND_GetStatus(void);
uint32_t FSMC_NAND_ReadStatus(void);
uint32_t FSMC_NAND_AddressIncrement(NAND_ADDRESS* Address);

#endif /* __FSMC_NAND_H */

/******************* (C) COPYRIGHT 2010 STMicroelectronics *****END OF FILE****/
