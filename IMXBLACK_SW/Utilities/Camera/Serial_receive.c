#include "stdlib.h"
#include "stdio.h"
#include "string.h"
#include "stm32f4xx.h"                  // Device header
#include "main.h"
#include "serial_receive.h"
#include "stm32f4xx_dcmi.h"
#include "stm32f4xx_rcc.h"
//#include "thread.h"
//#include "windows.h"
#include <stdio.h>
//#include "conio.h"
#include "stm32f4xx_tim.h"

//  USART_ReceiveData(USART_TypeDef* USARTx)
//  USART_SendData(USART_TypeDef* USARTx, uint16_t Data)
//	USART_DMACmd(UART4,USART_DMAReq_Tx, ENABLE);     //DMA Request TX
//	USART_DMACmd(UART4,USART_DMAReq_Rx, ENABLE);     //DMA Request TX

#define START_ISR EXTI_Line0;

char buffer[1024];
char temp[1024],temp1[1024];
int cnt_f,cnt_s;
FILE *fp1,*fp2;
int START_O=0,STOP_O=0;
extern uint8_t frame_buffer[];
extern void TIMER_START(uint16_t numOfMilleseconds);
uint8_t stop_fn();
void imagesent(void);
void stop_capture(void);
void UART_SendFrame(void);

void open_stm32()
{
 
	fp1=fopen("test.jpg","r+");
	if(fp1<0)
	{
	   printf("File Open Failed");
	}

}
static int count_filename;
static int Interval_timer=0;

void receive_imxfilename()
{
	int i=0,j=0,k=0;
	char cmd_st[6],dat,cmd_res[3]={"ok"};
	char file_name[20];
	
	memset(temp,0,100);
	memset(temp1,0,100);

//FILENAME	
	while((dat=(USART_ReceiveData(UART4)!='\0')))
	{
		buffer[i]=dat;
		i++;
	};
	
	if((strcmp(buffer,"start")) !=0)
	{	
	//strcmp(buffer,temp);
	strcpy(temp,buffer); //Cp the file name to temp Here
	strcat(temp,".jpg"); //Add .jpg with  file name 
	printf("receiveFile Name  temp=%s",temp);
	//send Ok response to IMX
	for (i=0;i<3;i++)
	{
	USART_SendData(UART4,cmd_res[i]);
//		USART_DMACmd(UART4,USART_DMAReq_Tx, ENABLE);
	}
 }
	
 //FILE OPEN
 //   fp2 = fopen(temp, "w+");   //File open with Write access
 
 //INTERVAL//
	while((dat=(USART_ReceiveData(UART4)!='\0')))
	{
		buffer[j]=dat;
		j++;
	};
	
	Interval_timer=atoi(buffer);
	printf("receiveInterval_timer=%d",Interval_timer);
	
	if(Interval_timer !=0)
	{	
	//send Ok response to IMX
	for (j=0;j<3;j++)
	{
	USART_SendData(UART4,cmd_res[j]);
//		USART_DMACmd(UART4,USART_DMAReq_Tx, ENABLE);
	}
 }
	
//CAPTURE START 
 while((dat=(USART_ReceiveData(UART4)!='\0')))
	{
		buffer[k]=dat;
		k++;
	};
	
	if((strcmp(buffer,"start")) ==0)
	{	
	printf("receiveSTART=%s",buffer);
			//send Ok response to IMX
	for (k=0;k<3;k++)
	{
	USART_SendData(UART4,cmd_res[k]);
//		USART_DMACmd(UART4,USART_DMAReq_Tx, ENABLE);
	}	
  DMA_Cmd(DMA2_Stream1, ENABLE); 
  DCMI_Cmd(ENABLE);
	capture_start();
 }
}


int m;
uint8_t stop_fn()
{
	char dat,cmd_res[3]={"ok"};
   while((dat=(USART_ReceiveData(UART4)!='\0')))
	  {
		  buffer[m]=dat;
		  m++;
	  };
	if((strcmp(buffer,"stop")) ==0)
	{	
	printf("receiveSTOP=%s",buffer);
			//send Ok response to IMX
	  for (m=0;m<3;m++)
	  {
	    USART_SendData(UART4,cmd_res[m]);
//		USART_DMACmd(UART4,USART_DMAReq_Tx, ENABLE);
	  }
   return 0;		
  }
 return 1;
}



void stop_capture()
{
	DMA_Cmd(DMA2_Stream1,DISABLE); 
  DCMI_Cmd(DISABLE);
  DCMI_CaptureCmd(DISABLE); 
	//Clear the External Interrupt bit
	EXTI_ClearITPendingBit(EXTI_Line0);
	
	goto Again_start;
	Again_start:receive_imxfilename();	
}


void capture_start()
 {	
	// system("gst-launch v4l2src num-buffers=1 ! video/x-raw-rgb,width=640,height=480,framerate=8/1 ! ffmpegcolorspace ! jpegenc ! filesink location=test.jpg");
	    do
			 {
//FILE OPEN
       fp2 = fopen(temp, "w+");   //File open with Write access
				 
			 TIMER_START(Interval_timer); //ms
       DCMI_CaptureCmd(ENABLE);
			 EXTI_GenerateSWInterrupt(EXTI_Line0); //Set the SW interrupt for the external Line 0		 
			 snprintf(temp1,sizeof(temp1),"%d_%s",count_filename,temp);
			 count_filename++;
			 imagesent();	 
			 }while(stop_fn!=0);
			 
	 	 stop_capture();
 }       


 int n;
 char buffer[];
	void imagesent()
	{
	 int filelen,i;	
		uint8_t vv;

  //   fp2 = fopen(temp1, "r+");
                //get file size
	//	fprintf(fp2,"%s","This is just an example :)");
	              filelen=strlen(temp1);
		            for(i=0; i<filelen; i++)
                {		
                USART_SendData(UART4,temp1[i]);
                }	
								
//  							filelen=sizeof(frame_buffer)/sizeof(frame_buffer[614440]);
								
                for(i=0;i<filelen;i++)
                {								
								fwrite(frame_buffer,1,filelen,fp2 );
                }									
								fseek(fp2, 0, SEEK_END);
                filelen = ftell(fp2);
                fseek(fp2, 0, SEEK_SET);							
                //read file contents
//               filelen=sizeof(frame_buffer[614400]);  //Check
								USART_SendData(UART4,filelen);							
                fread(buffer,filelen,1,fp2);
                fclose(fp2);
								
                 for(i=0; i<=filelen; i++)
                {		
                USART_SendData(UART4,buffer[i]);
                }
						
	}
	

	
void DCMI_0V5670_PWDN_Init()
{
 GPIO_InitTypeDef GPIO_InitStructure;

 /* Enable GPIOs clocks */
 RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG, ENABLE);

 GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7 ;
 GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
 GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
 GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
 GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; 
 GPIO_Init(GPIOG, &GPIO_InitStructure);

	/*RESET*/
 GPIO_SetBits(GPIOG, GPIO_Pin_6);
 Delay(10);
 GPIO_ResetBits(GPIOG, GPIO_Pin_6);
 /*PWDN*/
 GPIO_ResetBits(GPIOG, GPIO_Pin_7);
 Delay(10);
 GPIO_SetBits(GPIOG, GPIO_Pin_7); 
}
	


// EXTI for PushButton

void EXTI0_IRQHandler(void)
{
if(EXTI_GetITStatus(EXTI_Line0) != RESET)
{

UART_SendFrame();
/* Clear the EXTI line 1 pending bit */
EXTI_ClearITPendingBit(EXTI_Line0);
}
}
/**********************************************************************/


void UART_SendFrame(void)
{
  int i,filelen=0;
	uint16_t Buff_Size=0;
	filelen=strlen(temp1);
	for(i=0; i<filelen; i++)
  {		
   USART_SendData(UART4,temp1[i]);
  }
	Buff_Size=sizeof(frame_buffer[614400]);  //Check
	USART_SendData(UART4,Buff_Size);
	
//for(i=0;i<50688;i++)
	for(i=0;i<614400;i++)
  {
//   USART_SendData(UART4,(uint16_t)frame_buffer[i]);
		 USART_SendData(UART4,frame_buffer[i]);
  }

#if 0
	/******Clear the Frame Buffer**************/
  for(i=0;i<614400;i++)
  {
   frame_buffer[i]=0;
  }
#endif
}
	
	
	int Serialcongig()
	{
 //   open_stm32();
		receive_imxfilename();
 //   imagesent();
	  return 0;
	}
	
	
	
	
#if 0	
int ImageFlag;	
void DMA2_Stream1_IRQHandler(void)
{
  if(DMA_GetITStatus(DMA2_Stream1, DMA_IT_TCIF1) != RESET)
  {
    // Clear interrupt pending bits
    DMA_ClearITPendingBit(DMA2_Stream1, DMA_IT_TCIF1);
    DCMI_CaptureCmd(DISABLE); 
    DCMI_Cmd(DISABLE); 
    // Image Buffer filled
    ImageFlag = 1;
  }
  else  if(DMA_GetITStatus(DMA2_Stream1 ,DMA_IT_TEIF1) != RESET)
  {
    // Clear interrupt pending bit
    DMA_ClearITPendingBit(DMA2_Stream1, DMA_IT_TEIF1);

    // Image Buffer Error
    ImageFlag = 0;	
	//	 NVIC_InitStructure.NVIC_IRQChannel = DMA_STREAM_IRQ;
  }
}
	
#endif
	
	
	
	
	
	
	
	