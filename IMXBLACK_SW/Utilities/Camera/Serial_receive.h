#ifndef __SERIAL_RECEIVE_H
#define __SERIAL_RECEIVE_H

#ifdef __cplusplus
 extern "C" {
#endif

	 
	 
	 
	 

int Serialcongig();	
void capture_start();	 
void open_stm32();
//void receive_imx();
void receive_imx_start();
void receive_imx_stop();
void receive_imx_send();
void receive_imxfilename();	 
void DCMI_0V5670_PWDN_Init(void);	 
	 
	 

	 
	 
 
#endif	 