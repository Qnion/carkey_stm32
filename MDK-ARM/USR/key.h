#ifndef __KEY_H
#define __KEY_H	 
//#include "sys.h"
#include "stm32f1xx_hal.h"
//////////////////////////////////////////////////////////////////////////////////	 
//±¾³ÌÐòÖ»¹©Ñ§Ï°Ê¹ÓÃ£¬Î´¾­×÷ÕßÐí¿É£¬²»µÃÓÃÓÚÆäËüÈÎºÎÓÃÍ¾
//ALIENTEK miniSTM32¿ª·¢°å
//°´¼üÇý¶¯´úÂë	   
//ÕýµãÔ­×Ó@ALIENTEK
//¼¼ÊõÂÛÌ³:www.openedv.com
//ÐÞ¸ÄÈÕÆÚ:2012/9/3
//°æ±¾£ºV1.0
//°æÈ¨ËùÓÐ£¬µÁ°æ±Ø¾¿¡£
//Copyright(C) ¹ãÖÝÊÐÐÇÒíµç×Ó¿Æ¼¼ÓÐÏÞ¹«Ë¾ 2009-2019
//All rights reserved									  
//////////////////////////////////////////////////////////////////////////////////   	 


//#define KEY0 PCin(5)   	
//#define KEY1 PAin(15)	 
//#define WK_UP  PAin(0)	 
 

#define KEY0  HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_5)//¶ÁÈ¡°´¼ü0GPIO_PIN_9
#define KEY1  HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_15)//¶ÁÈ¡°´¼ü1
#define WK_UP   HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_0)//¶ÁÈ¡°´¼ü2 
 

#define KEY0_PRES	1		//KEY0  
#define KEY1_PRES	2		//KEY1 
#define WKUP_PRES	3		//WK_UP  

void KEY_Init(void);//IO³õÊ¼»¯
u8 KEY_Scan(u8 mode);  	//°´¼üÉ¨Ãèº¯Êý					    
#endif
