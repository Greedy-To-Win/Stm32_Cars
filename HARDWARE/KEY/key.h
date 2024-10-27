#ifndef __KEY_H
#define __KEY_H	 
#include "sys.h"

#define KEY1  GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_4)//读取按键1
#define KEY2   GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_5)//读取按键2
#define KEY3  GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_6)//读取按键1
#define KEY4   GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_7)//读取按键2

	 
#define KEY1_PRES	1	
#define KEY2_PRES	2	 


void KEY_Init(void);//IO初始化
u8 KEY_Scan(u8 mode);  	//按键扫描函数		
void EXTI15_10_IRQHandler(void);     //外部中断函数
#endif
