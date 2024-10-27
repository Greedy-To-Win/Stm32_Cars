#ifndef __USART_H
#define __USART_H
#include "stdio.h"	
#include "sys.h" 

//********************************************************************************
//V1.3修改说明 
//支持适应不同频率下的串口波特率设置.
//加入了对printf的支持
//增加了串口接收命令功能.
//修正了printf第一个字符丢失的bug
//V1.4修改说明
//1,修改串口初始化IO的bug
//2,修改了USART_RX_STA,使得串口最大接收字节数为2的14次方
//3,增加了USART_REC_LEN,用于定义串口最大允许接收的字节数(不大于2的14次方)
//4,修改了EN_USART1_RX的使能方式
//V1.5修改说明
//1,增加了对UCOSII的支持
#define USART_REC_LEN  			200  	//定义最大接收字节数 200
#define EN_USART1_RX 			1		//使能（1）/禁止（0）串口1接收

#define DeBug_Rcc_GPIOx   RCC_APB2Periph_GPIOB	//蓝牙串口对应GPIO口的时钟
#define DeBug_Rcc_USARTx  RCC_APB1Periph_USART3	//蓝牙串口对应串口的时钟
#define DeBug_Port        GPIOB					//蓝牙串口对应GPIO口
#define DeBug_TX          GPIO_Pin_10				//
#define DeBug_RX          GPIO_Pin_11			//蓝牙串口对应GPIO位
#define DeBug_USARTx	  USART3
#define DeBug_IRQn		  USART3_IRQn
#define DeBug_IRQHandler  USART3_IRQHandler

extern u8  USART_RX_BUF[USART_REC_LEN]; //接收缓冲,最大USART_REC_LEN个字节.末字节为换行符 
extern u16 USART_RX_STA;         		//接收状态标记	
//如果想串口中断接收，请不要注释以下宏定义
void DeBug_Init(u32 bound);
void USART3_IRQHandler(void);
void Usart_SendArray( USART_TypeDef * pUSARTx, uint8_t *array, uint16_t num);
#endif


