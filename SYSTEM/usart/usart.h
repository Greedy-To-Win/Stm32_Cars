#ifndef __USART_H
#define __USART_H
#include "stdio.h"	
#include "sys.h" 

//********************************************************************************
//V1.3�޸�˵�� 
//֧����Ӧ��ͬƵ���µĴ��ڲ���������.
//�����˶�printf��֧��
//�����˴��ڽ��������.
//������printf��һ���ַ���ʧ��bug
//V1.4�޸�˵��
//1,�޸Ĵ��ڳ�ʼ��IO��bug
//2,�޸���USART_RX_STA,ʹ�ô����������ֽ���Ϊ2��14�η�
//3,������USART_REC_LEN,���ڶ��崮�����������յ��ֽ���(������2��14�η�)
//4,�޸���EN_USART1_RX��ʹ�ܷ�ʽ
//V1.5�޸�˵��
//1,�����˶�UCOSII��֧��
#define USART_REC_LEN  			200  	//�����������ֽ��� 200
#define EN_USART1_RX 			1		//ʹ�ܣ�1��/��ֹ��0������1����

#define DeBug_Rcc_GPIOx   RCC_APB2Periph_GPIOB	//�������ڶ�ӦGPIO�ڵ�ʱ��
#define DeBug_Rcc_USARTx  RCC_APB1Periph_USART3	//�������ڶ�Ӧ���ڵ�ʱ��
#define DeBug_Port        GPIOB					//�������ڶ�ӦGPIO��
#define DeBug_TX          GPIO_Pin_10				//
#define DeBug_RX          GPIO_Pin_11			//�������ڶ�ӦGPIOλ
#define DeBug_USARTx	  USART3
#define DeBug_IRQn		  USART3_IRQn
#define DeBug_IRQHandler  USART3_IRQHandler

extern u8  USART_RX_BUF[USART_REC_LEN]; //���ջ���,���USART_REC_LEN���ֽ�.ĩ�ֽ�Ϊ���з� 
extern u16 USART_RX_STA;         		//����״̬���	
//����봮���жϽ��գ��벻Ҫע�����º궨��
void DeBug_Init(u32 bound);
void USART3_IRQHandler(void);
void Usart_SendArray( USART_TypeDef * pUSARTx, uint8_t *array, uint16_t num);
#endif


