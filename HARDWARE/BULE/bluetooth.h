#ifndef __BLUETOOTH_H
#define __BLUETOOTH_H 
#include "sys.h"	  	
#include "led.h"
#include "pid.h"

/***����ģ������߼� ��
 * 1�����ж��н������ݰ�
 * 2������ѭ���н������ݰ�����ִ�ж�Ӧ����
 */



#define Blutooth_Rcc_GPIOx   RCC_APB2Periph_GPIOA	//�������ڶ�ӦGPIO�ڵ�ʱ��
#define Blutooth_Rcc_USARTx  RCC_APB2Periph_USART1	//�������ڶ�Ӧ���ڵ�ʱ��
#define Blutooth_Port        GPIOA					//�������ڶ�ӦGPIO��
#define Blutooth_TX          GPIO_Pin_9				//
#define Blutooth_RX          GPIO_Pin_10			//�������ڶ�ӦGPIOλ
#define Blutooth_USARTx		 USART1
#define Blutooth_IRQn		 USART1_IRQn
#define Blutooth_IRQHandler  USART1_IRQHandler




// ���ݰ���ʽ
typedef struct {
    uint8_t *BUF_P;
    uint8_t Checksum;
    uint8_t date1;
    uint8_t date2;
    uint8_t date_head;
    uint8_t flag;
} Bluetooth_packet_t;

extern Bluetooth_packet_t bluetooth_packet;

// ��������
void bluetooth_uart_init(u32 bound);//��ʼ������ֵ������ָ��洢���������ָ��͸����ݰ�֡ͷ
void bluetooth_packet_init(Bluetooth_packet_t bluetooth_packet,uint8_t date_head,uint8_t *BUF_P);
void USART1_IRQHandler(void);
void blutooth_control(Bluetooth_packet_t bluetooth_packet);
int Get_packet_date(char* uartRxBuffer,  Bluetooth_packet_t bluetooth_packet);//���ж϶�ȡ��һ�����ݰ�ʱ�����ú��������ݰ���date1��date2д��ṹ����

#endif
