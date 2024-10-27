#ifndef __BLUETOOTH_H
#define __BLUETOOTH_H 
#include "sys.h"	  	
#include "led.h"
#include "pid.h"

/***蓝牙模块代码逻辑 ：
 * 1、在中断中接收数据包
 * 2、在主循环中解析数据包，并执行对应操作
 */



#define Blutooth_Rcc_GPIOx   RCC_APB2Periph_GPIOA	//蓝牙串口对应GPIO口的时钟
#define Blutooth_Rcc_USARTx  RCC_APB2Periph_USART1	//蓝牙串口对应串口的时钟
#define Blutooth_Port        GPIOA					//蓝牙串口对应GPIO口
#define Blutooth_TX          GPIO_Pin_9				//
#define Blutooth_RX          GPIO_Pin_10			//蓝牙串口对应GPIO位
#define Blutooth_USARTx		 USART1
#define Blutooth_IRQn		 USART1_IRQn
#define Blutooth_IRQHandler  USART1_IRQHandler




// 数据包格式
typedef struct {
    uint8_t *BUF_P;
    uint8_t Checksum;
    uint8_t date1;
    uint8_t date2;
    uint8_t date_head;
    uint8_t flag;
} Bluetooth_packet_t;

extern Bluetooth_packet_t bluetooth_packet;

// 函数声明
void bluetooth_uart_init(u32 bound);//初始化基础值，例如指向存储数据数组的指针和该数据包帧头
void bluetooth_packet_init(Bluetooth_packet_t bluetooth_packet,uint8_t date_head,uint8_t *BUF_P);
void USART1_IRQHandler(void);
void blutooth_control(Bluetooth_packet_t bluetooth_packet);
int Get_packet_date(char* uartRxBuffer,  Bluetooth_packet_t bluetooth_packet);//当中断读取完一次数据包时，调用函数将数据包的date1和date2写入结构体中

#endif
