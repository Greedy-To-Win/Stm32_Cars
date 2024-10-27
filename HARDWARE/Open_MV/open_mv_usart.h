#include "sys.h"
//串口2：A2-TXD,A3-RX

#define OPENMV_USART_PACKET_BUF_LEN 10//openmv串口数据包缓冲区长度------略大于数据包长度

#define Fput_to_openmv    1
#define Fput_to_bluetooth 0

#define USE_OPENMV 		0
#define USE_Vofa		0 //这里两个只能开一个.不然由于变量定义会报错


#define OpenMV_USART_Rcc_GPIOx   RCC_APB2Periph_GPIOA	//OpenMV串口对应GPIO口的时钟
#define OpenMV_USART_Rcc_USARTx  RCC_APB1Periph_USART2	//OpenMV串口对应串口的时钟
#define OpenMV_USART_Port        GPIOA					//OpenMV串口对应GPIO口
#define OpenMV_USART_TX          GPIO_Pin_2			    //
#define OpenMV_USART_RX          GPIO_Pin_3 			//OpenMV串口对应GPIO位
#define OpenMV_USART_USARTx		 USART2
#define OpenMV_USART_IRQn		 USART2_IRQn
#define OpenMV_USART_IRQHandler  USART2_IRQHandler

#define OpenMV_DMA_Channel       DMA1_Channel6
#define OpenMV_DMA_IRQn          DMA1_Channel6_IRQn
#define OpenMV_DMA_IRQHandler    DMA1_Channel6_IRQHandler
#define OpenMV_Rcc_DMA           RCC_AHBPeriph_DMA1

//DMA中断标志位
#define DMAx_IT_TCx              DMA1_IT_TC6//DMA传输完成
#define DMAx_IT_HTx              DMA1_IT_HT6//DMA半满
#define DMAx_IT_TEx              DMA1_IT_TE6//DMA错误



typedef struct 
{


    uint8_t date1;
    uint8_t date2;
    uint8_t date3;
    uint8_t date4;
    
//    uint8_t date_tail;
    uint8_t flag;
    /* data */
}Openmv_usart_packet_t;
typedef struct {
    uint8_t Vofa_Rx_Blance_kp;
    uint8_t Vofa_Rx_Blance_kd;
    uint8_t Vofa_Rx_Speed_kp;
    uint8_t Vofa_Rx_Turn_kp;
    uint8_t id;//4为直立环kd
    uint8_t Rx_point;//如果Flag
	uint8_t Rx_num;  //数据数量
	uint8_t RxState;//接收状态位
	uint8_t RxPacket[8];
	uint8_t contrl_flag;
} Vofa_packet_t;
extern Vofa_packet_t vofa_packet;
extern Openmv_usart_packet_t openmv_tracking_packet;

void openmv_usart_init(u32 bound);
float Get_error_form_openmv(Openmv_usart_packet_t openmv_tracking_packet);


void  Get_vofa_date_to_pid(Vofa_packet_t* vofa_packet);
float Pow_invert(uint8_t X,uint8_t n);//x除以n次10
//void USART2_DMA_Init(void);