#include "bluetooth.h"
#include "pid.h"
#include "usart.h"	
//��ʼ�����ݰ�
Bluetooth_packet_t bluetooth_packet;//�������ݰ�


uint8_t Bluetooth_RxBuf[5]={0};//�������ջ�����
uint8_t* BUF_P1=&Bluetooth_RxBuf[0];

/// @brief ��ʼ�����ݰ����ݣ���ҪΪ֡ͷ������Ҫ������������
/// @param bluetooth_packet 
/// @param date_head 
/// @param BUF_P 
void bluetooth_packet_init(Bluetooth_packet_t bluetooth_packet,uint8_t date_head,uint8_t *BUF_P)
{	
    bluetooth_packet.flag=0;
    bluetooth_packet.date_head=date_head;
    bluetooth_packet.BUF_P=BUF_P;
};

/**************************************************************************
�������ܣ��������ڳ�ʼ��
��ڲ����� bound:������
����  ֵ����
**************************************************************************/
void bluetooth_uart_init(u32 bound)
{  
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
/*******************GPIO�˿�����*******************************/
	
//��ʱ��	 
	RCC_APB2PeriphClockCmd(Blutooth_Rcc_GPIOx, ENABLE);	  //ʹ��UGPIOAʱ��
	RCC_APB1PeriphClockCmd(Blutooth_Rcc_USARTx, ENABLE);  //ʹ��USART1ʱ��
//USART1_TX  
	GPIO_InitStructure.GPIO_Pin = Blutooth_TX; 			  //PA9
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;		  //�����������
	GPIO_Init(Blutooth_Port, &GPIO_InitStructure);
   
//USART1_RX	  
	GPIO_InitStructure.GPIO_Pin = Blutooth_RX;			  //PA10
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING; //��������
	GPIO_Init(Blutooth_Port, &GPIO_InitStructure);
/*******************Usart1 NVIC ����*******************************/
	
NVIC_InitStructure.NVIC_IRQChannel = Blutooth_IRQn;
NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=1 ;//��ռ���ȼ�
NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;		//�����ȼ�
NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQͨ��ʹ��
NVIC_Init(&NVIC_InitStructure);				//����ָ���Ĳ�����ʼ��VIC�Ĵ���
/*******************USART1 ��ʼ������****************************/

	USART_InitStructure.USART_BaudRate = bound;					//���ڲ�����
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;	//�ֳ�Ϊ8λ���ݸ�ʽ
	USART_InitStructure.USART_StopBits = USART_StopBits_1;		//һ��ֹͣλ
	USART_InitStructure.USART_Parity = USART_Parity_No;			//����żУ��λ
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//��Ӳ������������
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//�շ�ģʽ
    USART_Init(Blutooth_USARTx, &USART_InitStructure);     //��ʼ������1
    USART_ITConfig(Blutooth_USARTx, USART_IT_RXNE, ENABLE);//�������ڽ����ж�
    USART_Cmd(Blutooth_USARTx, ENABLE);                    //ʹ�ܴ���1
    
	bluetooth_packet_init(bluetooth_packet,0xFF,BUF_P1);
}





/// @brief �������ݰ�
/// @param uartRxBuffer 
/// @param parsedData 
/// @return 0 or 1��ture or false��
int Get_packet_date(char* uartRxBuffer,Bluetooth_packet_t bluetooth_packet) {
    if (uartRxBuffer[0] != 0xFF) {
        printf("Error: Invalid frame header.\n");
        return -1; // �����֡ͷ
    }
    
    bluetooth_packet.Checksum = (uartRxBuffer[1] + uartRxBuffer[2])&& 0xFF;//У���*********?????????????
    if (bluetooth_packet.Checksum != uartRxBuffer[3]) {
        printf("Error: Checksum mismatch.\n");
        return -2; // У��Ͳ�ƥ��
    }
    // ���һ�������������ݴ���parsedData
    bluetooth_packet.date1= uartRxBuffer[1]; // �洢date1
    bluetooth_packet.date2= uartRxBuffer[2]; // �洢date2
    return 0; // �ɹ�
}





/**************************************************************************
�������ܣ�����1�����ж�
��ڲ�������
����  ֵ����
**************************************************************************/
__IO uint8_t usart_value=0;//����һ���ֽ����ݵı���
void Blutooth_IRQHandler(void)
{	
	if(USART_GetITStatus(Blutooth_USARTx, USART_IT_RXNE) != RESET) //���յ�����
	{	  
	 USART_ClearITPendingBit(Blutooth_USARTx,USART_IT_RXNE);//�����־λ
  	 usart_value=USART_ReceiveData(Blutooth_USARTx); 

	
	/************ʹ������****************/				
     if(bluetooth_packet.date_head==usart_value)//֡ͷ
     {
        *(bluetooth_packet.BUF_P)=usart_value;
        bluetooth_packet.BUF_P++;
        bluetooth_packet.flag=1;//�������ݽ���
     }
     if(1==bluetooth_packet.flag)//date1
     {
        *(bluetooth_packet.BUF_P)=usart_value;
        bluetooth_packet.BUF_P++;
        bluetooth_packet.flag=2;//�������ݽ���
     }
     if(2==bluetooth_packet.flag)//date2
     {
        *(bluetooth_packet.BUF_P)=usart_value;
        bluetooth_packet.BUF_P++;
        bluetooth_packet.flag=3;//�������ݽ���
     }
     if(3==bluetooth_packet.flag)//У���
     {
        *(bluetooth_packet.BUF_P)=usart_value;
        bluetooth_packet.flag=0;//�������ݽ���
       
        while(0!=Get_packet_date(Bluetooth_RxBuf,bluetooth_packet));///�����ȡʧ�ܣ�����Ῠ��
        bluetooth_packet.BUF_P=BUF_P1;//�ɹ���ȡ������ָ��
     }
	 
	}  											 
} 

/// @brief ����������յ������ݣ������з���
/// @param  Bluetooth_packet(ȡ��������Ҫ�����ĸ���������)
void blutooth_control(Bluetooth_packet_t bluetooth_packet)
{
    switch (bluetooth_packet.date1)
    {
    case 0x01://��date2����

        break;
    
    default:
        break;
    }



    

}


