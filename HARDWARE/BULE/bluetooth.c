#include "bluetooth.h"
#include "pid.h"
#include "usart.h"	
//初始化数据包
Bluetooth_packet_t bluetooth_packet;//蓝牙数据包


uint8_t Bluetooth_RxBuf[5]={0};//蓝牙接收缓存区
uint8_t* BUF_P1=&Bluetooth_RxBuf[0];

/// @brief 初始化数据包内容，主要为帧头，和主要数据所在数组
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
函数功能：蓝牙串口初始化
入口参数： bound:波特率
返回  值：无
**************************************************************************/
void bluetooth_uart_init(u32 bound)
{  
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
/*******************GPIO端口设置*******************************/
	
//开时钟	 
	RCC_APB2PeriphClockCmd(Blutooth_Rcc_GPIOx, ENABLE);	  //使能UGPIOA时钟
	RCC_APB1PeriphClockCmd(Blutooth_Rcc_USARTx, ENABLE);  //使能USART1时钟
//USART1_TX  
	GPIO_InitStructure.GPIO_Pin = Blutooth_TX; 			  //PA9
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;		  //复用推挽输出
	GPIO_Init(Blutooth_Port, &GPIO_InitStructure);
   
//USART1_RX	  
	GPIO_InitStructure.GPIO_Pin = Blutooth_RX;			  //PA10
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING; //浮空输入
	GPIO_Init(Blutooth_Port, &GPIO_InitStructure);
/*******************Usart1 NVIC 配置*******************************/
	
NVIC_InitStructure.NVIC_IRQChannel = Blutooth_IRQn;
NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=1 ;//抢占优先级
NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;		//子优先级
NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
NVIC_Init(&NVIC_InitStructure);				//根据指定的参数初始化VIC寄存器
/*******************USART1 初始化设置****************************/

	USART_InitStructure.USART_BaudRate = bound;					//串口波特率
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;	//字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;		//一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;			//无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式
    USART_Init(Blutooth_USARTx, &USART_InitStructure);     //初始化串口1
    USART_ITConfig(Blutooth_USARTx, USART_IT_RXNE, ENABLE);//开启串口接受中断
    USART_Cmd(Blutooth_USARTx, ENABLE);                    //使能串口1
    
	bluetooth_packet_init(bluetooth_packet,0xFF,BUF_P1);
}





/// @brief 解析数据包
/// @param uartRxBuffer 
/// @param parsedData 
/// @return 0 or 1（ture or false）
int Get_packet_date(char* uartRxBuffer,Bluetooth_packet_t bluetooth_packet) {
    if (uartRxBuffer[0] != 0xFF) {
        printf("Error: Invalid frame header.\n");
        return -1; // 错误的帧头
    }
    
    bluetooth_packet.Checksum = (uartRxBuffer[1] + uartRxBuffer[2])&& 0xFF;//校验和*********?????????????
    if (bluetooth_packet.Checksum != uartRxBuffer[3]) {
        printf("Error: Checksum mismatch.\n");
        return -2; // 校验和不匹配
    }
    // 如果一切正常，将数据存入parsedData
    bluetooth_packet.date1= uartRxBuffer[1]; // 存储date1
    bluetooth_packet.date2= uartRxBuffer[2]; // 存储date2
    return 0; // 成功
}





/**************************************************************************
函数功能：串口1接收中断
入口参数：无
返回  值：无
**************************************************************************/
__IO uint8_t usart_value=0;//接收一个字节数据的变量
void Blutooth_IRQHandler(void)
{	
	if(USART_GetITStatus(Blutooth_USARTx, USART_IT_RXNE) != RESET) //接收到数据
	{	  
	 USART_ClearITPendingBit(Blutooth_USARTx,USART_IT_RXNE);//清除标志位
  	 usart_value=USART_ReceiveData(Blutooth_USARTx); 

	
	/************使用蓝牙****************/				
     if(bluetooth_packet.date_head==usart_value)//帧头
     {
        *(bluetooth_packet.BUF_P)=usart_value;
        bluetooth_packet.BUF_P++;
        bluetooth_packet.flag=1;//进入数据接收
     }
     if(1==bluetooth_packet.flag)//date1
     {
        *(bluetooth_packet.BUF_P)=usart_value;
        bluetooth_packet.BUF_P++;
        bluetooth_packet.flag=2;//进入数据接收
     }
     if(2==bluetooth_packet.flag)//date2
     {
        *(bluetooth_packet.BUF_P)=usart_value;
        bluetooth_packet.BUF_P++;
        bluetooth_packet.flag=3;//进入数据接收
     }
     if(3==bluetooth_packet.flag)//校验和
     {
        *(bluetooth_packet.BUF_P)=usart_value;
        bluetooth_packet.flag=0;//进入数据接收
       
        while(0!=Get_packet_date(Bluetooth_RxBuf,bluetooth_packet));///如果获取失败，程序会卡死
        bluetooth_packet.BUF_P=BUF_P1;//成功读取后重置指针
     }
	 
	}  											 
} 

/// @brief 处理接蓝牙收到的数据，并进行反馈
/// @param  Bluetooth_packet(取决于你需要处理哪个包的数据)
void blutooth_control(Bluetooth_packet_t bluetooth_packet)
{
    switch (bluetooth_packet.date1)
    {
    case 0x01://以date2匀速

        break;
    
    default:
        break;
    }



    

}


