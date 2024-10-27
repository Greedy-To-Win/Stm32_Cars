#include "open_mv_usart.h"
#include "usart.h"	
#include "oled.h"
#include "pid.h"

uint8_t Openmv_usart_packet_buf[OPENMV_USART_PACKET_BUF_LEN];
Openmv_usart_packet_t openmv_tracking_packet;
Vofa_packet_t vofa_packet;

void openmv_usart_init(u32 bound)//openmv串口初始化
{
    USART_InitTypeDef USART_InitStructure;
    DMA_InitTypeDef DMA_InitStructure;
    GPIO_InitTypeDef GPIO_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
/*******************GPIO端口设置*******************************/
//开时钟	 
	RCC_APB2PeriphClockCmd(OpenMV_USART_Rcc_GPIOx, ENABLE);	  //使能UGPIOA时钟
	RCC_APB1PeriphClockCmd(OpenMV_USART_Rcc_USARTx, ENABLE);  //使能USART2时钟
    RCC_AHBPeriphClockCmd(OpenMV_Rcc_DMA, ENABLE);
//USART2_TX  
	GPIO_InitStructure.GPIO_Pin = OpenMV_USART_TX; 			  //PA2
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;		  //复用推挽输出
	GPIO_Init(OpenMV_USART_Port, &GPIO_InitStructure);
   
//USART2_RX	  
	GPIO_InitStructure.GPIO_Pin = OpenMV_USART_RX;			  //PA3
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING; //浮空输入
	GPIO_Init(OpenMV_USART_Port, &GPIO_InitStructure);
/*******************Usart2 NVIC 配置*******************************/
	
NVIC_InitStructure.NVIC_IRQChannel = OpenMV_USART_IRQn;
NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=1 ;//抢占优先级
NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;		//子优先级
NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
NVIC_Init(&NVIC_InitStructure);				//根据指定的参数初始化VIC寄存器
/*******************USART2 初始化设置****************************/

    // 配置DMA
    DMA_DeInit(OpenMV_DMA_Channel);// 清除DMA1_Channel6寄存器,串口2的RX为通道六
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t) &OpenMV_USART_USARTx->DR;
    DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t) Openmv_usart_packet_buf;
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
    DMA_InitStructure.DMA_BufferSize = OPENMV_USART_PACKET_BUF_LEN;
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
    DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;//优先获取openmv的数据
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
    DMA_Init(OpenMV_DMA_Channel, &DMA_InitStructure);
    // 配置USART2
    USART_InitStructure.USART_BaudRate = bound;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
    USART_Init(OpenMV_USART_USARTx, &USART_InitStructure);
#if USE_OPENMV
    // 使能DMA通道
    DMA_Cmd(OpenMV_DMA_Channel, ENABLE);
    // 向dma发送请求
    USART_DMACmd(OpenMV_USART_USARTx, USART_DMAReq_Rx, ENABLE);
   //使能空闲中断
	USART_ITConfig(OpenMV_USART_USARTx,USART_IT_IDLE,ENABLE);
#endif
#if USE_Vofa
	vofa_packet.contrl_flag=0;
	vofa_packet.RxState=0;
	USART_ITConfig(OpenMV_USART_USARTx,USART_IT_RXNE,ENABLE);
#endif
	// 使能串口
	USART_Cmd(OpenMV_USART_USARTx, ENABLE);	    
    // 使能DMA中断
    //NVIC_EnableIRQ(OpenMV_DMA_IRQn);

}

float Pow_invert(uint8_t X,uint8_t n)//x除以n次10
{
  float result=X;
	while(n--)
	{
		result/=10;
	}
	return result;
}
/// @brief 传进一个结构体指针，获取数据包里的数据到pid
/// @param Vofa_packet_t* vofa_packet 
void Get_vofa_date_to_pid(Vofa_packet_t* vofa_packet)
{
  uint8_t i;
  float Data=0.0;
  uint8_t dot_Flag=0;//小数点标志位，能区分小数点后或小数点前 0为小数点前，1为小数点后
  uint8_t dot_after_num=1;//小数点后的第几位
  for(i=0;i<vofa_packet->Rx_num;i++)
  {
    if(dot_Flag==0)
    {
      if(vofa_packet->RxPacket[i]==0x2E)//如果识别到小数点，则将dot_Flag置1
      {
        dot_Flag=1;
      }
      else//还没遇到小数点前的运算
      {
        Data = Data*10 + vofa_packet->RxPacket[i]-30;
      }
    }
    else//遇到小数点后的运算
    {
      Data = Data + Pow_invert(vofa_packet->RxPacket[i]-30,dot_after_num);
      dot_after_num++;
    }
  }//数据计算完毕
  switch(vofa_packet->id)
  {
	 
	   case 1://直立环kp
//		  pid.Kp=Data;
		  break;
	  
	   case 2://速度环kp
//		  pid.speed_Kp=Data;
		  break;
	  
	   case 3://转向环kp
//	  	  pid.Kp_turn=Data;
		  break;
	  
	   case 4://直立环kd
//		  pid.Kd=Data;
		  break;
	  
  }
	
}



float Get_error_form_openmv(Openmv_usart_packet_t openmv_tracking_packet)
{
	
}

// DMA中断服务程序
void OpenMV_DMA_IRQHandler(void)
{
    // 处理DMA中断
    if (DMA_GetITStatus(DMA1_IT_TC6) != RESET)
    {
        // DMA传输完成
        DMA_ClearITPendingBit(DMA1_IT_TC6);
    }
    if (DMA_GetITStatus(DMA1_IT_HT6) != RESET)
    {
        // DMA半满中断
        DMA_ClearITPendingBit(DMA1_IT_HT6);
    }
    if (DMA_GetITStatus(DMA1_IT_TE6) != RESET)
    {
        // DMA传输错误中断
        DMA_ClearITPendingBit(DMA1_IT_TE6);
    }
}

// USART2中断服务程序
void OpenMV_USART_IRQHandler(void)
{
#if USE_OPENMV
	uint16_t t;
	if(USART_GetITStatus(OpenMV_USART_USARTx,USART_IT_IDLE) == SET)          //检查中断是否发生
	{	
		DMA_Cmd(OpenMV_DMA_Channel,DISABLE); 		//关闭DMA传输
		if(0==openmv_tracking_packet.flag&&Openmv_usart_packet_buf[0]==0xFF&&Openmv_usart_packet_buf[1]==0xFF)
		{openmv_tracking_packet.flag++;}
		if(1==openmv_tracking_packet.flag&&Openmv_usart_packet_buf[6]==0xFF)
		{
			openmv_tracking_packet.date1=Openmv_usart_packet_buf[2];
			openmv_tracking_packet.date2=Openmv_usart_packet_buf[3];
			openmv_tracking_packet.date3=Openmv_usart_packet_buf[4];
			openmv_tracking_packet.date4=Openmv_usart_packet_buf[5];
			openmv_tracking_packet.flag=2;//可处理数据获取误差标志位
			OLED_Refresh();
			OLED_ShowString(1,1,"openmv_success!");

	
		}
		
		t = DMA_GetCurrDataCounter(OpenMV_DMA_Channel);              //获取剩余的数据数量
		
        Usart_SendArray(OpenMV_USART_USARTx,Openmv_usart_packet_buf,OPENMV_USART_PACKET_BUF_LEN-t);       //向电脑返回数据（接收数据数量 = SENDBUFF_SIZE - 剩余未传输的数据数量）
		
		DMA_SetCurrDataCounter(OpenMV_DMA_Channel,OPENMV_USART_PACKET_BUF_LEN);    //重新设置传输的数据数量

		DMA_Cmd(OpenMV_DMA_Channel,ENABLE);                          //开启DMA传输
		
		USART_ReceiveData(OpenMV_USART_USARTx);                              //读取一次数据，不然会一直进中断
		USART_ClearFlag(OpenMV_USART_USARTx,USART_FLAG_IDLE);                //清除串口空闲中断标志位
	}
#endif	
	
		/************VOFA+调试PID************/	
#if USE_Vofa
uint16_t usart_value;
	if(USART_GetITStatus(OpenMV_USART_USARTx, USART_IT_RXNE) != RESET) //接收到数据
	{	  
	 USART_ClearITPendingBit(OpenMV_USART_USARTx,USART_IT_RXNE);//清除标志位
  	 usart_value=USART_ReceiveData(OpenMV_USART_USARTx); 
		 printf("==================================================================");
		 if(vofa_packet.RxState==0&&usart_value==0x50) //第1个帧头  "P"==0x50
		{
			 printf("!!!!!!!!!!!!!!!!!!!!!!!!!!");
			vofa_packet.RxState=1;
		}
		else if(vofa_packet.RxState==1)//确认传参的对象 即修改id
		{	
			 printf("111");
			vofa_packet.id=usart_value-30;
			vofa_packet.RxState=2;
		}
		else if(vofa_packet.RxState==2&&usart_value==0x3D) //判断等号，也可以类比为数据开始的帧头
		{
			 printf("222");
			vofa_packet.RxState=3;
		}
		else if(vofa_packet.RxState==3)//开始接收传输的数据
		{	
			 printf("333");
			  if(usart_value==0x3D)//结束的帧尾   如果没有接收到！即还有数据来，就一直接收
				{
					 printf("999");
					vofa_packet.Rx_num=vofa_packet.Rx_point;//获取位数
					vofa_packet.Rx_point=0;//清除索引方便下次进行接收数据
					vofa_packet.RxState=0;//数据包接受完，开始准备接收下一个
					vofa_packet.contrl_flag=1;
				}
			  else
			  {
					vofa_packet.RxPacket[vofa_packet.Rx_point++]=usart_value;//把数据放在数据包内
			  }
		}
	}	

		
#endif		
	
}



/*****************  发送一个字节 **********************/
void Usart_SendByte( USART_TypeDef * pUSARTx, uint8_t ch)
{
	/* 发送一个字节数据到USART */
	USART_SendData(pUSARTx,ch);
		
	/* 等待发送数据寄存器为空 */
	while (USART_GetFlagStatus(pUSARTx, USART_FLAG_TXE) == RESET);	
}

/****************** 发送8位的数组 ************************/
void Usart_SendArray( USART_TypeDef * pUSARTx, uint8_t *array, uint16_t num)
{
  uint8_t i;
	
	for(i=0; i<num; i++)
  {
	    /* 发送一个字节数据到USART */
	    Usart_SendByte(pUSARTx,array[i]);	
  
  }
	/* 等待发送完成 */
	while(USART_GetFlagStatus(pUSARTx,USART_FLAG_TC)==RESET);
}

/*****************  发送字符串 **********************/
void Usart_SendString( USART_TypeDef * pUSARTx, char *str)
{
	unsigned int k=0;
  do 
  {
      Usart_SendByte( pUSARTx, *(str + k) );
      k++;
  } while(*(str + k)!='\0');
  
  /* 等待发送完成 */
  while(USART_GetFlagStatus(pUSARTx,USART_FLAG_TC)==RESET)
  {}
}

/*****************  发送一个16位数 **********************/
void Usart_SendHalfWord( USART_TypeDef * pUSARTx, uint16_t ch)
{
	uint8_t temp_h, temp_l;
	
	/* 取出高八位 */
	temp_h = (ch&0XFF00)>>8;
	/* 取出低八位 */
	temp_l = ch&0XFF;
	
	/* 发送高八位 */
	USART_SendData(pUSARTx,temp_h);	
	while (USART_GetFlagStatus(OpenMV_USART_USARTx, USART_FLAG_TXE) == RESET);
	
	/* 发送低八位 */
	USART_SendData(pUSARTx,temp_l);	
	while (USART_GetFlagStatus(OpenMV_USART_USARTx, USART_FLAG_TXE) == RESET);	
}

///重定向c库函数printf到串口，重定向后可使用printf函数
#if Fput_to_openmv
int fputc(int ch, FILE *f)
{
		/* 发送一个字节数据到串口 */
		USART_SendData(OpenMV_USART_USARTx, (uint8_t) ch);
		
		/* 等待发送完毕 */
		while (USART_GetFlagStatus(OpenMV_USART_USARTx, USART_FLAG_TXE) == RESET);		
	
		return (ch);
}
#endif
#if Fput_to_bluetooth
int fputc(int ch, FILE *f)
{
		/* 发送一个字节数据到串口 */
		USART_SendData(USART1, (uint8_t) ch);
		
		/* 等待发送完毕 */
		while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);		
	
		return (ch);
}
#endif
///重定向c库函数scanf到串口，重写向后可使用scanf、getchar等函数
int fgetc(FILE *f)
{
		/* 等待串口输入数据 */
		while (USART_GetFlagStatus(OpenMV_USART_USARTx, USART_FLAG_RXNE) == RESET);

		return (int)USART_ReceiveData(OpenMV_USART_USARTx);
}








