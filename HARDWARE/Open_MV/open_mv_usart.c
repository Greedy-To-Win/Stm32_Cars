#include "open_mv_usart.h"
#include "usart.h"	
#include "oled.h"
#include "pid.h"

uint8_t Openmv_usart_packet_buf[OPENMV_USART_PACKET_BUF_LEN];
Openmv_usart_packet_t openmv_tracking_packet;
Vofa_packet_t vofa_packet;

void openmv_usart_init(u32 bound)//openmv���ڳ�ʼ��
{
    USART_InitTypeDef USART_InitStructure;
    DMA_InitTypeDef DMA_InitStructure;
    GPIO_InitTypeDef GPIO_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
/*******************GPIO�˿�����*******************************/
//��ʱ��	 
	RCC_APB2PeriphClockCmd(OpenMV_USART_Rcc_GPIOx, ENABLE);	  //ʹ��UGPIOAʱ��
	RCC_APB1PeriphClockCmd(OpenMV_USART_Rcc_USARTx, ENABLE);  //ʹ��USART2ʱ��
    RCC_AHBPeriphClockCmd(OpenMV_Rcc_DMA, ENABLE);
//USART2_TX  
	GPIO_InitStructure.GPIO_Pin = OpenMV_USART_TX; 			  //PA2
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;		  //�����������
	GPIO_Init(OpenMV_USART_Port, &GPIO_InitStructure);
   
//USART2_RX	  
	GPIO_InitStructure.GPIO_Pin = OpenMV_USART_RX;			  //PA3
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING; //��������
	GPIO_Init(OpenMV_USART_Port, &GPIO_InitStructure);
/*******************Usart2 NVIC ����*******************************/
	
NVIC_InitStructure.NVIC_IRQChannel = OpenMV_USART_IRQn;
NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=1 ;//��ռ���ȼ�
NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;		//�����ȼ�
NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQͨ��ʹ��
NVIC_Init(&NVIC_InitStructure);				//����ָ���Ĳ�����ʼ��VIC�Ĵ���
/*******************USART2 ��ʼ������****************************/

    // ����DMA
    DMA_DeInit(OpenMV_DMA_Channel);// ���DMA1_Channel6�Ĵ���,����2��RXΪͨ����
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t) &OpenMV_USART_USARTx->DR;
    DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t) Openmv_usart_packet_buf;
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
    DMA_InitStructure.DMA_BufferSize = OPENMV_USART_PACKET_BUF_LEN;
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
    DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;//���Ȼ�ȡopenmv������
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
    DMA_Init(OpenMV_DMA_Channel, &DMA_InitStructure);
    // ����USART2
    USART_InitStructure.USART_BaudRate = bound;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
    USART_Init(OpenMV_USART_USARTx, &USART_InitStructure);
#if USE_OPENMV
    // ʹ��DMAͨ��
    DMA_Cmd(OpenMV_DMA_Channel, ENABLE);
    // ��dma��������
    USART_DMACmd(OpenMV_USART_USARTx, USART_DMAReq_Rx, ENABLE);
   //ʹ�ܿ����ж�
	USART_ITConfig(OpenMV_USART_USARTx,USART_IT_IDLE,ENABLE);
#endif
#if USE_Vofa
	vofa_packet.contrl_flag=0;
	vofa_packet.RxState=0;
	USART_ITConfig(OpenMV_USART_USARTx,USART_IT_RXNE,ENABLE);
#endif
	// ʹ�ܴ���
	USART_Cmd(OpenMV_USART_USARTx, ENABLE);	    
    // ʹ��DMA�ж�
    //NVIC_EnableIRQ(OpenMV_DMA_IRQn);

}

float Pow_invert(uint8_t X,uint8_t n)//x����n��10
{
  float result=X;
	while(n--)
	{
		result/=10;
	}
	return result;
}
/// @brief ����һ���ṹ��ָ�룬��ȡ���ݰ�������ݵ�pid
/// @param Vofa_packet_t* vofa_packet 
void Get_vofa_date_to_pid(Vofa_packet_t* vofa_packet)
{
  uint8_t i;
  float Data=0.0;
  uint8_t dot_Flag=0;//С�����־λ��������С������С����ǰ 0ΪС����ǰ��1ΪС�����
  uint8_t dot_after_num=1;//С�����ĵڼ�λ
  for(i=0;i<vofa_packet->Rx_num;i++)
  {
    if(dot_Flag==0)
    {
      if(vofa_packet->RxPacket[i]==0x2E)//���ʶ��С���㣬��dot_Flag��1
      {
        dot_Flag=1;
      }
      else//��û����С����ǰ������
      {
        Data = Data*10 + vofa_packet->RxPacket[i]-30;
      }
    }
    else//����С����������
    {
      Data = Data + Pow_invert(vofa_packet->RxPacket[i]-30,dot_after_num);
      dot_after_num++;
    }
  }//���ݼ������
  switch(vofa_packet->id)
  {
	 
	   case 1://ֱ����kp
//		  pid.Kp=Data;
		  break;
	  
	   case 2://�ٶȻ�kp
//		  pid.speed_Kp=Data;
		  break;
	  
	   case 3://ת��kp
//	  	  pid.Kp_turn=Data;
		  break;
	  
	   case 4://ֱ����kd
//		  pid.Kd=Data;
		  break;
	  
  }
	
}



float Get_error_form_openmv(Openmv_usart_packet_t openmv_tracking_packet)
{
	
}

// DMA�жϷ������
void OpenMV_DMA_IRQHandler(void)
{
    // ����DMA�ж�
    if (DMA_GetITStatus(DMA1_IT_TC6) != RESET)
    {
        // DMA�������
        DMA_ClearITPendingBit(DMA1_IT_TC6);
    }
    if (DMA_GetITStatus(DMA1_IT_HT6) != RESET)
    {
        // DMA�����ж�
        DMA_ClearITPendingBit(DMA1_IT_HT6);
    }
    if (DMA_GetITStatus(DMA1_IT_TE6) != RESET)
    {
        // DMA��������ж�
        DMA_ClearITPendingBit(DMA1_IT_TE6);
    }
}

// USART2�жϷ������
void OpenMV_USART_IRQHandler(void)
{
#if USE_OPENMV
	uint16_t t;
	if(USART_GetITStatus(OpenMV_USART_USARTx,USART_IT_IDLE) == SET)          //����ж��Ƿ���
	{	
		DMA_Cmd(OpenMV_DMA_Channel,DISABLE); 		//�ر�DMA����
		if(0==openmv_tracking_packet.flag&&Openmv_usart_packet_buf[0]==0xFF&&Openmv_usart_packet_buf[1]==0xFF)
		{openmv_tracking_packet.flag++;}
		if(1==openmv_tracking_packet.flag&&Openmv_usart_packet_buf[6]==0xFF)
		{
			openmv_tracking_packet.date1=Openmv_usart_packet_buf[2];
			openmv_tracking_packet.date2=Openmv_usart_packet_buf[3];
			openmv_tracking_packet.date3=Openmv_usart_packet_buf[4];
			openmv_tracking_packet.date4=Openmv_usart_packet_buf[5];
			openmv_tracking_packet.flag=2;//�ɴ������ݻ�ȡ����־λ
			OLED_Refresh();
			OLED_ShowString(1,1,"openmv_success!");

	
		}
		
		t = DMA_GetCurrDataCounter(OpenMV_DMA_Channel);              //��ȡʣ�����������
		
        Usart_SendArray(OpenMV_USART_USARTx,Openmv_usart_packet_buf,OPENMV_USART_PACKET_BUF_LEN-t);       //����Է������ݣ������������� = SENDBUFF_SIZE - ʣ��δ���������������
		
		DMA_SetCurrDataCounter(OpenMV_DMA_Channel,OPENMV_USART_PACKET_BUF_LEN);    //�������ô������������

		DMA_Cmd(OpenMV_DMA_Channel,ENABLE);                          //����DMA����
		
		USART_ReceiveData(OpenMV_USART_USARTx);                              //��ȡһ�����ݣ���Ȼ��һֱ���ж�
		USART_ClearFlag(OpenMV_USART_USARTx,USART_FLAG_IDLE);                //������ڿ����жϱ�־λ
	}
#endif	
	
		/************VOFA+����PID************/	
#if USE_Vofa
uint16_t usart_value;
	if(USART_GetITStatus(OpenMV_USART_USARTx, USART_IT_RXNE) != RESET) //���յ�����
	{	  
	 USART_ClearITPendingBit(OpenMV_USART_USARTx,USART_IT_RXNE);//�����־λ
  	 usart_value=USART_ReceiveData(OpenMV_USART_USARTx); 
		 printf("==================================================================");
		 if(vofa_packet.RxState==0&&usart_value==0x50) //��1��֡ͷ  "P"==0x50
		{
			 printf("!!!!!!!!!!!!!!!!!!!!!!!!!!");
			vofa_packet.RxState=1;
		}
		else if(vofa_packet.RxState==1)//ȷ�ϴ��εĶ��� ���޸�id
		{	
			 printf("111");
			vofa_packet.id=usart_value-30;
			vofa_packet.RxState=2;
		}
		else if(vofa_packet.RxState==2&&usart_value==0x3D) //�жϵȺţ�Ҳ�������Ϊ���ݿ�ʼ��֡ͷ
		{
			 printf("222");
			vofa_packet.RxState=3;
		}
		else if(vofa_packet.RxState==3)//��ʼ���մ��������
		{	
			 printf("333");
			  if(usart_value==0x3D)//������֡β   ���û�н��յ�������������������һֱ����
				{
					 printf("999");
					vofa_packet.Rx_num=vofa_packet.Rx_point;//��ȡλ��
					vofa_packet.Rx_point=0;//������������´ν��н�������
					vofa_packet.RxState=0;//���ݰ������꣬��ʼ׼��������һ��
					vofa_packet.contrl_flag=1;
				}
			  else
			  {
					vofa_packet.RxPacket[vofa_packet.Rx_point++]=usart_value;//�����ݷ������ݰ���
			  }
		}
	}	

		
#endif		
	
}



/*****************  ����һ���ֽ� **********************/
void Usart_SendByte( USART_TypeDef * pUSARTx, uint8_t ch)
{
	/* ����һ���ֽ����ݵ�USART */
	USART_SendData(pUSARTx,ch);
		
	/* �ȴ��������ݼĴ���Ϊ�� */
	while (USART_GetFlagStatus(pUSARTx, USART_FLAG_TXE) == RESET);	
}

/****************** ����8λ������ ************************/
void Usart_SendArray( USART_TypeDef * pUSARTx, uint8_t *array, uint16_t num)
{
  uint8_t i;
	
	for(i=0; i<num; i++)
  {
	    /* ����һ���ֽ����ݵ�USART */
	    Usart_SendByte(pUSARTx,array[i]);	
  
  }
	/* �ȴ�������� */
	while(USART_GetFlagStatus(pUSARTx,USART_FLAG_TC)==RESET);
}

/*****************  �����ַ��� **********************/
void Usart_SendString( USART_TypeDef * pUSARTx, char *str)
{
	unsigned int k=0;
  do 
  {
      Usart_SendByte( pUSARTx, *(str + k) );
      k++;
  } while(*(str + k)!='\0');
  
  /* �ȴ�������� */
  while(USART_GetFlagStatus(pUSARTx,USART_FLAG_TC)==RESET)
  {}
}

/*****************  ����һ��16λ�� **********************/
void Usart_SendHalfWord( USART_TypeDef * pUSARTx, uint16_t ch)
{
	uint8_t temp_h, temp_l;
	
	/* ȡ���߰�λ */
	temp_h = (ch&0XFF00)>>8;
	/* ȡ���Ͱ�λ */
	temp_l = ch&0XFF;
	
	/* ���͸߰�λ */
	USART_SendData(pUSARTx,temp_h);	
	while (USART_GetFlagStatus(OpenMV_USART_USARTx, USART_FLAG_TXE) == RESET);
	
	/* ���͵Ͱ�λ */
	USART_SendData(pUSARTx,temp_l);	
	while (USART_GetFlagStatus(OpenMV_USART_USARTx, USART_FLAG_TXE) == RESET);	
}

///�ض���c�⺯��printf�����ڣ��ض�����ʹ��printf����
#if Fput_to_openmv
int fputc(int ch, FILE *f)
{
		/* ����һ���ֽ����ݵ����� */
		USART_SendData(OpenMV_USART_USARTx, (uint8_t) ch);
		
		/* �ȴ�������� */
		while (USART_GetFlagStatus(OpenMV_USART_USARTx, USART_FLAG_TXE) == RESET);		
	
		return (ch);
}
#endif
#if Fput_to_bluetooth
int fputc(int ch, FILE *f)
{
		/* ����һ���ֽ����ݵ����� */
		USART_SendData(USART1, (uint8_t) ch);
		
		/* �ȴ�������� */
		while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);		
	
		return (ch);
}
#endif
///�ض���c�⺯��scanf�����ڣ���д����ʹ��scanf��getchar�Ⱥ���
int fgetc(FILE *f)
{
		/* �ȴ������������� */
		while (USART_GetFlagStatus(OpenMV_USART_USARTx, USART_FLAG_RXNE) == RESET);

		return (int)USART_ReceiveData(OpenMV_USART_USARTx);
}








