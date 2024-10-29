#include "key.h"
#include "delay.h"
#include "pid.h"
#include "Debug.h"
#include "Schedule.h"
/**************************************************************************
函数功能：按键初始化函数
入口参数：无
返回  值：无
**************************************************************************/	 
void KEY_Init(void)
{
	
	
	GPIO_InitTypeDef GPIO_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
    EXTI_InitTypeDef EXTI_InitStructure;
    /*****开启时钟*************/
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO  , ENABLE);
    
    /******配置GPIO口**********/
 	//GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);//关闭jtag，使能SWD，可以用SWD模式调试
	
	GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7;//PA5.PA4
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; //设置成上拉输入
 	GPIO_Init(GPIOA, &GPIO_InitStructure);
    
    /*******配置AFIO 中断引脚选择********/
	//GPIO_EXTILineConfig(GPIO_PortSourceGPIOB , GPIO_PinSource14);
    
    /*****配置EXTI********/

    EXTI_InitStructure.EXTI_Line = EXTI_Line4|EXTI_Line5| EXTI_Line6|EXTI_Line7;                     //选择中断线
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;                       //使能
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;             //中断模式
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;         //选择下降沿触发
    EXTI_Init(&EXTI_InitStructure);
    
    /*****配置NVIC*****/

    NVIC_InitStructure.NVIC_IRQChannel = EXTI4_IRQn;                       // 使能所在的外部中断通道
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;                             //使能外部中断通道
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;                   //抢占优先级
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;                          //响应优先级
    NVIC_Init(&NVIC_InitStructure);
    
	NVIC_InitStructure.NVIC_IRQChannel =EXTI9_5_IRQn ;                       // 使能所在的外部中断通道
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;                             //使能外部中断通道
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;                   //抢占优先级
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;                          //响应优先级
    NVIC_Init(&NVIC_InitStructure);
}





void  EXTI4_IRQHandler(void)     //外部中断函数
{
    if(EXTI_GetITStatus(EXTI_Line4) == SET)    //判断是否通道4来的中断
    {
		if(Holding_State==Active_FSM.State)//如果待机状态按下则切换到开机直立状态
		{
			Active_FSM.State_change=1;
			Active_FSM.State=Upright_State;//开机直立
		}
        if (GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_4) == 0&&Active_FSM.State!=Holding_State)         //如果4号脚为低电平
		{
#if DEBUG_Angle_speed
			Angle_speed_loop.Kp+=0.01;
#endif
#if DEBUG_standing			
				Angle_loop.Kp+=10;
		
#endif	
#if DEBUG_speed
			Motor_Velocity_loop.Ki+=2;
#endif			
#if DEBUG_turn			
			Turn_loop.Kp+=1;
#endif						
	

        EXTI_ClearITPendingBit(EXTI_Line4);        //清除中断请求
		}
	}
}
	
void EXTI9_5_IRQHandler	(void)
{	
/**************************按键A5*******************************************/		
    if(EXTI_GetITStatus(EXTI_Line5) == SET)    //判断是否通道5来的中断
    {
        if (GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_5) == 0&&Active_FSM.State!=Holding_State)         //如果5号脚为低电平
		{
#if DEBUG_Angle_speed
			Angle_speed_loop.Kp-=0.01;
#endif			
#if DEBUG_standing	

				Angle_loop.Kp-=10;
#endif
#if DEBUG_speed			
			Motor_Velocity_loop.Ki-=2;
#endif			
#if DEBUG_turn			
			Turn_loop.Kp-=1;
#endif			
		
		}
        EXTI_ClearITPendingBit(EXTI_Line5);        //清除中断请求
    }

/**************************按键A6*******************************************/		
	 if(EXTI_GetITStatus(EXTI_Line6) == SET)    //判断是否通道5来的中断
    {
        if (GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_6) == 0&&Active_FSM.State!=Holding_State)         //如果5号脚为低电平
		{
#if DEBUG_Angle_speed
			Angle_speed_loop.Ki+=0.001;
#endif					
#if DEBUG_standing	
				Angle_loop.Kd+=0.01;
#endif
#if DEBUG_speed			
			    Motor_Velocity_loop.Kp+=0.1;
#endif			


        EXTI_ClearITPendingBit(EXTI_Line6);        //清除中断请求
		}
	}
/**************************按键A7*******************************************/	
	
	if(EXTI_GetITStatus(EXTI_Line7) == SET)    //判断是否通道5来的中断
	{
        if (GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_7) == 0&&Active_FSM.State!=Holding_State)         //如果5号脚为低电平
		{
#if DEBUG_Angle_speed
			Angle_speed_loop.Ki-=0.001;
#endif	
			
#if DEBUG_standing	
				Angle_loop.Kd-=0.01;
			
#endif
#if DEBUG_speed		
			Motor_Velocity_loop.Kp-=0.1;			
#endif			

		}
        EXTI_ClearITPendingBit(EXTI_Line7);        //清除中断请求
    }
}
