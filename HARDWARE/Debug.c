#include "Debug.h"
#include "delay.h"	
#include "motor.h"   
#include "oled.h"
#include "Date.h"

char oledBuf[50];//显示缓冲区
/**************************************************************************
函数功能：开机调试显示数据函数
入口参数：未知
返回  值：无
**************************************************************************/
void Init_Display(void)
{
		sprintf(oledBuf,"gyrox:%d",mpu6050_date.gyrox+35);//角速度
	OLED_ShowString(1,1,(u8*)oledBuf);
		sprintf(oledBuf,"Roll:%.3f",mpu6050_date.roll);
	OLED_ShowString(2,1,(u8*)oledBuf);
		sprintf(oledBuf,"yaw:%.3f",mpu6050_date.yaw);
	OLED_ShowString(3,1,(u8*)oledBuf);
	
	OLED_ShowString(4,1,"Init finish");
}

void Debug_Display(void)
{	
	
#if DEBUG_Angle_speed
	
		sprintf(oledBuf,"gyrox:%d",mpu6050_date.gyrox+35);//角速度
	OLED_ShowString(1,1,(u8*)oledBuf);	
		sprintf(oledBuf,"Out:%d",Blance_motor.Moto1_pwm_out);
	OLED_ShowString(2,1,(u8*)oledBuf);
		sprintf(oledBuf,"I_sum:%d",Angle_speed_loop.Error_sum);
	OLED_ShowString(3,1,(u8*)oledBuf);	
	
		sprintf(oledBuf,"p:%.2f",Angle_speed_loop.Kp);
	OLED_ShowString(4,1,(u8*)oledBuf);
		sprintf(oledBuf,"i:%.2f",Angle_speed_loop.Ki);
	OLED_ShowString(4,8,(u8*)oledBuf);
#endif
#if DEBUG_standing 
	
	
	sprintf(oledBuf,"Roll:%f",mpu6050_date.roll);
		OLED_ShowString(1,1,(u8*)oledBuf);	
	sprintf(oledBuf,"out:%.3f",Angle_loop.Out);
		OLED_ShowString(2,1,(u8*)oledBuf);
	sprintf(oledBuf,"->Ero:%.3f",Angle_speed_loop.Error);
		OLED_ShowString(3,1,(u8*)oledBuf);
	
		sprintf(oledBuf,"Kp:%3f",Angle_loop.Kp);
	OLED_ShowString(4,1,(u8*)oledBuf);
		sprintf(oledBuf,"kd:%3f",Angle_loop.Kd);
	OLED_ShowString(4,8,(u8*)oledBuf);
#endif
	
#if DEBUG_speed		
	sprintf(oledBuf,"n_dt:%.1f",Motor_Velocity_loop.Now_Date);
		OLED_ShowString(1,1,(u8*)oledBuf);	
	sprintf(oledBuf,"Inc:%f",Motor_Velocity_loop.Out_Inc);
		OLED_ShowString(1,8,(u8*)oledBuf);	
	sprintf(oledBuf,"out:%.3f",Motor_Velocity_loop.Out);
		OLED_ShowString(2,1,(u8*)oledBuf);
	sprintf(oledBuf,"->Ero:%.3f",Angle_loop.Error);
		OLED_ShowString(3,1,(u8*)oledBuf);
	
		sprintf(oledBuf,"Kp:%3f",Motor_Velocity_loop.Kp);
	OLED_ShowString(4,1,(u8*)oledBuf);
		sprintf(oledBuf,"Ki:%3f",Motor_Velocity_loop.Ki);
	OLED_ShowString(4,8,(u8*)oledBuf);
	

#endif	
#if DEBUG_turn		

		sprintf(oledBuf,"yaw:%f",mpu6050_date.yaw);
	OLED_ShowString(1,1,(u8*)oledBuf);
		sprintf(oledBuf,"pid.Kp_turn:%.1f",Turn_loop.Kp);
	

#endif
		
		
		
#if DEBUG_bluetooth

		sprintf(oledBuf,"date1:%d",bluetooth_packet.date1);
	OLED_ShowString(1,1,(u8*)oledBuf);
		sprintf(oledBuf,"date2:%d",bluetooth_packet.date2);
	OLED_ShowString(2,1,(u8*)oledBuf);	
		sprintf(oledBuf,"check:%d",bluetooth_packet.Checksum);
	OLED_ShowString(3,1,(u8*)oledBuf);	

#endif
		
		
#if DEBUG_OPENMV 
	 OLED_ShowString(1,1,"openmv......");
	 printf("发送OK\r\n");\
	 Usart_SendString( OpenMV_USART_USARTx,"这是一个串口中断接收回显实验\n");
#endif	

	OLED_Refresh();

}
