#include "pid.h"
#include "mpu6050.h"
/******PID控制器对象初始化***********/
Position_PID_t 	Angle_loop;
Position_PID_t 	Angle_speed_loop;
Position_PID_t 	Turn_loop;
Inc_PID_t 		Motor_Velocity_loop; 

/**************************************************************************
函数功能：转向环控制器
入口参数：PID对象指针，
			输出最大值,
			微分项的选择，为1则直接使用读取的数据，为0则需要前后两次误差进行计算
返回  值：无
***************************************************************************/				
void Turn_PID_Control(Position_PID_t* Turn_PID,uint16_t Max_Output,uint8_t Using_kd_date)
{

		
	if(Turn_PID->Target_Date <= -180&& Turn_PID->Now_Date  >0)
		Turn_PID->Error = (Turn_PID->Now_Date -(Turn_PID->Target_Date+360))*10;	
	else if(Turn_PID->Target_Date<= -360&&Turn_PID->Now_Date <0)
		Turn_PID->Error = (Turn_PID->Now_Date -(Turn_PID->Target_Date+360))*10;	
	else if(Turn_PID->Target_Date >= 180&&Turn_PID->Now_Date <0)
		 Turn_PID->Error = (Turn_PID->Now_Date -(Turn_PID->Target_Date-360))*10;
	else if(Turn_PID->Target_Date>= 360&&Turn_PID->Now_Date >0)
		Turn_PID->Error = (Turn_PID->Now_Date -(Turn_PID->Target_Date-360))*10;
	else
		Turn_PID->Error = (Turn_PID->Now_Date -Turn_PID->Target_Date)*10;	
	if(Turn_PID->Error>-0.15&&Turn_PID->Error<0.15)
		Turn_PID->Error=0;
	//转向环输出
	 Turn_PID->Out = Turn_PID->Kp*Turn_PID->Error; 
	
				/*进行转向速度的单独限幅*/
	if(Turn_PID->Out>=1500)	Turn_PID->Out =1500;
	else if(Turn_PID->Out<=-1500)Turn_PID->Out =-1500;
			
	
}

/**************************************************************************
函数功能：普通增量式控制器
入口参数：增量式PID对象指针，输出最大值
返回  值：无
***************************************************************************/

void Inc_PID_Control(Inc_PID_t* Inc_PID,uint16_t Max_Output)
{
	//误差计算
	Inc_PID->Error=Inc_PID->Now_Date - Inc_PID->Target_Date;
/**************滤波***************/
	Inc_PID->Error*=0.8;
	if(Inc_PID->Error>-1.5&&Inc_PID->Error<1.5)
	{
		Inc_PID->Error=0;
	}
/********************************/
	//增量计算
	Inc_PID->Out_Inc =Inc_PID->Kp*(Inc_PID->Error-Inc_PID->Last_Error)+(Inc_PID->Ki*Inc_PID->Error);//增量式PI控制器 
	//保存上一次偏差 
	Inc_PID->Last_Error=Inc_PID->Error;
	if(Inc_PID->Out_Inc>2||Inc_PID->Out_Inc<-2)
	{
		Inc_PID->Out+=Inc_PID->Out_Inc;
	}
/*******输出限幅*********/
	if(Inc_PID->Out>Max_Output)
		Inc_PID->Out=Max_Output;
		
	if(Inc_PID->Out<-Max_Output)
		Inc_PID->Out=-Max_Output;
}

/**************************************************************************
函数功能：普通位置式控制器
入口参数：位置式PID对象指针，
			输出最大值,
			积分最大值，
			微分项的选择，为1则直接使用读取的数据，为0则需要前后两次误差进行计算
返回  值：无
***************************************************************************/

void Position_PID_Control(Position_PID_t* Position_PID,uint16_t Max_Output,uint16_t Max_i_date,uint8_t Using_kd_date)
{
	//误差计算
	Position_PID->Error=Position_PID->Now_Date-Position_PID->Target_Date;
	Position_PID->Error*=0.8;
	Position_PID->Error_sum+=Position_PID->Error;
	//积分限幅
	if(Position_PID->Error_sum>Max_i_date)
		Position_PID->Error_sum=Max_i_date;
		
	if(Position_PID->Error_sum<-Max_i_date)
		Position_PID->Error_sum=-Max_i_date;
	
	if(1==Using_kd_date)//能直接读出微分项
	{
		Position_PID->Out=Position_PID->Kp*Position_PID->Error\
					  +Position_PID->Ki*Position_PID->Error_sum\
					  +Position_PID->Kd*Position_PID->Kd_date;
	}
	else//不能直接读出微分项，用上下两次误差相减
	{
		Position_PID->Out=Position_PID->Kp*Position_PID->Error\
					  +Position_PID->Ki*Position_PID->Error_sum\
			+Position_PID->Kd*(Position_PID->Error-Position_PID->Last_Error);
	//保存上一次偏差 	
	Position_PID->Last_Error=Position_PID->Error;
	}
	
	
	/*******输出限幅*********/
	if(Position_PID->Out>Max_Output)
		Position_PID->Out=Max_Output;
		
	if(Position_PID->Out<-Max_Output)
		Position_PID->Out=-Max_Output;
}

void Motor_Velocity_Set(int16_t speed)
{
	Motor_Velocity_loop.Target_Date=speed;
}


