#include "pid.h"
#include "mpu6050.h"
/******PID控制器对象初始化***********/
Position_PID_t 	Angle_loop;
Position_PID_t 	Angle_speed_loop;
Position_PID_t 	Turn_loop;
Inc_PID_t 		Motor_Velocity_loop; 
Exper_Inc_Pid_TCB_t	 Exper_Motor_Velocity_TCB;
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
	
//	if(Inc_PID->Error > Inc_PID->Error_Max)		//误差太大，外部干扰
//		Inc_PID->Out=Max_Output;
//	else if(Inc_PID->Error < Inc_PID->Error_Min)//误差太大，外部干扰
//		Inc_PID->Out=-Max_Output;
	if(Inc_PID->Error > Inc_PID->Error_Max||Inc_PID->Error < Inc_PID->Error_Min)
	 {
		 if(Inc_PID->Error*Inc_PID->Kp>0)
			Inc_PID->Out=Max_Output;
		 else
			Inc_PID->Out=-Max_Output; 
	 }
	else	//正常误差	
	{
		Inc_PID->Out_Inc =Inc_PID->Kp*(Inc_PID->Error-Inc_PID->Last_Error)+(Inc_PID->Ki*Inc_PID->Error);//增量式PI控制器 

		if(Inc_PID->Out_Inc>2||Inc_PID->Out_Inc<-2)
		{
		/************************/
		 Inc_PID->Out+=Inc_PID->Out_Inc;//增量累加
		/*************************/
		}
		/*******输出限幅*********/
		if(Inc_PID->Out>Max_Output)
			Inc_PID->Out=Max_Output;
			
		if(Inc_PID->Out<-Max_Output)
			Inc_PID->Out=-Max_Output;
	}

/****保存上一次偏差 ****/
	Inc_PID->Last_Error=Inc_PID->Error;
}	

/**************************************************************************
函数功能：专家增量式控制器
入口参数：增量式PID对象指针，增量式专家控制块指针
返回  值：无
***************************************************************************/

void Inc_PID_Control_Expert(Inc_PID_t* Inc_PID,Exper_Inc_Pid_TCB_t* Exper_Motor_Velocity_TCB)
{
//	//计算误差变化
//	Exper_Motor_Velocity_TCB->inc_iError=Inc_PID->Error - Inc_PID->Last_Error;
////	if(Inc_PID->Error<0)
////		Inc_PID->Error=-Inc_PID->Error; //本语句后误差变为绝对值
//	// 小偏差 
//	if (Inc_PID->Error < Exper_Motor_Velocity_TCB->Error_abs_Min) 
//		{
//			//较小误差, 调小比例
//			Inc_PID->Out_Inc = Exper_Motor_Velocity_TCB->control_parameter * Inc_PID->Out_Inc;
//		} 
//		
//	// 大偏差
//	else if(Inc_PID->Error > Exper_Motor_Velocity_TCB->Error_abs_Max) 
//		{
//			// 串联BangBang-PID
//			Inc_PID->Out_Inc= (Inc_PID->Error > 0) ? (6999) : (-6999);
//		}
//	// 正常偏差
//	else {
//			// 偏差在朝向偏差绝对值增大的方向变化(偏差越来越大), 或者偏差一直为某一固定值
//			if ((Inc_PID->Error * Exper_Motor_Velocity_TCB->inc_iError > 0 && Error_Inc_PID->Last_Error* Exper_Motor_Velocity_TCB->inc_Error1 > 0) || (Inc_PID->Error - Inc_PID->Last_Error) == 0) 
//				{
//				
//				if (Inc_PID->Error > Exper_Motor_Velocity_TCB->Error_abs_Mid) 
//					{
//						// 控制器实施较强的控制作用
//						Inc_PID->Out_Inc = (1.2+Exper_Motor_Velocity_TCB->control_parameter )* Inc_PID->Out_Inc;
//					} 
//				else 
//					{
//						// 但是偏差绝对值本身并不是很大
//						Inc_PID->Out_Inc =Inc_PID->Out_Inc + 0;
//					}
//				}
//			// 偏差的绝对值向减小的方向变化，或者已经达到平衡状态
//			else if ((Inc_PID->Error * Exper_Motor_Velocity_TCB->inc_iError  < 0 )&&( Exper_Motor_Velocity_TCB->inc_iError * Exper_Motor_Velocity_TCB->inc_Error1 > 0) ||( Inc_PID->Error  == 0 && Inc_PID->Last_Error == 0)) 
//				{
//					
//					// 此时可以保持控制器输出不变
//				} 
//			else if (pid->errNow * (pid->errNow - pid->errLast) < 0 && ((pid->errNow - pid->errLast) * (pid->errLast - pid->errLastLast) < 0)) 
//				{
//					// 偏差处于极值极限状态
//					if (pid->errABS > midErr) 
//						{
//						pid->outPut = onlineK * pid->outPut;
//					} else {
//						// 但是偏差绝对值本身并不是很大
//						pid->outPut = pid->outPut + 0;
//					}
//				
//				} 
//			else 
//			{
//				// 正常情况
//				pid->outPut = pid->outPut + 0;
//			}
//		}

//	//更新误差
//	Exper_Motor_Velocity_TCB->Error1=Inc_PID->Error;
//	Exper_Motor_Velocity_TCB->Error2=Exper_Motor_Velocity_TCB->Error1;

//	//更新误差增量
//	Exper_Motor_Velocity_TCB->inc_Error1=Exper_Motor_Velocity_TCB->inc_iError;
	
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
	//误差过大
	 if(Position_PID->Error>Position_PID->Error_Max||Position_PID->Error < Position_PID->Error_Min)
	 {
		 if(Position_PID->Error*Position_PID->Kp>0)
			Position_PID->Out=Max_Output;
		 else
			Position_PID->Out=-Max_Output; 
	 }
	 else{
				 
			//误差正常，正常积分
			if(Position_PID->Error > -Position_PID->Error_temp && Position_PID->Error < Position_PID->Error_temp)
			{
				if(Position_PID->Error_sum>Max_i_date)
					Position_PID->Error_sum=Max_i_date;
					
				if(Position_PID->Error_sum<-Max_i_date)
					Position_PID->Error_sum=-Max_i_date;
			}
			
			else 
				Position_PID->Error_sum=0;//大误差不采用积分

			
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
			}
			
			
			/*******输出限幅*********/

			if(Position_PID->Out>Max_Output)
				Position_PID->Out=Max_Output;
				
			if(Position_PID->Out<-Max_Output)
				Position_PID->Out=-Max_Output;
			//保存上一次偏差
			Position_PID->Last_Error=Position_PID->Error;
		}
}

void Motor_Velocity_Set(int16_t speed)
{
	Motor_Velocity_loop.Target_Date=speed;
}


