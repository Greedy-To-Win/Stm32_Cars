#include "pid.h"
#include "motor.h"
#include "mpu6050.h"


//动态PID参数
typedef struct {
	float Kp;
	float Ki;
	float Kd;
}PID_Date_t;
extern PID_Date_t Dynamic[30];
//数据整定函数
void PID_Date_Init(void);
void Motor_Date_Init(void);
void Dynamic_PID_Control(uint16_t speed);//分段PID整定
