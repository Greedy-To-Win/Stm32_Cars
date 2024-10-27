#include "pid.h"
#include "motor.h"
#include "mpu6050.h"


//��̬PID����
typedef struct {
	float Kp;
	float Ki;
	float Kd;
}PID_Date_t;
extern PID_Date_t Dynamic[30];
//������������
void PID_Date_Init(void);
void Motor_Date_Init(void);
void Dynamic_PID_Control(uint16_t speed);//�ֶ�PID����
