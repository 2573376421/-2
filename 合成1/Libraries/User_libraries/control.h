#ifndef _CONTROL_H
#define _CONTROL_H


#define DIRL P50
#define DIRR P52


#define key3 P30
#define key1 P40

#define Bkey4 P41
#define Bkey3 P42
#define Bkey2 P43
#define Bkey1 P44


#include "common.h"

extern int16 MOTOR1,MOTOR2;
extern float Vertical_Kp,Vertical_Kd,Velocity_Kp,Turn_Kp,Turn_Kd;
extern float Med_Angle,Angle_T1;      //角度
extern uint16 in_col1[12],in_col2[12],in_col3[12],in_col4[12];
extern int16 AD_Err,AD_Sum;
extern int8 Time_Flag,stop_flag,count;

extern float check;

void Init(void);       //总初始化
void PID_config(void);
int16 Vertical(float V_Angle,float Angle,int16 gyro_y);
int16 Velocity(int16 encoder_left,int16 encoder_right);
int16 AD_Turn(void);
void control(void);
void Charge_judge(void);
void AD_Collect(void);
void ccd_judge(void);
#endif