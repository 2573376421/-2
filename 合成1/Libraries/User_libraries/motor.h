#ifndef  _MOTOR_H
#define  _MOTOR_H

#include "common.h"
#include "board.h"

#define PWM_MAX 9999
#define PWM_MIN -9999



void Limit(int16 *motoA,int16 *motoB);
int16 abs(int16 p);
void Load(int16 moto1,int16 moto2);

#endif