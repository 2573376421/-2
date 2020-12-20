#ifndef _ALGORITHM_h
#define _ALGORITHM_h

#include "common.h"


#define FILTER_N 12

#define max(a,b) (a>b?a:b)
#define min(a,b) (a<b?a:b)

extern uint16 Filter_AD1,Filter_AD2,Filter_AD3,Filter_AD4;
extern float Angle_lv;


void Filter(void);
void Angle_Calcu(void);

#endif