#include "algorithm.h"
#include "SEEKFREE_ICM20602.h"
#include "math.h" 
#include "control.h"
#include "headfile.h"

uint16 Filter_AD1,Filter_AD2,Filter_AD3,Filter_AD4;

float Accel_ax, Accel_az,Gyro_y;             //X轴加速度值暂存

float Angle_lv;

/*------------------------------------------------------------------
平均滤波法（又称防脉冲干扰平均滤波法）
-------------------------------------------------------------------*/
void Filter(void)
{
    uint16 i;
    uint16 filter_max1 = 0,filter_max2 = 0,filter_max3 = 0,filter_max4 = 0;
    uint16 filter_min1 = 255,filter_min2 = 255,filter_min3 = 255,filter_min4 = 255;
    uint16 filter_sum1 = 0,filter_sum2 = 0,filter_sum3 = 0,filter_sum4 = 0;
    uint16 filter_col1[FILTER_N],filter_col2[FILTER_N],filter_col3[FILTER_N],filter_col4[FILTER_N];

    for(i = 0; i < FILTER_N; i++) 
    {
			  filter_col1[i] = adc_once(ADC_P14, ADC_8BIT);  //获取采样数据
        filter_col2[i] = adc_once(ADC_P15, ADC_8BIT);
        filter_col3[i] = adc_once(ADC_P16, ADC_8BIT);
        filter_col4[i] = adc_once(ADC_P17, ADC_8BIT);
        filter_max1 = max(filter_max1, filter_col1[i]);
        filter_min1 = min(filter_min1, filter_col1[i]);
        filter_max2 = max(filter_max2, filter_col2[i]);
        filter_min2 = min(filter_min2, filter_col2[i]);
        filter_max3 = max(filter_max3, filter_col3[i]);
        filter_min3 = min(filter_min3, filter_col3[i]);        
        filter_max4 = max(filter_max4, filter_col4[i]);
        filter_min4 = min(filter_min4, filter_col4[i]);

        filter_sum1 += filter_col1[i];
        filter_sum2 += filter_col2[i];
        filter_sum3 += filter_col3[i];            
        filter_sum4 += filter_col4[i];
    }

    Filter_AD1 = (filter_sum1 - filter_max1 - filter_min1) / (FILTER_N - 2);
    Filter_AD2 = (filter_sum2 - filter_max2 - filter_min2)  / (FILTER_N - 2);
    Filter_AD3 = (filter_sum3 - filter_max3 - filter_min3)  / (FILTER_N - 2);
    Filter_AD4 = (filter_sum4 - filter_max4 - filter_min4)  / (FILTER_N - 2);      
}



//-------------------------------------------------------------------------------------------------------------------
//  @brief      角度计算      
//倾角计算（互补滤波）
//-------------------------------------------------------------------------------------------------------------------
void Angle_Calcu(void)
{
		static float Angle_g,Gyro_speed_y;
    static uint8 first_angle;
		float Angle_m; 
    float d = 0.095;
    float dt = 0.00012; //积分系数    
    if (!first_angle)
    {
        first_angle = 1;
        Angle_g = Angle_m;
    }
    Accel_ax  = icm_acc_x;          //读取X轴加速度
    Accel_az  = icm_acc_z;          //读取z轴加速度
    Gyro_y = icm_gyro_y;
    Gyro_speed_y = Gyro_y * dt;
    Angle_g = (int16)(Gyro_speed_y + Angle_g);
    Angle_m = (int16)(atan(Accel_ax/Accel_az)*180/3.14);
    Angle_lv = d * Angle_m + (1 - d) * Angle_lv ; 
}


