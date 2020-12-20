#include "headfile.h"
/*-------------------------------------------------------------------------

数据定义部分

--------------------------------------------------------------------------*/

float VOLTAGE;//实测电压值
int8 Time_Flag=0,stop_flag=0,Out_flag=0,End_Flag=0;
int8 count;

uint16 OUT_AD_Max[4];
int16 AD_Err,AD_Sum; //测试时用于测量两侧偏差

int16 MOTOR1,MOTOR2;//电机PWM值（带正负值）

int16 Encoder_Left, Encoder_Right;//编码器
static Encoder_all;	

float Car_speed;   //当前车速
int16 speed;       //速度设定

int16 Velocity_Out,Turn_Out;

float Vertical_Out;
float Med_Angle; //机械中植设定(默认为0)

//直立环Kp，Ki，Kd调节区
float Vertical_Kp;
float Vertical_Kd;		    
//速度环Kp，Ki，Kd调节区
float Velocity_Kp;
//转向环Kp，Ki，Kd调节区
float Turn_Kp;
float Turn_Kd;

float check; //检查

/*---------------------------------------
总初始化定义

----------------------------------------*/
void PID_config(void)
{
	//直立环Kp，Ki，Kd调节区
	 Vertical_Kp = 280;		//280
	 Vertical_Kd = -1.3;		//-1.18    
	//速度环Kp，Ki，Kd调节区
	 Velocity_Kp =-0.18;	//-0.18			-0.013
	//转向环Kp，Ki，Kd调节区
	 Turn_Kp 	= 40;//80.5					75
	 Turn_Kd 	= -0.145;//0.659			-0.53
	 Med_Angle = 6; //机械中植设定(默认为0)
   speed 		= 70;       //速度设定

}

/*---------------------------------------
总初始化定义

----------------------------------------*/
void Init(void)
{
	
	delay_init();
	delay_ms(1000);
	delay_ms(1000);
	delay_ms(1000);
	delay_ms(1000);
	DisableGlobalIRQ();					//关闭总中断
	board_init();						//初始化内部寄存器，勿删除此句代码。	
	delay_init();
//	simiic_init();				    //IIC初始化
  oled_init();		//oled初始化			
	icm20602_init_simiic();		    //陀螺仪初始化
	// ccd_init();						//CCD初始化
	PID_config();
	adc_init(ADC_P06,ADC_SYSclk_DIV_2);	//adc引脚初始化，其中P06为电源测量脚，其他为电磁检测
	adc_init(ADC_P14,ADC_SYSclk_DIV_2);
	adc_init(ADC_P15,ADC_SYSclk_DIV_2);
	adc_init(ADC_P16,ADC_SYSclk_DIV_2);
	adc_init(ADC_P17,ADC_SYSclk_DIV_2);
		
	ctimer_count_init(CTIM0_P34);	  //编码器初始化
	ctimer_count_init(CTIM3_P04);

	pwm_init(PWM5_P00, 17000,0);		//PWM初始化
	pwm_init(PWM6_P01, 17000,0);
	pwm_init(PWM2P_P22,17000,0);
	pwm_init(PWM4P_P26,17000,0);

	 
	
//	uart_init(UART_1,UART1_RX_P30,UART1_TX_P31,115200,TIM_2);		//串口初始化
	ccd_init();
//  adc_init(AD_CHANNEL,0X01);      
//	pit_timer_ms(TIM_4,10);
	pit_timer_ms(TIM_1,5);			//中断进入设置
	EnableGlobalIRQ();		//开启总中断

}
/*---------------------------------------
电磁AD采集处理函数
----------------------------------------*/

void AD_Collect(void)
{	
	int8 i,j;
	uint8 temp;


	Filter(); //采集数据并滤波

	//4路平均滤波处理后数据
	OUT_AD_Max[0] = Filter_AD1;
	OUT_AD_Max[1] = Filter_AD2;
	OUT_AD_Max[2] = Filter_AD3;
	OUT_AD_Max[3] = Filter_AD4;

	

	AD_Err = (int16)(Filter_AD1+Filter_AD2-Filter_AD3-Filter_AD4);  
	AD_Sum = (int16)(Filter_AD1+Filter_AD2+Filter_AD3+Filter_AD4);
	//电感大小排序
	for(i = 0;i < 4;i++)
	{
		for ( j = 0; j < 3; j++)
		{
			if (OUT_AD_Max[i] > OUT_AD_Max[i + 1])
			{
				temp = OUT_AD_Max[i];
				OUT_AD_Max[i] = OUT_AD_Max[i + 1];
				OUT_AD_Max[i + 1] = temp;
			}		
		}		
	}
}


/*---------------------------------------
速度环
----------------------------------------*/
int16 Velocity(int16 encoder_left,int16 encoder_right)
{
	static int16 Enc_Err_Lowout,PWM_out,Encoder_Err,Enc_Err_Lowout_last;									//速度未定义，默认为0
	float a = 0.7;
	if (Time_Flag == 18)
	{
		//计算速度偏差
		Encoder_Err = (encoder_left + encoder_right) / 2 - speed;
		//对速度进行低通滤波
		Enc_Err_Lowout = a * Encoder_Err + (1 - a) * Enc_Err_Lowout_last;
		Enc_Err_Lowout_last = Enc_Err_Lowout;
		//速度环控制输出进行运算
		PWM_out = Velocity_Kp * Enc_Err_Lowout;
		Time_Flag = 0; //标志位清零
	}
	PWM_out = PWM_out > 8 ? 8 : (PWM_out<(-3) ? (-3):PWM_out);
	return PWM_out;
}

/*---------------------------------------
直立环
----------------------------------------*/
int16 Vertical(float V_Angle,float Angle,int16 gyro_y)
{
	int16 PWM_out;
	PWM_out = Vertical_Kp * (Angle - V_Angle) + Vertical_Kd * gyro_y ; //(Angle - Med)换成Angle
	return PWM_out;
}


/*---------------------------------------
电磁AD转向环函数（速度越大越敏感）
----------------------------------------*/
int16 AD_Turn(void)
{
	float Bias,Gyro_Ratio; //偏差和转向D的动态系数
	int16 PWM_out;
	static int16 Last_Gyro_Ratio;
	static uint8 first_Gyro;
	if (!first_Gyro)
	{
			first_Gyro = 1;
			Last_Gyro_Ratio = Gyro_Ratio;
	}

	Bias = (float)AD_Err / (float)AD_Sum * 100.0;
	Bias = Bias * (Bias * Bias / 1250.0 + 2) / 10;
	if (Bias > 100)
	Bias = 100;
	else if (Bias < -100)
	Bias = -100;	
	Gyro_Ratio = icm_gyro_z * icm_gyro_z / 4000000 + 4.65;
	Gyro_Ratio = 0.7 * Gyro_Ratio + Last_Gyro_Ratio * 0.3;
	Car_speed = (Encoder_Left + Encoder_Right) / 2;
	PWM_out = (Car_speed * Car_speed / (speed * speed) + 1) * Turn_Kp * Bias + Gyro_Ratio * Turn_Kd * icm_gyro_z;
	Last_Gyro_Ratio = Gyro_Ratio;
	return PWM_out;
}


/*---------------------------------------
主控制部分，需修改
----------------------------------------*/

void control(void)
{
	int16 PWM_out;
		if(Out_flag==0)								//出库
		{	
			get_icm20602_accdata_simiic();	  //加速度
			get_icm20602_gyro_simiic();		  //角速度
			Angle_Calcu();            			//数据计算	
			AD_Collect();	
			Vertical_Out = Vertical(12,Angle_lv,icm_gyro_y);
			PWM_out = Vertical_Out;				
			if(Filter_AD2<10&&Filter_AD3<10)
			{				
				MOTOR1=PWM_out+50;	
				MOTOR2=PWM_out;	
			}					
			else if((Filter_AD2>=10||Filter_AD3>=10)&&(Filter_AD1<=20||Filter_AD4<=20))
			{	
				MOTOR1=PWM_out+1500;	
				MOTOR2=PWM_out-1500;	
			}
			else
			{
				Out_flag=1;
				MOTOR1=PWM_out;	
				MOTOR2=PWM_out;
			}
		}
	
if(Out_flag==1)	
	{
		Time_Flag++;

		get_icm20602_accdata_simiic();	  //加速度
		get_icm20602_gyro_simiic();		  //角速度
		Angle_Calcu();            			//数据计算	
		AD_Collect();		
//		ccd_collect();	 //CCD采集数据
//		ccd_judge();
	//左右编码器转向判断及其输出（1，0调节转向）
	if(DIRL == 1)
		{
			Encoder_Left = ctimer_count_read(CTIM0_P34);
		}
	else
		{
			Encoder_Left = -ctimer_count_read(CTIM0_P34);
		}
	if(DIRR == 0)											   //左右成对立关系故相反
		{
			Encoder_Right = ctimer_count_read(CTIM3_P04);
		}
	else
		{
			Encoder_Right = -ctimer_count_read(CTIM3_P04);
		}

//编码器数据清零
	ctimer_count_clean(CTIM0_P34);
	ctimer_count_clean(CTIM3_P04);
		
//将数据放入闭环系统，计算控制输出量
	Velocity_Out = Velocity(Encoder_Left,Encoder_Right);
	Vertical_Out = Vertical(Velocity_Out+Med_Angle,Angle_lv,icm_gyro_y);
	Turn_Out = AD_Turn();
	PWM_out = Vertical_Out; 
	MOTOR1=PWM_out - Turn_Out;	
	MOTOR2=PWM_out + Turn_Out;
 	if(Filter_AD1<5&&Filter_AD2<5&&Filter_AD3<5&&Filter_AD4<5)
 	{
 		Load(0,0);
 		stop_flag=1;
 	}
				if(End_Flag==1)										//入库
				{	
						Out_flag=3;
				}
	}
	
	if(Out_flag==3)								//入库
		{	
			get_icm20602_accdata_simiic();	  //加速度
			get_icm20602_gyro_simiic();		  //角速度
			Angle_Calcu();            			//数据计算	
			AD_Collect();	
			Vertical_Out = Vertical(12,Angle_lv,icm_gyro_y);
			PWM_out = Vertical_Out;				
			if(Filter_AD1>10&&Filter_AD4>10)
			{				
				MOTOR1=PWM_out+3000;	
				MOTOR2=PWM_out-3000;	
			}					
			else
			{
				stop_flag=1;
				Load(0,0);
			}
		}
	 	if(stop_flag==0)
 	{		
 		Limit(&MOTOR1,&MOTOR2);//PWM限幅			
		Load(MOTOR1,MOTOR2);//加载到电机上。（左，右）
 	}

}

/*---------------------------------------
充电电压判断函数
----------------------------------------*/

//void Charge_judge(void)
//{
//	int16 v[10];		  //judge为预设电压值
//	float judge = 10.0;
//	uint16 i;
//	int16 sum = 0,V_Max = 0;
//	while(1)
//	{
//		for ( i = 0; i < 10; i++)
//		{
//			v[i] = adc_voltage();
//		}
//		for ( i = 0; i < 10; i++)
//		{
//			V_Max = max(v[i],V_Max);
//			sum += v[i];
//		}		
//		VOLTAGE = ((float)(sum - V_Max))/9.0;
//		if(VOLTAGE = judge)
//			break;	
//	}
//}

void ccd_judge(void)
{
	uint8 i;
	int8 measure;
	for ( i = 0; i < 128; i++)
	{
		measure = ccd_data[i] - ccd_data[i + 1];
		if (measure >= 10)
		{
			count++;
		}
	}
		if (count >= 10)
		{
			End_Flag ++;
		}	
	
}

