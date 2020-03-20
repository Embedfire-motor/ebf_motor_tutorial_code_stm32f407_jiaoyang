#ifndef __BSP_STEPPER_S_SPEED_H
#define	__BSP_STEPPER_S_SPEED_H

#include "stm32f4xx.h"
#include "stm32f4xx_hal.h"
#include "./usart/bsp_debug_usart.h"
#include "./stepper/bsp_stepper_init.h"
#include "math.h"
#include "stdlib.h"
#include "string.h"


#define CONVER(speed)  ((speed) * FSPR * MICRO_STEP / 60)  // 根据电机转速（r/min），计算电机步速（step/s）

extern int32_t  Step_Position  ;           // 当前位置
extern uint16_t    Toggle_Pulse;        // 脉冲频率控制
extern uint8_t  MotionStatus   ;  

//typedef struct 
//{
//	int		Vo;  //初速度 单位 Step/s
//	int		Vt;  //末速度 单位 Step/s
//	int		AccelHalfStep;   //加速阶段半路程	单位 Step   
//	int		AccelTotalStep;  //总路程，加速阶段总步数 Step
//	float   Form[FORM_LEN]; // 速度表格 单位 Step/s
//	
//}Speed_s;
/**************************************************************************************/
#define FORM_LEN 	   1000

typedef struct {
	uint8_t 	status;	//状态
	uint8_t 	dir;		//方向
	uint32_t 	pos;		//位置
	uint32_t  pluse_time;//脉冲时间	
}Stepper_Typedef;

/*S加减速所用到的参数*/
typedef struct {
  int32_t   Vo;               // 初速度   单位 Step/s
  int32_t   Vt;               // 末速度   单位 Step/s
  int32_t AccelStep;          // 加速段的步数单位 Step
  float   Form[FORM_LEN];       // 速度表格 单位 Step/s  步进电机的脉冲频率
}SpeedCalc_TypeDef;

extern SpeedCalc_TypeDef Speed ;

#define CW                0 // 顺时针
#define CCW               1 // 逆时针

/*电机速度决策中的四个状态*/
#define ACCEL                 1   //  加速状态
#define AVESPEED              2   //  匀速状态
#define DECEL                 3   //  减速状态
#define STOP                  0   //  停止状态
																												  

/*频率相关参数*/
//定时器实际时钟频率为：168MHz/(TIM_PRESCALER+1)
//其中 高级定时器的 频率为168MHz,其他定时器为84MHz
//168/(168)=1Mhz
//具体需要的频率可以自己计算
#define TIM_PRESCALER               168-1 
#define T1_FREQ                               (SystemCoreClock/(TIM_PRESCALER+1)) // 频率ft值


/*电机单圈参数*/
#define STEP_ANGLE				1.8									//步进电机的步距角 单位：度
#define FSPR              (360.0f/1.8f)         //步进电机的一圈所需脉冲数

#define MICRO_STEP        32          				//细分器细分数 
#define SPR               (FSPR*MICRO_STEP)   //细分后一圈所需脉冲数

/**/

#define ROUNDPS_2_STEPPS(RPM)                 ((RPM) * FSPR * MICRO_STEP / 60)         // 根据电机转速（r/min），计算电机步速（step/s）
#define MIDDLEVELOCITY(Vo,Vt)                 ( ( (Vo) + (Vt) ) / 2 )                  // S型加减速加速段的中点速度 
#define INCACCEL(Vo,V,T)                      ( ( 2 * ((V) - (Vo)) ) / pow((T),2) )    // 加加速度:加速度增加量   V - V0 = 1/2 * J * t^2
#define INCACCELSTEP(J,T)                     ( ( (J) * pow( (T) , 3 ) ) / 6 )         // 加加速段的位移量(步数)  S = 1/6 * J * t^3
#define ACCEL_TIME(T)                         ( (T) / 2 )                              // 加加速段和减加速段的时间是相等的

#define TRUE                                   1
#define FALSE                                  0



#define SPEED_MIN                              (T1_FREQ / (65535.0f))// 最低频率/速度

extern uint8_t print_flag;
void CalculateSpeedTab(int Vo, int Vt, float Time);
void CalcSpeed(int32_t Vo, int32_t Vt, float Time);
void stepper_move_S(int start_speed,int end_speed,float time);
void speed_decision(void);
#endif 
