#ifndef __BSP_CREAT_S_TAB_H
#define	__BSP_CREAT_S_TAB_H

#include "stm32f4xx.h"
#include "stm32f4xx_hal.h"
#include "./usart/bsp_debug_usart.h"
#include "math.h"
#include "stdlib.h"
#include "string.h"

extern uint8_t print_flag;
void CalculateSpeedTab(int Vo, int Vt, float Time);

typedef struct 
{
	int		Vo;  //初速度 单位 Step/s
	int		Vt;  //末速度 单位 Step/s
	int		AccelHalfStep;   //加速阶段半路程	单位 Step   
	int		AccelTotalStep;  //总路程，加速阶段总步数 Step
	float   Form[1000]; // 速度表格 单位 Step/s
	
}Speed_s;


#define MICRO_STEP                    32    // 驱动器细分数
#define FSPR                          200   // 步进电机单圈步数


#define ROUNDPS_2_STEPPS(RoundSpeed)  ((RoundSpeed) * FSPR * MICRO_STEP / 60)  // 根据电机转速（r/min），计算电机步速（step/s）

#define MIDDLEVELOCITY(Vo,Vt)         ( ( (Vo) + (Vt) ) / 2 )                  // S型加减速加速段的中点速度
#define ACCELL_INCREASE(Vo,V,T)       ( ( 2 * ((V) - (Vo)) ) / pow((T),2) )    // 加加速度:加速度增加量 V - V0 = 1/2 *J*t^2
#define ACCEL_DISPLACEMENT(J,T)       ( ( (J) * pow( (T) ,3) ) / 6 )           // 加速段的位移量(步数)  S = 1/6 *J *t^3
#define ACCEL_TIME(T)                 ( (T) / 2 )    


//#define MICRO_STEP                    32    // 驱动器细分数
//#define FSPR                          200   // 步进电机单圈步数


//#define ROUNDPS_2_STEPPS(RoundSpeed)  ((RoundSpeed) * FSPR * MICRO_STEP / 60)  // 根据电机转速（r/min），计算电机步速（step/s）

//#define MIDDLEVELOCITY(Vo,Vt)         ( ( (Vo) + (Vt) ) / 2 )                  // S型加减速加速段的中点速度
//#define ACCELL_INCREASE(Vo,V,T)       ( ( 2 * ((V) - (Vo)) ) / pow((T),2) )    // 加加速度:加速度增加量 V - V0 = 1/2 *J*t^2
//#define ACCEL_DISPLACEMENT(J,T)       ( ( (J) * pow( (T) ,3) ) / 6 )           // 加速段的位移量(步数)  S = 1/6 *J *t^3
//#define ACCEL_TIME(T)                 ( (T) / 2 )                              // 加速段的加 加速所需要的时间 





#endif 
