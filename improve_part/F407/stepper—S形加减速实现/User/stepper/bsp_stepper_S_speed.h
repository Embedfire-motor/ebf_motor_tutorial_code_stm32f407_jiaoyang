#ifndef __BSP_STEPPER_S_SPEED_H
#define	__BSP_STEPPER_S_SPEED_H

#include "stm32f4xx.h"
#include "stm32f4xx_hal.h"
#include "./usart/bsp_debug_usart.h"
#include "./stepper/bsp_stepper_init.h"
#include "math.h"
#include "stdlib.h"
#include "string.h"

#define FORM_LEN 	   1000
#define MICRO_STEP     32    // 驱动器细分数
#define FSPR           200   // 步进电机单圈步数
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

/* 类型定义 ------------------------------------------------------------------*/
typedef struct {
  int32_t   Vo;               // 初速度   单位 Step/s
  int32_t   Vt;               // 末速度   单位 Step/s
  int32_t AccelStep;          // 加速段的步数单位 Step
  float   VelocityTab[534];       // 速度表格 单位 Step/s  步进电机的脉冲频率
}SpeedCalc_TypeDef;




extern SpeedCalc_TypeDef Speed ;


// 定义定时器预分频，定时器实际时钟频率为：168MHz/（STEPMOTOR_TIMx_PRESCALER+1）
#define TIM_PRESCALER               168-1 // 步进电机驱动器细分需要设置为32


// 定义定时器周期，输出比较模式周期设置为0xFFFF
#define STEPMOTOR_TIM_PERIOD                   0xFFFF
// 定义高级定时器重复计数寄存器值
#define STEPMOTOR_TIM_REPETITIONCOUNTER       0


#define T1_FREQ                               (SystemCoreClock/(TIM_PRESCALER+1)) // 频率ft值
#define FSPR                                  200         //步进电机单圈步数
#define MICRO_STEP                            32          // 步进电机驱动器细分数

#define ROUNDPS_2_STEPPS(RPM)                 ((RPM) * FSPR * MICRO_STEP / 60)         // 根据电机转速（r/min），计算电机步速（step/s）
#define MIDDLEVELOCITY(Vo,Vt)                 ( ( (Vo) + (Vt) ) / 2 )                  // S型加减速加速段的中点速度 
#define INCACCEL(Vo,V,T)                      ( ( 2 * ((V) - (Vo)) ) / pow((T),2) )    // 加加速度:加速度增加量   V - V0 = 1/2 * J * t^2
#define INCACCELSTEP(J,T)                     ( ( (J) * pow( (T) , 3 ) ) / 6 )         // 加加速段的位移量(步数)  S = 1/6 * J * t^3
#define ACCEL_TIME(T)                         ( (T) / 2 )                              // 加加速段和减加速段的时间是相等的

#define TRUE                                   1
#define FALSE                                  0

#define ACCEL                                  1   //  电机状态标记: 加速
#define AVESPEED                               2   //  电机状态标记: 匀速
#define DECEL                                  3   //  电机状态标记: 减速
#define STOP                                   0   //  电机状态标记: 停止

#define SPEED_MIN                              (T1_FREQ / (65535.0f))// 最低频率/速度

extern uint8_t print_flag;
void CalculateSpeedTab(int Vo, int Vt, float Time);
void CalcSpeed(int32_t Vo, int32_t Vt, float Time);
void stepper_move_S(int start_speed,int end_speed,float time);
void speed_decision(void);
#endif 
