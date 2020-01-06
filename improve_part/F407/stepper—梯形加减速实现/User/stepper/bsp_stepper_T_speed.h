#ifndef __BSP_STEPPER_T_SPEED_H
#define	__BSP_STEPPER_T_SPEED_H

#include "stm32f4xx.h"
#include "./stepper/bsp_stepper_init.h"
#include "math.h"

//梯形加减速相关变量
typedef struct {
  __IO uint8_t  run_state ;  // 电机旋转状态
  __IO uint8_t  dir ;        // 电机旋转方向
  __IO int32_t  step_delay;  // 下个脉冲周期（时间间隔），启动时为加速度
  __IO uint32_t decel_start; // 启动减速位置
  __IO int32_t  decel_val;   // 减速阶段步数
  __IO int32_t  min_delay;   // 最小脉冲周期(最大速度，即匀速段速度)
  __IO int32_t  accel_count; // 加减速阶段计数值
}speedRampData;





#define FALSE             0
#define TRUE              1
#define CW                0 // 顺时针
#define CCW               1 // 逆时针

/*电机速度决策中的四个状态*/
#define STOP              0 // 加减速曲线状态：停止
#define ACCEL             1 // 加减速曲线状态：加速阶段
#define DECEL             2 // 加减速曲线状态：减速阶段
#define RUN               3 // 加减速曲线状态：匀速阶段

/*频率相关参数*/
//定时器实际时钟频率为：168MHz/(TIM_PRESCALER+1)
//其中 高级定时器的 频率为168MHz,其他定时器为84MHz
//168/(5+1)=28Mhz
#define TIM_PRESCALER      5 
#define T1_FREQ           (SystemCoreClock/(TIM_PRESCALER+1)) // 频率ft值

/*电机单圈参数*/
#define STEP_ANGLE				1.8									//步进电机的步距角 单位：度
#define FSPR              (360.0/1.8)         //步进电机的一圈所需脉冲数
#define MICRO_STEP        32          				//细分器细分数 
#define SPR               (FSPR*MICRO_STEP)   //细分后一圈所需脉冲数

/*数学常数*/
#define ALPHA             ((float)(2*3.14159/SPR))       // α= 2*pi/spr
#define A_T_x10           ((float)(10*ALPHA*T1_FREQ))
#define T1_FREQ_148       ((float)((T1_FREQ*0.676)/10)) // 0.676为误差修正值
#define A_SQ              ((float)(2*100000*ALPHA)) 
#define A_x200            ((float)(200*ALPHA))


extern void speed_decision(void);

#endif
