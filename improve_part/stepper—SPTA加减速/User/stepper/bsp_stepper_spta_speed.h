#ifndef __BSP_STEPPER_SPTA_SPEED_H
#define __BSP_STEPPER_SPTA_SPEED_H

#include "stm32f4xx.h"
#include "./stepper/bsp_stepper_init.h"

/* SPTA运行状态 */
typedef enum
{
  IDLE = 0U,         /* 停止 */
  ACCELERATING,      /* 加速 */
  UNIFORM,           /* 匀速 */
  DECELERATING,      /* 减速 */
}STEP_STATE_TypeDef;

/* SPTA相关参数结构体 */
typedef struct 
{
  __IO int32_t steps_required;       //总运行步数
  __IO uint32_t steps_taken;         //已经运行的步数
  
  __IO uint32_t steps_acced;         //加速阶段的步数
  __IO uint32_t steps_middle;        //总步数的中点，三角轮廓时超过该步数就要减速
  
  __IO uint32_t acceleration;        //加速度的数值，不包括方向
  __IO uint32_t speed_accumulator;   //速度累加器，每次累加step_accel，当累加器第17位置位后step_speed增加
  
  __IO uint32_t step_speed;          //SPTA速度值，speed_accumulator的溢出值加到这里
  __IO uint32_t step_accumulator;    //步数累加器，每次累加step_speed，当累加器第17位置位后产生一个步进脉冲

  __IO uint32_t speed_lim;           //电机的最大速度限制，运行速度小于速度限制时为三角轮廓，大于等于速度限制时为梯形轮廓
  
  STEP_STATE_TypeDef step_state;     //电机运行状态枚举
}SPTAData_Typedef;


/*电机单圈参数*/
#define STEP_ANGLE        1.8f                 //步进电机的步距角 单位：度
#define FSPR              (360.0f/STEP_ANGLE)  //步进电机的一圈所需脉冲数

#define MICRO_STEP        32                   //细分器细分数 
#define SPR               (FSPR*MICRO_STEP)    //细分后一圈所需脉冲数


extern SPTAData_Typedef spta_data;
extern unsigned int spta_maxspeed;
extern unsigned int spta_accspeed;

void Stepper_Move_SPTA(int32_t steps, uint32_t speed_lim, uint32_t accel);
void SPTA_Speed_Decision(SPTAData_Typedef *pspta);

#endif /* __STEP_MOTOR_SPTA_H */

