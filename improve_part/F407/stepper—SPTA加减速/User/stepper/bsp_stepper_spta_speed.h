#ifndef __BSP_STEPPER_SPTA_SPEED_H
#define __BSP_STEPPER_SPTA_SPEED_H

#include "stm32f4xx.h"
#include "./stepper/bsp_stepper_init.h"
#include "math.h"

/* SPTA运行状态 */
typedef enum
{
  IDLE,              /* 停止 */
  ACCELERATING,      /* 加速 */
  UNIFORM,           /* 匀速 */
  DECELERATING,      /* 减速 */
}STEP_STATE_TypeDef;

//typedef __packed struct 
//{
//  uint32_t step_move     ;        //total move requested,设定运行的步数
//  uint32_t step_count    ;        //step counter;已经运行的步数
//  
//  uint32_t step_acced    ;        //steps in acceled stage;加速阶段的加速步数
//  uint32_t step_middle   ;        //mid-point of move, = (step_move - 1) >> 1;设定运行步数的1/2，超过该步数就要减速
//  
//  uint32_t step_accel    ;        //accel/decel rate, 8.8 bit format；设定的电机加速度数值
//  uint32_t speed_frac    ;        //speed counter fraction;速度片段累加器，每次都累加step_accel，该数值大于某个值后step_speed增加
//  
//  uint32_t step_speed    ;        //current speed, 16.8 bit format (HI byte always 0)；步数片段累加速度值，speed_frac溢出值加到这里
//  uint32_t step_frac     ;        //step counter fraction;步数片段累加器，每次累加step_speed，该数值大于某个值后产生一个步进脉冲，运行一步

//  uint32_t step_spmax    ;        //maximum speed,设定的电机最大速度 
//  
//  STEP_STATE_TypeDef step_state;//电机运转状态变量，加速、运行、减速
// 
//  TIM_HandleTypeDef* Handle_x;  
//} MOTOR_CONTROL_SPTA ;

typedef volatile struct 
{
  int32_t steps_required; //预计的总运行步数
  uint32_t steps_taken    ;        //step counter;已经运行的步数
  
  uint32_t steps_acced    ;        //加速阶段的步数
  uint32_t step_middle   ;        //mid-point of move, = (step_move - 1) >> 1;设定运行步数的1/2，超过该步数就要减速
  
  uint32_t acceleration;        //加速度数值
  uint32_t speed_accumulator;   //速度累加器，每次都累加step_accel，该数值大于某个值后step_speed增加
  
  uint32_t step_speed    ;        //current speed, 16.8 bit format (HI byte always 0)；步数片段累加速度值，speed_accumulator溢出值加到这里
  uint32_t step_accumulator;    //步数累加器，每次累加step_speed，该数值大于某个值后产生一个步进脉冲，运行一步

  uint32_t speed_lim;           //电机的最大速度限制，三角轮廓时不一定能达到
  
  STEP_STATE_TypeDef step_state;//电机运行状态枚举
} SPTAData_Typedef;


/*电机单圈参数*/
#define STEP_ANGLE        1.8                 //步进电机的步距角 单位：度
#define FSPR              (360.0/1.8)         //步进电机的一圈所需脉冲数

#define MICRO_STEP        32                  //细分器细分数 
#define SPR               (FSPR*MICRO_STEP)   //细分后一圈所需脉冲数


extern SPTAData_Typedef spta_data;
extern unsigned int spta_maxspeed;
extern unsigned int spta_accspeed;

//void SPTA_Init(uint8_t StepDive);
void Stepper_Move_SPTA(SPTAData_Typedef *pspta, int32_t steps, uint32_t speed_lim, uint32_t accel);
void SPTA_Speed_Decision(SPTAData_Typedef *pspta);

#endif /* __STEP_MOTOR_SPTA_H */

