/**
  ******************************************************************************
  * @file    bsp_stepper_spta_speed.c
  * @author  fire
  * @version V1.0
  * @date    2020-xx-xx
  * @brief   步进电机SPTA加减速算法
  ******************************************************************************
  * @attention
  *
  * 实验平台:野火  STM32 F407 开发板  
  * 论坛    :http://www.firebbs.cn
  * 淘宝    :http://firestm32.taobao.com
  *
  ******************************************************************************
  */
#include "./stepper/bsp_stepper_spta_speed.h"
#include "./stepper/bsp_stepper_init.h"
#include <stdlib.h>

#define MAXSPEED_SPTA 90000 //SPTA最大速度  80000
#define ACCSPEED_SPTA 10000 //SPTA加速度   150000

unsigned int spta_maxspeed = MAXSPEED_SPTA;
unsigned int spta_accspeed = ACCSPEED_SPTA;

SPTAData_Typedef spta_data = {0};

/**  @brief 启动spta加减速
  *  @param *pspta：spta数据结构体指针
  *  @param steps：移动的步数。正数为顺时针，负数为逆时针。
  *  @param speed_lim：最大速度，用于切换三角轮廓或梯形轮廓
  *  @param accel：减速度,如果取值为100，实际值为100*0.01*rad/sec^2=1rad/sec^2
  */
void Stepper_Move_SPTA(SPTAData_Typedef *pspta, int32_t steps, uint32_t speed_lim, uint32_t accel)
{
  /* 判断方向 */
  (steps < 0) ? MOTOR_DIR(CCW) : MOTOR_DIR(CW);
  steps = abs(steps);
  
  /*电机运行参数初始化*/
  pspta->steps_required = steps * MICRO_STEP;
  pspta->step_middle = pspta->steps_required >> 1;

  pspta->speed_lim = spta_maxspeed;
  pspta->acceleration = spta_accspeed;

  pspta->step_state = ACCELERATING;

  HAL_TIM_Base_Start_IT(&TIM_StepperHandle);
}

/**
  * @brief  spta速度决策
  * @param  *pspta：spta数据结构体指针
  * @retval 无
  */
void SPTA_Speed_Decision(SPTAData_Typedef *pspta)
{
  static uint8_t overflow_flag = 0;

  /* 拉低脉冲信号 */
  MOTOR_PUL(LOW);

  /*根据步数累加器是否溢出，决定是否产生一个步进脉冲
  步数累加器由步数速度器累加溢出产生*/
  pspta->step_accumulator += pspta->step_speed;
  //判断是否溢出
//  overflow_flag = pspta->step_accumulator >> 17;
//  if(overflow_flag != 0)
//  {
//    //溢出则产生一个脉冲
//    pspta->steps_taken++;
//    /*拉高脉冲信号产生一个步进脉冲*/
//    MOTOR_PUL(HIGH);
//    pspta->step_accumulator &= (0 << 17);
//  }
  overflow_flag = pspta->step_accumulator >> 17;
  pspta->step_accumulator -= overflow_flag << 17;
  if (overflow_flag != 0)
  {
    //溢出则产生一个脉冲
    pspta->steps_taken++;
    /*拉高脉冲信号产生一个步进脉冲*/
    MOTOR_PUL(HIGH);
  }

  /*根据电机的状态进行状态转换以及参数变换*/
  switch (pspta->step_state)
  {
  /* 步进电机加速状态 */
  case ACCELERATING:
    if (pspta->step_speed >= pspta->speed_lim)
    {
      //加速阶段到达一定的时刻，就达到了设定的最大速度
      pspta->step_speed = pspta->speed_lim;
      //转为最大速度状态
      pspta->step_state = UNIFORM;
      break;
    }
    //计算加速阶段的步数
    if (overflow_flag)
    {
      pspta->steps_acced++;
    }
    //加速阶段，速度累加器要根据设定的加速度值进行累加
    pspta->speed_accumulator += pspta->acceleration;
    //溢出判断
//    if((pspta->speed_accumulator >> 17) == 1)
//    {
//      /*如果速度累加器溢出，则步数速度器需要加上该值,
//        以便推动步数速度器的增长，为步数累加器溢出准备*/
//      pspta->step_speed++;
//      pspta->speed_accumulator &= (0 << 17);
//      //记录加速阶段的步数
//      pspta->steps_acced++;
//    }
    overflow_flag = pspta->speed_accumulator >> 17;
    pspta->speed_accumulator -= (overflow_flag << 17);
    if (overflow_flag)
    {
      //如果速度累加器溢出，则步数速度器需要加上该值，以便推动
      //步数速度器的增长，为步数累加器溢出准备
      pspta->step_speed += overflow_flag;
    }
    /*although we are ACCELERATING,but middle point reached,we need DECELERATING*/
    if (pspta->step_middle != 0)
    {
      if (pspta->steps_taken > pspta->step_middle)
      {
        pspta->step_state = DECELERATING;
      }
    }
    else if (pspta->steps_taken > 0)
    {
      //只运行一步？
      pspta->step_state = DECELERATING;
    }
    break;
  /* 步进电机匀速状态 */
  case UNIFORM:
    //在最大速度运行时，如果剩余需要运行的步数小余等于加速步则需要减速了，减速与加速
    //步数相同
    if (pspta->steps_required - pspta->steps_taken < pspta->steps_acced)
    {
      pspta->step_state = DECELERATING;
    }
    break;
  /* 步进电机减速状态 */
  case DECELERATING:
    //在运行设定步数以后停止运行
    if (pspta->steps_taken >= pspta->steps_required)
    {
      //设定电机状态
      pspta->step_state = IDLE;
      break;
    }
//    //减速阶段与加速阶段是相反的过程，原理也相同，只是很多+号变成-号
//    if (overflow_flag && pspta->steps_acced > 0)
//    {
//      pspta->steps_acced--;
//    }
    pspta->speed_accumulator += pspta->acceleration;
    overflow_flag = pspta->speed_accumulator >> 17;
    pspta->speed_accumulator -= (overflow_flag << 17);
    if (overflow_flag && pspta->step_speed > overflow_flag)
    {
      pspta->step_speed -= overflow_flag;
    }
    break;
  case IDLE:
    pspta->speed_lim = 0;
    pspta->step_accumulator = 0;
    pspta->speed_accumulator = 0;
    pspta->steps_acced = 0;
    pspta->step_speed = 0;
    pspta->steps_taken = 0;
    //停止定时器
    HAL_TIM_Base_Stop_IT(&TIM_StepperHandle);
    break;
  default:
    break;
  }
}

//void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
//{
//  TIMX_IRQHandler_SPTA(&stepper_motor);
//}
