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
#include <stdlib.h>

SPTAData_Typedef spta_data = {0};

/**  @brief 启动spta加减速
  *  @param steps：移动的步数，不考虑细分的整步数。正数为顺时针，负数为逆时针。
  *  @param speed_lim：最大速度限制，其值大小决定spta的速度曲线轮廓为三角轮廓或梯形轮廓
  *  @param accel：加速度数值
  */
void Stepper_Move_SPTA(int32_t steps, uint32_t speed_lim, uint32_t accel)
{
  SPTAData_Typedef *pspta = &spta_data;
  
  /* 判断是否正在加减速 */
  if(pspta->step_state != IDLE)
    return;
  
  /* 判断方向 */
  (steps < 0) ? MOTOR_DIR(CCW) : MOTOR_DIR(CW);
  steps = abs(steps);
  
  /* SPTA运行参数初始化 */
  pspta->steps_required = steps * MICRO_STEP;
  pspta->steps_middle = pspta->steps_required >> 1;

  pspta->speed_lim = speed_lim;
  pspta->acceleration = accel;

  pspta->step_state = ACCELERATING;

  /* 启动定时器 */
  HAL_TIM_Base_Start_IT(&TIM_StepperHandle);
}

/**
  * @brief  spta速度决策
  * @param  *pspta：spta数据结构体指针
  * @retval 无
  */
void SPTA_Speed_Decision(SPTAData_Typedef *pspta)
{
  /* 拉低脉冲信号 */
  MOTOR_PUL(LOW);

  /* 速度值累加到步数累加器 */
  pspta->step_accumulator += pspta->step_speed;
  
  /* 步数累加器是否溢出 */
  if((pspta->step_accumulator >> 17) == 1)
  {
    /* 溢出标志清零 */
    pspta->step_accumulator &= ~(1 << 17);
    /* 已走步数+1 */
    pspta->steps_taken++;
    /* 拉高脉冲信号产生一个步进脉冲 */
    MOTOR_PUL(HIGH);
  }

  /* 根据电机的状态进行状态转换以及参数计算 */
  switch (pspta->step_state)
  {
    /* SPTA加速状态 */
    case ACCELERATING:
      /* 是否达到最大速度限制*/
      if(pspta->step_speed >= pspta->speed_lim)
      {
        /* 达到最大速度 */
        pspta->step_speed = pspta->speed_lim;
        /* 此时走过的步数记为加速阶段的步数 */
        pspta->steps_acced = pspta->steps_taken;
        /* 转为匀速状态 */
        pspta->step_state = UNIFORM;
        break;
      }
      
      /* 加速度值加到速度累加器 */
      pspta->speed_accumulator += pspta->acceleration;
      /* 速度累加器是否溢出 */
      if((pspta->speed_accumulator >> 17) == 1)
      {
        /* 溢出标志清零 */
        pspta->speed_accumulator &= ~(1 << 17);
        /* 加速阶段，速度累加器溢出，则速度值递增 */
        pspta->step_speed++;
      }
      
      if(pspta->steps_middle != 0)
      {
        /* 如果已经运行的步数≥中点步数，则开始减速 */
        if (pspta->steps_taken >= pspta->steps_middle)
        {
          pspta->step_state = DECELERATING;
        }
      }
      else if(pspta->steps_taken > 0)
      {
        /* 只运行一步，直接变为减速状态 */
        pspta->step_state = DECELERATING;
      }
      break;
    /* SPTA匀速状态 */
    case UNIFORM:
      /* 匀速运行阶段，当剩余步数≤加速阶段步数，则开始减速 */
      if ((pspta->steps_required - pspta->steps_taken) <= pspta->steps_acced)
      {
        pspta->step_state = DECELERATING;
      }
      break;
    /* SPTA减速状态 */
    case DECELERATING:
      /* 到达设定的步数之后停止运行 */
      if(pspta->steps_taken >= pspta->steps_required)
      {
        /* 转为空闲状态 */
        pspta->step_state = IDLE;
        break;
      }
      
      /* 加速度值加到速度累加器 */
      pspta->speed_accumulator += pspta->acceleration;
      /* 速度累加器是否溢出 */
      if((pspta->speed_accumulator >> 17) == 1)
      {
        /* 溢出标志清零 */
        pspta->speed_accumulator &= ~(1 << 17);
        /* 减速阶段，速度累加器溢出，速度值递减 */
        pspta->step_speed--;
      }
      break;
    /* SPTA空闲状态 */
    case IDLE:
      /* 清零spta相关数据 */
      pspta->speed_lim = 0;
      pspta->step_accumulator = 0;
      pspta->speed_accumulator = 0;
      pspta->steps_acced = 0;
      pspta->step_speed = 0;
      pspta->steps_taken = 0;
      /* 关闭定时器 */
      HAL_TIM_Base_Stop_IT(&TIM_StepperHandle);
      break;
    default:
      break;
  }
}
