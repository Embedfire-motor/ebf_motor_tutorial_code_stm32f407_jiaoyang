/**
  ******************************************************************************
  * @file    bsp_bldcm_control.c
  * @author  fire
  * @version V1.0
  * @date    2019-xx-xx
  * @brief   无刷电机控制接口
  ******************************************************************************
  * @attention
  *
  * 实验平台:野火  STM32 F407 开发板 
  * 论坛    :http://www.firebbs.cn
  * 淘宝    :http://firestm32.taobao.com
  *
  ******************************************************************************
  */ 

#include ".\bldcm_control\bsp_bldcm_control.h"
#include "./pid/bsp_pid.h"

/* 私有变量 */
static bldcm_data_t bldcm_data;

/**
  * @brief  电机初始化
  * @param  无
  * @retval 无
  */
void bldcm_init(void)
{
  PWM_TIMx_Configuration();    // 电机控制定时器，引脚初始化
  hall_tim_config();           // 霍尔传感器初始化
}

/**
  * @brief  设置电机速度
  * @param  v: 速度（占空比）
  * @retval 无
  */
void set_bldcm_speed(uint16_t v)
{
  bldcm_data.dutyfactor = v;
  
  set_pwm_pulse(v);     // 设置速度
}

/**
  * @brief  设置电机方向
  * @param  无
  * @retval 无
  */
void set_bldcm_direction(motor_dir_t dir)
{
  bldcm_data.direction = dir;
}

/**
  * @brief  获取电机当前方向
  * @param  无
  * @retval 无
  */
motor_dir_t get_bldcm_direction(void)
{
  return bldcm_data.direction;
}

/**
  * @brief  使能电机
  * @param  无
  * @retval 无
  */
void set_bldcm_enable(void)
{
  bldcm_data.is_enable = 1;
  hall_enable();
}

/**
  * @brief  禁用电机
  * @param  无
  * @retval 无
  */
void set_bldcm_disable(void)
{
  /* 禁用霍尔传感器接口 */
  hall_disable();
  
  /* 停止 PWM 输出 */
  stop_pwm_output();
  
  bldcm_data.is_enable = 0;
}

/**
  * @brief  电机位置式 PID 控制实现(定时调用)
  * @param  无
  * @retval 无
  */
extern uint8_t dir;
void bldcm_pid_control(void)
{
  if (bldcm_data.is_enable)
  {
    float cont_val = 0;    // 当前控制值
    
    int32_t actual = get_motor_speed();   // 电机旋转的当前速度

    cont_val = PID_realize(actual);
//    printf("控制变量前：%0.2f，", cont_val);
    if (cont_val < 0)
    {
      cont_val = -cont_val;
    }
//    else
//    {
//      set_bldcm_direction(MOTOR_FWD);
//    }
    
    cont_val = cont_val > PWM_PERIOD_COUNT ? PWM_PERIOD_COUNT : cont_val;
    set_bldcm_speed(cont_val);
    
    float get_motor_dir(void);
    if (get_motor_dir() != 0)
    {
      actual = -actual;
    }
    
  #ifdef PID_ASSISTANT_EN
    set_computer_value(SEND_FACT_CMD, CURVES_CH1, &actual, 1);     // 给通道 1 发送实际值
  #else
//    printf("实际值：%d. 目标值：%d\n", actual, get_pid_target());
    printf("控制变量：%0.2f，实际值 ：%d, 目标值：%.0f, dir = %d\n", cont_val, actual, get_pid_target(), dir);
  #endif
  }
}

 
///**
//  * @brief  定时器每100ms产生一次中断回调函数
//  * @param  htim：定时器句柄
//  * @retval 无
//  */
//void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
//{
//    if(htim==(&TIM_TimeBaseStructure))
//    {
//        bldcm_pid_control();
//    }
//}

/*********************************************END OF FILE**********************/
