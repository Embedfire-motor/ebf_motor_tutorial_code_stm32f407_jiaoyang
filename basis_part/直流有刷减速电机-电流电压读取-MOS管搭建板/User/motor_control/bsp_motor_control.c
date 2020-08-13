/**
  ******************************************************************************
  * @file    bsp_motor_control.c
  * @author  fire
  * @version V1.0
  * @date    2019-xx-xx
  * @brief   电机控制接口
  ******************************************************************************
  * @attention
  *
  * 实验平台:野火  STM32 F407 开发板 
  * 论坛    :http://www.firebbs.cn
  * 淘宝    :http://firestm32.taobao.com
  *
  ******************************************************************************
  */ 

#include ".\motor_control\bsp_motor_control.h"
#include "./tim/bsp_motor_tim.h"

/* 私有变量 */
static motor_dir_t direction  = MOTOR_FWD;     // 记录方向
static uint16_t    dutyfactor = 0;             // 记录占空比

static void sd_gpio_config(void)
{
  GPIO_InitTypeDef GPIO_InitStruct;
  
  /* 定时器通道功能引脚端口时钟使能 */
	SHUTDOWN_GPIO_CLK_ENABLE();
  
  /* 引脚IO初始化 */
	/*设置输出类型*/
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	/*设置引脚速率 */ 
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	/*选择要控制的GPIO引脚*/	
	GPIO_InitStruct.Pin = SHUTDOWN_PIN;
  
	/*调用库函数，使用上面配置的GPIO_InitStructure初始化GPIO*/
  HAL_GPIO_Init(SHUTDOWN_GPIO_PORT, &GPIO_InitStruct);
}

/**
  * @brief  电机初始化
  * @param  无
  * @retval 无
  */
void motor_init(void)
{
  Motor_TIMx_Configuration();     // 初始化电机 1
  sd_gpio_config();
}

/**
  * @brief  设置电机速度
  * @param  v: 速度（占空比）
  * @retval 无
  */
void set_motor_speed(uint16_t v)
{
  dutyfactor = v;
  
  if (direction == MOTOR_FWD)
  {
    SET_FWD_COMPAER(dutyfactor);     // 设置速度
  }
  else
  {
    SET_REV_COMPAER(dutyfactor);     // 设置速度
  }
}

/**
  * @brief  设置电机方向
  * @param  无
  * @retval 无
  */
void set_motor_direction(motor_dir_t dir)
{
  direction = dir;
  
  if (direction == MOTOR_FWD)
  {
    SET_FWD_COMPAER(dutyfactor);     // 设置速度
    SET_REV_COMPAER(0);              // 设置速度
  }
  else
  {
    SET_FWD_COMPAER(0);              // 设置速度
    SET_REV_COMPAER(dutyfactor);     // 设置速度
  }
}

/**
  * @brief  使能电机
  * @param  无
  * @retval 无
  */
void set_motor_enable(void)
{
  MOTOR_ENABLE_SD();    // SD 脚输出高电平使能
  MOTOR_FWD_ENABLE();   // 使能 PWM 输出
  MOTOR_REV_ENABLE();   // 使能 PWM 输出
}

/**
  * @brief  禁用电机
  * @param  无
  * @retval 无
  */
void set_motor_disable(void)
{
  MOTOR_DISABLE_SD();    // SD 脚输出高电平禁用
  MOTOR_FWD_DISABLE();   // 禁用 PWM 输出
  MOTOR_REV_DISABLE();   // 禁用 PWM 输出
}

/*********************************************END OF FILE**********************/

