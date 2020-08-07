/**
  ******************************************************************************
  * @file    bsp_stepper_init.c
  * @author  fire
  * @version V1.0
  * @date    2019-xx-xx
  * @brief   步进电机初始化
  ******************************************************************************
  * @attention
  *
  * 实验平台:野火  STM32 F407 开发板  
  * 论坛    :http://www.firebbs.cn
  * 淘宝    :http://firestm32.taobao.com
  *
  ******************************************************************************
  */
#include "./stepper/bsp_stepper_init.h"
#include "./delay/core_delay.h"   
#include "stm32f4xx.h"

void TIM_SetTIMxCompare(TIM_TypeDef *TIMx,uint32_t channel,uint32_t compare);
void TIM_SetPWM_period(TIM_TypeDef* TIMx,uint32_t TIM_period);

/**
  * @brief  配置TIM复用输出PWM时用到的I/O
  * @param  无
  * @retval 无
  */
static void Stepper_GPIO_Config(void) 
{
  GPIO_InitTypeDef GPIO_InitStruct;
  /*开启Motor相关的GPIO外设时钟*/
  MOTOR_DIR_GPIO_CLK_ENABLE();
  MOTOR_PUL_GPIO_CLK_ENABLE();
  MOTOR_EN_GPIO_CLK_ENABLE();
  
  /*选择要控制的GPIO引脚*/                                 
  GPIO_InitStruct.Pin = MOTOR_DIR_PIN;  
  /*设置引脚的输出类型为推挽输出*/
  GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP; 
  GPIO_InitStruct.Pull =GPIO_PULLUP;
  /*设置引脚速率为高速 */   
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  /*Motor 方向引脚 初始化*/
  HAL_GPIO_Init(MOTOR_DIR_GPIO_PORT, &GPIO_InitStruct); 
  
  /*Motor 使能引脚 初始化*/
  GPIO_InitStruct.Pin = MOTOR_EN_PIN; 
  HAL_GPIO_Init(MOTOR_EN_GPIO_PORT, &GPIO_InitStruct);  
  
  /* 定时器通道1功能引脚IO初始化 */
  /*设置输出类型*/
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  /*设置引脚速率 */ 
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  /*设置复用*/
  GPIO_InitStruct.Alternate = MOTOR_PUL_GPIO_AF;
  /*设置复用*/
  GPIO_InitStruct.Pull =GPIO_PULLUP;
  /*选择要控制的GPIO引脚*/  
  GPIO_InitStruct.Pin = GENERAL_TIM_CH1_PIN;
  /*Motor 脉冲引脚 初始化*/
  HAL_GPIO_Init(GENERAL_TIM_CH1_GPIO_PORT, &GPIO_InitStruct);     
}


/*
 * 注意：TIM_TimeBaseInitTypeDef结构体里面有5个成员，TIM6和TIM7的寄存器里面只有
 * TIM_Prescaler和TIM_Period，所以使用TIM6和TIM7的时候只需初始化这两个成员即可，
 * 另外三个成员是通用定时器和高级定时器才有.
 *-----------------------------------------------------------------------------
 * TIM_Prescaler         都有
 * TIM_CounterMode       TIMx,x[6,7]没有，其他都有（基本定时器）
 * TIM_Period            都有
 * TIM_ClockDivision     TIMx,x[6,7]没有，其他都有(基本定时器)
 * TIM_RepetitionCounter TIMx,x[1,8]才有(高级定时器)
 *-----------------------------------------------------------------------------
 */

TIM_OC_InitTypeDef  TIM_OCInitStructure;  
TIM_HandleTypeDef  TIM_TimeBaseStructure; 
static void TIM_PWMOUTPUT_Config(void)
{
  
  int tim_per=1000;//定时器周期

  /*使能定时器*/
  MOTOR_PUL_CLK_ENABLE();

  TIM_TimeBaseStructure.Instance = MOTOR_PUL_TIM;
  /* 累计 TIM_Period个后产生一个更新或者中断*/    
  //当定时器从0计数到10000，即为10000次，为一个定时周期
  TIM_TimeBaseStructure.Init.Period = tim_per;
  // 通用控制定时器时钟源TIMxCLK = HCLK/2=84MHz 
  // 设定定时器频率为=TIMxCLK/(TIM_Prescaler+1)=1MHz
  TIM_TimeBaseStructure.Init.Prescaler = 168-1; 

  /*计数方式*/
  TIM_TimeBaseStructure.Init.CounterMode = TIM_COUNTERMODE_UP;
  /*采样时钟分频*/
  TIM_TimeBaseStructure.Init.ClockDivision=TIM_CLOCKDIVISION_DIV1;
  /*初始化定时器*/
  HAL_TIM_Base_Init(&TIM_TimeBaseStructure);

  /*PWM模式配置*/
  TIM_OCInitStructure.OCMode = TIM_OCMODE_PWM1;//配置为PWM模式1 
  TIM_OCInitStructure.Pulse = tim_per/2;//默认占空比为50%
  TIM_OCInitStructure.OCFastMode = TIM_OCFAST_DISABLE;
  /*当定时器计数值小于CCR1_Val时为高电平*/
  TIM_OCInitStructure.OCPolarity = TIM_OCPOLARITY_HIGH; 

  /*配置PWM通道*/
  HAL_TIM_PWM_ConfigChannel(&TIM_TimeBaseStructure, &TIM_OCInitStructure, MOTOR_PUL_CHANNEL_x);
  /*开始输出PWM*/
  HAL_TIM_PWM_Start(&TIM_TimeBaseStructure,MOTOR_PUL_CHANNEL_x);
      
}

/**
  * @brief  设置TIM通道的占空比
  * @param  channel   通道  （1,2,3,4）
  * @param  compare   占空比
  * @note   无
  * @retval 无
  */
void TIM2_SetPWM_pulse(int channel,int compare)
{
  switch(channel)
  {
    case 1:   __HAL_TIM_SET_COMPARE(&TIM_TimeBaseStructure,TIM_CHANNEL_1,compare);break;
    case 2:   __HAL_TIM_SET_COMPARE(&TIM_TimeBaseStructure,TIM_CHANNEL_2,compare);break;
    case 3:   __HAL_TIM_SET_COMPARE(&TIM_TimeBaseStructure,TIM_CHANNEL_3,compare);break;
    case 4:   __HAL_TIM_SET_COMPARE(&TIM_TimeBaseStructure,TIM_CHANNEL_4,compare);break;
  }
}



/**
  * @brief  引脚初始化
  * @retval 无
  */
void stepper_Init()
{
  Stepper_GPIO_Config();

  TIM_PWMOUTPUT_Config();
            
}



















