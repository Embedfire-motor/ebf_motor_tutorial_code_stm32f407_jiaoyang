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

TIM_HandleTypeDef TIM_StepperHandle = {0};

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
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	/*Motor 方向引脚 初始化*/
	HAL_GPIO_Init(MOTOR_DIR_GPIO_PORT, &GPIO_InitStruct);	
  HAL_GPIO_WritePin(MOTOR_DIR_GPIO_PORT, MOTOR_DIR_PIN, GPIO_PIN_SET);
	/*Motor 使能引脚 初始化*/
	GPIO_InitStruct.Pin = MOTOR_EN_PIN;	
	HAL_GPIO_Init(MOTOR_EN_GPIO_PORT, &GPIO_InitStruct);	

	/*选择要控制的GPIO引脚*/	
	GPIO_InitStruct.Pin = MOTOR_PUL_PIN;
	/*Motor 脉冲引脚 初始化*/
	HAL_GPIO_Init(MOTOR_PUL_GPIO_PORT, &GPIO_InitStruct);			
  HAL_GPIO_WritePin(MOTOR_PUL_GPIO_PORT, MOTOR_PUL_PIN, GPIO_PIN_RESET);
}

/**
  * @brief  配置步进电机的TIM
  * @param  无
  * @retval 无
  */
static void Stepper_TIM_Init(void)
{
  MOTOR_TIM_CLK_ENABLE();
  
  TIM_StepperHandle.Instance = MOTOR_TIM;
  TIM_StepperHandle.Init.Prescaler = MOTOR_TIM_PRESCALER - 1;
  TIM_StepperHandle.Init.CounterMode = TIM_COUNTERMODE_UP;
  TIM_StepperHandle.Init.Period = MOTOR_TIM_PERIOD;
  TIM_StepperHandle.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  HAL_TIM_Base_Init(&TIM_StepperHandle);

  /* 清零中断标志位 */
  __HAL_TIM_CLEAR_IT(&TIM_StepperHandle,TIM_IT_UPDATE);
  /* 使能定时器的更新事件中断 */
  __HAL_TIM_ENABLE_IT(&TIM_StepperHandle,TIM_IT_UPDATE);
  /* 设置更新事件请求源为：计数器溢出 */
  __HAL_TIM_URS_ENABLE(&TIM_StepperHandle);
  
  /* 关闭定时器 */
  HAL_TIM_Base_Stop_IT(&TIM_StepperHandle);
}

/**
  * @brief  中断优先级配置
  * @param  无
  * @retval 无
  */
static void Stepper_NVIC_Init(void)
{
  /* 外设中断配置 */
  HAL_NVIC_SetPriority(MOTOR_TIM_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(MOTOR_TIM_IRQn);
}

/**
  * @brief  步进电机初始化
  * @retval 无
  */
void stepper_Init()
{
	/* 电机IO配置 */
	Stepper_GPIO_Config();
  /* 电机定时器初始化 */
  Stepper_TIM_Init();
  /* 中断优先级配置 */
  Stepper_NVIC_Init();
}

