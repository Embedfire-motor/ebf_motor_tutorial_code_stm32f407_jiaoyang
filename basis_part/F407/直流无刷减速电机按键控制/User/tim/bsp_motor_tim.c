/**
  ******************************************************************************
  * @file    bsp_motor_tim.c
  * @author  STMicroelectronics
  * @version V1.0
  * @date    2015-xx-xx
  * @brief   电机相关定时器配置
  ******************************************************************************
  * @attention
  *
  * 实验平台:野火  STM32 F407 开发板  
  * 论坛    :http://www.firebbs.cn
  * 淘宝    :http://firestm32.taobao.com
  *
  ******************************************************************************
  */
  
#include "./tim/bsp_motor_tim.h"

TIM_HandleTypeDef  TIM_TimeBaseStructure;
TIM_OC_InitTypeDef TIM_OCInitStructure;

__IO uint16_t ChannelPulse = 500;

/**
  * @brief  配置TIM复用输出PWM时用到的I/O
  * @param  无
  * @retval 无
  */
static void TIMx_GPIO_Config(void) 
{
	/*定义一个GPIO_InitTypeDef类型的结构体*/
	GPIO_InitTypeDef GPIO_InitStructure;

	/*开启定时器相关的GPIO外设时钟*/
	ADVANCE_OCPWM1_GPIO_CLK_ENABLE();
	ADVANCE_OCNPWM1_GPIO_CLK_ENABLE();
  ADVANCE_OCPWM2_GPIO_CLK_ENABLE();
	ADVANCE_OCNPWM2_GPIO_CLK_ENABLE();
  ADVANCE_OCPWM3_GPIO_CLK_ENABLE();
	ADVANCE_OCNPWM3_GPIO_CLK_ENABLE();
//	ADVANCE_BKIN_GPIO_CLK_ENABLE(); 

	/* 定时器功能引脚初始化 */															   
	GPIO_InitStructure.Pin = ADVANCE_OCPWM1_PIN;	
	GPIO_InitStructure.Mode = GPIO_MODE_AF_PP;    
	GPIO_InitStructure.Pull = GPIO_NOPULL;
	GPIO_InitStructure.Speed = GPIO_SPEED_HIGH; 	
	GPIO_InitStructure.Alternate = ADVANCE_OCPWM1_AF;
	HAL_GPIO_Init(ADVANCE_OCPWM1_GPIO_PORT, &GPIO_InitStructure);	

	GPIO_InitStructure.Pin = ADVANCE_OCNPWM1_PIN;	
	GPIO_InitStructure.Alternate = ADVANCE_OCNPWM1_AF;	
	HAL_GPIO_Init(ADVANCE_OCNPWM1_GPIO_PORT, &GPIO_InitStructure);
  
  /* 通道 2 */
  GPIO_InitStructure.Pin = ADVANCE_OCPWM2_PIN;	
	GPIO_InitStructure.Alternate = ADVANCE_OCPWM2_AF;	
	HAL_GPIO_Init(ADVANCE_OCPWM2_GPIO_PORT, &GPIO_InitStructure);
  
  GPIO_InitStructure.Pin = ADVANCE_OCNPWM2_PIN;	
	GPIO_InitStructure.Alternate = ADVANCE_OCNPWM2_AF;	
	HAL_GPIO_Init(ADVANCE_OCNPWM2_GPIO_PORT, &GPIO_InitStructure);
  
  
  /* 通道 3 */
  GPIO_InitStructure.Pin = ADVANCE_OCPWM3_PIN;	
	GPIO_InitStructure.Alternate = ADVANCE_OCPWM3_AF;	
	HAL_GPIO_Init(ADVANCE_OCPWM3_GPIO_PORT, &GPIO_InitStructure);
  
  GPIO_InitStructure.Pin = ADVANCE_OCNPWM3_PIN;	
	GPIO_InitStructure.Alternate = ADVANCE_OCNPWM3_AF;	
	HAL_GPIO_Init(ADVANCE_OCNPWM3_GPIO_PORT, &GPIO_InitStructure);
	
//	GPIO_InitStructure.Pin = ADVANCE_BKIN_PIN;	
//	GPIO_InitStructure.Alternate = ADVANCE_BKIN_AF;	
//	HAL_GPIO_Init(ADVANCE_BKIN_GPIO_PORT, &GPIO_InitStructure);
}

/*
 * 注意：TIM_TimeBaseInitTypeDef结构体里面有5个成员，TIM6和TIM7的寄存器里面只有
 * TIM_Prescaler和TIM_Period，所以使用TIM6和TIM7的时候只需初始化这两个成员即可，
 * 另外三个成员是通用定时器和高级定时器才有.
 *-----------------------------------------------------------------------------
 * TIM_Prescaler         都有
 * TIM_CounterMode			 TIMx,x[6,7]没有，其他都有（基本定时器）
 * TIM_Period            都有
 * TIM_ClockDivision     TIMx,x[6,7]没有，其他都有(基本定时器)
 * TIM_RepetitionCounter TIMx,x[1,8]才有(高级定时器)
 *-----------------------------------------------------------------------------
 */
static void TIM_Mode_Config(void)
{
//	TIM_BreakDeadTimeConfigTypeDef TIM_BDTRInitStructure;
	// 开启TIMx_CLK,x[1,8] 
	ADVANCE_TIM_CLK_ENABLE(); 
	/* 定义定时器的句柄即确定定时器寄存器的基地址*/
	TIM_TimeBaseStructure.Instance = ADVANCE_TIM;
	/* 累计 TIM_Period个后产生一个更新或者中断*/		
	//当定时器从0计数到999，即为1000次，为一个定时周期
	TIM_TimeBaseStructure.Init.Period = PWM_PERIOD_COUNT;
	// 高级控制定时器时钟源TIMxCLK = HCLK=216MHz 
	// 设定定时器频率为=TIMxCLK/(TIM_Prescaler+1)=1MHz
	TIM_TimeBaseStructure.Init.Prescaler = PWM_PRESCALER_COUNT;	
	// 采样时钟分频
	TIM_TimeBaseStructure.Init.ClockDivision=TIM_CLOCKDIVISION_DIV1;
	// 计数方式
	TIM_TimeBaseStructure.Init.CounterMode=TIM_COUNTERMODE_UP;
	// 重复计数器
	TIM_TimeBaseStructure.Init.RepetitionCounter=0;	
	// 初始化定时器TIMx, x[1,8]
	HAL_TIM_PWM_Init(&TIM_TimeBaseStructure);

	/*PWM模式配置*/
	//配置为PWM模式1
	TIM_OCInitStructure.OCMode = TIM_OCMODE_PWM1;
	TIM_OCInitStructure.Pulse = ChannelPulse;
	TIM_OCInitStructure.OCPolarity = TIM_OCPOLARITY_HIGH;
	TIM_OCInitStructure.OCNPolarity = TIM_OCNPOLARITY_HIGH;
	TIM_OCInitStructure.OCIdleState = TIM_OCIDLESTATE_SET;
	TIM_OCInitStructure.OCNIdleState = TIM_OCNIDLESTATE_RESET;
	
	HAL_TIM_PWM_ConfigChannel(&TIM_TimeBaseStructure,&TIM_OCInitStructure,TIM_CHANNEL_1);    // 初始化通道 1 输出 PWM 
  HAL_TIM_PWM_ConfigChannel(&TIM_TimeBaseStructure,&TIM_OCInitStructure,TIM_CHANNEL_2);    // 初始化通道 2 输出 PWM
  HAL_TIM_PWM_ConfigChannel(&TIM_TimeBaseStructure,&TIM_OCInitStructure,TIM_CHANNEL_3);    // 初始化通道 3 输出 PWM

	/* 自动输出使能，断路、死区时间和锁定配置 */
//	TIM_BDTRInitStructure.OffStateRunMode = TIM_OSSR_ENABLE;
//	TIM_BDTRInitStructure.OffStateIDLEMode = TIM_OSSI_ENABLE;
//	TIM_BDTRInitStructure.LockLevel = TIM_LOCKLEVEL_1;
//	TIM_BDTRInitStructure.DeadTime = 11;
//	TIM_BDTRInitStructure.BreakState = TIM_BREAK_ENABLE;
//	TIM_BDTRInitStructure.BreakPolarity = TIM_BREAKPOLARITY_LOW;
//	TIM_BDTRInitStructure.AutomaticOutput = TIM_AUTOMATICOUTPUT_ENABLE;
//	HAL_TIMEx_ConfigBreakDeadTime(&TIM_TimeBaseStructure, &TIM_BDTRInitStructure);

	/* 关闭定时器通道1输出PWM */
	HAL_TIM_PWM_Stop(&TIM_TimeBaseStructure,TIM_CHANNEL_1);
	/* 关闭定时器通道1互补输出PWM */
	HAL_TIMEx_PWMN_Stop(&TIM_TimeBaseStructure,TIM_CHANNEL_1);
  
  /* 关闭定时器通道2输出PWM */
	HAL_TIM_PWM_Stop(&TIM_TimeBaseStructure,TIM_CHANNEL_2);
	/* 关闭定时器通道2互补输出PWM */
	HAL_TIMEx_PWMN_Stop(&TIM_TimeBaseStructure,TIM_CHANNEL_2);
  
  /* 关闭定时器通道3输出PWM */
	HAL_TIM_PWM_Stop(&TIM_TimeBaseStructure,TIM_CHANNEL_3);
	/* 关闭定时器通道3互补输出PWM */
	HAL_TIMEx_PWMN_Stop(&TIM_TimeBaseStructure,TIM_CHANNEL_3);
}

/**
  * @brief  初始化高级控制定时器
  * @param  无
  * @retval 无
  */
void TIMx_Configuration(void)
{
	TIMx_GPIO_Config();
	TIM_Mode_Config();
}

/* 霍尔传感器相关定时器初始出 */
TIM_HandleTypeDef htimx_hall;

/**
  * 函数功能: 定时器霍尔传感器接口硬件初始化配置
  * 输入参数: htim：定时器句柄类型指针
  * 返 回 值: 无
  * 说    明: 该函数由库函数调用
  */
static void hall_gpio_init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct;

  HALL_INPUT1_GPIO_CLK_ENABLE();
  HALL_INPUT2_GPIO_CLK_ENABLE();
  HALL_INPUT3_GPIO_CLK_ENABLE();
  
  /* 定时器通道 1 引脚初始化 */
  GPIO_InitStruct.Pin = HALL_INPUT1_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull=GPIO_PULLUP;
  GPIO_InitStruct.Alternate = HALL_INPUT1_AF;
  HAL_GPIO_Init(HALL_INPUT1_GPIO_PORT, &GPIO_InitStruct);
  
  /* 定时器通道 2 引脚初始化 */
  GPIO_InitStruct.Pin = HALL_INPUT2_PIN;
  HAL_GPIO_Init(HALL_INPUT2_GPIO_PORT, &GPIO_InitStruct);
  
  /* 定时器通道 3 引脚初始化 */
  GPIO_InitStruct.Pin = HALL_INPUT3_PIN;
  HAL_GPIO_Init(HALL_INPUT3_GPIO_PORT, &GPIO_InitStruct);
}
/**
  * 函数功能: 定时器霍尔传感器接口初始化
  * 输入参数: 无
  * 返 回 值: 无
  * 说    明: 无
  */
static void hall_tim_init(void)
{
  TIM_HallSensor_InitTypeDef  hall_sensor_onfig;  
  
  /* 基本定时器外设时钟使能 */
  HALL_TIM_CLK_ENABLE();
  
  /* 定时器基本功能配置 */
  htimx_hall.Instance = HALL_TIM;
  htimx_hall.Init.Prescaler = HALL_PRESCALER_COUNT;         // 预分频
  htimx_hall.Init.CounterMode = TIM_COUNTERMODE_UP;         // 向上计数
  htimx_hall.Init.Period = HALL_PERIOD_COUNT;               // 计数周期
  htimx_hall.Init.ClockDivision=TIM_CLOCKDIVISION_DIV1;     // 时钟分频
  
  hall_sensor_onfig.IC1Prescaler=TIM_ICPSC_DIV1;            // 输入捕获分频
  hall_sensor_onfig.IC1Polarity=TIM_ICPOLARITY_BOTHEDGE;    // 输入捕获极性
  hall_sensor_onfig.IC1Filter= 10;                          // 输入滤波
  hall_sensor_onfig.Commutation_Delay = 0U;                 // 不使用延迟触发
  HAL_TIMEx_HallSensor_Init(&htimx_hall,&hall_sensor_onfig);
  
  HAL_NVIC_SetPriority(HALL_TIM_IRQn, 0, 0);    // 设置中断优先级
  HAL_NVIC_EnableIRQ(HALL_TIM_IRQn);            // 使能中断
}

/**
  * @brief  初始化霍尔传感器定时器
  * @param  无
  * @retval 无
  */
void hall_tim_config(void)
{
	hall_gpio_init();	    // 初始化引脚
	hall_tim_init();      // 初始化定时器
}

/*********************************************END OF FILE**********************/
