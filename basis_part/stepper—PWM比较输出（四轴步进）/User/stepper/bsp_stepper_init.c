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

/* 定时器句柄 */
TIM_HandleTypeDef TIM_StepperHandle;

/* 步进电机数组 */
Stepper_TypeDef step_motor[4] = 
{
  {MOTOR_PUL1_PIN, MOTOR_DIR1_PIN, MOTOR_EN1_PIN, MOTOR_PUL1_CHANNEL, MOTOR_PUL1_PORT, MOTOR_DIR1_GPIO_PORT, MOTOR_EN1_GPIO_PORT, 100},
  {MOTOR_PUL2_PIN, MOTOR_DIR2_PIN, MOTOR_EN2_PIN, MOTOR_PUL2_CHANNEL, MOTOR_PUL2_PORT, MOTOR_DIR2_GPIO_PORT, MOTOR_EN2_GPIO_PORT, 150},
  {MOTOR_PUL3_PIN, MOTOR_DIR3_PIN, MOTOR_EN3_PIN, MOTOR_PUL3_CHANNEL, MOTOR_PUL3_PORT, MOTOR_DIR3_GPIO_PORT, MOTOR_EN3_GPIO_PORT, 200},
  {MOTOR_PUL4_PIN, MOTOR_DIR4_PIN, MOTOR_EN4_PIN, MOTOR_PUL4_CHANNEL, MOTOR_PUL4_PORT, MOTOR_DIR4_GPIO_PORT, MOTOR_EN4_GPIO_PORT, 250},
};

/**
  * @brief  中断优先级配置
  * @param  无
  * @retval 无
  */
static void TIMx_NVIC_Configuration(void)
{
  /* 外设中断配置 */
  HAL_NVIC_SetPriority(MOTOR_PUL_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(MOTOR_PUL_IRQn);
}

/**
  * @brief  配置TIM复用输出PWM时用到的I/O
  * @param  无
  * @retval 无
  */
static void Stepper_GPIO_Config(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;
  
  MOTOR_DIR1_GPIO_CLK_ENABLE();
  MOTOR_EN1_GPIO_CLK_ENABLE();
  MOTOR_PUL1_GPIO_CLK_ENABLE();

  MOTOR_DIR2_GPIO_CLK_ENABLE();
  MOTOR_EN2_GPIO_CLK_ENABLE();
  MOTOR_PUL2_GPIO_CLK_ENABLE();
  
  MOTOR_DIR3_GPIO_CLK_ENABLE();
  MOTOR_EN3_GPIO_CLK_ENABLE();
  MOTOR_PUL3_GPIO_CLK_ENABLE();
  
  MOTOR_DIR4_GPIO_CLK_ENABLE();
  MOTOR_EN4_GPIO_CLK_ENABLE();
  MOTOR_PUL4_GPIO_CLK_ENABLE();

  /* 获取数组元素个数 */
  uint8_t member_count = sizeof(step_motor)/sizeof(Stepper_TypeDef);
  
  for(uint8_t i = 0; i < member_count; i++)
  {
    /*选择要控制的GPIO引脚*/															   
    GPIO_InitStruct.Pin = step_motor[i].dir_pin;	
    /*设置引脚的输出类型为推挽输出*/
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;  
    GPIO_InitStruct.Pull =GPIO_PULLUP;
    /*设置引脚速率为高速 */   
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    /*Motor 方向引脚 初始化*/
    HAL_GPIO_Init(step_motor[i].dir_port, &GPIO_InitStruct);
    MOTOR_DIR(step_motor[i].dir_port, step_motor[i].dir_pin, CW);
    
    /*Motor 使能引脚 初始化*/
    GPIO_InitStruct.Pin = step_motor[i].en_pin;
    HAL_GPIO_Init(step_motor[i].en_port, &GPIO_InitStruct);
    MOTOR_OFFLINE(step_motor[i].en_port, step_motor[i].en_pin, ON);
    
    /* 定时器输出通道功能引脚IO初始化 */
    /*设置输出类型*/
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    /*设置引脚速率 */ 
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    /*设置复用*/
    GPIO_InitStruct.Alternate = MOTOR_PUL_GPIO_AF;
    /*设置复用*/
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    /*选择要控制的GPIO引脚*/	
    GPIO_InitStruct.Pin = step_motor[i].pul_pin;
    /*Motor 脉冲引脚 初始化*/
    HAL_GPIO_Init(step_motor[i].pul_port, &GPIO_InitStruct);
  }
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
static void TIM_PWMOUTPUT_Config(void)
{
	TIM_OC_InitTypeDef  TIM_OCInitStructure;

  /* 获取数组元素个数 */
  uint8_t member_count = sizeof(step_motor)/sizeof(Stepper_TypeDef);
  
	/*使能定时器*/
	MOTOR_PUL_CLK_ENABLE();

	TIM_StepperHandle.Instance = MOTOR_PUL_TIM;    
	/* 累计 TIM_Period个后产生一个更新或者中断*/		
	//当定时器从0计数到TIM_PERIOD，即为TIM_PERIOD次，为一个定时周期
	TIM_StepperHandle.Init.Period = TIM_PERIOD;
	// 通用控制定时器时钟源TIMxCLK = HCLK/2=168MHz 
	// 设定定时器频率为=TIMxCLK/(TIM_Prescaler+1)=2MHz
	TIM_StepperHandle.Init.Prescaler = TIM_PRESCALER-1;                

	/*计数方式*/
	TIM_StepperHandle.Init.CounterMode = TIM_COUNTERMODE_UP;            
	/*采样时钟分频*/	
	TIM_StepperHandle.Init.ClockDivision=TIM_CLOCKDIVISION_DIV1;   
	TIM_StepperHandle.Init.RepetitionCounter = 0 ;  		
	/*初始化定时器*/
	HAL_TIM_OC_Init(&TIM_StepperHandle);

	/*PWM模式配置--这里配置为输出比较模式*/
	TIM_OCInitStructure.OCMode = TIM_OCMODE_TOGGLE; 
	/*比较输出的计数值*/
	TIM_OCInitStructure.Pulse = TIM_PERIOD;                    
	/*当定时器计数值小于CCR1_Val时为高电平*/
	TIM_OCInitStructure.OCPolarity = TIM_OCPOLARITY_HIGH;          
	/*设置互补通道输出的极性*/
	TIM_OCInitStructure.OCNPolarity = TIM_OCNPOLARITY_LOW; 
	/*快速模式设置*/
	TIM_OCInitStructure.OCFastMode = TIM_OCFAST_DISABLE;   
	/*空闲电平*/
	TIM_OCInitStructure.OCIdleState = TIM_OCIDLESTATE_RESET;  
	/*互补通道设置*/
	TIM_OCInitStructure.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  
  for(uint8_t i = 0; i < member_count; i++)
  {
    /* 启动比较输出并使能中断 */
    HAL_TIM_OC_ConfigChannel(&TIM_StepperHandle, &TIM_OCInitStructure, step_motor[i].pul_channel);
    HAL_TIM_OC_Start_IT(&TIM_StepperHandle, step_motor[i].pul_channel);
  }

  /* 启动定时器 */
	HAL_TIM_Base_Start(&TIM_StepperHandle);
}

/**
  * @brief  定时器中断服务函数
	*	@note   无
  * @retval 无
  */
void MOTOR_PUL_IRQHandler(void)
{
  HAL_TIM_IRQHandler(&TIM_StepperHandle);
}

/**
  * @brief  定时器比较中断回调函数
  * @param  htim：定时器句柄指针
	*	@note   无
  * @retval 无
  */
void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim)
{
  uint32_t channel = htim->Channel;
  
  /* 获取当前计数 */
  uint32_t count = __HAL_TIM_GET_COUNTER(htim);
  
  switch(channel)
  {
    case HAL_TIM_ACTIVE_CHANNEL_1:
      /* 设置比较数值 */
      __HAL_TIM_SET_COMPARE(&TIM_StepperHandle, MOTOR_PUL1_CHANNEL, count + step_motor[0].oc_pulse_num);
      break;
    case HAL_TIM_ACTIVE_CHANNEL_2:
      /* 设置比较数值 */
      __HAL_TIM_SET_COMPARE(&TIM_StepperHandle, MOTOR_PUL2_CHANNEL, count + step_motor[1].oc_pulse_num);
      break;
    case HAL_TIM_ACTIVE_CHANNEL_3:
      /* 设置比较数值 */
      __HAL_TIM_SET_COMPARE(&TIM_StepperHandle, MOTOR_PUL3_CHANNEL, count + step_motor[2].oc_pulse_num);
      break;
    case HAL_TIM_ACTIVE_CHANNEL_4:
      /* 设置比较数值 */
      __HAL_TIM_SET_COMPARE(&TIM_StepperHandle, MOTOR_PUL4_CHANNEL, count + step_motor[3].oc_pulse_num);
      break;
  }
}


/**
  * @brief  步进电机初始化
  * @param  *step_motor：步进电机结构体指针
  * @param  member_count：想要初始化的步进电机个数
	*	@note   无
  * @retval 无
  */
void stepper_Init(void)
{
	/*电机IO配置*/
	Stepper_GPIO_Config();
	/*定时器PWM输出配置*/
	TIM_PWMOUTPUT_Config();
	/*中断配置*/
	TIMx_NVIC_Configuration();
}

/**
  * @brief  启动电机
  * @param  channel：电机通道
	*	@note   无
  * @retval 无
  */
void stepper_Start(uint32_t channel)
{
  TIM_CCxChannelCmd(MOTOR_PUL_TIM, channel, TIM_CCx_ENABLE);
}

/**
  * @brief  停止电机
  * @param  channel：电机通道
	*	@note   无
  * @retval 无
  */
void stepper_Stop(uint32_t channel)
{
  TIM_CCxChannelCmd(MOTOR_PUL_TIM, channel, TIM_CCx_DISABLE);
}
