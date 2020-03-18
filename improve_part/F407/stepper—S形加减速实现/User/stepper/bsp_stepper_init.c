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
#include "./stepper/bsp_creat_S_tab.h"
#include "./delay/core_delay.h"   
#include "stm32f4xx.h"
#include "math.h"
#include <math.h>

TIM_HandleTypeDef TIM_TimeBaseStructure;

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
	GPIO_InitStruct.Pin = MOTOR_PUL_PIN;
	/*Motor 脉冲引脚 初始化*/
	HAL_GPIO_Init(MOTOR_PUL_PORT, &GPIO_InitStruct);			
}

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
void TIM_PWMOUTPUT_Config(void)
{
	TIM_OC_InitTypeDef  TIM_OCInitStructure;  	
	/*使能定时器*/
	MOTOR_PUL_CLK_ENABLE();

	TIM_TimeBaseStructure.Instance = MOTOR_PUL_TIM;    
	/* 累计 TIM_Period个后产生一个更新或者中断*/		
	//当定时器从0计数到10000，即为10000次，为一个定时周期
	TIM_TimeBaseStructure.Init.Period = TIM_PERIOD; 
	// 通用控制定时器时钟源TIMxCLK = HCLK/2	=	84MHz 
	// 高级控制定时器时钟源TIMxCLK = HCLK		=	168MHz 
	// 设定定时器频率为=TIMxCLK/(TIM_Prescaler+1)
	TIM_TimeBaseStructure.Init.Prescaler = TIM_PRESCALER;//TIM_PRESCALER;                

	/*计数方式*/
	TIM_TimeBaseStructure.Init.CounterMode = TIM_COUNTERMODE_UP;            
	/*采样时钟分频*/	
	TIM_TimeBaseStructure.Init.ClockDivision=TIM_CLOCKDIVISION_DIV1;   
	TIM_TimeBaseStructure.Init.RepetitionCounter = 0 ;  		
	/*初始化定时器*/
	HAL_TIM_OC_Init(&TIM_TimeBaseStructure);

	/*PWM模式配置--这里配置为输出比较模式*/
	TIM_OCInitStructure.OCMode = TIM_OCMODE_TOGGLE; 
	/*比较输出的计数值*/
	TIM_OCInitStructure.Pulse = 0;                    
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
	HAL_TIM_OC_ConfigChannel(&TIM_TimeBaseStructure, &TIM_OCInitStructure, MOTOR_PUL_CHANNEL_x);

	/* 确定定时器 */
	HAL_TIM_Base_Start(&TIM_TimeBaseStructure);
	/* 启动比较输出并使能中断 */
	HAL_TIM_OC_Start_IT(&TIM_TimeBaseStructure,MOTOR_PUL_CHANNEL_x);
	/*使能比较通道*/
	TIM_CCxChannelCmd(MOTOR_PUL_TIM,MOTOR_PUL_CHANNEL_x,TIM_CCx_ENABLE);

}

/**
  * @brief  通道开关
	* @param  temp 		1:开    0：关
  * @retval 无
  */
void channel_sw(int temp)
{
	if(temp)
		HAL_TIM_OC_Start_IT(&TIM_TimeBaseStructure,MOTOR_PUL_CHANNEL_x);
	else
		HAL_TIM_OC_Stop_IT(&TIM_TimeBaseStructure,MOTOR_PUL_CHANNEL_x);
}


__IO int32_t  Step_Position   = 0;           // 当前位置
__IO uint16_t    Toggle_Pulse = 0;        // 脉冲频率控制
__IO uint8_t  MotionStatus    = 0;  

/**
  * @brief  速度决策
	*	@note 	在中断中使用，每进一次中断，决策一次
  * @retval 无
  */
void speed_decision()
{
	  __IO uint32_t Tim_Count = 0;
  __IO uint32_t tmp = 0;
  __IO float Tim_Pulse = 0;
  __IO static uint8_t i = 0;  
	
  
	if(__HAL_TIM_GET_IT_SOURCE(&TIM_TimeBaseStructure, MOTOR_TIM_IT_CCx) !=RESET)
	{
		// 清楚定时器中断
		__HAL_TIM_CLEAR_IT(&TIM_TimeBaseStructure, MOTOR_TIM_IT_CCx);
		/******************************************************************/
/*		
		    i++;     // 定时器中断次数计数值
    if(i == 2) // 2次，说明已经输出一个完整脉冲
    {
      i = 0;   // 清零定时器中断次数计数值
      if(MotionStatus == ACCEL || MotionStatus == DECEL)
      {
        Step_Position ++;
        if(Step_Position  < Speed.AccelStep )
        { 
          Tim_Pulse = T1_FREQ / Speed.VelocityTab[Step_Position];// 由速度表得到每一步的定时器计数值
          if((Tim_Pulse / 2) >= 0xFFFF)
            Tim_Pulse = 0xFFFF;
          Toggle_Pulse = (uint16_t) (Tim_Pulse / 2);
        }
        else
        {
          if(MotionStatus == ACCEL)   
            MotionStatus = AVESPEED;
          else
          {
            MotionStatus = STOP; 
//            free(Speed.VelocityTab);          //  运动完要释放内存
            TIM_CCxChannelCmd(MOTOR_PUL_TIM, MOTOR_PUL_CHANNEL_x, TIM_CCx_DISABLE);// 使能定时器通道 
            
          }
        }
      }
    }
	*/	
		/**********************************************************************/
		// 设置比较值
		uint32_t tim_count=__HAL_TIM_GET_COUNTER(&TIM_TimeBaseStructure);
		uint32_t tmp = tim_count+30;
		__HAL_TIM_SET_COMPARE(&TIM_TimeBaseStructure,MOTOR_PUL_CHANNEL_x,tmp);
	}
}

/**
  * 函数功能: 步进电机做S型加减速运动
  * 输入参数: 无
  * 返 回 值: 无
  * 说    明: 无
  */
void STEPMOTOR_MoveRel()
{
  Step_Position = 0;
  MotionStatus = ACCEL; // 电机为运动状态
  // 第一步速度是0,则定时器从0xFFFF开始;
  if(Speed.VelocityTab[0] == 0)
    Toggle_Pulse = 0xFFFF;
  else
    Toggle_Pulse  = (uint32_t)(T1_FREQ/Speed.VelocityTab[0]);
  __HAL_TIM_SET_COUNTER(&TIM_TimeBaseStructure,0);
  __HAL_TIM_SET_COMPARE(&TIM_TimeBaseStructure,MOTOR_PUL_CHANNEL_x,(uint16_t)(Toggle_Pulse)); // 设置定时器比较值
  TIM_CCxChannelCmd(MOTOR_PUL_TIM, MOTOR_PUL_CHANNEL_x, TIM_CCx_ENABLE);// 使能定时器通道 
  //STEPMOTOR_OUTPUT_ENABLE();
}




void stepper_move_T( int32_t step, uint32_t accel, uint32_t decel, uint32_t speed)
{  

	/*获取当前计数值*/
	int tim_count=__HAL_TIM_GET_COUNTER(&TIM_TimeBaseStructure);
	/*在当前计数值基础上设置定时器比较值*/
	__HAL_TIM_SET_COMPARE(&TIM_TimeBaseStructure,MOTOR_PUL_CHANNEL_x,tim_count+500); 
	/*使能定时器通道*/
	TIM_CCxChannelCmd(MOTOR_PUL_TIM, MOTOR_PUL_CHANNEL_x, TIM_CCx_ENABLE);

}

/**
  * @brief  引脚初始化
  * @retval 无
  */
void stepper_Init()
{
	/*电机IO配置*/
	Stepper_GPIO_Config();
	/*定时器PWM输出配置*/
	TIM_PWMOUTPUT_Config();
	/*中断配置*/
	TIMx_NVIC_Configuration();
}

/**
  * @brief  定时器中断服务函数
  * @retval 无
  */
void MOTOR_PUL_IRQHandler(void)
{ 
	/*速度状态决策*/
	speed_decision();
}

