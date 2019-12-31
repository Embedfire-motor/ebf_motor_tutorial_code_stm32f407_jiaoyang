#include "./stepper/bsp_stepper_init.h"
#include "./led/bsp_led.h"   
#include "./delay/core_delay.h"   
#include "stm32f4xx.h"


TIM_HandleTypeDef TIM_TimeBaseStructure;
 /**
  * @brief  通用定时器 TIMx,x[6,7]中断优先级配置
  * @param  无
  * @retval 无
  */
static void TIMx_NVIC_Configuration(void)
{
    /* 外设中断配置 */
    HAL_NVIC_SetPriority(GENERAL_TIM_IRQ, 0, 0);
    HAL_NVIC_EnableIRQ(GENERAL_TIM_IRQ);
}

/*
 * 注意：TIM_TimeBaseInitTypeDef结构体里面有5个成员，TIM6和TIM7的寄存器里面只有
 * TIM_Prescaler和TIM_Period，所以使用TIM6和TIM7的时候只需初始化这两个成员即可，
 * 另外三个成员是通用定时器和高级定时器才有.
 *-----------------------------------------------------------------------------
 * TIM_Prescaler         都有
 * TIM_CounterMode			 TIMx,x[6,7]没有，其他都有（通用定时器）
 * TIM_Period            都有
 * TIM_ClockDivision     TIMx,x[6,7]没有，其他都有(通用定时器)
 * TIM_RepetitionCounter TIMx,x[1,8]才有(高级定时器)
 *-----------------------------------------------------------------------------
 */
static void TIM_Mode_Config(void)
{

	GENERAL_TIM_CLK_ENABLE();
  TIM_OC_InitTypeDef  TIM_OCInitStructure;  
	
	int tim_per=512;
	
	/*定时器配置*/
  TIM_TimeBaseStructure.Instance = GENERAL_TIM;
  TIM_TimeBaseStructure.Init.Period = tim_per-1;									//周期
  TIM_TimeBaseStructure.Init.Prescaler = 21-1;								//分频系数
	TIM_TimeBaseStructure.Init.CounterMode=TIM_COUNTERMODE_UP;	//向上计数
	TIM_TimeBaseStructure.Init.ClockDivision=TIM_CLOCKDIVISION_DIV1;
	HAL_TIM_Base_Init(&TIM_TimeBaseStructure);
	
	/*pwm配置*/

	TIM_OCInitStructure.OCMode = TIM_OCMODE_PWM1; 
	TIM_OCInitStructure.OCPolarity = TIM_OCPOLARITY_LOW;          
	TIM_OCInitStructure.Pulse = 0;  
	TIM_OCInitStructure.OCFastMode = TIM_OCFAST_DISABLE;   
	TIM_OCInitStructure.OCIdleState = TIM_OCIDLESTATE_RESET;  


	HAL_TIM_OC_ConfigChannel(&TIM_TimeBaseStructure, &TIM_OCInitStructure, MOTOR_PUL_CHANNEL_x);

	/* 确定定时器 */
	HAL_TIM_Base_Start(&TIM_TimeBaseStructure);
	/* 启动比较输出并使能中断 */
	HAL_TIM_OC_Start_IT(&TIM_TimeBaseStructure,MOTOR_PUL_CHANNEL_x);
	/*使能比较通道*/
	TIM_CCxChannelCmd(GENERAL_TIM,MOTOR_PUL_CHANNEL_x,TIM_CCx_ENABLE);

	// 开启定时器更新中断
	HAL_TIM_Base_Start_IT(&TIM_TimeBaseStructure);	
}

/**
  * @brief  初始化通用定时器定时
  * @param  无
  * @retval 无
  */
void TIMx_Configuration(void)
{
	TIMx_NVIC_Configuration();	
  
	TIM_Mode_Config();
}

/**
  * @brief  配置TIM复用输出PWM时用到的I/O
  * @param  无
  * @retval 无
  */
static void Stepper_GPIO_Config(void) 
{
	GPIO_InitTypeDef GPIO_InitStruct;
	/*开启Motor相关的GPIO外设时钟*/
	MOTOR_PUL_GPIO_CLK_ENABLE();

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
  HAL_GPIO_Init(MOTOR_PUL_GPIO_PORT, &GPIO_InitStruct);			
	
}


/**
  * @brief  设置TIM通道的占空比
	* @param  channel		通道	（1,2,3,4）
	* @param  compare		占空比
	*	@note 	无
  * @retval 无
  */
void TIM2_SetPWM_pulse(int channel,int compare)
{
		switch(channel)
	{
		case 1:  	__HAL_TIM_SET_COMPARE(&TIM_TimeBaseStructure,TIM_CHANNEL_1,compare);break;
		case 2:	  __HAL_TIM_SET_COMPARE(&TIM_TimeBaseStructure,TIM_CHANNEL_2,compare);break;
		case 3:	  __HAL_TIM_SET_COMPARE(&TIM_TimeBaseStructure,TIM_CHANNEL_3,compare);break;
		case 4:	  __HAL_TIM_SET_COMPARE(&TIM_TimeBaseStructure,TIM_CHANNEL_4,compare);break;
	}
}

/**
  * @brief  引脚初始化
  * @retval 无
  */
void stepper_Init()
{
		Stepper_GPIO_Config();

		TIMx_Configuration();
}


/**
  * @brief  定时器中断函数
	*	@note 		无
  * @retval 无
  */
void  GENERAL_TIM_INT_IRQHandler (void)
{
	HAL_TIM_IRQHandler(&TIM_TimeBaseStructure);	 	
}


/* SPWM表,正弦曲线，此表使用工程目录下的python脚本sin_wave.py生成*/
const uint16_t indexWave[] = {
0, 9, 18, 27, 36, 45, 54, 63, 72, 81, 89, 98,
	107, 116, 125, 133, 142, 151, 159, 168, 176,
	184, 193, 201, 209, 218, 226, 234, 242, 249,
	257, 265, 273, 280, 288, 295, 302, 310, 317, 
	324, 331, 337, 344, 351, 357, 364, 370, 376, 
	382, 388, 394, 399, 405, 410, 416, 421, 426, 
	431, 436, 440, 445, 449, 454, 458, 462, 465, 
	469, 473, 476, 479, 482, 485, 488, 491, 493, 
	496, 498, 500, 502, 503, 505, 506, 508, 509, 
	510, 510, 511, 512, 512, 512, 512, 512, 512,
	511, 510, 510, 509, 508, 506, 505, 503, 502,
	500, 498, 496, 493, 491, 488, 485, 482, 479,
	476, 473, 469, 465, 462, 458, 454, 449, 445, 
	440, 436, 431, 426, 421, 416, 410, 405, 399, 
	394, 388, 382, 376, 370, 364, 357, 351, 344, 
	337, 331, 324, 	317, 310, 302, 295, 288, 280, 
	273, 265, 257, 249, 242, 234, 226, 218, 209, 
	201, 193, 184, 176, 168, 159, 151, 142, 133, 
125, 116, 107, 98, 89, 81, 72, 63, 54, 45, 36,
27, 18, 9, 0
};

//计算PWM表有多少个元素
uint16_t POINT_NUM = sizeof(indexWave)/sizeof(indexWave[0]); 
/*电压幅值等级数*/
#define AMPLITUDE_CLASS 256
//控制输出波形的频率
__IO uint16_t period_class = 1;


//该变量在定时器中断服务函数中使用，用于控制各通道的输出
//修改该变量的值可直接改变呼吸灯的颜色
//变量格式：RGB888
__IO uint32_t rgb_color = 0xFF00FF;

/**
  * @brief  回调函数
	*	@note 		无
  * @retval 无
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	static uint16_t pwm_index = 0;			//用于PWM查表
	static uint16_t period_cnt = 0;		//用于计算周期数
	static uint16_t amplitude_cnt = 0;	//用于计算幅值等级
	if(htim==(&TIM_TimeBaseStructure))
	{
		amplitude_cnt++;
				
			//每个PWM表中的每个元素有AMPLITUDE_CLASS个等级，
		  //每增加一级多输出一次脉冲，即PWM表中的元素多使用一次
		  //使用256次，根据RGB颜色分量设置通道输出
			if(amplitude_cnt > (AMPLITUDE_CLASS-1))		 						
			{		
				period_cnt++;
				
				//每个PWM表中的每个元素使用period_class次
				if(period_cnt > period_class)
				{				

					pwm_index++;												//标志PWM表指向下一个元素
				
					//若PWM表已到达结尾，重新指向表头
					if( pwm_index >=  POINT_NUM)			
					{
						pwm_index=0;								
					}
					
					period_cnt = 0;											//重置周期计数标志
				}
				
				amplitude_cnt=0;											//重置幅值+计数标志
			}
			else
			{	
					//每个PWM表中的每个元素有AMPLITUDE_CLASS个等级，
					//每增加一级多输出一次脉冲，即PWM表中的元素多使用一次

					//根据RGB颜色分量值，设置各个通道是否输出当前的PWM表元素表示的亮度
			
					//蓝
				TIM2_SetPWM_pulse(1,indexWave[pwm_index]);
//					if((rgb_color&0x0000FF) >= amplitude_cnt)				
//						TIM2_SetPWM_pulse(1,indexWave[pwm_index]);	//根据PWM表修改定时器的比较寄存器值
//					else
//						//BRE_BLUE_TIM->BRE_BLUE_CCRx = 0;		//比较寄存器值为0，通道输出高电平，该通道LED灯灭	
//						TIM2_SetPWM_pulse(1,0);

			}						
	}
}

