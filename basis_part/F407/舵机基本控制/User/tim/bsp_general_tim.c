/**
  ******************************************************************************
  * @file    bsp_basic_tim.c
  * @author  STMicroelectronics
  * @version V1.0
  * @date    2015-xx-xx
  * @brief   通用定时器定时范例
  ******************************************************************************
  * @attention
  *
  * 实验平台:野火  STM32 F407 开发板  
  * 论坛    :http://www.firebbs.cn
  * 淘宝    :http://firestm32.taobao.com
  *
  ******************************************************************************
  */
  
#include "./tim/bsp_general_tim.h"
#include "./usart/bsp_debug_usart.h"
#include <math.h>
#include <stdlib.h>

void TIM_SetTIMxCompare(TIM_TypeDef *TIMx,uint32_t channel,uint32_t compare);
void TIM_SetPWM_period(TIM_TypeDef* TIMx,uint32_t TIM_period);

/**
  * @brief  配置TIM复用输出PWM时用到的I/O
  * @param  无
  * @retval 无
  */
static void TIMx_GPIO_Config(void) 
{
  GPIO_InitTypeDef GPIO_InitStruct;
  
  /* 定时器通道功能引脚端口时钟使能 */
	
	__HAL_RCC_GPIOD_CLK_ENABLE();
  
  /* 定时器通道1功能引脚IO初始化 */
	/*设置输出类型*/
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	/*设置引脚速率 */ 
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	/*设置复用*/
  GPIO_InitStruct.Alternate = GENERAL_TIM_GPIO_AF;
	
	/*选择要控制的GPIO引脚*/	
	GPIO_InitStruct.Pin = GENERAL_TIM_CH1_PIN;
	/*调用库函数，使用上面配置的GPIO_InitStructure初始化GPIO*/
  HAL_GPIO_Init(GENERAL_TIM_CH1_GPIO_PORT, &GPIO_InitStruct);
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
TIM_HandleTypeDef  TIM_TimeBaseStructure;
static void TIM_PWMOUTPUT_Config(void)
{
  TIM_OC_InitTypeDef  TIM_OCInitStructure;  
	
  /*使能定时器*/
  GENERAL_TIM_CLK_ENABLE();
	
  TIM_TimeBaseStructure.Instance = GENERAL_TIM;
  /* 累计 TIM_Period个后产生一个更新或者中断*/		
  //当定时器从0计数到PWM_PERIOD_COUNT，即为PWM_PERIOD_COUNT+1次，为一个定时周期
	TIM_TimeBaseStructure.Init.Period = PWM_PERIOD_COUNT;
	// 通用控制定时器时钟源TIMxCLK = HCLK/2=84MHz 
	// 设定定时器频率为=TIMxCLK/(PWM_PRESCALER_COUNT+1)
  TIM_TimeBaseStructure.Init.Prescaler = PWM_PRESCALER_COUNT;	
	
	/*计数方式*/
  TIM_TimeBaseStructure.Init.CounterMode = TIM_COUNTERMODE_UP;
	/*采样时钟分频*/
  TIM_TimeBaseStructure.Init.ClockDivision=TIM_CLOCKDIVISION_DIV1;
	/*初始化定时器*/
  HAL_TIM_Base_Init(&TIM_TimeBaseStructure);
  
	/*PWM模式配置*/
  TIM_OCInitStructure.OCMode = TIM_OCMODE_PWM2;      // 配置为PWM模式1
  TIM_OCInitStructure.Pulse = 0.5/20.0*PWM_PERIOD_COUNT;    // 默认占空比
  TIM_OCInitStructure.OCFastMode = TIM_OCFAST_DISABLE;
	/*当定时器计数值小于CCR1_Val时为高电平*/
  TIM_OCInitStructure.OCPolarity = TIM_OCPOLARITY_HIGH;	
	
	/*配置PWM通道*/
  HAL_TIM_PWM_ConfigChannel(&TIM_TimeBaseStructure, &TIM_OCInitStructure, PWM_CHANNEL_1);
	/*开始输出PWM*/
	HAL_TIM_PWM_Start(&TIM_TimeBaseStructure,PWM_CHANNEL_1);
}

/**
  * @brief  设置TIM通道的占空比
	* @param  channel		通道	（1,2,3,4）
	* @param  compare		占空比
	*	@note 	无
  * @retval 无
  */
void TIM2_SetPWM_pulse(uint32_t channel,int compare)
{
		switch(channel)
	{
		case TIM_CHANNEL_1:  	__HAL_TIM_SET_COMPARE(&TIM_TimeBaseStructure,TIM_CHANNEL_1,compare);break;
		case TIM_CHANNEL_2:	  __HAL_TIM_SET_COMPARE(&TIM_TimeBaseStructure,TIM_CHANNEL_2,compare);break;
		case TIM_CHANNEL_3:	  __HAL_TIM_SET_COMPARE(&TIM_TimeBaseStructure,TIM_CHANNEL_3,compare);break;
		case TIM_CHANNEL_4:	  __HAL_TIM_SET_COMPARE(&TIM_TimeBaseStructure,TIM_CHANNEL_4,compare);break;
	}
}


/**
  * @brief  初始化控制通用定时器
  * @param  无
  * @retval 无
  */
void TIMx_Configuration(void)
{
	TIMx_GPIO_Config();
  
  TIM_PWMOUTPUT_Config();
}

/**
  * @brief  设置舵机占空比
  * @param  angle: 占空比，（0.5/20.0*PWM_PERIOD_COUNT 到 2.5/20.0*PWM_PERIOD_COUNT）
  * @retval 无
  */
void set_steering_gear_dutyfactor(uint16_t dutyfactor)
{
  #if 1
  {
    /* 对超过范围的占空比进行边界处理 */
    dutyfactor = 0.5/20.0*PWM_PERIOD_COUNT > dutyfactor ? 0.5/20.0*PWM_PERIOD_COUNT : dutyfactor;
    dutyfactor = 2.5/20.0*PWM_PERIOD_COUNT < dutyfactor ? 2.5/20.0*PWM_PERIOD_COUNT : dutyfactor;
  }
  #endif
  
	TIM2_SetPWM_pulse(PWM_CHANNEL_1, dutyfactor);
}

/**
  * @brief  设置舵机角度
  * @param  angle: 角度，（0 到 180（舵机为0°-180°））
  * @retval 无
  */
void set_steering_gear_angle(uint16_t angle_temp)
{
  angle_temp = (0.5 + angle_temp / 180.0 * (2.5 - 0.5)) / 20.0 * PWM_PERIOD_COUNT;    // 计算角度对应的占空比
  
  set_steering_gear_dutyfactor(angle_temp);    // 设置占空比
}

/**
  * @brief  打印帮助命令
  * @param  无
  * @retval 无
  */
void show_help(void)
{
    printf("——————————————野火舵机驱动演示程序——————————————\n\r");
    printf("输入命令(以回车结束)：\n\r");
    printf("< ? >     -帮助菜单\n\r");
    printf("a[data]   -设置舵机的角度（范围：%d—%d）\n\r", 0, 180);
}

extern uint16_t ChannelPulse;

/**
  * @brief  处理串口接收到的数据
  * @param  无
  * @retval 无
  */
void deal_serial_data(void)
{
    int angle_temp=0;
    
    //接收到正确的指令才为1
    char okCmd = 0;

    //检查是否接收到指令
    if(receive_cmd == 1)
    {
      if(UART_RxBuffer[0] == 'a' || UART_RxBuffer[0] == 'A')
      {
        //设置速度
        if(UART_RxBuffer[1] == ' ')
        {
          angle_temp = atoi((char const *)UART_RxBuffer+2);
          if(angle_temp>=0 && angle_temp <= 180)
          {
            printf("\n\r角度: %d\n\r", angle_temp);
            ChannelPulse = (0.5 + angle_temp / 180.0 * (2.5 - 0.5)) / 20.0 * PWM_PERIOD_COUNT;    // 更新按钮控制的占空比
            set_steering_gear_angle(angle_temp);
//            printf("\n\r角度: %d\n\r", (uint16_t)(angle_temp/PWM_PERIOD_COUNT*20.0/(2.5-0.5)*180.0));
            okCmd = 1;
          }
        }
      }
      else if(UART_RxBuffer[0] == '?')
      {
        //打印帮助命令
        show_help();
        okCmd = 1;
      }
      //如果指令有无则打印帮助命令
      if(okCmd != 1)
      {
        printf("\n\r 输入有误，请重新输入...\n\r");
        show_help();
      }

      //清空串口接收缓冲数组
      receive_cmd = 0;
      uart_FlushRxBuffer();

    }
}


/*********************************************END OF FILE**********************/
