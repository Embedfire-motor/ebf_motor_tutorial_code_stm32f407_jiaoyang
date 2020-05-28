/**
  ******************************************************************************
  * @file    bsp_advance_tim.c
  * @author  STMicroelectronics
  * @version V1.0
  * @date    2015-xx-xx
  * @brief   高级控制定时器互补输出范例
  ******************************************************************************
  * @attention
  *
  * 实验平台:野火  STM32 F407开发板  
  * 论坛    :http://www.firebbs.cn
  * 淘宝    :http://firestm32.taobao.com
  *
  ******************************************************************************
  */
  
#include "./tim/bsp_advance_tim.h"

TIM_HandleTypeDef  TIM_PWMOUTPUT_Handle;
TIM_HandleTypeDef  TIM_PWMINPUT_Handle;
__IO uint16_t IC2Value = 0;
__IO uint16_t IC1Value = 0;
__IO float DutyCycle = 0;
__IO float Frequency = 0;

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
  GENERAL_OCPWM_GPIO_CLK_ENABLE();
  ADVANCE_ICPWM_GPIO_CLK_ENABLE(); 

  /* 定时器功能引脚初始化 */
  /* 通用定时器PWM输出引脚 */  
  GPIO_InitStructure.Pin = GENERAL_OCPWM_PIN; 
  GPIO_InitStructure.Mode = GPIO_MODE_AF_PP;    
  GPIO_InitStructure.Pull = GPIO_NOPULL;
  GPIO_InitStructure.Speed = GPIO_SPEED_HIGH;   
  GPIO_InitStructure.Alternate = GENERAL_OCPWM_AF;
  HAL_GPIO_Init(GENERAL_OCPWM_GPIO_PORT, &GPIO_InitStructure);
  
  /* 高级定时器输入捕获引脚 */
  GPIO_InitStructure.Pin = ADVANCE_ICPWM_PIN;
  GPIO_InitStructure.Alternate = ADVANCE_ICPWM_AF;  
  HAL_GPIO_Init(ADVANCE_ICPWM_GPIO_PORT, &GPIO_InitStructure);
}

 /**
  * @brief  高级控制定时器 TIMx,x[1,8]中断优先级配置
  * @param  无
  * @retval 无
  */
static void TIMx_NVIC_Configuration(void)
{
  //设置抢占优先级，子优先级
  HAL_NVIC_SetPriority(ADVANCE_TIM_IRQn, 0, 3);
  // 设置中断来源
  HAL_NVIC_EnableIRQ(ADVANCE_TIM_IRQn);
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
static void TIM_PWMOUTPUT_Config(void)
{ 
  TIM_OC_InitTypeDef TIM_OCInitStructure;
  // 开启TIMx_CLK,x[2,3,4,5,12,13,14] 
  GENERAL_TIM_CLK_ENABLE(); 
  /* 定义定时器的句柄即确定定时器寄存器的基地址*/
  TIM_PWMOUTPUT_Handle.Instance = GENERAL_TIM;
  /* 累计 TIM_Period个后产生一个更新或者中断*/    
  //当定时器从0计数到9999，即为10000次，为一个定时周期
  TIM_PWMOUTPUT_Handle.Init.Period = 500-1;
  // 高级控制定时器时钟源TIMxCLK = HCLK=84MHz 
  // 设定定时器频率为=TIMxCLK/(TIM_Prescaler+1)=100KHz
  TIM_PWMOUTPUT_Handle.Init.Prescaler = 84-1; 
  // 采样时钟分频
  TIM_PWMOUTPUT_Handle.Init.ClockDivision=TIM_CLOCKDIVISION_DIV1;
  // 计数方式
  TIM_PWMOUTPUT_Handle.Init.CounterMode=TIM_COUNTERMODE_UP;
  // 重复计数器
  TIM_PWMOUTPUT_Handle.Init.RepetitionCounter=0;  
  // 初始化定时器TIMx, x[1,8]
  HAL_TIM_PWM_Init(&TIM_PWMOUTPUT_Handle);

  /*PWM模式配置*/
  //配置为PWM模式1
  TIM_OCInitStructure.OCMode = TIM_OCMODE_PWM1;
  TIM_OCInitStructure.Pulse = 250;
  TIM_OCInitStructure.OCPolarity = TIM_OCPOLARITY_HIGH;
  TIM_OCInitStructure.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  TIM_OCInitStructure.OCIdleState = TIM_OCIDLESTATE_SET;
  TIM_OCInitStructure.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  //初始化通道3输出PWM 
  HAL_TIM_PWM_ConfigChannel(&TIM_PWMOUTPUT_Handle,&TIM_OCInitStructure,TIM_CHANNEL_3);

  /* 定时器通道3输出PWM */
  HAL_TIM_PWM_Start(&TIM_PWMOUTPUT_Handle,TIM_CHANNEL_3);

}
    
static void TIM_PWMINPUT_Config(void)
{ 
  TIM_IC_InitTypeDef    TIM_ICInitStructure;
  TIM_SlaveConfigTypeDef  TIM_SlaveConfigStructure;
  TIM_MasterConfigTypeDef TIM_MasterConfigStructure;
  
  // 开启TIMx_CLK,x[1,8] 
  ADVANCE_TIM_CLK_ENABLE(); 
  /* 定义定时器的句柄即确定定时器寄存器的基地址*/
  TIM_PWMINPUT_Handle.Instance = ADVANCE_TIM;
  TIM_PWMINPUT_Handle.Init.Period = 0xFFFF;   
  // 高级控制定时器时钟源TIMxCLK = HCLK=168MHz 
  // 设定定时器频率为=TIMxCLK/(TIM_Prescaler+1)=1MHz
  TIM_PWMINPUT_Handle.Init.Prescaler = 168-1; 
  // 采样时钟分频
  TIM_PWMINPUT_Handle.Init.ClockDivision=TIM_CLOCKDIVISION_DIV1;
  // 计数方式
  TIM_PWMINPUT_Handle.Init.CounterMode=TIM_COUNTERMODE_UP;  
  // 初始化定时器TIMx, x[1,8]
  HAL_TIM_IC_Init(&TIM_PWMINPUT_Handle);
  
  /* IC1捕获：上升沿触发 TI1FP1 */
  TIM_ICInitStructure.ICPolarity = TIM_ICPOLARITY_RISING;
  TIM_ICInitStructure.ICSelection = TIM_ICSELECTION_DIRECTTI;
  TIM_ICInitStructure.ICPrescaler = TIM_ICPSC_DIV1;
  TIM_ICInitStructure.ICFilter = 0x0;
  HAL_TIM_IC_ConfigChannel(&TIM_PWMINPUT_Handle,&TIM_ICInitStructure,ADVANCE_IC1PWM_CHANNEL);

  /* IC2捕获：下降沿触发 TI1FP2 */  
  TIM_ICInitStructure.ICPolarity = TIM_ICPOLARITY_FALLING;
  TIM_ICInitStructure.ICSelection = TIM_ICSELECTION_INDIRECTTI;
  TIM_ICInitStructure.ICPrescaler = TIM_ICPSC_DIV1;
  TIM_ICInitStructure.ICFilter = 0x0;
  HAL_TIM_IC_ConfigChannel(&TIM_PWMINPUT_Handle,&TIM_ICInitStructure,ADVANCE_IC2PWM_CHANNEL);
  
  /* 选择从模式: 复位模式 */
  TIM_SlaveConfigStructure.SlaveMode = TIM_SLAVEMODE_RESET;
  /* 选择定时器输入触发: TI1FP1 */
  TIM_SlaveConfigStructure.InputTrigger = TIM_TS_TI1FP1;
  HAL_TIM_SlaveConfigSynchronization(&TIM_PWMINPUT_Handle,&TIM_SlaveConfigStructure);
  
  /* 使能捕获/比较2中断请求 */
  HAL_TIM_IC_Start_IT(&TIM_PWMINPUT_Handle,TIM_CHANNEL_1);
  HAL_TIM_IC_Start_IT(&TIM_PWMINPUT_Handle,TIM_CHANNEL_2);
}
/**
  * @brief  初始化高级控制定时器定时，1ms产生一次中断
  * @param  无
  * @retval 无
  */
void TIMx_Configuration(void)
{
  TIMx_GPIO_Config();
  
  TIMx_NVIC_Configuration();  
  
  TIM_PWMOUTPUT_Config();
  
  TIM_PWMINPUT_Config();
}

/**
  * @brief  Conversion complete callback in non blocking mode 
  * @param  htim : hadc handle
  * @retval None
  */
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
  {
    /* 获取输入捕获值 */
    IC1Value = HAL_TIM_ReadCapturedValue(&TIM_PWMINPUT_Handle,ADVANCE_IC1PWM_CHANNEL);
    IC2Value = HAL_TIM_ReadCapturedValue(&TIM_PWMINPUT_Handle,ADVANCE_IC2PWM_CHANNEL);  
    if (IC1Value != 0)
    {
      /* 占空比计算 */
      DutyCycle = (float)((IC2Value+1) * 100) / (IC1Value+1);
      /* 频率计算 */
      Frequency = 168000000/168/(float)(IC1Value+1);
    }
    else
    {
      DutyCycle = 0;
      Frequency = 0;
    }

  }
}
/*********************************************END OF FILE**********************/
