/**
  ******************************************************************************
  * @file    main.c
  * @author  fire
  * @version V1.0
  * @date    2017-xx-xx
  * @brief   GPIO输出--使用固件库点亮LED灯
  ******************************************************************************
  * @attention
  *
  * 实验平台:野火  STM32 F407 开发板 
  * 论坛    :http://www.firebbs.cn
  * 淘宝    :http://firestm32.taobao.com
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx.h"
#include "./tim/bsp_motor_tim.h"
#include "./led/bsp_led.h"
#include ".\key\bsp_key.h" 
#include ".\motor_control\bsp_motor_control.h"
#include "./usart/bsp_debug_usart.h"
#include "./Encoder/bsp_encoder.h"
#include "./tim/bsp_basic_tim.h"
#include "./pid/bsp_pid.h"
#include "./adc/bsp_adc.h"
#include "./protocol/protocol.h"

int pulse_num=0;
	
void Delay(__IO uint32_t nCount)	 //简单的延时函数
{
	for(; nCount != 0; nCount--);
}	
	
/**
  * @brief  主函数
  * @param  无
  * @retval 无
  */
int main(void)
{
  int32_t target_location = PER_CYCLE_PULSES;
  
  /* HAL 库初始化 */
  HAL_Init();
  
	/* 初始化系统时钟为168MHz */
	SystemClock_Config();
  
	/* 初始化按键 GPIO */
	Key_GPIO_Config();
  
  /* 初始化 LED */
  LED_GPIO_Config();
  
  /* 协议初始化 */
  protocol_init();
  
  /* 初始化串口 */
  DEBUG_USART_Config();

  /* 电机初始化 */
  motor_init();
  
	set_motor_disable();     // 停止电机 

  /* ADC 初始化 */
  ADC_Init();
  
  /* 编码器接口初始化 */
	Encoder_Init();
  
  /* 初始化基本定时器，用于处理定时任务 */
  TIMx_Configuration();
  
  /* PID 参数初始化 */
  PID_param_init();
  
  set_pid_target(&pid_location, target_location);    // 设置目标值
  
#if defined(PID_ASSISTANT_EN)
  set_computer_value(SEND_STOP_CMD, CURVES_CH1, NULL, 0);    // 同步上位机的启动按钮状态
  set_computer_value(SEND_TARGET_CMD, CURVES_CH1, &target_location, 1);     // 给通道 1 发送目标值
#endif

	while(1)
	{
    /* 接收数据处理 */
    receiving_process();
    
    /* 扫描KEY1 */
    if( Key_Scan(KEY1_GPIO_PORT, KEY1_PIN) == KEY_ON)
    {
    #if defined(PID_ASSISTANT_EN) 
      set_computer_value(SEND_START_CMD, CURVES_CH1, NULL, 0);               // 同步上位机的启动按钮状态
    #endif
      set_pid_target(&pid_location, target_location);    // 设置目标值
      set_motor_enable();              // 使能电机
    }
    
    /* 扫描KEY2 */
    if( Key_Scan(KEY2_GPIO_PORT, KEY2_PIN) == KEY_ON)
    {
      set_motor_disable();     // 停止电机
      set_computer_value(SEND_STOP_CMD, CURVES_CH1, NULL, 0);               // 同步上位机的启动按钮状态
    }
    
    /* 扫描KEY3 */
    if( Key_Scan(KEY3_GPIO_PORT, KEY3_PIN) == KEY_ON)
    {
      /* 增大目标位置 */
      target_location += PER_CYCLE_PULSES;
      
      set_pid_target(&pid_location, target_location);
    #if defined(PID_ASSISTANT_EN)
      set_computer_value(SEND_TARGET_CMD, CURVES_CH1,  &target_location, 1);     // 给通道 1 发送目标值
    #endif
    }

    /* 扫描KEY4 */
    if( Key_Scan(KEY4_GPIO_PORT, KEY4_PIN) == KEY_ON)
    {
      /* 减小目标位置 */
      target_location -= PER_CYCLE_PULSES;
      
      set_pid_target(&pid_location, target_location);
    #if defined(PID_ASSISTANT_EN)
      set_computer_value(SEND_TARGET_CMD, CURVES_CH1,  &target_location, 1);     // 给通道 1 发送目标值
    #endif
    }
	}
}

/**
  * @brief  System Clock Configuration
  *         The system Clock is configured as follow : 
  *            System Clock source            = PLL (HSE)
  *            SYSCLK(Hz)                     = 168000000
  *            HCLK(Hz)                       = 168000000
  *            AHB Prescaler                  = 1
  *            APB1 Prescaler                 = 4
  *            APB2 Prescaler                 = 2
  *            HSE Frequency(Hz)              = 8000000
  *            PLL_M                          = 25
  *            PLL_N                          = 336
  *            PLL_P                          = 2
  *            PLL_Q                          = 7
  *            VDD(V)                         = 3.3
  *            Main regulator output voltage  = Scale1 mode
  *            Flash Latency(WS)              = 5
  * @param  None
  * @retval None
  */
 void SystemClock_Config(void)
{
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_OscInitTypeDef RCC_OscInitStruct;

  /* Enable Power Control clock */
  __HAL_RCC_PWR_CLK_ENABLE();
  
  /* The voltage scaling allows optimizing the power consumption when the device is 
     clocked below the maximum system frequency, to update the voltage scaling value 
     regarding system frequency refer to product datasheet.  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  
  /* Enable HSE Oscillator and activate PLL with HSE as source */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if(HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    while(1) {};
  }
  
  /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2 
     clocks dividers */
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;  
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;  
  if(HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    while(1) {};
  }

  /* STM32F405x/407x/415x/417x Revision Z devices: prefetch is supported  */
  if (HAL_GetREVID() == 0x1001)
  {
    /* Enable the Flash prefetch */
    __HAL_FLASH_PREFETCH_BUFFER_ENABLE();
  }
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
