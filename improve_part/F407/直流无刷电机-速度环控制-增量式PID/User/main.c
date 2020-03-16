/**
  ******************************************************************************
  * @file    main.c
  * @author  fire
  * @version V1.0
  * @date    2020-xx-xx
  * @brief   直流无刷电机速度环-增量式PID控制
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
#include "./led/bsp_led.h"
#include ".\key\bsp_key.h" 
#include ".\bldcm_control\bsp_bldcm_control.h"
#include "./usart/bsp_debug_usart.h"
#include "./tim/bsp_basic_tim.h"
#include "./pid/bsp_pid.h"

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
  __IO uint16_t target_speed = 500;
  uint8_t i = 0;
  
	/* 初始化系统时钟为168MHz */
	SystemClock_Config();
  
	/* 初始化按键GPIO */
	Key_GPIO_Config();
  
  /* LED 灯初始化 */
  LED_GPIO_Config();
  
  /* 调试串口初始化 */
  DEBUG_USART_Config();
  
  PID_param_init();
  
  /* 周期控制定时器 50ms */
  TIMx_Configuration();

  /* 电机初始化 */
  bldcm_init();
  
  /* 使能电机 */
  set_bldcm_speed(300);
  set_bldcm_enable();
  
  Delay(0xFFFFFF);
	
	while(1)
	{
    /* 扫描KEY1 */
    if( Key_Scan(KEY1_GPIO_PORT,KEY1_PIN) == KEY_ON  )
    {
      /* 使能电机 */
      set_bldcm_speed(300);
      set_bldcm_enable();
      
      Delay(0xFFFFFF);
    }
    
    /* 扫描KEY2 */
    if( Key_Scan(KEY2_GPIO_PORT,KEY2_PIN) == KEY_ON  )
    {
      /* 增大占空比 */
      target_speed+=100;
      
      if(target_speed>2000)
        target_speed=2000;
      
      set_pid_actual(target_speed);
    }
    
    /* 扫描KEY3 */
    if( Key_Scan(KEY3_GPIO_PORT,KEY3_PIN) == KEY_ON  )
    {
      if(target_speed<100)
        target_speed=100;
      else
        target_speed-=100;

      set_pid_actual(target_speed);
    }
    
    /* 扫描KEY4 */
    if( Key_Scan(KEY4_GPIO_PORT,KEY4_PIN) == KEY_ON  )
    {
      /* 转换方向 */
      set_bldcm_direction( (++i % 2) ? MOTOR_FWD : MOTOR_REV);
    }
    
    /* 扫描KEY4 */
    if( Key_Scan(KEY5_GPIO_PORT,KEY5_PIN) == KEY_ON  )
    {
      /* 停止电机 */
      set_bldcm_disable();
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
