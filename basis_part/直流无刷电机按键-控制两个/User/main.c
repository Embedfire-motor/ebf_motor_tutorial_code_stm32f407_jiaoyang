/**
  ******************************************************************************
  * @file    main.c
  * @author  fire
  * @version V1.0
  * @date    2020-xx-xx
  * @brief   直流无刷电机控制
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

/**
  * @brief  主函数
  * @param  无
  * @retval 无
  */
int main(void) 
{
  __IO uint16_t MOTOR1_ChannelPulse = MOTOR1_PWM_MAX_PERIOD_COUNT/10;
	__IO uint16_t MOTOR2_ChannelPulse = MOTOR2_PWM_MAX_PERIOD_COUNT/10;
	
	uint8_t i = 0;
	uint8_t j = 0;
	
  uint8_t motor1_en_flag = 0;
  uint8_t motor2_en_flag = 0;
	
	/* 初始化系统时钟为168MHz */
	SystemClock_Config();
  
	/* 初始化按键GPIO */
	Key_GPIO_Config();
  
  /* LED 灯初始化 */
  LED_GPIO_Config();
  
  /* 调试串口初始化 */
  DEBUG_USART_Config();
  
  printf("野火直流无刷电机按键控制例程\r\n");

  /* 电机初始化 */
  bldcm_init();

	while(1)
	{
    /* 扫描KEY1 */
    if( Key_Scan(KEY1_GPIO_PORT, KEY1_PIN) == KEY_ON)
    {
      /* 使能电机 */
			if(!motor1_en_flag)
			{
			    set_motor1_bldcm_speed(MOTOR1_ChannelPulse);
					set_motor1_bldcm_enable();				
			}
			else
			{
					set_motor1_bldcm_disable();
			}
			motor1_en_flag = !motor1_en_flag;
    }
    
    /* 扫描KEY2 */
    if( Key_Scan(KEY2_GPIO_PORT, KEY2_PIN) == KEY_ON)
    {
      /* 使能电机 */
			if(!motor2_en_flag)
			{
			    set_motor2_bldcm_speed(MOTOR2_ChannelPulse);
					set_motor2_bldcm_enable();
			}
			else
			{
					set_motor2_bldcm_disable();
			}
			motor2_en_flag = !motor2_en_flag;
    }
    /* 扫描KEY3 */
    if( Key_Scan(KEY3_GPIO_PORT, KEY3_PIN) == KEY_ON)
    {
      /* 增大占空比 */
      MOTOR1_ChannelPulse += MOTOR1_PWM_MAX_PERIOD_COUNT/10;
      
      if(MOTOR1_ChannelPulse > MOTOR1_PWM_MAX_PERIOD_COUNT)
        MOTOR1_ChannelPulse = MOTOR1_PWM_MAX_PERIOD_COUNT;
      
      set_motor1_bldcm_speed(MOTOR1_ChannelPulse);
			
			MOTOR2_ChannelPulse += MOTOR2_PWM_MAX_PERIOD_COUNT/10;
      
      if(MOTOR2_ChannelPulse > MOTOR2_PWM_MAX_PERIOD_COUNT)
        MOTOR2_ChannelPulse = MOTOR2_PWM_MAX_PERIOD_COUNT;
      
      set_motor2_bldcm_speed(MOTOR2_ChannelPulse);
    }
    
    /* 扫描KEY4 */
    if( Key_Scan(KEY4_GPIO_PORT, KEY4_PIN) == KEY_ON)
    {
      if(MOTOR1_ChannelPulse < MOTOR1_PWM_MAX_PERIOD_COUNT/10)
        MOTOR1_ChannelPulse = 0;
      else
        MOTOR1_ChannelPulse -= MOTOR1_PWM_MAX_PERIOD_COUNT/10;

      set_motor1_bldcm_speed(MOTOR1_ChannelPulse);
			
      if(MOTOR2_ChannelPulse < MOTOR2_PWM_MAX_PERIOD_COUNT/10)
        MOTOR2_ChannelPulse = 0;
      else
        MOTOR2_ChannelPulse -= MOTOR2_PWM_MAX_PERIOD_COUNT/10;

      set_motor2_bldcm_speed(MOTOR2_ChannelPulse);
    }

		/* 扫描KEY5 */
    if( Key_Scan(KEY5_GPIO_PORT, KEY5_PIN) == KEY_ON)
    {
      /* 转换方向 */
      set_motor1_bldcm_direction( (++i % 2) ? MOTOR_FWD : MOTOR_REV);
		
      set_motor2_bldcm_direction( (++j % 2) ? MOTOR_FWD : MOTOR_REV);
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
