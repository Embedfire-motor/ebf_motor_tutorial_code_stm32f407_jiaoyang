/**
  ******************************************************************************
  * @file    main.c
  * @author  fire
  * @version V1.0
  * @date    2019-xx-xx
  * @brief   PID算法实现
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
#include "./tim/bsp_basic_tim.h"
#include "./usart/bsp_debug_usart.h"
#include "./led/bsp_led.h"
#include "./pid/bsp_pid.h"
#include "./key/bsp_key.h" 
#include "./protocol/protocol.h"

extern _pid pid;
extern float set_point;
extern int pid_status;

/**
  * @brief  主函数
  * @param  无
  * @retval 无
  */
int main(void) 
{
	/* 初始化系统时钟为168MHz */
	SystemClock_Config();
	/* 协议初始化 */
	protocol_init();
	/*初始化USART 配置模式为 115200 8-N-1，中断接收*/
	DEBUG_USART_Config();
	/* 初始化基本定时器定时，20ms产生一次中断 */
	TIMx_Configuration();
	/* PID算法参数初始化 */
	PID_param_init();	
	/*led初始化*/
	LED_GPIO_Config();
	/*按键初始化*/
	Key_GPIO_Config();
	int run_i=0;
#if defined(PID_ASSISTANT_EN) 
    int temp = set_point;    // 上位机需要整数参数，转换一下
    set_computer_value(SEND_TARGET_CMD, CURVES_CH1, &temp, 1);     // 给通道 1 发送目标值
#endif
  
	while(1)
	{      
		/* 接收数据处理 */
		receiving_process();		
		/*模拟修改PID目标值*/
		if( Key_Scan(KEY2_GPIO_PORT,KEY2_PIN) == KEY_ON  )
		{
			if(run_i%2==0)
			{
					set_point=200;
					pid.target_val = set_point;
			}
			else
			{
					set_point=0;
					pid.target_val = set_point;
			}
			run_i++;
        
#if defined(PID_ASSISTANT_EN) 
        temp = set_point;    // 上位机需要整数参数，转换一下
        set_computer_value(SEND_TARGET_CMD, CURVES_CH1, &temp, 1);     // 给通道 1 发送目标值
#endif
		}   

		if( Key_Scan(KEY3_GPIO_PORT,KEY3_PIN) == KEY_ON  )
		{
			pid_status=!pid_status;//取反状态
		  
#if defined(PID_ASSISTANT_EN) 
		  if (!pid_status)
		  {
			set_computer_value(SEND_START_CMD, CURVES_CH1, NULL, 0);     // 同步上位机的启动按钮状态
		  }
		  else
		  {
			set_computer_value(SEND_STOP_CMD, CURVES_CH1, NULL, 0);     // 同步上位机的启动按钮状态
		  }      
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
