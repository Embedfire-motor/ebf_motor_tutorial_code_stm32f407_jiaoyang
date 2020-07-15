/**
  ******************************************************************************
  * @file    main.c
  * @author  fire
  * @version V1.0
  * @date    2020-xx-xx
  * @brief   梯形加减速
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
#include "./usart/bsp_debug_usart.h"
#include "./delay/core_delay.h"
#include "./stepper/bsp_stepper_init.h"
#include "./stepper/bsp_stepper_spta_speed.h"
#include "./key/bsp_exti.h"
#include "./led/bsp_led.h"
#include "./Encoder/bsp_encoder.h"


extern void motot_start(void);

/* 当前时刻总计数值 */
__IO int32_t capture_count = 0;
/* 上一时刻总计数值 */
__IO int32_t last_count = 0;
/* 单位时间内总计数值 */
__IO int32_t count_per_unit = 0;
/* 电机转轴转速 */
__IO float shaft_speed = 0.0f;
/* 累积圈数 */
__IO float number_of_rotations = 0.0f;

/**
  * @brief  主函数
  * @param  无
  * @retval 无
  */
int main(void) 
{
  HAL_InitTick(0);
	/* 初始化系统时钟为168MHz */
	SystemClock_Config();
	/*初始化USART 配置模式为 115200 8-N-1，中断接收*/
	DEBUG_USART_Config();
	printf("欢迎使用野火 电机开发板 步进电机 SPTA加减速正反旋转 例程\r\n");
	/*步进电机初始化*/
	stepper_Init();
  /* 编码器接口初始化 */
	Encoder_Init();
  
  int i=0;
  char j = 0;
//  uint32_t time = 0;

	while(1)
	{
//    motot_start();

    /* 当前时刻总计数值 = 计数器值 + 计数溢出次数 * ENCODER_TIM_PERIOD  */
    capture_count =__HAL_TIM_GET_COUNTER(&TIM_EncoderHandle) + (Encoder_Overflow_Count * ENCODER_TIM_PERIOD);
    /* 单位时间内总计数值 = 当前时刻总计数值 - 上一时刻总计数值 */
    count_per_unit = capture_count - last_count;
    /* 转轴转速 = 单位时间内的计数值 / 编码器总分辨率 * 时间系数  */
    shaft_speed = (float)count_per_unit / ENCODER_TOTAL_RESOLUTION * 100 ;
    /* 记录当前总计数值，供下一时刻计算使用 */
    last_count = capture_count;
    /* 累积圈数 = 当前时刻总计数值 / 编码器总分辨率  */
    number_of_rotations = (float)capture_count / ENCODER_TOTAL_RESOLUTION;
    
    printf("%.3f %.3f\r\n", shaft_speed, number_of_rotations);
//    printf("%d %.3f\r\n", time, shaft_speed);
    
    if(i == 250)
    {
      j > 0 ? MOTOR_DIR(CCW) : MOTOR_DIR(CW);
//      motot_start();
      j > 0 ? Stepper_Move_SPTA(&spta_data, -200*10, spta_maxspeed, spta_accspeed) : Stepper_Move_SPTA(&spta_data, 200*10, spta_maxspeed, spta_accspeed);
      i = 0;
      j=~j;
    }
    i++;
    HAL_Delay(10);
//    time+=5;
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



