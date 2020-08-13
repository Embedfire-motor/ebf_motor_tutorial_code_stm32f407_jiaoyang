#ifndef __BSP_GENERAL_TIM_H
#define	__BSP_GENERAL_TIM_H

#include "stm32f4xx.h"

/*宏定义*/
#define GENERAL_TIM                        	TIM4
#define GENERAL_TIM_GPIO_AF                 GPIO_AF2_TIM4
#define GENERAL_TIM_CLK_ENABLE()  					__TIM4_CLK_ENABLE()

#define PWM_CHANNEL_1                       TIM_CHANNEL_1
//#define PWM_CHANNEL_2                       TIM_CHANNEL_2
//#define PWM_CHANNEL_3                       TIM_CHANNEL_3
//#define PWM_CHANNEL_4                       TIM_CHANNEL_4

/* 累计 TIM_Period个后产生一个更新或者中断*/		
/* 当定时器从0计数到PWM_PERIOD_COUNT，即为PWM_PERIOD_COUNT+1次，为一个定时周期 */
#define PWM_PERIOD_COUNT     999

/* 通用控制定时器时钟源TIMxCLK = HCLK/2=84MHz */
/* 设定定时器频率为=TIMxCLK/(PWM_PRESCALER_COUNT+1) */
#define PWM_PRESCALER_COUNT     1679

/*PWM引脚*/
#define GENERAL_TIM_CH1_GPIO_PORT           GPIOD
#define GENERAL_TIM_CH1_PIN                 GPIO_PIN_12

//#define GENERAL_TIM_CH2_GPIO_PORT           GPIOD
//#define GENERAL_TIM_CH2_PIN                 GPIO_PIN_13

extern TIM_HandleTypeDef  TIM_TimeBaseStructure;

extern void TIMx_Configuration(void);
extern void TIM2_SetPWM_pulse(uint32_t channel,int compare);
void set_steering_gear_dutyfactor(uint16_t dutyfactor);
void set_steering_gear_angle(uint16_t angle);
void show_help(void);
void deal_serial_data(void);

#endif

