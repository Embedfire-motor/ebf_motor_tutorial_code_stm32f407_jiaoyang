#ifndef __BSP_MOTOR_TIM_H
#define	__BSP_MOTOR_TIM_H

#include "stm32f4xx.h"

/*宏定义*/
#define PWM_TIM                        	TIM1
#define PWM_TIM_GPIO_AF                 GPIO_AF1_TIM1
#define PWM_TIM_CLK_ENABLE()  					__TIM1_CLK_ENABLE()

#define PWM_CHANNEL_1                   TIM_CHANNEL_1
#define PWM_CHANNEL_2                   TIM_CHANNEL_2

/* 累计 TIM_Period个后产生一个更新或者中断*/		
/* 当定时器从0计数到PWM_PERIOD_COUNT，即为PWM_PERIOD_COUNT+1次，为一个定时周期 */
#define PWM_PERIOD_COUNT     (5600)

/* 通用控制定时器时钟源TIMxCLK = HCLK=168MHz */
/* 设定定时器频率为=TIMxCLK/(PWM_PRESCALER_COUNT+1) */
#define PWM_PRESCALER_COUNT     (2)

/*PWM引脚*/
#define PWM_TIM_CH1_GPIO_PORT           GPIOA
#define PWM_TIM_CH1_PIN                 GPIO_PIN_8

#define PWM_TIM_CH2_GPIO_PORT           GPIOA
#define PWM_TIM_CH2_PIN                 GPIO_PIN_9

#define PWM_TIM_CH3_GPIO_PORT           GPIOA
#define PWM_TIM_CH3_PIN                 GPIO_PIN_10

extern TIM_HandleTypeDef  TIM_TimeBaseStructure;

extern void TIMx_Configuration(void);
extern void TIM1_SetPWM_pulse(uint32_t channel,int compare);

#endif   /* __BSP_MOTOR_TIM_H */

