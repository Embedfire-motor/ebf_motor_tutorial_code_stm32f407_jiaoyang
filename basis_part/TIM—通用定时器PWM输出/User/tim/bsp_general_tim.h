#ifndef __BSP_GENERAL_TIM_H
#define	__BSP_GENERAL_TIM_H

#include "stm32f4xx.h"
/*ºê¶¨Òå*/

#define GENERAL_TIM                        	TIM2
#define GENERAL_TIM_GPIO_AF                 GPIO_AF1_TIM2
#define GENERAL_TIM_CLK_ENABLE()  					__TIM2_CLK_ENABLE()


/*PWMÒý½Å*/
#define GENERAL_TIM_CH1_GPIO_PORT           GPIOA
#define GENERAL_TIM_CH1_PIN                 GPIO_PIN_15

#define GENERAL_TIM_CH2_GPIO_PORT           GPIOB
#define GENERAL_TIM_CH2_PIN                 GPIO_PIN_3

#define GENERAL_TIM_CH3_GPIO_PORT           GPIOB
#define GENERAL_TIM_CH3_PIN                 GPIO_PIN_10

#define GENERAL_TIM_CH4_GPIO_PORT           GPIOB
#define GENERAL_TIM_CH4_PIN                 GPIO_PIN_11

extern void TIMx_Configuration(void);
extern void TIM2_SetPWM_pulse(int channel,int compare);

#endif

