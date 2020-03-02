#ifndef __BASIC_TIM_H
#define	__BASIC_TIM_H

#include "stm32f4xx.h"

#define BASIC_TIM           		  TIM7
#define BASIC_TIM_CLK_ENABLE()   	__TIM7_CLK_ENABLE()

//#define BASIC_TIM_IRQn				    TIM6_DAC_IRQn
//#define BASIC_TIM_IRQHandler    	TIM6_DAC_IRQHandler

#define BASIC_TIM_IRQn				    TIM7_IRQn
#define BASIC_TIM_IRQHandler    	TIM7_IRQHandler

extern TIM_HandleTypeDef TIM_TimeBaseStructure;
void TIMx_Configuration(void);

#endif /* __BASIC_TIM_H */

