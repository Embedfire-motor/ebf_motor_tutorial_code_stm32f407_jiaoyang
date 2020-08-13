#ifndef __ADVANCE_TIM_H
#define	__ADVANCE_TIM_H

#include "stm32f4xx.h"

#define ADVANCE_TIM           		TIM1
#define ADVANCE_TIM_CLK_ENABLE()  	__TIM1_CLK_ENABLE()

#define ADVANCE_TIM_IRQn		    TIM1_UP_TIM10_IRQn
#define ADVANCE_TIM_IRQHandler      TIM1_UP_TIM10_IRQHandler

extern TIM_HandleTypeDef TIM_TimeBaseStructure;

void TIMx_Configuration(void);

#endif /* __ADVANCE_TIM_H */

