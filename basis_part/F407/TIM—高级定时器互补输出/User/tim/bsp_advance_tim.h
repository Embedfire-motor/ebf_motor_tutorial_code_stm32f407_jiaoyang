#ifndef __ADVANCE_TIM_H
#define	__ADVANCE_TIM_H

#include "stm32f4xx.h"

/* 定时器 */
#define ADVANCE_TIM           				TIM8
#define ADVANCE_TIM_CLK_ENABLE()  			__TIM8_CLK_ENABLE()

/* TIM8通道1输出引脚 */
#define ADVANCE_OCPWM_PIN           		GPIO_PIN_6              
#define ADVANCE_OCPWM_GPIO_PORT     		GPIOC                      
#define ADVANCE_OCPWM_GPIO_CLK_ENABLE() 	__GPIOC_CLK_ENABLE()
#define ADVANCE_OCPWM_AF					GPIO_AF3_TIM8

/* TIM8通道1互补输出引脚 */
#define ADVANCE_OCNPWM_PIN            		GPIO_PIN_5              
#define ADVANCE_OCNPWM_GPIO_PORT      		GPIOA                      
#define ADVANCE_OCNPWM_GPIO_CLK_ENABLE()	__GPIOA_CLK_ENABLE()
#define ADVANCE_OCNPWM_AF					GPIO_AF3_TIM8

/* TIM8断路输入引脚 */
#define ADVANCE_BKIN_PIN              		GPIO_PIN_6              
#define ADVANCE_BKIN_GPIO_PORT        		GPIOA                      
#define ADVANCE_BKIN_GPIO_CLK_ENABLE()  	__GPIOA_CLK_ENABLE()
#define ADVANCE_BKIN_AF						GPIO_AF3_TIM8


extern TIM_HandleTypeDef TIM_TimeBaseStructure;

void TIMx_Configuration(void);

#endif /* __ADVANCE_TIM_H */

