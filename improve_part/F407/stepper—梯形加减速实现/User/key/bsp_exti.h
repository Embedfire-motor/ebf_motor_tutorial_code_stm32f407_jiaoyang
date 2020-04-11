#ifndef __EXTI_H
#define	__EXTI_H

#include "stm32f4xx.h"

//Òý½Å¶¨Òå
/*******************************************************/
#define KEY1_INT_GPIO_PORT                GPIOA
#define KEY1_INT_GPIO_CLK_ENABLE()        __GPIOA_CLK_ENABLE();
#define KEY1_INT_GPIO_PIN                 GPIO_PIN_0
#define KEY1_INT_EXTI_IRQ                 EXTI0_IRQn
#define KEY1_IRQHandler                   EXTI0_IRQHandler

#define KEY2_INT_GPIO_PORT                GPIOG
#define KEY2_INT_GPIO_CLK_ENABLE()        __GPIOG_CLK_ENABLE();
#define KEY2_INT_GPIO_PIN                 GPIO_PIN_2
#define KEY2_INT_EXTI_IRQ                 EXTI2_IRQn
#define KEY2_IRQHandler                   EXTI2_IRQHandler
/*******************************************************/


void EXTI_Key_Config(void);

#endif /* __EXTI_H */
