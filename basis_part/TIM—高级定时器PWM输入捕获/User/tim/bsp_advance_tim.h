#ifndef __ADVANCE_TIM_H
#define __ADVANCE_TIM_H

#include "stm32f4xx.h"

/* 通用定时器 */
#define GENERAL_TIM                       TIM4
#define GENERAL_TIM_CLK_ENABLE()          __TIM4_CLK_ENABLE()

/* 通用定时器PWM输出 */
/* PWM输出引脚 */
#define GENERAL_OCPWM_PIN                 GPIO_PIN_14              
#define GENERAL_OCPWM_GPIO_PORT           GPIOD                      
#define GENERAL_OCPWM_GPIO_CLK_ENABLE()   __GPIOD_CLK_ENABLE()
#define GENERAL_OCPWM_AF                  GPIO_AF2_TIM4

/* 高级控制定时器 */
#define ADVANCE_TIM                       TIM1
#define ADVANCE_TIM_CLK_ENABLE()          __TIM1_CLK_ENABLE()

/* 捕获/比较中断 */
#define ADVANCE_TIM_IRQn                  TIM1_CC_IRQn
#define ADVANCE_TIM_IRQHandler            TIM1_CC_IRQHandler
/* 高级控制定时器PWM输入捕获 */
/* PWM输入捕获引脚 */
#define ADVANCE_ICPWM_PIN                 GPIO_PIN_9              
#define ADVANCE_ICPWM_GPIO_PORT           GPIOE                     
#define ADVANCE_ICPWM_GPIO_CLK_ENABLE()   __GPIOE_CLK_ENABLE()
#define ADVANCE_ICPWM_AF                  GPIO_AF1_TIM1
#define ADVANCE_IC1PWM_CHANNEL            TIM_CHANNEL_1
#define ADVANCE_IC2PWM_CHANNEL            TIM_CHANNEL_2

extern TIM_HandleTypeDef  TIM_PWMOUTPUT_Handle;
extern TIM_HandleTypeDef  TIM_PWMINPUT_Handle;

void TIMx_Configuration(void);
#endif /* __ADVANCE_TIM_H */

