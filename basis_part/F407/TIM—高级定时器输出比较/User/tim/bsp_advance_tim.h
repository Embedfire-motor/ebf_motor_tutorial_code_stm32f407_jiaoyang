#ifndef __ADVANCE_TIM_H
#define	__ADVANCE_TIM_H

#include "stm32f4xx.h"

/* 高级定时器配置 */
#define ADVANCE_TIM           		      TIM8
#define ADVANCE_TIM_CLK_ENABLE()  	    __HAL_RCC_TIM8_CLK_ENABLE()
#define ADVANCE_TIM_IRQn		            TIM8_CC_IRQn
#define ADVANCE_TIM_IRQHandler          TIM8_CC_IRQHandler

/* TIM8通道1输出引脚 */
#define CHANNEL1_OC_PIN           		  GPIO_PIN_5              
#define CHANNEL1_OC_GPIO_PORT     		  GPIOI                     
#define CHANNEL1_OC_GPIO_CLK_ENABLE() 	__HAL_RCC_GPIOI_CLK_ENABLE()
#define CHANNEL1_OC_AF					        GPIO_AF3_TIM8
#define ADVANCE_TIM_CHANNEL_1           TIM_CHANNEL_1

/* TIM8通道2输出引脚 */
#define CHANNEL2_OC_PIN           		  GPIO_PIN_6              
#define CHANNEL2_OC_GPIO_PORT     		  GPIOI                     
#define CHANNEL2_OC_GPIO_CLK_ENABLE() 	__HAL_RCC_GPIOI_CLK_ENABLE()
#define CHANNEL2_OC_AF					        GPIO_AF3_TIM8
#define ADVANCE_TIM_CHANNEL_2           TIM_CHANNEL_2

/* TIM8通道3输出引脚 */
#define CHANNEL3_OC_PIN           		  GPIO_PIN_7              
#define CHANNEL3_OC_GPIO_PORT     		  GPIOI                     
#define CHANNEL3_OC_GPIO_CLK_ENABLE() 	__HAL_RCC_GPIOI_CLK_ENABLE()
#define CHANNEL3_OC_AF					        GPIO_AF3_TIM8
#define ADVANCE_TIM_CHANNEL_3           TIM_CHANNEL_3

/* TIM8通道4输出引脚 */
#define CHANNEL4_OC_PIN           		  GPIO_PIN_9              
#define CHANNEL4_OC_GPIO_PORT     		  GPIOC                     
#define CHANNEL4_OC_GPIO_CLK_ENABLE() 	__HAL_RCC_GPIOC_CLK_ENABLE()
#define CHANNEL4_OC_AF					        GPIO_AF3_TIM8
#define ADVANCE_TIM_CHANNEL_4           TIM_CHANNEL_4

/*频率相关参数*/
//定时器实际时钟频率为：168MHz/(TIM_PRESCALER-1)
//其中 高级定时器的 频率为168MHz,其他定时器为84MHz
//168/(TIM_PRESCALER-1)=1Mhz
//具体需要的频率可以自己计算
#define TIM_PRESCALER                168
// 定义定时器周期，输出比较模式周期设置为0xFFFF
#define TIM_PERIOD                   0xFFFF

extern TIM_HandleTypeDef TIM_AdvanceHandle;

void TIMx_AdvanceConfig(void);

#endif /* __ADVANCE_TIM_H */

