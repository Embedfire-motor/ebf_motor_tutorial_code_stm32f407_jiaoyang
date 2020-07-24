#ifndef __BSP_PROXIMITY_SWITCH_H
#define	__BSP_PROXIMITY_SWITCH_H

#include "stm32f4xx_hal.h"

/* 接近开关结构体 */
typedef struct{
  GPIO_TypeDef *port;            //端口号
  uint16_t pin;                  //引脚序号
  uint32_t gpio_mode;            //触发模式
  IRQn_Type IRQn;                //中断源
}ProximitySwitch_TypeDef;


//引脚定义
/*******************************************************/
#define SWITCH1_INT_GPIO_PORT                GPIOF
#define SWITCH1_INT_GPIO_CLK_ENABLE()        __HAL_RCC_GPIOF_CLK_ENABLE()
#define SWITCH1_INT_GPIO_PIN                 GPIO_PIN_13
#define SWITCH1_INT_EXTI_IRQ                 EXTI15_10_IRQn
#define SWITCH1_IRQHandler                   EXTI15_10_IRQHandler

#define SWITCH2_INT_GPIO_PORT                GPIOF
#define SWITCH2_INT_GPIO_CLK_ENABLE()        __HAL_RCC_GPIOF_CLK_ENABLE()
#define SWITCH2_INT_GPIO_PIN                 GPIO_PIN_14
#define SWITCH2_INT_EXTI_IRQ                 EXTI15_10_IRQn
#define SWITCH2_IRQHandler                   EXTI15_10_IRQHandler

#define SWITCH3_INT_GPIO_PORT                GPIOF
#define SWITCH3_INT_GPIO_CLK_ENABLE()        __HAL_RCC_GPIOF_CLK_ENABLE()
#define SWITCH3_INT_GPIO_PIN                 GPIO_PIN_15
#define SWITCH3_INT_EXTI_IRQ                 EXTI15_10_IRQn
#define SWITCH3_IRQHandler                   EXTI15_10_IRQHandler

#define SWITCH4_INT_GPIO_PORT                GPIOG
#define SWITCH4_INT_GPIO_CLK_ENABLE()        __HAL_RCC_GPIOG_CLK_ENABLE()
#define SWITCH4_INT_GPIO_PIN                 GPIO_PIN_0
#define SWITCH4_INT_EXTI_IRQ                 EXTI0_IRQn
#define SWITCH4_IRQHandler                   EXTI0_IRQHandler
/*******************************************************/


void ProximitySwitch_Config(void);

#endif /* __BSP_PROXIMITY_SWITCH_H */
