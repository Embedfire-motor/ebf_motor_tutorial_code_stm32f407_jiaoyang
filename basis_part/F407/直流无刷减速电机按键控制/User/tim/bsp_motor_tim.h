#ifndef __BSP_MOTOR_TIM_H
#define	__BSP_MOTOR_TIM_H

#include "stm32f4xx.h"
#include ".\bldcm_control\bsp_bldcm_control.h"

/* 电机控制定时器 */
#define ADVANCE_TIM           				      TIM8
#define ADVANCE_TIM_CLK_ENABLE()  			    __TIM8_CLK_ENABLE()
extern TIM_HandleTypeDef  htimx_bldcm;

/* 累计 TIM_Period个后产生一个更新或者中断		
	当定时器从0计数到4999，即为5000次，为一个定时周期 */
#define PWM_PERIOD_COUNT     (5000-1)

/* 高级控制定时器时钟源TIMxCLK = HCLK=168MHz 
	 设定定时器频率为=TIMxCLK/(PWM_PRESCALER_COUNT+1)/PWM_PERIOD_COUNT = 20KHz*/
#define PWM_PRESCALER_COUNT     (168-1)

/* TIM8通道1输出引脚 */
#define ADVANCE_OCPWM1_PIN           		    GPIO_PIN_5
#define ADVANCE_OCPWM1_GPIO_PORT     		    GPIOI
#define ADVANCE_OCPWM1_GPIO_CLK_ENABLE() 	  __GPIOI_CLK_ENABLE()
#define ADVANCE_OCPWM1_AF					          GPIO_AF3_TIM8

/* TIM8通道2输出引脚 */
#define ADVANCE_OCPWM2_PIN           		    GPIO_PIN_6
#define ADVANCE_OCPWM2_GPIO_PORT     		    GPIOI
#define ADVANCE_OCPWM2_GPIO_CLK_ENABLE() 	  __GPIOI_CLK_ENABLE()
#define ADVANCE_OCPWM2_AF					          GPIO_AF3_TIM8

/* TIM8通道3输出引脚 */
#define ADVANCE_OCPWM3_PIN           		    GPIO_PIN_7
#define ADVANCE_OCPWM3_GPIO_PORT     		    GPIOI
#define ADVANCE_OCPWM3_GPIO_CLK_ENABLE() 	  __GPIOI_CLK_ENABLE()
#define ADVANCE_OCPWM3_AF					          GPIO_AF3_TIM8


/* TIM8通道1互补输出引脚 */
#define ADVANCE_OCNPWM1_PIN            		  GPIO_PIN_13
#define ADVANCE_OCNPWM1_GPIO_PORT      		  GPIOH
#define ADVANCE_OCNPWM1_GPIO_CLK_ENABLE()	  __GPIOH_CLK_ENABLE()
#define ADVANCE_OCNPWM1_AF					        GPIO_AF3_TIM8

/* TIM8通道2互补输出引脚 */
#define ADVANCE_OCNPWM2_PIN            		  GPIO_PIN_14
#define ADVANCE_OCNPWM2_GPIO_PORT      		  GPIOH
#define ADVANCE_OCNPWM2_GPIO_CLK_ENABLE()	  __GPIOH_CLK_ENABLE()
#define ADVANCE_OCNPWM2_AF					        GPIO_AF3_TIM8

/* TIM8通道3互补输出引脚 */
#define ADVANCE_OCNPWM3_PIN            		  GPIO_PIN_15
#define ADVANCE_OCNPWM3_GPIO_PORT      		  GPIOH
#define ADVANCE_OCNPWM3_GPIO_CLK_ENABLE()	  __GPIOH_CLK_ENABLE()
#define ADVANCE_OCNPWM3_AF					        GPIO_AF3_TIM8

/* TIM8断路输入引脚 */
//#define ADVANCE_BKIN_PIN              	   	GPIO_PIN_6              
//#define ADVANCE_BKIN_GPIO_PORT        		  GPIOA                      
//#define ADVANCE_BKIN_GPIO_CLK_ENABLE()  	  __GPIOA_CLK_ENABLE()
//#define ADVANCE_BKIN_AF						          GPIO_AF3_TIM8

/* 霍尔传感器定时器 */
#define HALL_TIM           				      TIM5
#define HALL_TIM_CLK_ENABLE()  			    __TIM5_CLK_ENABLE()

extern TIM_HandleTypeDef htimx_hall;

/* 累计 TIM_Period个后产生一个更新或者中断		
	当定时器从0计数到4999，即为5000次，为一个定时周期 */
#define HALL_PERIOD_COUNT     (0xFFFF)

/* 高级控制定时器时钟源TIMxCLK = HCLK / 2 = 84MHz
	 设定定时器频率为 = TIMxCLK / (PWM_PRESCALER_COUNT + 1) / PWM_PERIOD_COUNT = 7.6Hz
   周期 T = 131ms */
#define HALL_PRESCALER_COUNT     (168-1)

/* TIM5 通道 1 引脚 */
#define HALL_INPUT1_PIN           		    GPIO_PIN_10
#define HALL_INPUT1_GPIO_PORT     		    GPIOH
#define HALL_INPUT1_GPIO_CLK_ENABLE() 	  __GPIOH_CLK_ENABLE()
#define HALL_INPUT1_AF					          GPIO_AF2_TIM5

/* TIM5 通道 2 引脚 */
#define HALL_INPUT2_PIN           		    GPIO_PIN_11
#define HALL_INPUT2_GPIO_PORT     		    GPIOH
#define HALL_INPUT2_GPIO_CLK_ENABLE() 	  __GPIOH_CLK_ENABLE()
#define HALL_INPUT2_AF					          GPIO_AF2_TIM5

/* TIM5 通道 3 引脚 */
#define HALL_INPUT3_PIN           		    GPIO_PIN_12
#define HALL_INPUT3_GPIO_PORT     		    GPIOH
#define HALL_INPUT3_GPIO_CLK_ENABLE() 	  __GPIOH_CLK_ENABLE()
#define HALL_INPUT3_AF					          GPIO_AF2_TIM5

#define HALL_TIM_IRQn                    TIM5_IRQn
#define HALL_TIM_IRQHandler              TIM5_IRQHandler

extern TIM_HandleTypeDef TIM_TimeBaseStructure;

void TIMx_Configuration(void);
void stop_pwm_output(void);
void set_pwm_pulse(uint16_t pulse);

void hall_enable(void);
void hall_disable(void);
void hall_tim_config(void);

#endif /* __BSP_MOTOR_TIM_H */

