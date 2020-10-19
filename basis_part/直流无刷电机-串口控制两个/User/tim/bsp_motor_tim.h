#ifndef __BSP_MOTOR_TIM_H
#define	__BSP_MOTOR_TIM_H

#include "stm32f4xx.h"
#include ".\bldcm_control\bsp_bldcm_control.h"

/*****************************电机接口1宏定义*******************************************/
/* 电机控制定时器 */
#define MOTOR1_TIM           				      TIM1
#define MOTOR1_TIM_CLK_ENABLE()  			    __TIM1_CLK_ENABLE()
extern TIM_HandleTypeDef  motor1_htimx_bldcm;

/* 累计 TIM_Period个后产生一个更新或者中断		
	当定时器从0计数到5599，即为5600次，为一个定时周期 */
#define MOTOR1_PWM_PERIOD_COUNT     (5600)

#define MOTOR1_PWM_MAX_PERIOD_COUNT    (MOTOR1_PWM_PERIOD_COUNT - 100)

/* 高级控制定时器时钟源TIMxCLK = HCLK = 168MHz 
	 设定定时器频率为=TIMxCLK/(PWM_PRESCALER_COUNT+1)/PWM_PERIOD_COUNT = 15KHz*/
#define MOTOR1_PWM_PRESCALER_COUNT     (2)

/* TIM1通道1输出引脚 */
#define MOTOR1_OCPWM1_PIN           		    GPIO_PIN_8
#define MOTOR1_OCPWM1_GPIO_PORT     		    GPIOA
#define MOTOR1_OCPWM1_GPIO_CLK_ENABLE() 	  __GPIOA_CLK_ENABLE()
#define MOTOR1_OCPWM1_AF					          GPIO_AF1_TIM1

/* TIM1通道2输出引脚 */
#define MOTOR1_OCPWM2_PIN           		    GPIO_PIN_9
#define MOTOR1_OCPWM2_GPIO_PORT     		    GPIOA
#define MOTOR1_OCPWM2_GPIO_CLK_ENABLE() 	  __GPIOA_CLK_ENABLE()
#define MOTOR1_OCPWM2_AF					          GPIO_AF1_TIM1

/* TIM1通道3输出引脚 */
#define MOTOR1_OCPWM3_PIN           		    GPIO_PIN_10
#define MOTOR1_OCPWM3_GPIO_PORT     		    GPIOA
#define MOTOR1_OCPWM3_GPIO_CLK_ENABLE() 	  __GPIOA_CLK_ENABLE()
#define MOTOR1_OCPWM3_AF					          GPIO_AF1_TIM1

/* TIM1通道1互补输出引脚 */
#define MOTOR1_OCNPWM1_PIN            		  GPIO_PIN_13
#define MOTOR1_OCNPWM1_GPIO_PORT      		  GPIOB
#define MOTOR1_OCNPWM1_GPIO_CLK_ENABLE()	  __GPIOB_CLK_ENABLE()
#define MOTOR1_OCNPWM1_AF					        	GPIO_AF1_TIM1

/* TIM1通道2互补输出引脚 */
#define MOTOR1_OCNPWM2_PIN            		  GPIO_PIN_14
#define MOTOR1_OCNPWM2_GPIO_PORT      		  GPIOB
#define MOTOR1_OCNPWM2_GPIO_CLK_ENABLE()	  __GPIOB_CLK_ENABLE()
#define MOTOR1_OCNPWM2_AF					       	  GPIO_AF1_TIM1

/* TIM1通道3互补输出引脚 */
#define MOTOR1_OCNPWM3_PIN            		  GPIO_PIN_15
#define MOTOR1_OCNPWM3_GPIO_PORT      		  GPIOB
#define MOTOR1_OCNPWM3_GPIO_CLK_ENABLE()	  __GPIOB_CLK_ENABLE()
#define MOTOR1_OCNPWM3_AF					        	GPIO_AF1_TIM1

#define MOTOR1_TIM_COM_TS_ITRx              TIM_TS_ITR3    // 内部触发配置(TIM1->ITR2->TIM3)

/* 霍尔传感器定时器 */
#define MOTOR1_HALL_TIM           				  TIM3
#define MOTOR1_HALL_TIM_CLK_ENABLE()  			__TIM3_CLK_ENABLE()

extern TIM_HandleTypeDef motor1_htimx_hall;

/* 累计 TIM_Period个后产生一个更新或者中断		
	当定时器从0计数到4999，即为5000次，为一个定时周期 */
#define MOTOR1_HALL_PERIOD_COUNT     (0xFFFF)

/* 高级控制定时器时钟源TIMxCLK = HCLK / 2 = 84MHz
	 设定定时器频率为 = TIMxCLK / (PWM_PRESCALER_COUNT + 1) / PWM_PERIOD_COUNT = 10.01Hz
   周期 T = 100ms */
#define MOTOR1_HALL_PRESCALER_COUNT     (128)

/* TIM3 通道 1 引脚 */
#define MOTOR1_HALL_INPUTU_PIN           		    GPIO_PIN_6
#define MOTOR1_HALL_INPUTU_GPIO_PORT     		    GPIOC
#define MOTOR1_HALL_INPUTU_GPIO_CLK_ENABLE() 	  __GPIOC_CLK_ENABLE()
#define MOTOR1_HALL_INPUTU_AF					          GPIO_AF2_TIM3

/* TIM3 通道 2 引脚 */
#define MOTOR1_HALL_INPUTV_PIN           		    GPIO_PIN_7
#define MOTOR1_HALL_INPUTV_GPIO_PORT     		    GPIOC
#define MOTOR1_HALL_INPUTV_GPIO_CLK_ENABLE() 	  __GPIOC_CLK_ENABLE()
#define MOTOR1_HALL_INPUTV_AF					          GPIO_AF2_TIM3

/* TIM3 通道 3 引脚 */
#define MOTOR1_HALL_INPUTW_PIN           		    GPIO_PIN_8
#define MOTOR1_HALL_INPUTW_GPIO_PORT     		    GPIOC
#define MOTOR1_HALL_INPUTW_GPIO_CLK_ENABLE() 	  __GPIOC_CLK_ENABLE()
#define MOTOR1_HALL_INPUTW_AF					          GPIO_AF2_TIM3

#define MOTOR1_HALL_TIM_IRQn                    TIM3_IRQn
#define MOTOR1_HALL_TIM_IRQHandler              TIM3_IRQHandler


/***************************************************************************************/

/*****************************电机接口2宏定义*******************************************/
/* 电机控制定时器 */
#define MOTOR2_TIM           				      TIM8
#define MOTOR2_TIM_CLK_ENABLE()  			    __TIM8_CLK_ENABLE()
extern TIM_HandleTypeDef  motor2_htimx_bldcm;

/* 累计 TIM_Period个后产生一个更新或者中断		
	当定时器从0计数到5599，即为5600次，为一个定时周期 */
#define MOTOR2_PWM_PERIOD_COUNT     (5600)

#define MOTOR2_PWM_MAX_PERIOD_COUNT    (MOTOR2_PWM_PERIOD_COUNT - 100)

/* 高级控制定时器时钟源TIMxCLK = HCLK = 168MHz 
	 设定定时器频率为=TIMxCLK/(PWM_PRESCALER_COUNT+1)/PWM_PERIOD_COUNT = 15KHz*/
#define MOTOR2_PWM_PRESCALER_COUNT     (2)

/* TIM8通道1输出引脚 */
#define MOTOR2_OCPWM1_PIN           		    GPIO_PIN_5
#define MOTOR2_OCPWM1_GPIO_PORT     		    GPIOI
#define MOTOR2_OCPWM1_GPIO_CLK_ENABLE() 	  __GPIOI_CLK_ENABLE()
#define MOTOR2_OCPWM1_AF					          GPIO_AF3_TIM8

/* TIM8通道2输出引脚 */
#define MOTOR2_OCPWM2_PIN           		    GPIO_PIN_6
#define MOTOR2_OCPWM2_GPIO_PORT     		    GPIOI
#define MOTOR2_OCPWM2_GPIO_CLK_ENABLE() 	  __GPIOI_CLK_ENABLE()
#define MOTOR2_OCPWM2_AF					          GPIO_AF3_TIM8

/* TIM8通道3输出引脚 */
#define MOTOR2_OCPWM3_PIN           		    GPIO_PIN_7
#define MOTOR2_OCPWM3_GPIO_PORT     		    GPIOI
#define MOTOR2_OCPWM3_GPIO_CLK_ENABLE() 	  __GPIOI_CLK_ENABLE()
#define MOTOR2_OCPWM3_AF					          GPIO_AF3_TIM8

/* TIM8通道1互补输出引脚 */
#define MOTOR2_OCNPWM1_PIN            		  GPIO_PIN_13
#define MOTOR2_OCNPWM1_GPIO_PORT      		  GPIOH
#define MOTOR2_OCNPWM1_GPIO_CLK_ENABLE()	  __GPIOH_CLK_ENABLE()
#define MOTOR2_OCNPWM1_AF					        	GPIO_AF3_TIM8

/* TIM8通道2互补输出引脚 */
#define MOTOR2_OCNPWM2_PIN            		  GPIO_PIN_14
#define MOTOR2_OCNPWM2_GPIO_PORT      		  GPIOH
#define MOTOR2_OCNPWM2_GPIO_CLK_ENABLE()	  __GPIOH_CLK_ENABLE()
#define MOTOR2_OCNPWM2_AF					        	GPIO_AF3_TIM8

/* TIM8通道3互补输出引脚 */
#define MOTOR2_OCNPWM3_PIN            		  GPIO_PIN_15
#define MOTOR2_OCNPWM3_GPIO_PORT      		  GPIOH
#define MOTOR2_OCNPWM3_GPIO_CLK_ENABLE()	  __GPIOH_CLK_ENABLE()
#define MOTOR2_OCNPWM3_AF					        	GPIO_AF3_TIM8

#define MOTOR2_TIM_COM_TS_ITRx              TIM_TS_ITR3    // 内部触发配置(TIM8->ITR3->TIM5)

/* 霍尔传感器定时器 */
#define MOTOR2_HALL_TIM           				  TIM5
#define MOTOR2_HALL_TIM_CLK_ENABLE()  			__TIM5_CLK_ENABLE()

extern TIM_HandleTypeDef motor2_htimx_hall;

/* 累计 TIM_Period个后产生一个更新或者中断		
	当定时器从0计数到4999，即为5000次，为一个定时周期 */
#define MOTOR2_HALL_PERIOD_COUNT     (0xFFFF)

/* 高级控制定时器时钟源TIMxCLK = HCLK / 2 = 84MHz
	 设定定时器频率为 = TIMxCLK / (PWM_PRESCALER_COUNT + 1) / PWM_PERIOD_COUNT = 10.01Hz
   周期 T = 100ms */
#define MOTOR2_HALL_PRESCALER_COUNT     (128)

/* TIM5 通道 1 引脚 */
#define MOTOR2_HALL_INPUTU_PIN           		    GPIO_PIN_10
#define MOTOR2_HALL_INPUTU_GPIO_PORT     		    GPIOH
#define MOTOR2_HALL_INPUTU_GPIO_CLK_ENABLE() 	  __GPIOH_CLK_ENABLE()
#define MOTOR2_HALL_INPUTU_AF					          GPIO_AF2_TIM5

/* TIM5 通道 2 引脚 */
#define MOTOR2_HALL_INPUTV_PIN           		    GPIO_PIN_11
#define MOTOR2_HALL_INPUTV_GPIO_PORT     		    GPIOH
#define MOTOR2_HALL_INPUTV_GPIO_CLK_ENABLE() 	  __GPIOH_CLK_ENABLE()
#define MOTOR2_HALL_INPUTV_AF					          GPIO_AF2_TIM5

/* TIM5 通道 3 引脚 */
#define MOTOR2_HALL_INPUTW_PIN           		    GPIO_PIN_12
#define MOTOR2_HALL_INPUTW_GPIO_PORT     		    GPIOH
#define MOTOR2_HALL_INPUTW_GPIO_CLK_ENABLE() 	  __GPIOH_CLK_ENABLE()
#define MOTOR2_HALL_INPUTW_AF					          GPIO_AF2_TIM5

#define MOTOR2_HALL_TIM_IRQn                    TIM5_IRQn
#define MOTOR2_HALL_TIM_IRQHandler              TIM5_IRQHandler


/***************************************************************************************/

extern TIM_HandleTypeDef motor1_TIM_TimeBaseStructure;
extern TIM_HandleTypeDef motor2_TIM_TimeBaseStructure;

void TIMx_Configuration(void);

void stop_motor1_pwm_output(void);
void set_motor1_pwm_pulse(uint16_t pulse);

void hall_motor1_enable(void);
void hall_motor1_disable(void);
void hall_motor1_tim_config(void);

void stop_motor2_pwm_output(void);
void set_motor2_pwm_pulse(uint16_t pulse);

void hall_motor2_enable(void);
void hall_motor2_disable(void);
void hall_motor2_tim_config(void);

#endif /* __BSP_MOTOR_TIM_H */

