#ifndef __BSP_STEP_MOTOR_INIT_H
#define	__BSP_STEP_MOTOR_INIT_H

#include "stm32f4xx.h"
#include "stm32f4xx_hal.h"
#include "./stepper/bsp_stepper_T_speed.h"

/*宏定义*/
/*******************************************************/
//Motor 方向 
#define MOTOR_DIR_PIN                  						GPIO_PIN_13   
#define MOTOR_DIR_GPIO_PORT            						GPIOB                     
#define MOTOR_DIR_GPIO_CLK_ENABLE()   						__HAL_RCC_GPIOB_CLK_ENABLE()
					
//Motor 使能 					
#define MOTOR_EN_PIN                  						GPIO_PIN_6
#define MOTOR_EN_GPIO_PORT            						GPIOA                       
#define MOTOR_EN_GPIO_CLK_ENABLE()    						__HAL_RCC_GPIOA_CLK_ENABLE()
	
//Motor 脉冲
#define MOTOR_PUL_TIM                        			TIM8
#define MOTOR_PUL_CLK_ENABLE()  									__TIM8_CLK_ENABLE()

#define MOTOR_PUL_IRQn                   					TIM8_CC_IRQn
#define MOTOR_PUL_IRQHandler             					TIM8_CC_IRQHandler

#define STEPMOTOR_TIM_IT_CCx                 		 	TIM_IT_CC1
#define STEPMOTOR_TIM_FLAG_CCx               		 	TIM_FLAG_CC1

#define MOTOR_PUL_PORT                						GPIOC                       
#define MOTOR_PUL_PIN                 						GPIO_PIN_6                  
#define MOTOR_PUL_GPIO_CLK_ENABLE()       				__HAL_RCC_GPIOC_CLK_ENABLE()

#define MOTOR_PUL_GPIO_AF                         GPIO_AF3_TIM8
#define MOTOR_PUL_CHANNEL_x               				TIM_CHANNEL_1


/****************************************************************/

#define HIGH 1		//高电平
#define LOW  0		//低电平

#define ON  1			//开
#define OFF 0			//关

#define CLOCKWISE 			1//顺时针
#define ANTI_CLOCKWISE	0//逆时针

//输出比较模式周期设置为0xFFFF
#define TIM_PERIOD                   0xFFFF

//控制使能引脚
#define	GPIO_H(p,i)					{p->BSRR=i;}			  								//设置为高电平		
#define GPIO_L(p,i)					{p->BSRR=(uint32_t)i << 16;}				//输出低电平
#define GPIO_T(p,i)					{p->ODR ^=i;}												//输出反转状态

#define MOTOR_EN(x)					if(x)																			\
														{GPIO_H(MOTOR_EN_GPIO_PORT,MOTOR_EN_PIN);}\
														else																			\
														{GPIO_L(MOTOR_EN_GPIO_PORT,MOTOR_EN_PIN);}

#define MOTOR_PUL(x)				if(x)																				\
														{GPIO_H(MOTOR_PUL_GPIO_PORT,MOTOR_PUL_PIN);}\
														else																				\
														{GPIO_L(MOTOR_PUL_GPIO_PORT,MOTOR_PUL_PIN);}	

#define MOTOR_DIR(x)				if(x)																				\
														{GPIO_H(MOTOR_DIR_GPIO_PORT,MOTOR_DIR_PIN);}\
                            else																				\
                            {GPIO_L(MOTOR_DIR_GPIO_PORT,MOTOR_DIR_PIN);}	
														

#define MOTOR_PUL_T()				GPIO_T(MOTOR_PUL_GPIO_PORT,MOTOR_PUL_PIN);
	


//函数以及变量声明
extern TIM_HandleTypeDef TIM_TimeBaseStructure;
void stepper_Init(void);
void stepper_move_T(int32_t step, uint32_t accel, uint32_t decel, uint32_t speed);

#endif
