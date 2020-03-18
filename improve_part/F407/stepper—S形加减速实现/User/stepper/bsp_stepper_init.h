#ifndef __BSP_STEP_MOTOR_INIT_H
#define	__BSP_STEP_MOTOR_INIT_H

#include "stm32f4xx.h"
#include "stm32f4xx_hal.h"

/*宏定义*/
/*******************************************************/
//宏定义对应开发板的接口 1 、2 、3 、4
#define CHANNEL_SW 1

#if(CHANNEL_SW == 1)

//Motor 方向 
#define MOTOR_DIR_PIN                  	GPIO_PIN_1   
#define MOTOR_DIR_GPIO_PORT            	GPIOE                    
#define MOTOR_DIR_GPIO_CLK_ENABLE()   	__HAL_RCC_GPIOE_CLK_ENABLE()

//Motor 使能 
#define MOTOR_EN_PIN                  	GPIO_PIN_0
#define MOTOR_EN_GPIO_PORT            	GPIOE                       
#define MOTOR_EN_GPIO_CLK_ENABLE()    	__HAL_RCC_GPIOE_CLK_ENABLE()
	
//Motor 脉冲
#define MOTOR_PUL_IRQn                  TIM8_CC_IRQn
#define MOTOR_PUL_IRQHandler            TIM8_CC_IRQHandler

#define MOTOR_PUL_TIM                   TIM8
#define MOTOR_PUL_CLK_ENABLE()  		__TIM8_CLK_ENABLE()

#define MOTOR_PUL_PORT       			GPIOI
#define MOTOR_PUL_PIN             		GPIO_PIN_5
#define MOTOR_PUL_GPIO_CLK_ENABLE()		__HAL_RCC_GPIOI_CLK_ENABLE()

//#define MOTOR_PUL_PORT       			GPIOC
//#define MOTOR_PUL_PIN             		GPIO_PIN_6
//#define MOTOR_PUL_GPIO_CLK_ENABLE()		__HAL_RCC_GPIOC_CLK_ENABLE()

#define MOTOR_PUL_GPIO_AF               GPIO_AF3_TIM8
#define MOTOR_PUL_CHANNEL_x             TIM_CHANNEL_1

#define MOTOR_TIM_IT_CCx                TIM_IT_CC1
#define MOTOR_TIM_FLAG_CCx              TIM_FLAG_CC1

#elif(CHANNEL_SW == 2)

//Motor 方向 
#define MOTOR_DIR_PIN                  	GPIO_PIN_8
#define MOTOR_DIR_GPIO_PORT            	GPIOI          
#define MOTOR_DIR_GPIO_CLK_ENABLE()   	__HAL_RCC_GPIOI_CLK_ENABLE()

//Motor 使能 
#define MOTOR_EN_PIN                  	GPIO_PIN_4
#define MOTOR_EN_GPIO_PORT            	GPIOE                       
#define MOTOR_EN_GPIO_CLK_ENABLE()    	__HAL_RCC_GPIOE_CLK_ENABLE()
	
//Motor 脉冲
#define MOTOR_PUL_IRQn                  TIM8_CC_IRQn
#define MOTOR_PUL_IRQHandler            TIM8_CC_IRQHandler

#define MOTOR_PUL_TIM                   TIM8
#define MOTOR_PUL_CLK_ENABLE()  		__TIM8_CLK_ENABLE()

#define MOTOR_PUL_PORT       			GPIOI
#define MOTOR_PUL_PIN             		GPIO_PIN_6
#define MOTOR_PUL_GPIO_CLK_ENABLE()		__HAL_RCC_GPIOI_CLK_ENABLE()

#define MOTOR_PUL_GPIO_AF               GPIO_AF3_TIM8
#define MOTOR_PUL_CHANNEL_x             TIM_CHANNEL_2

#define MOTOR_TIM_IT_CCx                TIM_IT_CC2
#define MOTOR_TIM_FLAG_CCx              TIM_FLAG_CC2

#elif(CHANNEL_SW == 3)

//Motor 方向 
#define MOTOR_DIR_PIN                  	GPIO_PIN_11
#define MOTOR_DIR_GPIO_PORT            	GPIOI          
#define MOTOR_DIR_GPIO_CLK_ENABLE()   	__HAL_RCC_GPIOI_CLK_ENABLE()

//Motor 使能 
#define MOTOR_EN_PIN                  	GPIO_PIN_10
#define MOTOR_EN_GPIO_PORT            	GPIOI                 
#define MOTOR_EN_GPIO_CLK_ENABLE()    	__HAL_RCC_GPIOI_CLK_ENABLE()
	
//Motor 脉冲
#define MOTOR_PUL_IRQn                  TIM8_CC_IRQn
#define MOTOR_PUL_IRQHandler            TIM8_CC_IRQHandler

#define MOTOR_PUL_TIM                   TIM8
#define MOTOR_PUL_CLK_ENABLE()  		__TIM8_CLK_ENABLE()

#define MOTOR_PUL_PORT       			GPIOI
#define MOTOR_PUL_PIN             		GPIO_PIN_7
#define MOTOR_PUL_GPIO_CLK_ENABLE()		__HAL_RCC_GPIOI_CLK_ENABLE()

#define MOTOR_PUL_GPIO_AF               GPIO_AF3_TIM8
#define MOTOR_PUL_CHANNEL_x             TIM_CHANNEL_3

#define MOTOR_TIM_IT_CCx                TIM_IT_CC3
#define MOTOR_TIM_FLAG_CCx              TIM_FLAG_CC3

#elif(CHANNEL_SW == 4)

//Motor 方向 
#define MOTOR_DIR_PIN                  	GPIO_PIN_2
#define MOTOR_DIR_GPIO_PORT            	GPIOF
#define MOTOR_DIR_GPIO_CLK_ENABLE()   	__HAL_RCC_GPIOF_CLK_ENABLE()

//Motor 使能 
#define MOTOR_EN_PIN                  	GPIO_PIN_1
#define MOTOR_EN_GPIO_PORT            	GPIOF       
#define MOTOR_EN_GPIO_CLK_ENABLE()    	__HAL_RCC_GPIOF_CLK_ENABLE()
	
//Motor 脉冲
#define MOTOR_PUL_IRQn                  TIM8_CC_IRQn
#define MOTOR_PUL_IRQHandler            TIM8_CC_IRQHandler

#define MOTOR_PUL_TIM                   TIM8
#define MOTOR_PUL_CLK_ENABLE()  		__TIM8_CLK_ENABLE()

#define MOTOR_PUL_PORT       			GPIOC
#define MOTOR_PUL_PIN             		GPIO_PIN_9
#define MOTOR_PUL_GPIO_CLK_ENABLE()		__HAL_RCC_GPIOC_CLK_ENABLE()

#define MOTOR_PUL_GPIO_AF               GPIO_AF3_TIM8
#define MOTOR_PUL_CHANNEL_x             TIM_CHANNEL_4

#define MOTOR_TIM_IT_CCx                TIM_IT_CC4
#define MOTOR_TIM_FLAG_CCx              TIM_FLAG_CC4

#endif




/****************************************************************/

#define HIGH 1		//高电平
#define LOW  0		//低电平

#define ON  0			//开
#define OFF !0			//关

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
//void stepper_move_T(int32_t step, uint32_t accel, uint32_t decel, uint32_t speed);

#endif
