#ifndef __BSP_STEP_MOTOR_INIT_H
#define	__BSP_STEP_MOTOR_INIT_H

#include "stm32f4xx.h"
#include "stm32f4xx_hal.h"

/*宏定义*/
/*******************************************************/

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

#define MOTOR_PUL_GPIO_AF               GPIO_AF3_TIM8
#define MOTOR_PUL_CHANNEL_x             TIM_CHANNEL_1

// 定义定时器周期，输出比较模式周期设置为0xFFFF
#define TIM_PERIOD                   0xFFFF


extern void TIMx_Configuration(void);
extern void TIM8_SetPWM_pulse(int channel,int compare);

	
/************************************************************/
#define HIGH 1	//高电平
#define LOW 0		//低电平

#define ON 1	//开
#define OFF 0		//关

#define CLOCKWISE 			1//顺时针
#define ANTI_CLOCKWISE	0//逆时针



//控制使能引脚
/* 带参宏，可以像内联函数一样使用 */
#define MOTOR_EN(x)					HAL_GPIO_WritePin(MOTOR_EN_GPIO_PORT,MOTOR_EN_PIN,x)
#define MOTOR_PLU(x)				HAL_GPIO_WritePin(MOTOR_PUL_GPIO_PORT,MOTOR_PUL_PIN,x)
#define MOTOR_DIR(x)				HAL_GPIO_WritePin(MOTOR_DIR_GPIO_PORT,MOTOR_DIR_PIN,x)

extern void stepper_Init(void);
extern void stepper_turn(int tim,float angle,float subdivide,uint8_t dir);
#endif /* __STEP_MOTOR_INIT_H */
