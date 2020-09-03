#ifndef __BSP_STEP_MOTOR_INIT_H
#define	__BSP_STEP_MOTOR_INIT_H

#include "stm32f4xx_hal.h"

/*宏定义*/
/*******************************************************/
#define MOTOR_TIM                       TIM6
#define MOTOR_TIM_CLK_ENABLE()          __HAL_RCC_TIM6_CLK_ENABLE()
#define MOTOR_TIM_IRQn                  TIM6_DAC_IRQn
#define MOTOR_TIM_IRQHandler            TIM6_DAC_IRQHandler

/*频率相关参数*/
//定时器实际时钟频率为：84MHz/(TIM_PRESCALER-1)
//其中 高级定时器的 频率为168MHz,其他定时器为84MHz
//84M/(MOTOR_TIM_PERIOD-1)=500KHz
//具体需要的频率可以自己计算
#define MOTOR_TIM_PRESCALER             1
// 定义定时器中断周期设置为10
#define MOTOR_TIM_PERIOD                168


//对应开发板的接口 1 、2 、3 、4
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
#define MOTOR_PUL_PIN             		  GPIO_PIN_5
#define MOTOR_PUL_GPIO_PORT       	    GPIOI
#define MOTOR_PUL_GPIO_CLK_ENABLE()		  __HAL_RCC_GPIOI_CLK_ENABLE()

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
#define MOTOR_PUL_PIN             		  GPIO_PIN_6
#define MOTOR_PUL_GPIO_PORT       			GPIOI
#define MOTOR_PUL_GPIO_CLK_ENABLE()		  __HAL_RCC_GPIOI_CLK_ENABLE()

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
#define MOTOR_PUL_PIN             		  GPIO_PIN_7
#define MOTOR_PUL_GPIO_PORT       			GPIOI
#define MOTOR_PUL_GPIO_CLK_ENABLE()		  __HAL_RCC_GPIOI_CLK_ENABLE()

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
#define MOTOR_PUL_PIN             		  GPIO_PIN_9
#define MOTOR_PUL_GPIO_PORT       			GPIOC
#define MOTOR_PUL_GPIO_CLK_ENABLE()		  __HAL_RCC_GPIOC_CLK_ENABLE()

#endif

/************************************************************/
#define HIGH GPIO_PIN_SET	  //高电平
#define LOW  GPIO_PIN_RESET	//低电平

#define ON  LOW	            //开
#define OFF HIGH	          //关

#define CW 	HIGH		        //顺时针
#define CCW LOW      	      //逆时针

//控制使能引脚
/* 带参宏，可以像内联函数一样使用 */
#define MOTOR_EN(x)					HAL_GPIO_WritePin(MOTOR_EN_GPIO_PORT,MOTOR_EN_PIN,x)
#define MOTOR_PUL(x)				HAL_GPIO_WritePin(MOTOR_PUL_GPIO_PORT,MOTOR_PUL_PIN,x)
#define MOTOR_DIR(x)				HAL_GPIO_WritePin(MOTOR_DIR_GPIO_PORT,MOTOR_DIR_PIN,x)

extern TIM_HandleTypeDef TIM_StepperHandle;

void stepper_Init(void);

#endif /* __STEP_MOTOR_INIT_H */
