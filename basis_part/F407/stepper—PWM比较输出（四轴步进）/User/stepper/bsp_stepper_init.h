#ifndef __BSP_STEPPER_INIT_H
#define	__BSP_STEPPER_INIT_H

#include "stm32f4xx_hal.h"


/* 步进电机结构体 */
typedef struct{
  uint16_t pul_pin;                     //电机脉冲输出引脚号
  uint16_t dir_pin;                     //电机方向引脚号
  uint16_t en_pin;                      //电机使能引脚号
  uint32_t pul_channel;                 //电机脉冲输出通道
  GPIO_TypeDef *pul_port;               //电机脉冲引脚端口
  GPIO_TypeDef *dir_port;               //电机方向引脚端口
  GPIO_TypeDef *en_port;                //电机使能引脚端口
  uint16_t oc_pulse_num;                //输出比较计数值，值越小电机转速越快
}Stepper_TypeDef;

/*宏定义*/
/*******************************************************/
#define MOTOR_PUL_IRQn                   TIM8_CC_IRQn
#define MOTOR_PUL_IRQHandler             TIM8_CC_IRQHandler
#define MOTOR_PUL_TIM                    TIM8
#define MOTOR_PUL_CLK_ENABLE()  		     __TIM8_CLK_ENABLE()
#define MOTOR_PUL_GPIO_AF                GPIO_AF3_TIM8

//通道1
//Motor 方向
#define MOTOR_DIR1_PIN                  	GPIO_PIN_1   
#define MOTOR_DIR1_GPIO_PORT            	GPIOE
#define MOTOR_DIR1_GPIO_CLK_ENABLE()   	  __HAL_RCC_GPIOE_CLK_ENABLE()

//Motor 使能
#define MOTOR_EN1_PIN                  	  GPIO_PIN_0
#define MOTOR_EN1_GPIO_PORT            	  GPIOE
#define MOTOR_EN1_GPIO_CLK_ENABLE()    	  __HAL_RCC_GPIOE_CLK_ENABLE()

//Motor 脉冲
#define MOTOR_PUL1_PORT       			      GPIOI
#define MOTOR_PUL1_PIN             		    GPIO_PIN_5
#define MOTOR_PUL1_GPIO_CLK_ENABLE()		  __HAL_RCC_GPIOI_CLK_ENABLE()

//定时器通道
#define MOTOR_PUL1_CHANNEL                TIM_CHANNEL_1


//通道2
//Motor 方向
#define MOTOR_DIR2_PIN                  	GPIO_PIN_8
#define MOTOR_DIR2_GPIO_PORT            	GPIOI          
#define MOTOR_DIR2_GPIO_CLK_ENABLE()      __HAL_RCC_GPIOI_CLK_ENABLE()

//Motor 使能
#define MOTOR_EN2_PIN                     GPIO_PIN_4
#define MOTOR_EN2_GPIO_PORT               GPIOE                       
#define MOTOR_EN2_GPIO_CLK_ENABLE()    	  __HAL_RCC_GPIOE_CLK_ENABLE()

//Motor 脉冲
#define MOTOR_PUL2_PORT       			      GPIOI
#define MOTOR_PUL2_PIN             		    GPIO_PIN_6
#define MOTOR_PUL2_GPIO_CLK_ENABLE()	  	__HAL_RCC_GPIOI_CLK_ENABLE()

//定时器通道
#define MOTOR_PUL2_CHANNEL                TIM_CHANNEL_2


//通道3
//Motor 方向
#define MOTOR_DIR3_PIN                  	GPIO_PIN_11
#define MOTOR_DIR3_GPIO_PORT            	GPIOI          
#define MOTOR_DIR3_GPIO_CLK_ENABLE()   	  __HAL_RCC_GPIOI_CLK_ENABLE()

//Motor 使能
#define MOTOR_EN3_PIN                  	  GPIO_PIN_10
#define MOTOR_EN3_GPIO_PORT            	  GPIOI                 
#define MOTOR_EN3_GPIO_CLK_ENABLE()    	  __HAL_RCC_GPIOI_CLK_ENABLE()

//Motor 脉冲
#define MOTOR_PUL3_PORT       			      GPIOI
#define MOTOR_PUL3_PIN             		    GPIO_PIN_7
#define MOTOR_PUL3_GPIO_CLK_ENABLE()		  __HAL_RCC_GPIOI_CLK_ENABLE()

//定时器通道
#define MOTOR_PUL3_CHANNEL                TIM_CHANNEL_3


//通道4
//Motor 方向
#define MOTOR_DIR4_PIN                  	GPIO_PIN_2
#define MOTOR_DIR4_GPIO_PORT            	GPIOF
#define MOTOR_DIR4_GPIO_CLK_ENABLE()   	  __HAL_RCC_GPIOF_CLK_ENABLE()

//Motor 使能
#define MOTOR_EN4_PIN                  	  GPIO_PIN_1
#define MOTOR_EN4_GPIO_PORT            	  GPIOF       
#define MOTOR_EN4_GPIO_CLK_ENABLE()    	  __HAL_RCC_GPIOF_CLK_ENABLE()

//Motor 脉冲
#define MOTOR_PUL4_PORT       			      GPIOC
#define MOTOR_PUL4_PIN             		    GPIO_PIN_9
#define MOTOR_PUL4_GPIO_CLK_ENABLE()      __HAL_RCC_GPIOC_CLK_ENABLE()

//定时器通道
#define MOTOR_PUL4_CHANNEL                TIM_CHANNEL_4


//控制使能引脚
/* 带参宏，可以像内联函数一样使用 */
#define MOTOR_OFFLINE(port, pin, x)       HAL_GPIO_WritePin(port, pin, x)
#define MOTOR_PUL(port, pin, x)           HAL_GPIO_WritePin(port, pin, x)
#define MOTOR_DIR(port, pin, x)           HAL_GPIO_WritePin(port, pin, x)

/*频率相关参数*/
//定时器实际时钟频率为：168MHz/TIM_PRESCALER
//其中 高级定时器的 频率为168MHz,其他定时器为84MHz
//168/TIM_PRESCALER=2MHz
//具体需要的频率可以自己计算
#define TIM_PRESCALER                84
// 定义定时器周期，输出比较模式周期设置为0xFFFF
#define TIM_PERIOD                   0xFFFF

/************************************************************/
#define HIGH GPIO_PIN_SET	  //高电平
#define LOW  GPIO_PIN_RESET	//低电平

#define ON   LOW	          //开
#define OFF  HIGH	          //关

#define CW   HIGH		        //顺时针
#define CCW  LOW      	    //逆时针

/* 步进电机数组 */
extern Stepper_TypeDef step_motor[4];

void stepper_Init(void);
void stepper_Start(uint32_t channel);
void stepper_Stop(uint32_t channel);

#endif /* __BSP_STEPPER_INIT_H */
