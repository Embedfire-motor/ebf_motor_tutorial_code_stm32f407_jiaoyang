#ifndef __BSP_MOTOR_CONTROL_H
#define	__BSP_MOTOR_CONTROL_H

#include "stm32f4xx.h"
#include "./tim/bsp_motor_tim.h"
#include "./pid/bsp_pid.h"
#include "main.h"

/* 电机方向控制枚举 */
typedef enum
{
  MOTOR_FWD = 0,
  MOTOR_REV,
}motor_dir_t;

#define PPR    (ENCODER_TOTAL_RESOLUTION * REDUCTION_RATIO)    // 编码器一圈可以捕获的脉冲

/* 设置速度（占空比） */
#define SET_FWD_COMPAER(ChannelPulse)     TIM1_SetPWM_pulse(PWM_CHANNEL_1,ChannelPulse)    // 设置比较寄存器的值
#define SET_REV_COMPAER(ChannelPulse)     TIM1_SetPWM_pulse(PWM_CHANNEL_2,ChannelPulse)    // 设置比较寄存器的值

/* 使能输出 */
#define MOTOR_FWD_ENABLE()      HAL_TIM_PWM_Start(&DCM_TimeBaseStructure,PWM_CHANNEL_1);
#define MOTOR_REV_ENABLE()      HAL_TIM_PWM_Start(&DCM_TimeBaseStructure,PWM_CHANNEL_2);

/* 禁用输出 */
#define MOTOR_FWD_DISABLE()     HAL_TIM_PWM_Stop(&DCM_TimeBaseStructure,PWM_CHANNEL_1);
#define MOTOR_REV_DISABLE()     HAL_TIM_PWM_Stop(&DCM_TimeBaseStructure,PWM_CHANNEL_2);

#define PID_ASSISTANT_EN    1    // 1:使用PID调试助手显示波形，0：使用串口直接打印数据

void set_motor_speed(uint16_t v);
void set_motor_direction(motor_dir_t dir);
void show_help(void);
void deal_serial_data(void);
void set_motor_enable(void);
void set_motor_disable(void);
void motor_pid_control(void);

#endif /* __LED_H */

