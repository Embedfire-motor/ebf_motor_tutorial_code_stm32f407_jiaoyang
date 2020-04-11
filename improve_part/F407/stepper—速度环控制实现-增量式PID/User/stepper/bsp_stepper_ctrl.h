#ifndef __BSP_STEP_MOTOR_CTRL_H
#define	__BSP_STEP_MOTOR_CTRL_H

#include "./stepper/bsp_stepper_init.h"
#include "./Encoder/bsp_encoder.h"

/*宏定义*/
/*******************************************************/
#define TIM_STEP_FREQ     (SystemCoreClock/TIM_PRESCALER) // 频率ft值

/*电机单圈参数*/
#define STEP_ANGLE				1.8f                 //步进电机的步距角 单位：度
#define FSPR              (360.0f/STEP_ANGLE)  //步进电机的一圈所需脉冲数

#define MICRO_STEP        32          				 //细分器细分数 
#define SPR               (FSPR*MICRO_STEP)    //细分后一圈所需脉冲数

#define PULSE_RATIO       (float)(SPR/ENCODER_TOTAL_RESOLUTION)//步进电机单圈脉冲数与编码器单圈脉冲的比值

#define TARGET_SPEED      4                   //步进电机运动时的目标转速，单位：转/秒
//#define SAMPLES_PER_SEC   50                  //每秒采样次数 = 1000ms / 采样周期，
#define PID_ASSISTANT_EN  1

typedef struct {
  unsigned char stepper_dir : 1;               //步进电机方向
  unsigned char stepper_running : 1;           //步进电机运行状态
  unsigned char MSD_ENA : 1;                   //驱动器使能状态
}__SYS_STATUS;


void MSD_ENA(int NewState);
void Set_Stepper_Stop(void);
void Set_Stepper_Start(void);
void Set_Stepper_Dir(int dir);
void Stepper_Speed_Ctrl(void);

#endif /* __STEP_MOTOR_CTRL_H */
