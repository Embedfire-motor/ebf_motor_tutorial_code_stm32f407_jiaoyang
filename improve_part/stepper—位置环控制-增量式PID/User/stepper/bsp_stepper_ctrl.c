/**
  ******************************************************************************
  * @file    bsp_stepper_init.c
  * @author  fire
  * @version V1.0
  * @date    2019-xx-xx
  * @brief   步进电机初始化
  ******************************************************************************
  * @attention
  *
  * 实验平台:野火  STM32 F407 开发板  
  * 论坛    :http://www.firebbs.cn
  * 淘宝    :http://firestm32.taobao.com
  *
  ******************************************************************************
  */
#include "math.h"

#include "./stepper/bsp_stepper_ctrl.h"
#include "./pid/bsp_pid.h"
#include "./protocol/protocol.h"

extern _pid pid;
extern __IO uint16_t OC_Pulse_num;     //比较输出的计数值

__SYS_STATUS sys_status = {0};

/**
  * @brief  驱动器紧急停止
  * @param  NewState：使能或者禁止
  * @retval 无
  */
void MSD_ENA(int NewState)
{
    if(NewState)
    {
      //ENA失能，禁止驱动器输出，（脱机状态）此时电机为无保持力矩状态，可以手动旋转电机
      MOTOR_EN(OFF);
      sys_status.MSD_ENA = 0;
    }
    else
    {
      //ENA使能，此时电机为保持力矩状态
      MOTOR_EN(ON);
      sys_status.MSD_ENA = 1;
    }
}

/**
  * @brief  步进电机刹车
  * @param  无
  * @retval 无
  */
void Set_Stepper_Stop(void)
{
  /*失能比较通道*/
	TIM_CCxChannelCmd(MOTOR_PUL_TIM,MOTOR_PUL_CHANNEL_x,TIM_CCx_DISABLE);
  sys_status.stepper_running = 0;
}

/**
  * @brief  启动步进电机
  * @param  无
  * @retval 无
  */
void Set_Stepper_Start(void)
{
  /*使能驱动器*/
  MSD_ENA(ON);
  /*使能比较通道输出*/
	TIM_CCxChannelCmd(MOTOR_PUL_TIM,MOTOR_PUL_CHANNEL_x,TIM_CCx_ENABLE);
  sys_status.MSD_ENA = 1;
  sys_status.stepper_running = 1;
}

/**
  * @brief  步进电机方向
  * @param  无
  * @retval 无
  */
void Set_Stepper_Dir(int dir)
{
  if(sys_status.stepper_dir != dir)
  {
    Set_Stepper_Stop();
  }
}

/**
  * @brief  步进电机增量式PID控制
  * @retval 无
  * @note   基本定时器中断内调用
  */
void Stepper_Speed_Ctrl(void)
{
  /* 编码器相关变量 */
  static __IO int32_t last_count = 0;
  __IO int32_t capture_count = 0;
  __IO int32_t capture_per_unit = 0;
  /* 经过pid计算后的期望值 */
  static __IO float cont_val = 0.0f;
  
  __IO float timer_delay = 0.0f;
  
  /* 当电机运动时才启动pid计算 */
  if((sys_status.MSD_ENA == 1) && (sys_status.stepper_running == 1))
  {
    /* 第一步 --> 计算当前编码器脉冲总数 */
		// 当前脉冲总计数值 = 当前定时器的计数值 + （定时器的溢出次数 * 定时器的重装载值）
    capture_count =__HAL_TIM_GET_COUNTER(&TIM_EncoderHandle) + (encoder_overflow_count * ENCODER_TIM_PERIOD);
    
    /* 第二步 --> 将当前编码器脉冲总数作为实际值传入pid控制器 */		
    cont_val += PID_realize((float)capture_count);
    
    /* 第三步 --> 用PID控制器的输出值(期望值)来调节步进电机 */
		// 判断PID控制器输出值的方向
    cont_val > 0 ? (MOTOR_DIR(CW)) : (MOTOR_DIR(CCW));
    // 由于比较计数器的增量不能为负值，对计算得出的期望值取绝对值 */
    timer_delay = fabsf(cont_val);
    /* 限制最大启动速度 ，防止堵转*/
    timer_delay >= SPEED_LIMIT ? (timer_delay = SPEED_LIMIT) : timer_delay;
    // 比较计数器的增量 =  (比较计时器的频率/(期望值的绝对值 * 步进电机与编码器的脉冲比 )) / 2
    OC_Pulse_num = ((uint16_t)(T1_FREQ / ((float)timer_delay * PULSE_RATIO))) >> 1;
 
    #if PID_ASSISTANT_EN
     int Temp = capture_count;    // 上位机需要整数参数，转换一下
     set_computer_value(SEND_FACT_CMD, CURVES_CH1, &Temp, 1);  // 给通道 1 发送实际值
    #else
     printf("实际值：%d，目标值：%.0f\r\n", capture_per_unit, pid.target_val);// 打印实际值和目标值 
    #endif
  }
  else
  {
    /*停机状态所有参数清零*/
    last_count = 0;
    cont_val = 0;
    pid.actual_val = 0;
    pid.err = 0;
    pid.err_last = 0;
    pid.err_next = 0;
  }
}
