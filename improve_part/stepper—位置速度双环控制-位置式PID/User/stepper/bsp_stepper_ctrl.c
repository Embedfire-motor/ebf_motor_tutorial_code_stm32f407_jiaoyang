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

extern _pid speed_pid,move_pid;
extern __IO uint16_t OC_Pulse_num;     //比较输出的计数值

/* 系统状态初始化 */
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
  * @brief  步进电机位置速度双闭环控制
  * @retval 无
  * @note   基本定时器中断内调用
  */
void Stepper_Ctrl(void)
{
  /* 编码器相关变量 */
  static __IO float last_count = 0;
  __IO float capture_count = 0;
  __IO float capture_per_unit = 0;
  /* 经过pid计算后的期望值 */
  static __IO float speed_cont_val = 0.0f;
  static __IO float move_cont_val = 0.0f;  
  static int cont_val = 0;  
	
  /* 当电机运动时才启动pid计算 */
  if((sys_status.MSD_ENA == 1) && (sys_status.stepper_running == 1))
  {
		/* 第一步 --> 计算当前编码器脉冲总数和单位时间内产生的脉冲数 */
		// 当前脉冲总计数值 = 当前定时器的计数值 + （定时器的溢出次数 * 定时器的重装载值）
    capture_count = (int)__HAL_TIM_GET_COUNTER(&TIM_EncoderHandle) + (encoder_overflow_count * ENCODER_TIM_PERIOD);
		// 单位时间的脉冲数 = 当前的脉冲总计数值 - 上一次的脉冲总计数值    
		capture_per_unit = capture_count - last_count;
		// 将当前当前的脉冲总计数值 赋值给 上一次的脉冲总计数值这个变量   
    last_count = capture_count;

    /* 第二步 --> 将当前脉冲总计数值作为实际值传入PID控制器，
		              进行位置环的计算，得到当前位置环期望值	*/
    move_cont_val = PID_realize(&move_pid, (float)capture_count);

    /* 第三步 --> 使用位置环期望值判断运动方向 */
    move_cont_val > 0 ? (MOTOR_DIR(CW)) : (MOTOR_DIR(CCW));
		/* 判断是否启用速度环 */
		if (fabsf(move_cont_val) >= MOVE_CTRL) 
		{
			/* 第四步 --> 速度环计算前的数据处理 */
			// 传递位置环计算的期望值给另一个变量，便于计算
			cont_val = move_cont_val;
			// 目标速度上限处理 
			if (cont_val > TARGET_SPEED_MAX)
			{
				cont_val = TARGET_SPEED_MAX;
			}
			else if (cont_val < -TARGET_SPEED_MAX)
			{
				cont_val = -TARGET_SPEED_MAX;
			}
			
#if defined(PID_ASSISTANT_EN)
			int32_t temp = cont_val;
			set_computer_value(SEND_TARGET_CMD, CURVES_CH2, &temp, 1);     // 给通道 2 发送目标值
#endif
			/* 第五步 --> 将位置环求得的期望值作为目标值，
										将单位时间的脉冲数作为实际值传入PID控制器，
		                进行速度环的计算，得到当前速度环期望值	*/
			// 设定速度环的目标值 
			set_pid_target(&speed_pid, cont_val);    
			// 单位时间内的编码器脉冲数作为实际值传入速度环pid控制器
			speed_cont_val = PID_realize(&speed_pid, (float)capture_per_unit);
			/* 第六步 --> 使用速度环求得的期望值控制步进电机 */
			// 由于OC_Pulse_num为uint16_t变量，取速度环输出值的绝对值进行后续计算
			cont_val = fabsf(speed_cont_val);	
			// 计算比较计数器的增量值 
			OC_Pulse_num = ((uint16_t)(TIM_STEP_FREQ / (cont_val * PULSE_RATIO * SAMPLING_PERIOD))) >> 1;
		} 
		else
		{
			/* 直接使用位置环控制步进电机，计算比较计数器的增量值 */
			OC_Pulse_num = ((uint16_t)(TIM_STEP_FREQ / ((float)move_cont_val * PULSE_RATIO))) >> 1;
		}
#if PID_ASSISTANT_EN
     int Temp_ch2 = capture_per_unit;    // 上位机需要整数参数，转换一下
		 int Temp_ch1 = capture_count;
     set_computer_value(SEND_FACT_CMD, CURVES_CH2, &Temp_ch2, 1);  // 给通道 1 发送实际值     // 给通道 2 发送实际值
     set_computer_value(SEND_FACT_CMD, CURVES_CH1, &Temp_ch1, 1);     // 给通道 1 发送实际值

#else
     printf("实际值：%d，目标值：%.0f\r\n", capture_per_unit, pid.target_val);// 打印实际值和目标值 
#endif
  }
  else
  {
    /*停机状态所有参数清零*/
    last_count = 0;
    speed_cont_val = 0;
    move_cont_val = 0;
    speed_pid.actual_val = 0;
    speed_pid.err = 0;
    speed_pid.err_last = 0;
		move_pid.actual_val = 0;
    move_pid.err = 0;
    move_pid.err_last = 0;
  }
}
