/**
  ******************************************************************************
  * @file    bsp_stepper_S_speed.c
  * @author  fire
  * @version V1.0
  * @date    2020-xx-xx
  * @brief   S形加减速
  ******************************************************************************
  * @attention
  *
  * 实验平台:野火  STM32 F407 开发板 
  * 论坛    :http://www.firebbs.cn
  * 淘宝    :http://firestm32.taobao.com
  *
  ******************************************************************************
  */
#include "./stepper/bsp_stepper_S_speed.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>

/*

在这里采用的是数学匀变速的数学模型，但是速度也并

不是匀速变化的，所以准确的说是加速度的匀变数学模型。

a-t 曲线如下 （V-t曲线 请看 文档）

	|    .
 a|   /|\             
	|  / | \
	| /  |  \            
	|/___|___\_t 		
	0   t/2
		
以上就是加速度的匀变模型（默认初始的加速度为0）

那么有了这个模型就很容易可以算出一些不为人知的秘密！！！

首先有了以上的模型就可以算出每一时刻的加速度，所以先看 

0~t/2 这段 a=K*T ，其中K为加速度a的斜率

将加速度积分就是速度，所以 V = a dt = K*T dt

得： V = K*T^2/2

将速度积分就是位移，所以 S = V dt = K*T^2/2 dt 

得： S = K*T^3/6

*/

SpeedCalc_TypeDef Speed ;

Stepper_Typedef Stepper;

uint8_t print_flag=0;

void CalcSpeed(int32_t Vo, int32_t Vt, float Time)
{
	
  uint8_t Is_Dec = FALSE;     
  int32_t i = 0;
  int32_t Vm =0;              // 中间点速度
  int32_t INCACCELStep;       // 加加速所需的步数
  int32_t DecAccelStep;       // 减加速所需的步数
  float Jerk = 0;             // 加加速度
  float Ti = 0;               // 时间间隔 dt
  float Sumt = 0;             // 时间累加量
  float DeltaV = 0;           // 速度的增量dv  
  float TiCube = 0;           // 时间间隔的立方

  if(Vo > Vt )                          // 初速度比末速度大,做减速运动,数值变化跟加速运动相同,
  {                                     // 只是建表的时候注意将速度倒序.    
    Is_Dec = TRUE;
    Speed.Vo = ROUNDPS_2_STEPPS(Vt);    // 起速:Step/s
    Speed.Vt = ROUNDPS_2_STEPPS(Vo);    // 末速:Step/s
  }
  else
  {
    Is_Dec = FALSE;
    Speed.Vo = ROUNDPS_2_STEPPS(Vo);    
    Speed.Vt = ROUNDPS_2_STEPPS(Vt);    
  }

  Time = ACCEL_TIME(Time);                                    // 得到加加速段的时间
  Vm =  MIDDLEVELOCITY( Speed.Vo , Speed.Vt );                // 计算中点速度
  Jerk = fabs(INCACCEL( Speed.Vo, Vm, Time ));                // 根据中点速度计算加加速度
  INCACCELStep = (int32_t)INCACCELSTEP(Jerk,Time);            // 加加速需要的步数
  DecAccelStep = (int32_t)(Speed.Vt * Time - INCACCELStep);   // 减加速需要的步数 S = Vt * Time - S1
  
  /* 申请内存空间存放速度表 */
  Speed.AccelStep = DecAccelStep + INCACCELStep;              // 加速需要的步数 
  if( Speed.AccelStep  % 2 != 0)     // 由于浮点型数据转换成整形数据带来了误差,所以这里加1
    Speed.AccelStep  += 1;
	
	/*判断内存长度*/
	if(FORM_LEN<Speed.AccelStep)
	{
		printf("FORM_LEN 缓存长度不足\r\n,请将 FORM_LEN 修改为 %d \r\n",Speed.AccelStep);
		return ;
	}
	
  /* 
   * 目标的S型速度曲线是对时间的方程,但是在控制电机的时候则是以步进的方式控制,所以这里对V-t曲线做转换,
   * 得到V-S曲线,计算得到的速度表是关于步数的速度值.使得步进电机每一步都在控制当中.
   */ 
// 计算第一步速度  //根据第一步的速度值达到下一步的时间
  TiCube  = 6.0f * 1.0f / Jerk;                 // 根据位移和时间的公式S = 1/2 * J * Ti^3 第1步的时间:Ti^3 = 6 * 1 / Jerk ;
  Ti = pow(TiCube,(1 / 3.0f) );                 // Ti
  Sumt += Ti;
  DeltaV = 0.5f * Jerk * pow(Sumt,2);//第一步的速度
  Speed.Form[0] = Speed.Vo + DeltaV;
  
	/***************************************************************************/
	/*最小速度限幅*/
  if( Speed.Form[0] <= MIN_SPEED )//以当前定时器频率所能达到的最低速度
    Speed.Form[0] = MIN_SPEED;
  /***************************************************************************/
	/*计算S形速度表*/
  for(i = 1; i < Speed.AccelStep; i++)
  {
    /* 步进电机的速度就是定时器脉冲输出频率,可以计算出每一步的时间 */
    /* 得到第i-1步的时间 */
     Ti = 1.0f / Speed.Form[i-1];             // 电机每走一步的时间 Ti = 1 / Vn-1
    /* 加加速段速度计算 */
    if( i < INCACCELStep)
    {
      Sumt += Ti;//从0开始到i的时间累积
      DeltaV = 0.5f * Jerk * pow(Sumt,2);            // 速度的变化量: dV = 1/2 * Jerk * Ti^2;
      Speed.Form[i] = Speed.Vo + DeltaV;      // 得到加加速段每一步对应的速度
      // 当最后一步的时候,时间并不严格等于Time,所以这里要稍作处理,作为减加速段的时间
      if(i == INCACCELStep - 1)
        Sumt  = fabs(Sumt - Time );
    }
    /* 减加速段速度计算 */
    else
    {
      Sumt += Ti;                                        // 时间累计
      DeltaV = 0.5f * Jerk * pow(fabs( Time - Sumt),2);  // dV = 1/2 * Jerk *(T-t)^2;
      Speed.Form[i] = Speed.Vt - DeltaV;          // V = Vt - DeltaV ;
    }
  }
	/***************************************************************************/
	/*减速运动，倒序排列*/
  if(Is_Dec == TRUE)
  {
    float tmp_Speed = 0;  
    /* 倒序排序 */
    for(i = 0; i< (Speed.AccelStep / 2); i++)
    {
      tmp_Speed = Speed.Form[i];
      Speed.Form[i] = Speed.Form[Speed.AccelStep-1 - i];
      Speed.Form[Speed.AccelStep-1 - i] = tmp_Speed;
    }
  }

}


/**
  * @brief  速度决策
	*	@note 	在中断中使用，每进一次中断，决策一次
  * @retval 无
  */
void speed_decision(void)
{
	/*计数临时变量*/
  float temp_p = 0;
	/*脉冲计数*/
  static uint8_t i = 0;  	
  
	if(__HAL_TIM_GET_IT_SOURCE(&TIM_TimeBaseStructure, MOTOR_TIM_IT_CCx) !=RESET)
	{
		/*清除定时器中断*/
		__HAL_TIM_CLEAR_IT(&TIM_TimeBaseStructure, MOTOR_TIM_IT_CCx);
		
		/******************************************************************/
		/*两次为一个脉冲周期*/
		i++; 
    if(i == 2)
    {
			/*脉冲周期完整后清零*/
      i = 0;   
			/*判断当前的状态，*/
      if(Stepper.status == ACCEL || Stepper.status == DECEL)
      {
				/*步数位置索引递增*/
        Stepper.pos++;
        if(Stepper.pos  < Speed.AccelStep )
        { 
					/*获取每一步的定时器计数值*/
          temp_p = T1_FREQ / Speed.Form[Stepper.pos];
          if((temp_p / 2) >= 0xFFFF)
            temp_p = 0xFFFF;
          Stepper.pluse_time = (uint16_t) (temp_p / 2);
        }
        else
        {
					/*加速部分结束后接下来就是匀速状态或者停止状态*/
          if(Stepper.status == ACCEL)   
					{
					  Stepper.status = AVESPEED;
					}          
          else
          {
						/*停止状态，清空速度表并且关闭通道*/
            Stepper.status = STOP; 
						memset(Speed.Form,0,sizeof(float)*FORM_LEN);
            TIM_CCxChannelCmd(MOTOR_PUL_TIM, MOTOR_PUL_CHANNEL_x, TIM_CCx_DISABLE);// 使能定时器通道 
            
          }
        }
      }
    }
		/**********************************************************************/
		// 获取当前计数器数值
		uint32_t tim_count=__HAL_TIM_GET_COUNTER(&TIM_TimeBaseStructure);
		/*计算下一次时间*/
		uint32_t tmp = tim_count+Stepper.pluse_time;
		/*设置比较值*/
		__HAL_TIM_SET_COMPARE(&TIM_TimeBaseStructure,MOTOR_PUL_CHANNEL_x,tmp);
		
	}
}


/**
  * @brief  初始化状态并且设置第一步的速度
  * @param  无
	* @param  无
	*	@note 		无
  * @retval 无
  */
void stepper_start_run()
{

	/*初始化结构体*/
	memset(&Stepper,0,sizeof(Stepper_Typedef));
	/*初始电机状态*/
	Stepper.status=ACCEL;
	/*初始电机位置*/
	Stepper.pos=0;
	
	/*计算第一次脉冲间隔*/
  if(Speed.Form[0] == 0)	//排除分母为0的情况
    Stepper.pluse_time = 0xFFFF;
  else										//分母不为0的情况
    Stepper.pluse_time  = (uint32_t)(T1_FREQ/Speed.Form[0]);
	
	/*获取当前计数值*/
	uint32_t temp=__HAL_TIM_GET_COUNTER(&TIM_TimeBaseStructure);
	/*在当前计数值基础上设置定时器比较值*/
	__HAL_TIM_SET_COMPARE(&TIM_TimeBaseStructure,MOTOR_PUL_CHANNEL_x,temp +Stepper.pluse_time); 
	/*开启中断输出比较*/
	HAL_TIM_OC_Start_IT(&TIM_TimeBaseStructure,MOTOR_PUL_CHANNEL_x);
	/*使能定时器通道*/
	TIM_CCxChannelCmd(MOTOR_PUL_TIM, MOTOR_PUL_CHANNEL_x, TIM_CCx_ENABLE);
}


/*! \brief 给固定的时间和速度，使得步进电机在固定时间内达到目标速度
 *  \param start_speed   	初始速度
 *  \param end_speed  		结束速度
 *  \param time  					时间
 */
void stepper_move_S(int start_speed,int end_speed,float time)
{
	/*计算参数*/
	CalcSpeed(start_speed,end_speed,time);
	/*开始旋转*/
	stepper_start_run();
}



