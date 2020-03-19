/**
  ******************************************************************************
  * @file    bsp_creat_S_tab.c
  * @author  fire
  * @version V1.0
  * @date    2020-xx-xx
  * @brief   S加减速生成表
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
uint8_t print_flag=0;



/** 
  * 函数功能： 速度表计算函数
  * 输入参数： V0      初速度，单位：转/min
  *           Vt      末速度，单位：转/min
  *           Time    加速时间 单位: s
  * 返 回 值：无
  * 说    明: 根据速度曲线和加速时间,将数据密集化,即计算每一步的速度值,并存放在内存当中
  *           这里采用的数学模型是匀变速直线运动
  *           加速段的曲线有两部分组成,第一是加速度递增的加加速段,
  *           第二是加速度递减的减加速段,两段曲线可以视为关于中心对称(中心是中点速度).这两段曲线所用时间相等
  *           所以中点速度 Vm = (Vt + Vo)/2;
  *       加加速段:
  *           加加速段的加速度曲线是一条过原点的递增的直线,对加速度积分得到的就是速度的增加量
  *           所以有:Vm - Vo = 1/2 * Jerk * t^2,得到加加速度Jerk.
  *           最后得到位移方程 S = 1/6 * Jerk * t^3
  *       减加速段:
  *           减加速段的曲线跟关于加加速段对称,速度公式V = Vt - 1/2 * Jerk * (T-t)^2,
  *           位移公式则需要通过对速度曲线求积分的方法得到,S = Vt * t - 1/6 * Jerk * t^3
  *
  *           :分析过程请结合工程文件夹下的曲线图理解.
  *     
  */
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
//  /* mallo申请内存空间,记得释放 */
//  Speed.VelocityTab = (float*)(malloc((Speed.AccelStep + 1) * sizeof(float) ));
//  if(Speed.VelocityTab == NULL)
//  {
//    printf("内存不足!请修改曲线参数,或者修改Heap大小\n");
//    return ;
//  }
  /* 
   * 目标的S型速度曲线是对时间的方程,但是在控制电机的时候则是以步进的方式控制,所以这里对V-t曲线做转换,
   * 得到V-S曲线,计算得到的速度表是关于步数的速度值.使得步进电机每一步都在控制当中.
   */ 
// 计算第一步速度  //根据第一步的速度值达到下一步的时间
  TiCube  = 6.0f * 1.0f / Jerk;                 // 根据位移和时间的公式S = 1/2 * J * Ti^3 第1步的时间:Ti^3 = 6 * 1 / Jerk ;
  Ti = pow(TiCube,(1 / 3.0f) );                 // Ti
  Sumt += Ti;
  DeltaV = 0.5f * Jerk * pow(Sumt,2);//第一步的速度
  Speed.VelocityTab[0] = Speed.Vo + DeltaV;
  
  if( Speed.VelocityTab[0] <= SPEED_MIN )//以当前定时器频率所能达到的最低速度
    Speed.VelocityTab[0] = SPEED_MIN;
  
  for(i = 1; i < Speed.AccelStep; i++)
  {
    /* 步进电机的速度就是定时器脉冲输出频率,可以计算出每一步的时间 */
    /* 得到第i-1步的时间 */
     Ti = 1.0f / Speed.VelocityTab[i-1];             // 电机每走一步的时间 Ti = 1 / Vn-1
    /* 加加速段速度计算 */
    if( i < INCACCELStep)
    {
      Sumt += Ti;//从0开始到i的时间累积
      DeltaV = 0.5f * Jerk * pow(Sumt,2);            // 速度的变化量: dV = 1/2 * Jerk * Ti^2;
      Speed.VelocityTab[i] = Speed.Vo + DeltaV;      // 得到加加速段每一步对应的速度
      // 当最后一步的时候,时间并不严格等于Time,所以这里要稍作处理,作为减加速段的时间
      if(i == INCACCELStep - 1)
        Sumt  = fabs(Sumt - Time );
    }
    /* 减加速段速度计算 */
    else
    {
      Sumt += Ti;                                        // 时间累计
      DeltaV = 0.5f * Jerk * pow(fabs( Time - Sumt),2);  // dV = 1/2 * Jerk *(T-t)^2;
      Speed.VelocityTab[i] = Speed.Vt - DeltaV;          // V = Vt - DeltaV ;
    }
  }
  if(Is_Dec == TRUE) // 减速运动
  {
    float tmp_Speed = 0;  
    /* 倒序排序 */
    for(i = 0; i< (Speed.AccelStep / 2); i++)
    {
      tmp_Speed = Speed.VelocityTab[i];
      Speed.VelocityTab[i] = Speed.VelocityTab[Speed.AccelStep-1 - i];
      Speed.VelocityTab[Speed.AccelStep-1 - i] = tmp_Speed;
    }
  }
//  打印输出速度值
//  for(i= 0; i < Speed.AccelStep;i++)
//  {
//    printf("Speed[%d] = %f\n",i,Speed.VelocityTab[i]);
//  }
}


///**
//  * @brief  计算加速表
//  * @param  Vo  初始速度
//  * @param  Vt	t时刻的速度
//  * @param  T	加速完成时间
//  *	@note 		无
//  * @retval 无
//  */

//void CalculateSpeedTab(int Vo, int Vt, float T)
//{
//	int32_t i = 0;

//	int32_t Vm =0;      // 中间点速度
//	float K = 0;   		// 加加速度 加速度的斜率
//	float Tn = 0;     	// Tn时刻
//	float DeltaV = 0; 	// 每一时刻的速度
//	float TimeDel = 0;	// 时间间隔

//	Speed.Vo = CONVER(Vo);    // 起速:Step/s
//	Speed.Vt = CONVER(Vt);    // 末速:Step/s
//	
//	T = T / 2;						//加加速段的时间（加速度斜率>0的时间）
//	
//	Vm = (Speed.Vo + Speed.Vt) / 2;	//计算中点的速度
//	
//	K = ( ( 2 * ((Vm) - (Speed.Vo)) ) / pow((T),2) );// 根据中点速度计算加加速度
//	
//	Speed.AccelHalfStep  = (int)( ( (K) * pow( (T) ,3) ) / 6 );// 加加速需要的步数
//	
//	/* 申请内存空间存放速度表 */
//	Speed.AccelHalfStep  = abs(Speed.AccelHalfStep ); // 减速计算的时候防止出现负数
//	if( Speed.AccelHalfStep  % 2 != 0)     // 由于浮点型数据转换成整形数据带来了误差,所以这里加1
//	{
//		Speed.AccelHalfStep  += 1;
//	}
//	Speed.AccelTotalStep = Speed.AccelHalfStep * 2;           // 加速段的步数
//	
//	if(FORM_LEN<Speed.AccelTotalStep)
//	{
//		printf("FORM_LEN 缓存长度不足\r\n,请将 FORM_LEN 修改为 %d \r\n",Speed.AccelTotalStep);
//		return ;
//	}
//	  
//	/* 目标的速度曲线是对时间的方程,将时间与步数对应，所以在此将时间分成
//		与步数对应的份数,并且计算出 ,这样就可以计算出相应的速度*/
//	TimeDel = T / Speed.AccelHalfStep;

//	for(i = 0; i <= Speed.AccelHalfStep; i++)
//	{
//		Tn = i * TimeDel;						// 第i个时刻的Tn
//		DeltaV = 0.5f * K * pow(Tn,2);        	// 在Tn时刻所对应的速度  dv = 1/2 * K * t^2;
//		Speed.Form[i] = Speed.Vo + DeltaV;		// 得到每一时刻对应的速度,由于考虑到初速度不为0的情况，所以与Vo相加求和
//																				
//		Speed.Form [ Speed.AccelTotalStep - i] = Speed.Vt - DeltaV ;        // 加加速过程与减加速是中心对称,可以直接求出后半段速度
//																			// 减加速过程对称点的速度
//	}
//	/* 输出速度表内容 */
//	for(i = 0; i <= Speed.AccelTotalStep ; i++)
//	{
//		printf("i,%.3f;speed,%.3f\n",(float)i,Speed.Form[i]);	
//	}
//	/* 清空表 */
//	memset(Speed.Form,0,FORM_LEN*sizeof(float));
//}


/**
  * @brief  速度决策
	*	@note 	在中断中使用，每进一次中断，决策一次
  * @retval 无
  */
void speed_decision(void)
{
	  __IO uint32_t Tim_Count = 0;
  __IO uint32_t tmp = 0;
  __IO float Tim_Pulse = 0;
  __IO static uint8_t i = 0;  
	
  
	if(__HAL_TIM_GET_IT_SOURCE(&TIM_TimeBaseStructure, MOTOR_TIM_IT_CCx) !=RESET)
	{
		// 清楚定时器中断
		__HAL_TIM_CLEAR_IT(&TIM_TimeBaseStructure, MOTOR_TIM_IT_CCx);
		/******************************************************************/
		
		    i++;     // 定时器中断次数计数值
    if(i == 2) // 2次，说明已经输出一个完整脉冲
    {
      i = 0;   // 清零定时器中断次数计数值
      if(MotionStatus == ACCEL || MotionStatus == DECEL)
      {
        Step_Position ++;
        if(Step_Position  < Speed.AccelStep )
        { 
          Tim_Pulse = T1_FREQ / Speed.VelocityTab[Step_Position];// 由速度表得到每一步的定时器计数值
          if((Tim_Pulse / 2) >= 0xFFFF)
            Tim_Pulse = 0xFFFF;
          Toggle_Pulse = (uint16_t) (Tim_Pulse / 2);
        }
        else
        {
          if(MotionStatus == ACCEL)   
            MotionStatus = AVESPEED;
          else
          {
            MotionStatus = STOP; 
//            free(Speed.VelocityTab);          //  运动完要释放内存
            TIM_CCxChannelCmd(MOTOR_PUL_TIM, MOTOR_PUL_CHANNEL_x, TIM_CCx_DISABLE);// 使能定时器通道 
            
          }
        }
      }
    }
	
		/**********************************************************************/
		// 设置比较值
		uint32_t tim_count=__HAL_TIM_GET_COUNTER(&TIM_TimeBaseStructure);
		uint32_t tmp = tim_count+Toggle_Pulse;
		__HAL_TIM_SET_COMPARE(&TIM_TimeBaseStructure,MOTOR_PUL_CHANNEL_x,tmp);
		
		
	}
}



int32_t  Step_Position   = 0;           // 当前位置
uint16_t    Toggle_Pulse = 0;        // 脉冲频率控制
uint8_t  MotionStatus    = 0;  



/**
  * 函数功能: 步进电机做S型加减速运动
  * 输入参数: 无
  * 返 回 值: 无
  * 说    明: 无
  */
void stepper_start_run()
{
  Step_Position = 0;
  MotionStatus = ACCEL; // 电机为运动状态
  // 第一步速度是0,则定时器从0xFFFF开始;
  if(Speed.VelocityTab[0] == 0)
    Toggle_Pulse = 0xFFFF;
  else
    Toggle_Pulse  = (uint32_t)(T1_FREQ/Speed.VelocityTab[0]);
  __HAL_TIM_SET_COUNTER(&TIM_TimeBaseStructure,0);
  __HAL_TIM_SET_COMPARE(&TIM_TimeBaseStructure,MOTOR_PUL_CHANNEL_x,(uint16_t)(Toggle_Pulse)); // 设置定时器比较值
  TIM_CCxChannelCmd(MOTOR_PUL_TIM, MOTOR_PUL_CHANNEL_x, TIM_CCx_ENABLE);// 使能定时器通道 
  //STEPMOTOR_OUTPUT_ENABLE();
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
