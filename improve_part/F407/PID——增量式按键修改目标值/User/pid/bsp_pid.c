#include "./pid/bsp_pid.h"
#include "math.h"
#include "./key/bsp_key.h" 
//定义全局变量

_pid pid;

/**
  * @brief  PID参数初始化
	*	@note 	无
  * @retval 无
  */
void PID_param_init()
{
		/* 初始化参数 */

    pid.target_val=0.0;				
    pid.actual_val=0.0;
		pid.err = 0.0;
		pid.err_last = 0.0;
		pid.err_next = 0.0;
		
//		pid.Kp = 0.20;
//		pid.Ki = 0.80;
//		pid.Kd = 0.01;
	
		pid.Kp = 0.05;
		pid.Ki = 0.10;
		pid.Kd = 0.01;

}
/**
  * @brief  PID算法实现
  * @param  val		目标值
	*	@note 	无
  * @retval 通过PID计算后的输出
  */
float PID_realize(float temp_val) 
{
	/*传入目标值*/
	pid.target_val = temp_val;
	/*计算目标值与实际值的误差*/
  pid.err=pid.target_val-pid.actual_val;
	/*PID算法实现*/
	float increment_val = pid.Kp*(pid.err - pid.err_next) + pid.Ki*pid.err + pid.Kd*(pid.err - 2 * pid.err_next + pid.err_last);
	/*累加*/
	pid.actual_val += increment_val;
	/*传递误差*/
	pid.err_last = pid.err_next;
	pid.err_next = pid.err;
	/*返回当前实际值*/
	return pid.actual_val;
}
/**
  * @brief  定时器周期调用函数
  * @param  无
	*	@note 	无
  * @retval 无
  */
float set_point=0.0;
int pid_status=0;
void time_period_fun()
{
	static int num=0;
	static int run_i=0;
			
	if(!pid_status)
	{
		float val=PID_realize(set_point);
		printf("val,%f;act,%f\n",set_point,val);	
	}
}







