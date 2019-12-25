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
//    printf("PID_init begin \n");
    pid.target_val=0.0;				
    pid.actual_val=0.0;
    pid.err=0.0;
    pid.err_last=0.0;
    pid.integral=0.0;
//		pid.Kp = 0.31;
//		pid.Ki = 0.070;
//		pid.Kd = 0.3;
//		pid.Kp = 0.21;
//		pid.Ki = 0.070;
//		pid.Kd = 0.3;
		pid.Kp = 0.01;//24
		pid.Ki = 0.80;
		pid.Kd = 0.04;
//    printf("PID_init end \n");

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
    pid.target_val=temp_val;
		/*计算目标值与实际值的误差*/
    pid.err=pid.target_val-pid.actual_val;
		/*误差累积*/
    pid.integral+=pid.err;
		/*PID算法实现*/
    pid.actual_val=pid.Kp*pid.err+pid.Ki*pid.integral+pid.Kd*(pid.err-pid.err_last);
		/*误差传递*/
    pid.err_last=pid.err;
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







