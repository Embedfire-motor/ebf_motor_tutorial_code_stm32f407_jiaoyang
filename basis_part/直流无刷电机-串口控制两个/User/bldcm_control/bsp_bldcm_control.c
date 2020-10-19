/**
  ******************************************************************************
  * @file    bsp_bldcm_control.c
  * @author  fire
  * @version V1.0
  * @date    2019-xx-xx
  * @brief   无刷电机控制接口
  ******************************************************************************
  * @attention
  *
  * 实验平台:野火  STM32 F407 开发板 
  * 论坛    :http://www.firebbs.cn
  * 淘宝    :http://firestm32.taobao.com
  *
  ******************************************************************************
  */ 

#include ".\bldcm_control\bsp_bldcm_control.h"
#include "./usart/bsp_debug_usart.h"
#include <math.h>
#include <stdlib.h>

/* 私有变量 */
static bldcm_data_t motor1_bldcm_data;
static bldcm_data_t motor2_bldcm_data;

/* 局部函数 */
static void sd_gpio_config(void);

/**
  * @brief  电机初始化
  * @param  无
  * @retval 无
  */
void bldcm_init(void)
{
  TIMx_Configuration();    // 电机控制定时器，引脚初始化
  hall_motor1_tim_config();       // 霍尔传感器初始化
	hall_motor2_tim_config();       // 霍尔传感器初始化
  sd_gpio_config();        // sd 引脚初始化
}

/**
  * @brief  电机 SD 控制引脚初始化
  * @param  无
  * @retval 无
  */
static void sd_gpio_config(void)
{
  GPIO_InitTypeDef GPIO_InitStruct;
  
  /* 定时器通道功能引脚端口时钟使能 */
	MOTOR1_SHUTDOWN_GPIO_CLK_ENABLE();
 	MOTOR2_SHUTDOWN_GPIO_CLK_ENABLE(); 
	
  /* 引脚IO初始化 */
	/*设置输出类型*/
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	/*设置引脚速率 */ 
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	/*选择要控制的GPIO引脚*/	
	GPIO_InitStruct.Pin = MOTOR1_SHUTDOWN_PIN;
  
	/*调用库函数，使用上面配置的GPIO_InitStructure初始化GPIO*/
  HAL_GPIO_Init(MOTOR1_SHUTDOWN_GPIO_PORT, &GPIO_InitStruct);
	/*选择要控制的GPIO引脚*/	
	GPIO_InitStruct.Pin = MOTOR2_SHUTDOWN_PIN;
	HAL_GPIO_Init(MOTOR2_SHUTDOWN_GPIO_PORT, &GPIO_InitStruct);
}

/**
  * @brief  设置电机速度
  * @param  v: 速度（占空比）
  * @retval 无
  */
void set_motor1_bldcm_speed(uint16_t v)
{
  motor1_bldcm_data.dutyfactor = v;
  
  set_motor1_pwm_pulse(v);     // 设置速度
}

void set_motor2_bldcm_speed(uint16_t v)
{
  motor2_bldcm_data.dutyfactor = v;
  
  set_motor2_pwm_pulse(v);     // 设置速度
}

/**
  * @brief  设置电机方向
  * @param  无
  * @retval 无
  */
void set_motor1_bldcm_direction(motor_dir_t dir)
{
  motor1_bldcm_data.direction = dir;
}

void set_motor2_bldcm_direction(motor_dir_t dir)
{
  motor2_bldcm_data.direction = dir;
}

/**
  * @brief  获取电机当前方向
  * @param  无
  * @retval 无
  */
motor_dir_t get_motor1_bldcm_direction(void)
{
  return motor1_bldcm_data.direction;
}

motor_dir_t get_motor2_bldcm_direction(void)
{
  return motor2_bldcm_data.direction;
}

/**
  * @brief  使能电机
  * @param  无
  * @retval 无
  */
void set_motor1_bldcm_enable(void)
{
  MOTOR1_BLDCM_ENABLE_SD();
  hall_motor1_enable();
}

void set_motor2_bldcm_enable(void)
{
  MOTOR2_BLDCM_ENABLE_SD();
  hall_motor2_enable();
}

/**
  * @brief  禁用电机
  * @param  无
  * @retval 无
  */
void set_motor1_bldcm_disable(void)
{
  /* 禁用霍尔传感器接口 */
  hall_motor1_disable();
  
  /* 停止 PWM 输出 */
  stop_motor1_pwm_output();
  
  /* 关闭 MOS 管 */
  MOTOR1_BLDCM_DISABLE_SD();
}

void set_motor2_bldcm_disable(void)
{
  /* 禁用霍尔传感器接口 */
  hall_motor2_disable();
  
  /* 停止 PWM 输出 */
  stop_motor2_pwm_output();
  
  /* 关闭 MOS 管 */
  MOTOR2_BLDCM_DISABLE_SD();
}

/**
  * @brief  打印帮助命令
  * @param  无
  * @retval 无
  */
void show_help(void)
{
    printf("——————————————野火直流无刷电机驱动演示程序——————————————\n\r");
    printf("输入命令(以回车结束)：\n\r");
    printf("< ? >        -帮助菜单\n\r");
    printf("v1 [data]     -设置电机1的速度（范围：0—%d）\n\r", MOTOR1_PWM_PERIOD_COUNT);
    printf("d1 [data]     -设置电机1的方向，%d:正向转，%d:反向转\n\r", MOTOR_FWD, MOTOR_REV);
	  printf("v2 [data]     -设置电机2的速度（范围：0—%d）\n\r", MOTOR2_PWM_PERIOD_COUNT);
    printf("d2 [data]     -设置电机2的方向，%d:正向转，%d:反向转\n\r", MOTOR_FWD, MOTOR_REV);
}

/**
  * @brief  处理串口接收到的数据
  * @param  无
  * @retval 无
  */
void deal_serial_data(void)
{
    static char showflag =1;
    int dec_temp=0;
    int speed_temp=0;
    
    //接收到正确的指令才为1
    char okCmd = 0;
  
    if (showflag)
    {
      show_help();
      showflag = !showflag;
    }

    //检查是否接收到指令
    if(receive_cmd == 1)
    {
      if(UART_RxBuffer[0] == 'v' || UART_RxBuffer[0] == 'V')
      {
        //设置速度
        if(UART_RxBuffer[1] == '1')
        {
          if (UART_RxBuffer[2] == ' ')
          {
            speed_temp = atoi((char const *)UART_RxBuffer+2);
            if(speed_temp>=0 && speed_temp <= MOTOR1_PWM_MAX_PERIOD_COUNT)
            {
              set_motor1_bldcm_speed(speed_temp);
							set_motor1_bldcm_enable();
              printf("\n\r电机1速度: %d\n\r", speed_temp);
              okCmd = 1;
            }
          }
        }
        //设置速度
        if(UART_RxBuffer[1] == '2')
        {
          if (UART_RxBuffer[2] == ' ')
          {
            speed_temp = atoi((char const *)UART_RxBuffer+2);
            if(speed_temp>=0 && speed_temp <= MOTOR2_PWM_MAX_PERIOD_COUNT)
            {
              set_motor2_bldcm_speed(speed_temp);
							set_motor2_bldcm_enable();
              printf("\n\r电机2速度: %d\n\r", speed_temp);
              okCmd = 1;
            }
          }
        }
      }
      else if(UART_RxBuffer[0] == 'd')
      {
        if (UART_RxBuffer[1] == '1')
        {
          //设置方向
          if(UART_RxBuffer[2] == ' ')
          {
            dec_temp = atoi((char const *)UART_RxBuffer+2);

            if(dec_temp>=0)
            {
              set_motor1_bldcm_direction(dec_temp);
              printf("\n\r电机1方向:%s\n\r", dec_temp ? "反向转" : "正向转");
              okCmd = 1;
            }
          }
        }
        if (UART_RxBuffer[1] == '2')
        {
          //设置方向
          if(UART_RxBuffer[2] == ' ')
          {
            dec_temp = atoi((char const *)UART_RxBuffer+2);

            if(dec_temp>=0)
            {
              set_motor2_bldcm_direction (dec_temp);
              printf("\n\r电机2方向:%s\n\r", dec_temp ? "反向转" : "正向转");
              okCmd = 1;
            }
          }
        }
      }
      else if(UART_RxBuffer[0] == '?')
      {
        //打印帮助命令
        show_help();
        okCmd = 1;
      }
      //如果指令有无则打印帮助命令
      if(okCmd != 1)
      {
        printf("\n\r 输入有误，请重新输入...\n\r");
        show_help();
      }

      //清空串口接收缓冲数组
      receive_cmd = 0;
      uart_FlushRxBuffer();

    }
}

/*********************************************END OF FILE**********************/
