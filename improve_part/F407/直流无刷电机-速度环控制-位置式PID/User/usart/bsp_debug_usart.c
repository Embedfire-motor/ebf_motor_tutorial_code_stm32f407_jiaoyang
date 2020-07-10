/**
  ******************************************************************************
  * @file    bsp_debug_usart.c
  * @author  fire
  * @version V1.0
  * @date    2016-xx-xx
  * @brief   使用串口1，重定向c库printf函数到usart端口，中断接收模式
  ******************************************************************************
  * @attention
  *
  * 实验平台:野火 STM32 F407 开发板  
  * 论坛    :http://www.firebbs.cn
  * 淘宝    :http://firestm32.taobao.com
  *
  ******************************************************************************
  */ 
  
#include "./usart/bsp_debug_usart.h"
#include "./led/bsp_led.h"

UART_HandleTypeDef UartHandle;

 /**
  * @brief  DEBUG_USART GPIO 配置,工作模式配置。115200 8-N-1
  * @param  无
  * @retval 无
  */  
void DEBUG_USART_Config(void)
{ 
  
  UartHandle.Instance          = DEBUG_USART;
  
  UartHandle.Init.BaudRate     = DEBUG_USART_BAUDRATE;
  UartHandle.Init.WordLength   = USART_WORDLENGTH_8B;
  UartHandle.Init.StopBits     = USART_STOPBITS_1;
  UartHandle.Init.Parity       = USART_PARITY_NONE;
//  UartHandle.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
  UartHandle.Init.Mode         = USART_MODE_TX_RX;
  
  HAL_UART_Init(&UartHandle);
    
  /* 使能串口空闲断 */
  __HAL_UART_ENABLE_IT(&UartHandle, USART_IT_RXNE);
  
  HAL_NVIC_SetPriority(DEBUG_USART_IRQ, 0, 0);	// 抢占优先级0，子优先级1
  HAL_NVIC_EnableIRQ(DEBUG_USART_IRQ );		      // 使能USART1中断通道 
}


/**
  * @brief UART MSP 初始化 
  * @param huart: UART handle
  * @retval 无
  */
void HAL_UART_MspInit(UART_HandleTypeDef *huart)
{  
  GPIO_InitTypeDef  GPIO_InitStruct;
  
  DEBUG_USART_CLK_ENABLE();
	
	DEBUG_USART_RX_GPIO_CLK_ENABLE();
  DEBUG_USART_TX_GPIO_CLK_ENABLE();
  
/**USART1 GPIO Configuration    
  PA9     ------> USART1_TX
  PA10    ------> USART1_RX 
  */
  /* 配置Tx引脚为复用功能  */
  GPIO_InitStruct.Pin = DEBUG_USART_TX_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = DEBUG_USART_TX_AF;
  HAL_GPIO_Init(DEBUG_USART_TX_GPIO_PORT, &GPIO_InitStruct);
  
  /* 配置Rx引脚为复用功能 */
  GPIO_InitStruct.Pin = DEBUG_USART_RX_PIN;
  GPIO_InitStruct.Alternate = DEBUG_USART_RX_AF;
  HAL_GPIO_Init(DEBUG_USART_RX_GPIO_PORT, &GPIO_InitStruct); 
}

/*****************  发送字符 **********************/
void Usart_SendByte(uint8_t str)
{
  HAL_UART_Transmit(&UartHandle, &str, 1, 1000);
}

/*****************  发送字符串 **********************/
void Usart_SendString(uint8_t *str)
{
	unsigned int k=0;
  do 
  {
      HAL_UART_Transmit(&UartHandle,(uint8_t *)(str + k) ,1,1000);
      k++;
  } while(*(str + k)!='\0');
}

///重定向c库函数printf到串口DEBUG_USART，重定向后可使用printf函数
int fputc(int ch, FILE *f)
{
	/* 发送一个字节数据到串口DEBUG_USART */
	HAL_UART_Transmit(&UartHandle, (uint8_t *)&ch, 1, 1000);	
	
	return (ch);
}

///重定向c库函数scanf到串口DEBUG_USART，重写向后可使用scanf、getchar等函数
int fgetc(FILE *f)
{
		
	int ch;
	HAL_UART_Receive(&UartHandle, (uint8_t *)&ch, 1, 1000);	
	return (ch);
}

///**
//  * @brief 解析串口接收到的数据
//  * @param cmd：命令
//  * @param ch: 曲线通道
//  * @param data：数据
//  * @retval 无
//  */
//void HAL_UART_AbortReceiveCpltCallback(UART_HandleTypeDef *husart)
//{
//  packet_head_t packet;
//    
//  packet.cmd = UART_RxBuffer[CMD_INDEX_VAL];
//  packet.len  = COMPOUND_32BIT(&UART_RxBuffer[LEN_INDEX_VAL]);     // 合成长度
//  packet.head = COMPOUND_32BIT(&UART_RxBuffer[HEAD_INDEX_VAL]);    // 合成包头
//  
//  if (packet.head == PACKET_HEAD)    // 检查包头
//  {
//    /* 包头正确 */
//    if (check_sum(0, UART_RxBuffer, packet.len - 1) == UART_RxBuffer[packet.len - 1])    // 检查校验和是否正确
//    {
//      switch(packet.cmd)
//      {
//        case SET_P_I_D_CMD:
//        {
//          uint32_t temp0 = COMPOUND_32BIT(&UART_RxBuffer[13]);
//          uint32_t temp1 = COMPOUND_32BIT(&UART_RxBuffer[17]);
//          uint32_t temp2 = COMPOUND_32BIT(&UART_RxBuffer[21]);
//          
//          float p_temp, i_temp, d_temp;
//          
//          p_temp = *(float *)&temp0;
//          i_temp = *(float *)&temp1;
//          d_temp = *(float *)&temp2;
//          
//          set_p_i_d(p_temp, i_temp, d_temp);    // 设置 P I D
//        }
//        break;

//        case SET_TARGET_CMD:
//        {
//          int actual_temp = COMPOUND_32BIT(&UART_RxBuffer[13]);    // 得到数据
//          
//          set_pid_target(actual_temp);    // 设置目标值
//        }
//        break;
//        
//        case START_CMD:
//        {
//          set_motor_enable();              // 启动电机
//        }
//        break;
//        
//        case STOP_CMD:
//        {
//          set_motor_disable();              // 停止电机
//        }
//        break;
//        
//        case RESET_CMD:
//        {
//          HAL_NVIC_SystemReset();          // 复位系统
//        }
//        break;
//        
//        case SET_PERIOD_CMD:
//        {
//          uint32_t temp = COMPOUND_32BIT(&UART_RxBuffer[13]);     // 周期数
//          SET_BASIC_TIM_PERIOD(temp);                             // 设置定时器周期1~1000ms
//        }
//        break;
//      }
//    }
//  }
//}

/*********************************************END OF FILE**********************/
