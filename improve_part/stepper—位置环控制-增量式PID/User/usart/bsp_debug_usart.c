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
#include "./pid/bsp_pid.h"
#include "./led/bsp_led.h"
#include "./tim/bsp_basic_tim.h"
#include "./stepper/bsp_stepper_ctrl.h"

extern _pid pid;
UART_HandleTypeDef UartHandle;

//串口接收数组
unsigned char UART_RxBuffer[UART_RX_BUFFER_SIZE];
//串口接收数组指针
unsigned char UART_RxPtr;
/* 命令接收完成 */
uint8_t receive_cmd = 0;

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
  __HAL_UART_ENABLE_IT(&UartHandle, USART_IT_IDLE);
  
  HAL_UART_Receive_IT(&UartHandle, UART_RxBuffer, sizeof(UART_RxBuffer));
  
  HAL_NVIC_SetPriority(DEBUG_USART_IRQ, 1, 0);	// 抢占优先级0，子优先级1
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

//清空接收缓冲
void uart_FlushRxBuffer(void)
{
  UART_RxPtr = 0;
  UART_RxBuffer[UART_RxPtr] = 0;
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

/*********************************************END OF FILE**********************/
