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
    
  /*使能串口接收断 */
//  __HAL_UART_ENABLE_IT(&UartHandle, USART_IT_RXNE);
//  HAL_USART_Receive_IT(&UartHandle, &data, sizeof(data));
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
 
//  HAL_NVIC_SetPriority(DEBUG_USART_IRQ ,5,0);	//抢占优先级0，子优先级1
//  HAL_NVIC_EnableIRQ(DEBUG_USART_IRQ );		    //使能USART1中断通道  
}

//清空接收缓冲
void uart_FlushRxBuffer(void)
{
  UART_RxPtr = 0;
  UART_RxBuffer[UART_RxPtr] = 0;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
//	if(__HAL_UART_GET_IT_SOURCE(&UartHandle, UART_IT_RXNE) != RESET)
//	{
//    //如果为退格键
//    if(data == '\b')
//    {
//      //如果指针不在数组的开始位置
//      if(UART_RxPtr)
//      {
//        Usart_SendByte('\b');
//        Usart_SendByte(' ');
//        Usart_SendByte('\b');
//        UART_RxPtr--;
//        UART_RxBuffer[UART_RxPtr]=0x00;
//      }
//    }
//    //如果不是退格键
//    else
//    {
//      //将数据填入数组UART_RxBuffer
//      //并且将后面的一个元素清零如果数组满了则写入最后一个元素为止
//      if(UART_RxPtr < (UART_RX_BUFFER_SIZE - 1))
//      {
//        UART_RxBuffer[UART_RxPtr] = data;
//        UART_RxBuffer[UART_RxPtr + 1]=0x00;
//        UART_RxPtr++;
//      }
//      else
//      {
//        UART_RxBuffer[UART_RxPtr - 1] = data;
//        Usart_SendByte('\b');
//      }
//      //如果为回车键，则开始处理串口数据
//      if(data == 13 || data == 10)
//      {
//        receive_cmd = 1;
//      }
//      else
//      {
//        Usart_SendByte(data);
//      }
//    }
//  }
//  
//  HAL_USART_Receive_IT(&UartHandle, &data, sizeof(data));
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

/**
  * @brief 计算校验和
  * @param ptr：需要计算的数据
  * @param len：需要计算的长度
  * @retval 校验和
  */
uint8_t check_sum(uint8_t init, uint8_t *ptr, uint8_t len )
{
  uint8_t sum = init;
  
  while(len--)
  {
    sum += *ptr;
    ptr++;
  }
  
  return sum;
}

/**
  * @brief 设置上位机的值
  * @param cmd：命令
  * @param ch: 曲线通道
  * @param data：参数指针
  * @param num：参数个数
  * @retval 无
  */
void set_computer_value(uint8_t cmd, uint8_t ch, void *data, uint8_t num)
{
  uint8_t sum = 0;    // 校验和
  num *= 4;           // 一个参数 4 个字节
  
  static packet_head_t set_packet;
  
  set_packet.head = PACKET_HEAD;     // 包头 0x59485A53
  set_packet.len  = 0x0B + num;      // 包长
  set_packet.ch   = ch;              // 设置通道
  set_packet.cmd  = cmd;             // 设置命令
  
  sum = check_sum(0, (uint8_t *)&set_packet, sizeof(set_packet));       // 计算包头校验和
  sum = check_sum(sum, (uint8_t *)data, num);                           // 计算参数校验和
  
  HAL_UART_Transmit(&UartHandle, (uint8_t *)&set_packet, sizeof(set_packet), 0xFFFFF);    // 发送数据头
  HAL_UART_Transmit(&UartHandle, (uint8_t *)data, num, 0xFFFFF);                          // 发送参数
  HAL_UART_Transmit(&UartHandle, (uint8_t *)&sum, sizeof(sum), 0xFFFFF);                  // 发送校验和
}

/*********************************************END OF FILE**********************/
