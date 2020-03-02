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
//extern uint8_t ucTemp;  

 /**
  * @brief  DEBUG_USART GPIO 配置,工作模式配置。115200 8-N-1
  * @param  无
  * @retval 无
  */  
void DEBUG_USART_Config(void)
{ 
  
  UartHandle.Instance          = DEBUG_USART;
  
  UartHandle.Init.BaudRate     = DEBUG_USART_BAUDRATE;
  UartHandle.Init.WordLength   = UART_WORDLENGTH_8B;
  UartHandle.Init.StopBits     = UART_STOPBITS_1;
  UartHandle.Init.Parity       = UART_PARITY_NONE;
  UartHandle.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
  UartHandle.Init.Mode         = UART_MODE_TX_RX;
  
  HAL_UART_Init(&UartHandle);
    
 /*使能串口接收断 */
  __HAL_UART_ENABLE_IT(&UartHandle,UART_IT_RXNE);  
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
 
  HAL_NVIC_SetPriority(DEBUG_USART_IRQ ,0,1);	//抢占优先级0，子优先级1
  HAL_NVIC_EnableIRQ(DEBUG_USART_IRQ );		    //使能USART1中断通道  
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

/* 数据格式化转换 */
typedef union {
  char Ch[4];
  float Float;
  int32_t Int;
}Format_UnionTypedef;

typedef struct {
  __IO uint8_t  Code ;  	
  __IO Format_UnionTypedef data[3];//数据帧有3个参数
}MSG_TypeDef;

// 协议相关定义
#define FRAME_LENTH               16    // 指令长度
#define FRAME_START               0xAA  // 协议帧开始
#define FRAME_END                 '/'   // 协议帧结束
#define FRAME_CHECK_BEGIN          1    // 校验码开始的位置 RxBuf[1]
#define FRAME_CHECKSUM            14    // 校验码的位置   RxBuf[14]
#define FRAME_CHECK_NUM           13    // 需要校验的字节数
#define FILL_VALUE                0x55  // 填充值
#define CODE_SETPID               0x07  // 设置PID参数
#define CODE_SETTGT               0x08  // 设置目标值
#define CODE_RESET                0x09   // 复位重启
#define CODE_STARTMOTOR           0x0A   // 启动电机

__IO uint8_t RxBuf[FRAME_LENTH] ; // 接收缓存区
__IO uint8_t TxBuf[FRAME_LENTH] ; // 发送缓存区
/* 扩展变量 ------------------------------------------------------------------*/
MSG_TypeDef Msg;

uint8_t CheckSum(uint8_t *Ptr,uint8_t Num )
{
  uint8_t Sum = 0;
  while(Num--)
  {
    Sum += *Ptr;
    Ptr++;
  }
  return Sum;
}

/**
  * 函数功能: 串口接收中断回调函数
  * 输入参数: huart：串口句柄指针
  * 返 回 值: 无
  * 说    明: 按固定字长接收数据帧,同时验证帧头帧尾和校验码
  */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if(huart == &UartHandle)
  {
    if(RxBuf[0] != FRAME_START )    // 帧头正确
    {
      return;
    }
    if(RxBuf[FRAME_LENTH-1] == FRAME_END ) // 帧尾正确
    { 
      /* 判断校验码 */
      if(CheckSum((uint8_t*)&RxBuf[FRAME_CHECK_BEGIN],FRAME_CHECK_NUM) != RxBuf[FRAME_CHECKSUM] )
      {
        Msg.Code = NULL;
        return;
      }
      else
      {
        /* 解析数据帧 */
//        ptr_Fun_();
      }
    }
    HAL_UART_Receive_IT(huart,(uint8_t *)&RxBuf,FRAME_LENTH); // 重新使能接收中断
  }
}
/**
  * 函数功能: 发送反馈值
  * 输入参数: 无
  * 返 回 值: 无
  * 说    明: 将反馈值发送到串口
  */
void Transmit_FB( __IO int32_t *Feedback)
{
  uint8_t i = 0;
  for(i=0;i<FRAME_LENTH;i++)
  {
    TxBuf[i] = FILL_VALUE;  // 参数填充 0x55
  }
  
  Msg.data[0].Int = *Feedback;//反馈值 速度
  
  TxBuf[0] = FRAME_START;   // 帧头
  TxBuf[1] = 0x80|CODE_SETTGT; // 指令码
  TxBuf[2] = Msg.data[0].Ch[0];
  TxBuf[3] = Msg.data[0].Ch[1];
  TxBuf[4] = Msg.data[0].Ch[2];
  TxBuf[5] = Msg.data[0].Ch[3];
  
  TxBuf[FRAME_CHECKSUM] = CheckSum((uint8_t*)&TxBuf[FRAME_CHECK_BEGIN],FRAME_CHECK_NUM);  // 计算校验和
  TxBuf[FRAME_LENTH-1] = FRAME_END;   // 加入帧尾

  HAL_UART_Transmit_IT(&UartHandle,(uint8_t *)&TxBuf,FRAME_LENTH); // 发送数据帧
}
/*********************************************END OF FILE**********************/
