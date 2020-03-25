#ifndef __BSP_ADC_H
#define	__BSP_ADC_H

#include "stm32f4xx.h"

// ADC GPIO 宏定义
#define CURR_ADC_GPIO_PORT              GPIOB
#define CURR_ADC_GPIO_PIN               GPIO_PIN_1
#define CURR_ADC_GPIO_CLK_ENABLE()      __GPIOB_CLK_ENABLE()
    
// ADC 序号宏定义
#define CURR_ADC                        ADC1
#define CURR_ADC_CLK_ENABLE()           __ADC1_CLK_ENABLE()
#define CURR_ADC_CHANNEL                ADC_CHANNEL_9

// ADC DR寄存器宏定义，ADC转换后的数字值则存放在这里
#define CURR_ADC_DR_ADDR                ((uint32_t)ADC1+0x4c)

// ADC DMA 通道宏定义，这里我们使用DMA传输
#define CURR_ADC_DMA_CLK_ENABLE()       __DMA2_CLK_ENABLE()
#define CURR_ADC_DMA_CHANNEL            DMA_CHANNEL_0
#define CURR_ADC_DMA_STREAM             DMA2_Stream0

#define ADC_DMA_IRQ                DMA2_Stream0_IRQn
#define ADC_DMA_IRQ_Handler        DMA2_Stream0_IRQHandler

#define ADC_NUM_MAX           1024    // ADC 转换结果缓冲区最大值

#define VREF                  3.259f     // 参考电压，理论上是3.3，可通过实际测量得3.259

#define GET_ADC_VDC_VAL(val)     ((float)val/(float)4096*VREF)          // 得到电压值
#define GET_ADC_CURR_VAL(val)    ((float)val/10.0/0.02*1000.0)          // 得到电流值，电压放大10倍，0.02是采样电阻，单位mA。

//#define GET_ADC_CURR_VAL(val)    ((val)/10.02/0.02*1000.0)
  
extern DMA_HandleTypeDef DMA_Init_Handle;

void CURR_ADC_Init(void);
int32_t get_curr_val(void);

#endif /* __BSP_ADC_H */



