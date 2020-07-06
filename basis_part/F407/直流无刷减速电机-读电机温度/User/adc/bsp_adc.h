#ifndef __BSP_ADC_H
#define	__BSP_ADC_H

#include "stm32f4xx.h"

// ADC 序号宏定义
#define CURR_ADC                        ADC3
#define CURR_ADC_CLK_ENABLE()           __ADC3_CLK_ENABLE()

#define ADC_VBUS_IRQ                    ADC_IRQn
#define ADC_VBUS_IRQHandler             ADC_IRQHandler

#define VREF                            3.3f     // 参考电压，理论上是3.3，可通过实际测量得3.258
#define ADC_NUM_MAX                     2048       // ADC 转换结果缓冲区最大值

#define GET_ADC_VDC_VAL(val)            ((float)val/(float)4096.0*VREF)          // 得到电压值
  
/*********************** 温度传感器电压采集 ******************/
// ADC GPIO 宏定义
#define CURR_ADC_GPIO_PORT              GPIOF
#define CURR_ADC_GPIO_PIN               GPIO_PIN_10
#define CURR_ADC_GPIO_CLK_ENABLE()      __GPIOB_CLK_ENABLE()

#define CURR_ADC_CHANNEL                ADC_CHANNEL_8

// ADC DR寄存器宏定义，ADC转换后的数字值则存放在这里
#define CURR_ADC_DR_ADDR                ((uint32_t)ADC1+0x4c)

// ADC DMA 通道宏定义，这里我们使用DMA传输
#define CURR_ADC_DMA_CLK_ENABLE()       __DMA2_CLK_ENABLE()
#define CURR_ADC_DMA_CHANNEL            DMA_CHANNEL_2
#define CURR_ADC_DMA_STREAM             DMA2_Stream0

#define ADC_DMA_IRQ                     DMA2_Stream0_IRQn
#define ADC_DMA_IRQ_Handler             DMA2_Stream0_IRQHandler

/*********************** 电源电压采集 ******************/

#define VBUS_GPIO_PORT                  GPIOF
#define VBUS_GPIO_PIN                   GPIO_PIN_9
#define VBUS_GPIO_CLK_ENABLE()          __GPIOF_CLK_ENABLE()

#define VBUS_ADC_CHANNEL                ADC_CHANNEL_7

#define GET_VBUS_VAL(val)               (((float)val-(float)0.5) / (float)8.0 * (float)301.0)      // 电压最大值（测量电压是电源电压的1/500）

extern DMA_HandleTypeDef DMA_Init_Handle;
extern ADC_HandleTypeDef ADC_Handle;

void ADC_Init(void);
float get_ntc_v_val(void);
float get_ntc_r_val(void);
float get_ntc_t_val(void);
float get_vbus_val(void);
 
#endif /* __BSP_ADC_H */
