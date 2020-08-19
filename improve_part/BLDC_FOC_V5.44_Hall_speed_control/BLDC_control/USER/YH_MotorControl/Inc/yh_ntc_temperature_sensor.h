/**
  ******************************************************************************
  * @file    yh_ntc_temperature_sensor.h
  * @author  LONGZR
  * @brief   This file contains all definitions and functions prototypes for the
  *          Temperature Sensor component of the Motor Control SDK.
  *          在这里实现了ST MC SDK不一样的部分，重新写温度获取函数.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 野火电子.
  * All rights reserved.</center></h2>
  *
  ******************************************************************************
  * @ingroup TemperatureSensor
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __YH_TEMPERATURESENSOR_H
#define __YH_TEMPERATURESENSOR_H

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/* Includes ------------------------------------------------------------------*/
#include "ntc_temperature_sensor.h"
#include "parameters_conversion.h"

/** @addtogroup MCSDK
  * @{
  */

/** @addtogroup TemperatureSensor
  * @{
  */
  
#define GET_ADC_VDC_VAL(val)            ((float)val / 65536.0f * (float)ADC_REFERENCE_VOLTAGE)          // 得到电压值
  
/* 参数宏 */
#define DICIDER_RESISTANCE    (4700.0f)    // 分压电阻阻值

#define NTC_Ka     (273.15f)         // 0℃ 时对应的温度（开尔文）
#define NTC_R25    (10000.0f)        // 25℃ 电阻值
#define NTC_T25    (NTC_Ka + 25)     // 25℃ 时对应的温度（开尔文）
#define NTC_B      (3950.0f)         /* B-常数：B = ln(R25 / Rt) / (1 / T – 1 / T25)，
                                        其中 T = 25 + 273.15 */

/* 温度保护阈值与清除报警值,
计算公式（_d = 3.3 * R / (Rt + R) / ADC_REFERENCE_VOLTAGE * 65536） */
#define YH_OV_TEMPERATURE_THRESHOLD_d      51837    /*!< 通过查表80℃对应的电阻值计算得出 */
#define YH_OV_TEMPERATURE_HYSTERESIS_d     47866    /*!< 通过查表70℃对应的电阻值计算得出 */

/**
  * @brief NTC_Handle_t structure used for temperature monitoring
  *
  */

/* Temperature sensing computation */
uint16_t NTC_CalcAvTemp( NTC_Handle_t * pHandle );

/* Get averaged temperature measurement expressed in Celsius degrees */
int16_t NTC_GetAvTemp_C( NTC_Handle_t * pHandle );

/**
  * @}
  */

/**
  * @}
  */

#ifdef __cplusplus
}
#endif /* __cpluplus */

#endif /* __YH_TEMPERATURESENSOR_H */

/************************ (C) COPYRIGHT 2019 STMicroelectronics *****END OF FILE****/
