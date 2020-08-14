/**
  ******************************************************************************
  * @file    yh_r_divider_bus_voltage_sensor.h
  * @author  LONGZR
  * @brief   This file contains all definitions and functions prototypes for the
  *          Resistor Divider Bus Voltage Sensor component of the Motor Control SDK.
  *          在这里实现了ST MC SDK不一样的部分，重新写电压检查函数.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 野火电子.
  * All rights reserved.</center></h2>
  *
  ******************************************************************************
  * @ingroup RDividerBusVoltageSensor
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __YH_RDIVIDER_BUSVOLTAGESENSOR_H
#define __YH_RDIVIDER_BUSVOLTAGESENSOR_H

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/* Includes ------------------------------------------------------------------*/
#include "regular_conversion_manager.h"
#include "bus_voltage_sensor.h"
#include "r_divider_bus_voltage_sensor.h"
#include "drive_parameters.h"
#include "parameters_conversion.h"

/** @addtogroup MCSDK
  * @{
  */

/** @addtogroup BusVoltageSensor
  * @{
  */

#define VBUS_MAGNIFICATION_TIMES     37.0    // 电压放大倍数
#define VBUS_VBIAS                   1.24    // 偏置电压

#define YH_OVERVOLTAGE_THRESHOLD_d   (uint16_t)((OV_VOLTAGE_THRESHOLD_V / VBUS_MAGNIFICATION_TIMES + VBUS_VBIAS)/\
                                                ADC_REFERENCE_VOLTAGE * 65535)
#define YH_UNDERVOLTAGE_THRESHOLD_d  (uint16_t)((UD_VOLTAGE_THRESHOLD_V / VBUS_MAGNIFICATION_TIMES + VBUS_VBIAS)/\
                                                ADC_REFERENCE_VOLTAGE * 65535)

/** @addtogroup RDividerBusVoltageSensor
  * @{
  */


/**
  * @brief  Rdivider class parameters definition
  */

/* Exported functions ------------------------------------------------------- */
void RVBS_Clear( RDivider_Handle_t * pHandle );
uint16_t RVBS_CheckFaultState( RDivider_Handle_t * pHandle );

/**
  * @}
  */

/**
  * @}
  */

/** @} */

#ifdef __cplusplus
}
#endif /* __cpluplus */

#endif /* __YH_RDividerBusVoltageSensor_H */

/************************ (C) COPYRIGHT 2019 STMicroelectronics *****END OF FILE****/

