/**
  ******************************************************************************
  * @file    yh_bus_voltage_sensor.h
  * @author  LONGZR
  * @brief   This file contains all definitions and functions prototypes for the
  *          BusVoltageSensor component of the Motor Control SDK, 
  *          在这里实现了ST MC SDK不一样的部分，重新写电压获取函数.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 野火电子.
  * All rights reserved.</center></h2>
  *
  ******************************************************************************
  * @ingroup YHBusVoltageSensor
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __YH_BUSVOLTAGESENSOR_H
#define __YH_BUSVOLTAGESENSOR_H

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/* Includes ------------------------------------------------------------------*/
#include "bus_voltage_sensor.h"
#include "drive_parameters.h"
#include "parameters_conversion.h"

/** @addtogroup MCSDK
  * @{
  */

/** @addtogroup BusVoltageSensor
  * @{
  */

/**
  * @brief  BusVoltageSensor handle definition
  */

/* Exported functions ------------------------------------------------------- */
uint16_t VBS_GetBusVoltage_d( BusVoltageSensor_Handle_t * pHandle );
uint16_t VBS_GetAvBusVoltage_d( BusVoltageSensor_Handle_t * pHandle );
uint16_t VBS_GetAvBusVoltage_V( BusVoltageSensor_Handle_t * pHandle );
uint16_t VBS_CheckVbus( BusVoltageSensor_Handle_t * pHandle );

/**
  * @}
  */

/**
  * @}
  */

#ifdef __cplusplus
}
#endif /* __cpluplus */

#endif /* __YH_BusVoltageSensor_H */

/************************ (C) COPYRIGHT 2019 STMicroelectronics *****END OF FILE****/
