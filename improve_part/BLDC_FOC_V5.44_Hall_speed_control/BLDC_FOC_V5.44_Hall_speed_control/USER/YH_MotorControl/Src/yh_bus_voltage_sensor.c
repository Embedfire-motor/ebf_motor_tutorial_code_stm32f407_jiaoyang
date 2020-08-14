/**
  ******************************************************************************
  * @file    yh_bus_voltage_sensor.c
  * @author  LONGZR
  * @brief   在这里实现了ST MC SDK不一样的部分，重新写电压获取函数.
  *
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 野火电子.
  * All rights reserved.</center></h2>
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "r_divider_bus_voltage_sensor.h"
#include "yh_r_divider_bus_voltage_sensor.h"
#include "yh_bus_voltage_sensor.h"


/** @addtogroup YH_MCSDK
  * @{
  */

/** @defgroup BusVoltageSensor Bus Voltage Sensor
  * @brief Bus Voltage Sensor components of the Motor Control SDK
  *
  * Two Bus Voltage Sensor implementations are provided:
  *
  * - The @ref RDividerBusVoltageSensor "Resistor Divider Bus Voltage Sensor" operates as the name suggests
  * - The @ref VirtualBusVoltageSensor "Virtual Bus Voltage Sensor" does not make measurement but rather
  *   returns a fixed, application defined value.
  *
  * @todo Document the Bus Voltage Sensor "module".
  *
  * @{
  */

#if defined (CCMRAM)
#if defined (__ICCARM__)
#pragma location = ".ccmram"
#elif defined (__CC_ARM) || defined(__GNUC__)
__attribute__( ( section ( ".ccmram" ) ) )
#endif
#endif

/**
  * @brief  It return latest averaged Vbus measurement expressed in Volts
  * @param  pHandle related Handle of BusVoltageSensor_Handle_t
  * @retval uint16_t Latest averaged Vbus measurement in Volts
  */
uint16_t VBS_GetAvBusVoltage_V( BusVoltageSensor_Handle_t * pHandle )
{
  uint16_t temp;

  temp = (pHandle->AvBusVoltage_d / 65536.0f * (float)ADC_REFERENCE_VOLTAGE - (float)VBUS_VBIAS)\
         * (float)VBUS_MAGNIFICATION_TIMES;

  return temp;
}

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT 2019 STMicroelectronics *****END OF FILE****/
