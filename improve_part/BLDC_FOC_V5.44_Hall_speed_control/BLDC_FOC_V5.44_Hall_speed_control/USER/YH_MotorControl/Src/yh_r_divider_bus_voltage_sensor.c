/**
  ******************************************************************************
  * @file    yh_r_divider_bus_voltage_sensor.c
  * @author  LONGZR
  * @brief   在这里实现了ST MC SDK不一样的部分，重新写电压检查函数.
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
#include "yh_r_divider_bus_voltage_sensor.h"
#include "regular_conversion_manager.h"

/** @addtogroup YH_MCSDK
  * @{
  */

/** @addtogroup BusVoltageSensor
  * @{
  */

/** @defgroup RDividerBusVoltageSensor Resistor Divider Bus Voltage Sensor
  * @brief Resistor Divider Bus Voltage Sensor implementation
  *
  * @todo Document the Resistor Divider Bus Voltage Sensor "module".
  *
  * @{
  */

/**
  * @brief  It clears bus voltage FW variable containing average bus voltage
  *         value
  * @param  pHandle related RDivider_Handle_t
  * @retval none
  */
void RVBS_Clear( RDivider_Handle_t * pHandle )
{
  uint16_t aux;
  uint16_t index;

  aux = ( YH_OVERVOLTAGE_THRESHOLD_d + YH_UNDERVOLTAGE_THRESHOLD_d ) / 2u;
  for ( index = 0u; index < pHandle->LowPassFilterBW; index++ )
  {
    pHandle->aBuffer[index] = aux;
  }
  pHandle->_Super.LatestConv = aux;
  pHandle->_Super.AvBusVoltage_d = aux;
  pHandle->index = 0;
}

/**
  * @brief  It returns MC_OVER_VOLT, MC_UNDER_VOLT or MC_NO_ERROR depending on
  *         bus voltage and protection threshold values
  * @param  pHandle related RDivider_Handle_t
  * @retval uint16_t Fault code error
  */
uint16_t RVBS_CheckFaultState( RDivider_Handle_t * pHandle )
{
  uint16_t fault;

  if ( pHandle->_Super.AvBusVoltage_d > YH_OVERVOLTAGE_THRESHOLD_d )
  {
    fault = MC_OVER_VOLT;
  }
  else if ( pHandle->_Super.AvBusVoltage_d < YH_UNDERVOLTAGE_THRESHOLD_d )
  {
    fault = MC_UNDER_VOLT;
  }
  else
  {
    fault = MC_NO_ERROR;
  }
  return fault;
}


/**
  * @}
  */

/**
  * @}
  */

/** @} */

/************************ (C) COPYRIGHT 2019 STMicroelectronics *****END OF FILE****/

