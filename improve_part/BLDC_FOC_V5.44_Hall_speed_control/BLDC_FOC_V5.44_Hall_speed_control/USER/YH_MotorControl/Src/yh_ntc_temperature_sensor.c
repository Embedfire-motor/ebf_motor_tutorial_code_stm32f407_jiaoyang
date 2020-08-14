/**
  ******************************************************************************
  * @file    yh_ntc_temperature_sensor.c
  * @author  LONGZR
  * @brief   在这里实现了ST MC SDK不一样的部分，重新写温度获取函数.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 野火电子.
  * All rights reserved.</center></h2>
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "yh_ntc_temperature_sensor.h"
#include <math.h>

/** @addtogroup YH_MCSDK
  * @{
  */

/** @defgroup TemperatureSensor NTC Temperature Sensor
  * @brief Allows to read the temperature of the heat sink
  *
  * This component implements both a virtual and a real temperature sensor,
  * depending on the sensor availability.
  *
  * Access to the MCU peripherals needed to acquire the temperature (GPIO and ADC
  * used for regular conversion) is managed by the PWM component used in the Motor
  * Control subsystem. As a consequence, this NTC temperature sensor implementation
  * is hardware-independent.
  *
  * If a real temperature sensor is available (Sensor Type = #REAL_SENSOR),
  * this component can handle NTC sensors or, more generally, analog temperature sensors
  * which output is related to the temperature by the following formula:
  *
  * @f[
  *               NTC_B * NTC_T25 / (NTC_B + log(Rt / NTC_R25) * NTC_T25) - NTC_Ka
  * @f]
  *
  * In case a real temperature sensor is not available (Sensor Type = #VIRTUAL_SENSOR),
  * This component will always returns a constant, programmable, temperature.
  *
  * @{
  */

/* Private function prototypes -----------------------------------------------*/
uint16_t NTC_SetFaultState( NTC_Handle_t * pHandle );

/* Private functions ---------------------------------------------------------*/

/**
  * @brief Returns fault when temperature exceeds the over voltage protection threshold
  *
  *  @p pHandle : Pointer on Handle structure of TemperatureSensor component
  *
  *  @r Fault status : Updated internal fault status
  */
uint16_t NTC_SetFaultState( NTC_Handle_t * pHandle )
{
  uint16_t hFault;

  if ( pHandle->hAvTemp_d > YH_OV_TEMPERATURE_THRESHOLD_d )
  {
    hFault = MC_OVER_TEMP;
  }
  else if ( pHandle->hAvTemp_d < YH_OV_TEMPERATURE_HYSTERESIS_d )
  {
    hFault = MC_NO_ERROR;
  }
  else
  {
    hFault = pHandle->hFaultState;
  }
  return hFault;
}

/* Functions ---------------------------------------------------- */

/**
  * @brief Performs the temperature sensing average computation after an ADC conversion
  *
  *  @p pHandle : Pointer on Handle structure of TemperatureSensor component
  *
  *  @r Fault status : Error reported in case of an over temperature detection
  */
uint16_t NTC_CalcAvTemp( NTC_Handle_t * pHandle )
{
  uint32_t wtemp;
  uint16_t hAux;

  if ( pHandle->bSensorType == REAL_SENSOR )
  {
    hAux = RCM_ExecRegularConv(pHandle->convHandle);

    if ( hAux != 0xFFFFu )
    {
      wtemp =  ( uint32_t )( pHandle->hLowPassFilterBW ) - 1u;
      wtemp *= ( uint32_t ) ( pHandle->hAvTemp_d );
      wtemp += hAux;
      wtemp /= ( uint32_t )( pHandle->hLowPassFilterBW );

      pHandle->hAvTemp_d = ( uint16_t ) wtemp;
    }

    pHandle->hFaultState = NTC_SetFaultState( pHandle );
  }
  else  /* case VIRTUAL_SENSOR */
  {
    pHandle->hFaultState = MC_NO_ERROR;
  }

  return ( pHandle->hFaultState );
}

/**
  * @brief  获取温度传感器端的电压值
  * @param  无
  * @retval 转换得到的电压值
  */
float get_ntc_v_val(NTC_Handle_t * pHandle)
{
  float vdc = GET_ADC_VDC_VAL(pHandle->hAvTemp_d);      // 获取电压值
  
  return vdc;
}

/**
  * @brief  获取温度传感器端的电阻值
  * @param  无
  * @retval 转换得到的电阻值
  */
float get_ntc_r_val(NTC_Handle_t * pHandle)
{
  float r = 0;
  float vdc = get_ntc_v_val(pHandle);
  
  r = ((float)ADC_REFERENCE_VOLTAGE - vdc) / (vdc / DICIDER_RESISTANCE);
  
  return r;
}

/**
  * @brief  Returns latest averaged temperature expressed in Celsius degrees
  *
  * @p pHandle : Pointer on Handle structure of TemperatureSensor component
  *
  * @r AverageTemperature : Latest averaged temperature measured (in Celsius degrees)
  */
int16_t NTC_GetAvTemp_C( NTC_Handle_t * pHandle )
{
  int32_t wTemp;

  if ( pHandle->bSensorType == REAL_SENSOR )
  {
    float Rt = 0;                   // 测量电阻
    Rt = get_ntc_r_val(pHandle);    // 获取当前电阻值

    wTemp = NTC_B * NTC_T25 / (NTC_B + log(Rt / NTC_R25) * NTC_T25) - NTC_Ka ;    // 使用公式计算
  }
  else
  {
    wTemp = pHandle->hExpectedTemp_C;
  }
  return ( ( int16_t )wTemp );
}

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT 2019 STMicroelectronics *****END OF FILE****/
