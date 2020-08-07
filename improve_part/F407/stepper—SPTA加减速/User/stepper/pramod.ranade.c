/*
	step.c
	Program to verify new algorithm for linear acceleration.
	Author: Pramod Ranade <pramod.ranade@spjsystems.com>
*/

#include <stdio.h>
#include <stdlib.h>
#include "./stepper/bsp_stepper_init.h"

#define SCAN_TIME_IN_MICRO_SEC	  	1
#define COUNT_FOR_COMPARISON		(1000000000UL / SCAN_TIME_IN_MICRO_SEC)

#define	SPEED_LIMIT				20000
#define	ACC_LIMIT					900000
#define	START_SPEED					100
#define	DES_POS						400*15
#define AVAILABLE_TIME_IN_MILI_SEC	2000

int OutputPinStatus = 1 ;
int g_nRisingEdgeCount = 0 ;


#define	MakeOutputHigh()	MOTOR_PUL(HIGH)
#define	MakeOutputLow()		MOTOR_PUL(LOW)

int FpgaRunMotor (unsigned int uiDesPos, unsigned int uiStartSpeedX1K, unsigned int uiDeltaSpeedX1K,
                  unsigned int uiTimeForAccInMicroSec, unsigned int uiTimeToStartDeccInMicroSec, unsigned int uiPeakSpeed)
{
/*
	This function performs the actions which must be done by FPGA.
	So, finally this should be converted to VHDL or Verilog code for the FPGA.
*/
	unsigned int uiDoneSteps ;
	unsigned int uiCurSpeedX1K ;
	unsigned int uiSpeedTimeProduct ;
	unsigned int uiCurTimeInMicroSec ;

	uiDoneSteps = 0 ;
	uiCurSpeedX1K = uiStartSpeedX1K ;
	uiDoneSteps ++ ;
	uiSpeedTimeProduct = uiCurSpeedX1K ;
	uiCurTimeInMicroSec = 0 ;
	MakeOutputHigh() ;	// first pulse output
	while(uiDoneSteps < uiDesPos)
	{
		// execute this loop after every N micro-seconds, where N = SCAN_TIME_IN_MICRO_SEC
		uiCurTimeInMicroSec += SCAN_TIME_IN_MICRO_SEC ;
		uiSpeedTimeProduct += uiCurSpeedX1K ;
		if (uiCurTimeInMicroSec >= uiTimeToStartDeccInMicroSec)
		{
			// it means deceleration is going on
			if (uiCurSpeedX1K > uiDeltaSpeedX1K)
			{
				uiCurSpeedX1K -= uiDeltaSpeedX1K ;
			}
			else
			{
				uiSpeedTimeProduct = COUNT_FOR_COMPARISON ;
			}
		}
		else if (uiCurTimeInMicroSec < uiTimeForAccInMicroSec)
		{
			// it means acceleration is going on
			uiCurSpeedX1K += uiDeltaSpeedX1K ;
		}
		else
		{
			// it means steady speed (plateau)
			uiCurSpeedX1K = uiPeakSpeed ;
		}
		if (uiSpeedTimeProduct >= COUNT_FOR_COMPARISON)
		{
			uiDoneSteps ++ ;
			MakeOutputLow() ;	// next pulse output
			uiSpeedTimeProduct -= COUNT_FOR_COMPARISON ;
		}
		else if (uiSpeedTimeProduct >= (COUNT_FOR_COMPARISON / 2))
		{
			MakeOutputHigh() ;
			g_nRisingEdgeCount ++ ;
		}
	}
	return 0 ;
}
int RunMotor (unsigned int uiStartSpeed, unsigned int uiAvailableTimeInMicroSec, int nDesPos, unsigned int uiSpeedLimit, unsigned int uiAccLimit)
{
/*
	This function performs the calculations which must be done by microprocessor.
	Finally, it calls another function FpgaRunMotor()
	i.e. it passes some parameters to FPGA and tells it to start motion.
*/
	unsigned int uiAverageSpeed ;
	unsigned int uiTimeForAccInMicroSec ;
	unsigned int uiTimeToStartDeccInMicroSec, uiRequiredAcc ;
	unsigned int uiPeakSpeed, uiDeltaSpeedX1K ;

	uiRequiredAcc = 0 ;
	if (uiAvailableTimeInMicroSec)
	{
		uiAverageSpeed = abs(nDesPos) * 1000000UL / uiAvailableTimeInMicroSec ;	// steps/sec
		if (uiAverageSpeed <= uiStartSpeed)
		{
			// no acceleration/deccelration required
			uiStartSpeed = uiAverageSpeed ;
		}
		else
		{
			uiRequiredAcc = 1 ;
		}
	}
	else
	{
		// no acceleration/deccelration required
	}
	if (uiRequiredAcc)
	{
		if (uiAverageSpeed <= ((uiSpeedLimit + uiStartSpeed) / 2))
		{
			// tri-angular profile
			uiTimeForAccInMicroSec = uiAvailableTimeInMicroSec / 2 ;
			uiPeakSpeed = ((uiAverageSpeed - uiStartSpeed) * 2) + uiStartSpeed ;
			uiRequiredAcc = (((uiPeakSpeed - uiStartSpeed) * 40000UL) / (uiAvailableTimeInMicroSec)) * 50 ;
			// please do not try to reduce above stmnt
			// becuase it results in overflow of intermediate result
		}
		else
		{
			// trapezoidal profile
			/*
				number of steps covered during acc and deacc = (Ta * (uiSpeedLimit + uiStartSpeed))
				number of steps covered during steady speed = nDesPos - (Ta * (uiSpeedLimit + uiStartSpeed))
				Ts = steady time can be computed from:
				uiSpeedLimit = (nDesPos - (Ta * (uiSpeedLimit + uiStartSpeed))) / Ts ;
				i.e.
				Ts = (nDesPos - (Ta * (uiSpeedLimit + uiStartSpeed))) / uiSpeedLimit ;
				(Ts + 2*Ta) * 1000000 = uiAvailableTimeInMicroSec ;
				Here, Ts and Ta are in seconds.
				(((nDesPos - (Ta * (uiSpeedLimit + uiStartSpeed))) / uiSpeedLimit) + (2*Ta)) * 1000000 = uiAvailableTimeInMicroSec ;
				((nDesPos - (Ta * (uiSpeedLimit + uiStartSpeed))) / uiSpeedLimit) + (2*Ta) = uiAvailableTimeInMicroSec / 1000000 ;
				multiply boths sides by uiSpeedLimit
				(nDesPos - (Ta * (uiSpeedLimit + uiStartSpeed))) + (2*Ta*uiSpeedLimit) = uiAvailableTimeInMicroSec*uiSpeedLimit / 1000000 ;
				nDesPos - Ta*uiSpeedLimit - Ta*uiStartSpeed + 2*Ta*uiSpeedLimit = uiAvailableTimeInMicroSec*uiSpeedLimit / 1000000 ;
				nDesPos - Ta*(uiStartSpeed - uiSpeedLimit) = uiAvailableTimeInMicroSec*uiSpeedLimit / 1000000 ;
				nDesPos - (uiAvailableTimeInMicroSec*uiSpeedLimit / 1000000) = Ta*(uiStartSpeed - uiSpeedLimit) ;
				Ta = (nDesPos - (uiAvailableTimeInMicroSec*uiSpeedLimit / 1000000)) / (uiStartSpeed - uiSpeedLimit) ;
				TaInUSec = (nDesPos*1000000 - uiAvailableTimeInMicroSec*uiSpeedLimit) / (uiStartSpeed - uiSpeedLimit) ;
				TaInUSec = (uiAvailableTimeInMicroSec*uiSpeedLimit - nDesPos*1000000) / (uiSpeedLimit - uiStartSpeed) ;
			*/
			uiPeakSpeed = uiSpeedLimit ;
			uiTimeForAccInMicroSec = (uiAvailableTimeInMicroSec*uiSpeedLimit - nDesPos*1000000) / (uiSpeedLimit - uiStartSpeed) ;
			uiRequiredAcc = (((uiSpeedLimit - uiStartSpeed) * 40000U) / uiTimeForAccInMicroSec) * 25 ;
			// please do not try to reduce above stmnt
			// becuase it results in overflow of intermediate result
			if (uiRequiredAcc > uiAccLimit)
			{
				// handle this error condition
				// TBD
			}
		}
		uiTimeToStartDeccInMicroSec = uiAvailableTimeInMicroSec - uiTimeForAccInMicroSec ;
		uiDeltaSpeedX1K = SCAN_TIME_IN_MICRO_SEC * uiRequiredAcc / 1000 ;
		if (!uiDeltaSpeedX1K)
		{
			uiDeltaSpeedX1K = 1 ;
		}
		// do something to set/clear direction pin here
		FpgaRunMotor(abs(nDesPos), uiStartSpeed * 1000, uiDeltaSpeedX1K, uiTimeForAccInMicroSec, uiTimeToStartDeccInMicroSec, uiPeakSpeed * 1000) ;
	}
	else
	{
		// run steady at uiStartSpeed for N steps and then stop
		// do something to set/clear direction pin here
		FpgaRunMotor(abs(nDesPos), uiStartSpeed * 1000, 0, 0, uiAvailableTimeInMicroSec, uiStartSpeed * 1000) ;
	}
	return uiRequiredAcc ;
}

void motot_start(void)
{
	unsigned int uiStartSpeed = START_SPEED ;
	unsigned int uiAvailableTimeInMiliSec = AVAILABLE_TIME_IN_MILI_SEC ;
 	int nDesPos = DES_POS ;
	unsigned int uiSpeedLimit = SPEED_LIMIT ;
	unsigned int uiAccLimit = ACC_LIMIT ;


	RunMotor(uiStartSpeed,(uiAvailableTimeInMiliSec * 1000),nDesPos,uiSpeedLimit,uiAccLimit) ;
}
