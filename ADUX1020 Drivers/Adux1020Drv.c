/**
    ***************************************************************************
    * @file     Adux1020Drv.c
    * @author   ADI
    * @version  V0.1
    * @date     17-October-2016
    * @brief    Reference design device driver to access ADI Adux1020 chip.
    ***************************************************************************
*/
/******************************************************************************
*                                                                             *
    Clear BSD license
*                                                                             *
    Copyright (c) 2016, Analog Devices, Inc.
    All rights reserved.
    Redistribution and use in source and binary forms, with or without
    modification, are permitted (subject to the limitations in the disclaimer
    below) provided that the following conditions are met:
*                                                                             *
    Redistributions of source code must retain the above copyright notice,
    this list of conditions and the following disclaimer.
*                                                                             *
*                                                                             *
    Redistributions in binary form must reproduce the above copyright notice,
    this list of conditions and the following disclaimer in the documentation
    and/or other materials provided with the distribution.
*                                                                             *
*                                                                             *
    Neither the name of Analog Devices, Inc. nor the names of its contributors
    may be used to endorse or promote products derived from this software
    without specific prior written permission.
*                                                                             *
    NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY
    THIS LICENSE.  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
    CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT
    NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
    A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER
    OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
    EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
    PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
    OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
    WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
    OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
    ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*                                                                             *
******************************************************************************/
/* ------------------------- Includes -------------------------------------- */
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdlib.h>
#include "Adux1020Drv.h"

/* ------------------------- Defines  --------------------------------------- */

/**
  * Define FIFO_WATERMARK value in the size of WORD, the fifo interrupt occure
  * after reaching this level. The value between 0x00U to 0x0FU 
  */
#define FIFO_WATERMARK	((0x04U) << 8U)

/** Define the INTERRUPT_ENABLE macro to enable the Interrupt related macro's */
#define INTERRUPT_ENABLE

/** Define the Adux1020 Chip Id*/
#define DEVICE_ID_1020           0x03FCU

/** Macro definition for Interrupt Enable */
#ifdef INTERRUPT_ENABLE
/**  Interrupt pin is enabled
Define INT_ENA as 0 to disable it */
#define INT_ENA ((0x01U) << 2U)
/**  
  * Interrupt polarity is set as active low Define INT_POL as 0 for active high,
  * 1 for active low 
  */
#define INT_POL ((0x01U) << 5U)
#endif


/* ------------------------- Private variables ----------------------------- */
#ifndef NDEBUG
static uint32_t gnOverFlowCnt = 0U;
#endif  /* NDEBUG */

static uint32_t gnAccessCnt[5];
/* This variable used to get the Device Id */
static uint16_t gnDeviceID = 0U;
/* This variable used to get the current fifo level */
static uint16_t gnFifoLevel;

/* Private function prototype's */
static int16_t SetAduxOffMode(void);
static int16_t SetAduxIdleMode(void);
static int16_t SetAduxSampleMode(void);
static int16_t SetAduxProximityMode(void);
static int16_t SetInterruptControl(void);
static void (*gpfnAduxCallBack)(void);


#ifndef NOTUSED
/** @brief  Synchronous register write to the Adux
  *
  * @param  nAddr - 16-bit register address
  * @param  nRegValue - 16-bit register data value
  * @return int16_t A 16-bit integer: 0 - success; < 0 - failure
  */
int16_t Adux1020DrvRegWrite(uint16_t nAddr, uint16_t nRegValue)
{
	/* Storing the error status to give feedback to called function */
	int16_t nRetCode = ADUXDrv_SUCCESS;

	/* 3 bytes of data array to store the function input values*/
	uint8_t anI2cData[3];

	/* Give the anI2cData reference to *pData pointer variable*/
	uint8_t *pData = &anI2cData[0];

	/* Assign the register address in 1st byte of array */
	anI2cData[0] = (uint8_t)nAddr;

	/* Assign the register MSB value in 2nd byte of array */
	anI2cData[1] = (uint8_t)(nRegValue >> 8U);

	/* Assign the register LSB value in 3rd byte of array */
	anI2cData[2] = (uint8_t)(nRegValue);

	/**
	    The 1st argument to the function Adux_I2C_Transmit is the pointer to
	    the buffer of the size of three bytes. The first byte is the
	    register address of the Adux device followed by the 16 bits data
	    value to be written to the device register.
	    The 2nd argument is the size of the buffer in bytes (3 bytes).
	    ADUX_I2C_Transmit() should be implemented in such a way that it
	    transmits the data from anI2cData buffer of size specified in the
	    second argument.
	  */
	if (ADUX_I2C_Transmit(pData, 3U) != ADUXDrv_SUCCESS) {
		nRetCode = ADUXDrv_ERROR;
	}

	return nRetCode;
}

/** @brief  Synchronous register read from the Adux
  *
  * @param  nAddr - 16-bit register address
  * @param  *pnData - Pointer to 16-bit register data value
  * @return int16_t A 16-bit integer: 0 - success; < 0 - failure
  */
int16_t Adux1020DrvRegRead(uint16_t nAddr, uint16_t *pnData)
{
	/* Storing the error status to give feedback to called function */
	int16_t nRetCode = ADUXDrv_SUCCESS;

	/* Define and assigning the register address in nRegAddr variable */
	uint8_t nRegAddr = (uint8_t)nAddr;

	/* Define 2 bytes of array to store the register value */
	uint8_t anRxData[2] = {0U};

	/* Give the anRxData reference to pRxData pointer variable */
	uint8_t *pRxData = &anRxData[0];
	/**
	The first argument to the function is the the register address of the
	Adux device from where the data is to be read.
	The 2nd argument is the pointer to the buffer of received data.
	The size of this buffer should be equal to the number of data requested.
	The 3rd argument is the size of requested data in bytes.
	ADUX_I2C_TxRx() should be implemented in such a way that it transmits
	the register address from the first argument and receives the data
	specified by the address in the second argument. The received data will
	be of size specified by 3rd argument.
	*/
	if (ADUX_I2C_TxRx(&nRegAddr, pRxData, 2U) != ADUXDrv_SUCCESS) {
		nRetCode = ADUXDrv_ERROR;
	}

	/* Assign the register value into *pnData pointer variable */
	*pnData = ((uint16_t)anRxData[0] << 8U) + anRxData[1];
	return nRetCode;
}

/** @brief Set device to Off mode, put the all block in power saving mode
  *
  * @param  void
  * @return int16_t A 16-bit integer: 0 - success; < 0 - failure
  */
static int16_t SetAduxOffMode(void)
{
	/* Storing the error status to give feedback to called function */
	uint16_t nRetCode = (uint16_t)ADUXDrv_SUCCESS;

	/* Force the sensor to set in active 4 state */
	nRetCode |= (uint16_t)Adux1020DrvRegWrite(REG_FORCE_MODE_ADDR,
	                                      ACTIVE_4_STATE);

	/* Force mode to set the active 4 state */
	nRetCode |= (uint16_t)Adux1020DrvRegWrite(REG_OP_MODE_ADDR, FORCE_MODE);

	/* Set the Sensor to Off mode */
	nRetCode |= (uint16_t)Adux1020DrvRegWrite(REG_OP_MODE_ADDR, MODE_OFF);

	/* Enable 32MHz fifo clock to reset the fifo */
	nRetCode |= (uint16_t)Adux1020DrvRegWrite(REG_TEST_MODES_3, 
                                                    FORCE_CLOCK_ON);

	/* Flush the fifo data */
	nRetCode |= (uint16_t)Adux1020DrvRegWrite(REG_INT_STATUS, FIFO_RESET);

	/* Disable 32MHz fifo clock */
	nRetCode |= (uint16_t)Adux1020DrvRegWrite(REG_TEST_MODES_3,
                                                    FORCE_CLOCK_RESET);

	/* Mask the all the interrupt */
	nRetCode |= (uint16_t)Adux1020DrvRegWrite(REG_INT_MASK, INT_DISABLE);

	return (int16_t)nRetCode;
}

/** @brief Set device to Idle mode, set the BG/REF/BIAS section ON and put the
  * other blocks in power saving mode
  *
  * @param  void
  * @return int16_t A 16-bit integer: 0 - success; < 0 - failure
  */
static int16_t SetAduxIdleMode(void)
{
	/* Storing the error status to give feedback to called function */
	uint16_t nRetCode = (uint16_t)ADUXDrv_SUCCESS;

	/* Force the sensor to set in active 4 state */
	nRetCode |= (uint16_t)Adux1020DrvRegWrite(REG_FORCE_MODE_ADDR, 
                                                    ACTIVE_4_STATE);

	/* Force mode to set the active 4 state */
	nRetCode |= (uint16_t)Adux1020DrvRegWrite(REG_OP_MODE_ADDR, FORCE_MODE);

	/* Set the Sensor to Idle mode */
	nRetCode |= (uint16_t)Adux1020DrvRegWrite(REG_OP_MODE_ADDR, MODE_IDLE);

	/* Enable 32MHz fifo clock to reset the fifo */
	nRetCode |= (uint16_t)Adux1020DrvRegWrite(REG_TEST_MODES_3,
                                                   FORCE_CLOCK_ON);

	/* Flush the fifo data */
	nRetCode |= (uint16_t)Adux1020DrvRegWrite(REG_INT_STATUS, FIFO_RESET);

	/* Disable 32MHz fifo clock */
	nRetCode |= (uint16_t)Adux1020DrvRegWrite(REG_TEST_MODES_3,
                                                   FORCE_CLOCK_RESET);

	/* Mask the all the interrupt */
	nRetCode |= (uint16_t)Adux1020DrvRegWrite(REG_INT_MASK, INT_DISABLE);

	return (int16_t)nRetCode;
}

/** @brief Set device to Sample mode, to read the sample data from the sensor
  *
  * @param  void
  * @return int16_t A 16-bit integer: 0 - success; < 0 - failure
  */
static int16_t SetAduxSampleMode(void)
{
	/* Storing the error status to give feedback to called function */
	uint16_t nRetCode = (uint16_t)ADUXDrv_SUCCESS;

	/* Switch to off mode before changing the register value*/
	nRetCode |= (uint16_t)Adux1020DrvRegWrite(REG_OP_MODE_ADDR, MODE_OFF);

	/* Switch to sample mode with prevent the fifo full */
	nRetCode |= (uint16_t)Adux1020DrvRegWrite(REG_OP_MODE_ADDR, 
                                                  SAMPLE_MODE_GO);

	return (int16_t)nRetCode;
}

/** @brief Set device to Proximity mode, to read the proximity data from sensor
  *
  * @param  void
  * @return int16_t A 16-bit integer: 0 - success; < 0 - failure
  */
static int16_t SetAduxProximityMode(void)
{
	/* Storing the error status to give feedback to called function */
	uint16_t nRetCode = (uint16_t)ADUXDrv_SUCCESS;

	/* Switch to off mode before changing the register value*/
	nRetCode |= (uint16_t)Adux1020DrvRegWrite(REG_OP_MODE_ADDR, MODE_OFF);

	/* Set fifo level to trigger the fifo filled interrupt */
	nRetCode |= (uint16_t)Adux1020DrvRegWrite(REG_I2C_2,0x0000U);

	/* Set the proximity trigger  type
	   Type 1. Only on crossing threshold use
	           TRIGGER_ON_CROSS_THRESHOLD  macro
	   Type 2. Above and below the threshold level use
	           TRIGGER_ON_BOTH_THRESHOLD macro
	*/
	nRetCode |= (uint16_t)Adux1020DrvRegWrite(REG_PROX_TYPE,
	                                      TRIGGER_ON_BOTH_THRESHOLD);

	/* Switch to proximity mode with prevent the fifo full */
	nRetCode |= (uint16_t)Adux1020DrvRegWrite(REG_OP_MODE_ADDR,
	                                      PROXIMITY_MODE_GO);

	return (int16_t)nRetCode;
}

/** @brief Set various interrupts
  *
  * @param  void
  * @return int16_t A 16-bit integer: 0 - success; < 0 - failure
  */
static int16_t SetInterruptControl(void)
{
	/* Storing the error status to give feedback to called function */
	uint16_t nRetCode = (uint16_t)ADUXDrv_SUCCESS;

	/* Define the variable for read and store the value from
	   interrupt enable and interrupt polarity register */
	uint16_t nRegValue = 0U;

	/**  Define INTERRUPT_ENABLE macro for the definition of INT_ENA and
	INT_POL macros. Change these values according to the requirements
	*/
	/* Read the interrupt enable register */
	nRetCode |= (uint16_t)Adux1020DrvRegRead(REG_INT_ENABLE, &nRegValue);

	/* Assign the value to nRegValue*/
	nRegValue = (((uint16_t)nRegValue | INT_ENA)) ;

	/* Enable int */
	nRetCode |= (uint16_t)Adux1020DrvRegWrite(REG_INT_ENABLE, nRegValue);

	/* Read the interrupt polarity register */
	nRetCode |= (uint16_t)Adux1020DrvRegRead(REG_INT_POLARITY, &nRegValue);

	/* Assign the value to nRegValue*/
	nRegValue = (nRegValue | INT_POL);

	/* Enable Polarity */
	nRetCode |= (uint16_t)Adux1020DrvRegWrite(REG_INT_POLARITY, nRegValue);

	return (int16_t)nRetCode;
}

/** @brief  Set various interrupt modes
  *
  * @param  nIntMask - Interrupt mask bits will be set according to data sheet
  * definition.
  *
  * @return int16_t A 16-bit integer: 0 - success; < 0 - failure
  */
int16_t Adux1020DrvSetInterrupt(uint16_t nIntMask)
{
	/* Storing the error status to give feedback to called function */
	int16_t nRetCode = ADUXDrv_SUCCESS;

	/* Write to the interrupt mask register */
	nRetCode  = Adux1020DrvRegWrite(REG_INT_MASK, nIntMask);

	return nRetCode;
}

/** @brief  Adux1020DrvSetFifoLevel, Set fifo level
  *
  * @param  void
  *
  * @return int16_t A 16-bit integer: 0 - success; < 0 - failure
  */
int16_t Adux1020DrvSetFifoLevel(void)
{
	/* Storing the error status to give feedback to called function */
	int16_t nRetCode = ADUXDrv_SUCCESS;

	/* Set fifo level to trigger the fifo filled interrupt */
	nRetCode = Adux1020DrvRegWrite(REG_I2C_2,FIFO_WATERMARK);

	return nRetCode;
}

/** @brief  Open Driver, setting up the interrupt and I2C lines
  *
  * @param  void
  * @return int16_t A 16-bit integer: 0 - success; < 0 - failure
  */
int16_t Adux1020DrvOpenDriver(void)
{
	/* Storing the error status to give feedback to called function */
	uint16_t nRetCode = (uint16_t)ADUXDrv_SUCCESS;

	/* Get the chip id */
	nRetCode |= (uint16_t)Adux1020DrvRegRead(REG_CHIP_ID, &gnDeviceID);

	/* Go further only chip id is matching with adux1020 */
	if(DEVICE_ID_1020 == gnDeviceID) {
		/* Switch to Idle mode */
		nRetCode |= (uint16_t)SetAduxIdleMode();

		/* Set the default interrupt */
		nRetCode |= (uint16_t)SetInterruptControl();
	}

	return (int16_t)nRetCode;
}

/** @brief  Close Driver, Clear up before existing
  *
  * @param  void
  * @return int16_t A 16-bit integer: 0 - success; < 0 - failure
  */
int16_t Adux1020DrvCloseDriver(void)
{
	return SetAduxIdleMode();
}

/** @brief  Set Adux operating mode, clear FIFO if needed
  *
  * @param  nOperationMode - 8-bit operating mode
  * @return int16_t A 16-bit integer: 0 - success; < 0 - failure
  */
int16_t Adux1020DrvSetOperationMode(uint8_t nOperationMode)
{
	/* Storing the error status to give feedback to called function */
	int16_t nRetCode = ADUXDrv_SUCCESS;

	switch(nOperationMode) {
	case ADUXDrv_MODE_OFF:
		/* Set to Off Mode */
		nRetCode = SetAduxOffMode();
		break;
	case ADUXDrv_MODE_IDLE:
		/* Set Idle Mode */
		nRetCode = SetAduxIdleMode();
		break;
	case ADUXDrv_MODE_SAMPLE:
		/* Set Sample Mode */
		nRetCode = SetAduxSampleMode();
		break;
	case ADUXDrv_MODE_PROXIMITY:
		/* Set Proximity Mode */
		nRetCode = SetAduxProximityMode();
		break;
	default :
		break;
	}
	return nRetCode;
}



/** @brief  Register data ready callback
  *
  * @param  pfAduxDataReady - Function Pointer callback for the reg data
  * @return void
  */
void Adux1020DrvDataReadyCallback(void (*pfAduxDataReady)(void))
{
	/* Assign the callback function in function pointer */
	gpfnAduxCallBack = pfAduxDataReady;
}

/** @brief  Adux interrupt service routine, This function should map with your 
  *         GPIO IRQ routine function 
  * @param  void
  * @return None
  */
void Adux1020DrvISR(void)
{
	if (gpfnAduxCallBack != NULL) {
		/* call the data ready callback function */
		(*gpfnAduxCallBack)();
	}
	/* Increment the function access count for debug purpose */
	gnAccessCnt[0]++;
}

/**
  * @brief Adux Soft Reset routine
  *
  * @param  void
  * @return None
*/
void Adux1020DrvSoftReset(void)
{
  /** 
      Implement the function in such a way of trun Off and turn On the power
      given to the Adux1020 sensor and give 500 milli sencond delay between power 
      On/Off. while doing this Disable the IRQ interrupt and enable it after 
      power given to sensor.
      ex:
        GPIO_IRQ_ADUX_Disable();
        GPIO_DUT_POWER_Off();
        MCU_HAL_Delay(500);
        GPIO_DUT_POWER_On();
        MCU_HAL_Delay(100);
        GPIO_IRQ_ADUX_Enable();
    */
}

/**
  * @brief Adux1020DrvEnable32MHzClock, enable fifo clock to access the fifo
  * @param void
  * @return int16_t A 16-bit integer: 0 - success; < 0 - failure
*/
int16_t Adux1020DrvEnable32MHzClock(void)
{
	/* Storing the error status to give feedback to called function */
	int16_t nRetCode = ADUXDrv_SUCCESS;

	/* Enable the fifo clock */
	nRetCode = Adux1020DrvRegWrite(REG_TEST_MODES_3, FORCE_CLOCK_ON);

	return nRetCode;
}

/**
  * @brief Adux1020DrvDisable32MHzClock Clock, disable fifo clock
  * @param void
  * @return int16_t A 16-bit integer: 0 - success; < 0 - failure
*/
int16_t Adux1020DrvDisable32MHzClock(void)
{
	/* Storing the error status to give feedback to called function */
	int16_t nRetCode = ADUXDrv_SUCCESS;

	/* Disable the fifo clock */
	nRetCode = Adux1020DrvRegWrite(REG_TEST_MODES_3, FORCE_CLOCK_RESET);

	return nRetCode;
}

/** @brief Adux Get parameter
  *
  * @param eCommands - for Watermark value
  * @param pnValue - DataSet Size to be get
  * @return int16_t A 16-bit integer: 0 - success; < 0 - failure
  */
int16_t Adux1020DrvGetParameter(AduxDrvParameterList eCommands,
                                uint16_t *pnValue)
{
	/* Storing the error status to give feedback to called function */
	int16_t nRetCode = ADUXDrv_SUCCESS;

	/* Define the nMask variable to assign the mask bits */
	uint16_t nMask = 0U;

	/* Define the nReg variable to assign the register address */
	uint16_t nReg = 0U;

	/* Define the nBitPos variable for bit position value */
	uint16_t nBitPos = 0U;

	/* Define the nAduxRxData variable for store the register value */
	uint16_t nAduxRxData = 0U;

	/* state machine block for read the parameter, based on the function
	       input state machine will decide which parameter need to read
	*/
	switch(eCommands) {
	case FIFO_LEVEL:
		/* Fifo threshold mask value assign to nMask variable */
		nMask = FIFO_LENGTH_THRESHOLD_MASK;

		/* Fifo bit position assign to nBitPos variable*/
		nBitPos = FIFO_LENGTH_THRESHOLD_BITPOS;

		/* Fifo register address assign to nReg variable */
		nReg = REG_INT_STATUS;

		/* Read the current register value */
		if (Adux1020DrvRegRead(nReg, &nAduxRxData) != ADUXDrv_SUCCESS) {
			/* Update the error status */
			nRetCode = ADUXDrv_ERROR;
		}

		/* Fifo level value assign to gnFifoLevel variable */
		gnFifoLevel = (nAduxRxData & (uint16_t)(~nMask)) >> nBitPos;

		/* Copy the fifo level to function output variable */
		*pnValue = gnFifoLevel;
		break;
	case INT_STATUS:
		/* Interrupt status register addresss assign to nReg variable */
		nReg = REG_INT_STATUS;

		/* Read the current register value */
		if (Adux1020DrvRegRead(nReg, &nAduxRxData) != ADUXDrv_SUCCESS) {
			nRetCode = ADUXDrv_ERROR;
		}

		/* Interrupt status value assign to function output variable */
		*pnValue = (nAduxRxData & (uint16_t)0x00FFU);
		break;
	default:
		break;
	}


	return nRetCode;
}

/** @brief Read data out from Adux FIFO
*
* @param  *pnData - output buffer pointer.
* @param nDataSetSize - DataSet Size to be get
* @return int16_t A 16-bit integer: 0 - success; < 0 - failure
*/
int16_t Adux1020DrvReadFifoData(uint8_t *pnData, uint16_t nDataSetSize)
{
	/* Storing the error status to give feedback to called function */
	int16_t nRetCode = ADUXDrv_SUCCESS;

	/* Define the nAddr variable to store register address */
	uint8_t nAddr;

#ifndef NDEBUG
	if (gnFifoLevel >= 128U) {
		gnOverFlowCnt++;
	}
#endif  /* NDEBUG */

	/* Read the data from fifo when fifo level reaching the data set size */
	if (gnFifoLevel >= nDataSetSize) {
		/* Increment the access count for debug purpose */
		gnAccessCnt[2]++;

		/* Fifo register address copy to nAddr variable */
		nAddr = REG_DATA_BUFFER;

		/**
		The first argument to the function is the the register (FIFO)
		address of the Adux device from where the data is to be read.
		The 2nd argument is the pointer to the buffer of received data.
		The size of this buffer should be equal to the number of data
		requested.
		The 3rd argument is the size of requested data in bytes.
		ADUX_I2C_TxRx() should be implemented in such a way that it
		transmits the register address from the first argument and
		receives the data specified by the address in the second
		argument. The received data will be of size nDataSetSize.
		*/
		if (ADUX_I2C_TxRx((uint8_t*) &nAddr, pnData,
		                  nDataSetSize) != ADUXDrv_SUCCESS) {
			/* Update the error status */
			nRetCode = ADUXDrv_ERROR;
		}

	}

	return nRetCode;
}

/**
 * @brief Set the LED current level (pulse peak value)
 *
 * @param        nLedCurrent - Specify as per Datasheet
 *                           - 0 --> 25mA
 *                           - 1 --> 40mA
 *                           - 2 --> 55mA
 *                           - 3 --> 70mA
 *                           - 4 --> 85mA
 *                           - 5 --> 100mA
 *                           - 6 --> 115mA
 *                           - 7 --> 130mA
 *                           - 8 --> 145mA
 *                           - 9 --> 160mA
 *                           - 10 --> 175mA
 *                           - 11 --> 190mA
 *                           - 12 --> 205mA
 *                           - 13 --> 220mA
 *                           - 14 --> 235mA
 *                           - 15 --> 250mA
 *
 *
 * @return int16_t A 16-bit integer: 0 - success; < 0 - failure
 */
int16_t Adux1020DrvSetLedcurrent(uint8_t nLedCurrent)
{
	/* Storing the error status to give feedback to called function */
	uint16_t nRetCode = (uint16_t)ADUXDrv_SUCCESS;

	/* Define the nMask variable to assign the mask bits */
	uint16_t nMask = 0U;

	/* Define the nReg variable to assign the register address */
	uint16_t nReg = 0U;

	/* Define the nBitPos variable for assign bit position value */
	uint16_t nBitPos = 0U;

	/* Define the nAduxRxData variable for store the register value */
	uint16_t nAduxRxData = 0U;

	/* Define the nAduxTxData variable, carry the register value to be write
	       in the register
	    */
	uint16_t nAduxTxData = 0U;

	/* Led current mask value assign to nMask variable */
	nMask = LED_CURRENT_MASK;

	/* Led current bit position assign to nBitPos variable*/
	nBitPos = LED_CURRENT_BITPOS;

	/* Led current register address assign to nReg variable */
	nReg = REG_LED_CURRENT;

	/* Read the current register value */
	if (Adux1020DrvRegRead(nReg, &nAduxRxData) != ADUXDrv_SUCCESS) {
		nRetCode |= (uint16_t) ADUXDrv_ERROR;
	}

	/* Led current value assign to nAduxTxData variable to be trasmit to
	   register address */
	nAduxTxData = ((uint16_t)((uint16_t)nAduxRxData & nMask) |
	               (uint16_t)(((uint16_t)nLedCurrent) << nBitPos)) ;

	/* Write the current register */
	if (Adux1020DrvRegWrite(nReg, nAduxTxData) != ADUXDrv_SUCCESS) {
		/* Update the error status */
		nRetCode |= (uint16_t)ADUXDrv_ERROR;
	}

	return (int16_t)nRetCode;
}

/**
 * @brief Get the LED current level (pulse peak value)
 * @param           *pLedCurrent - Specify as per Datasheet
 *                              - 0 --> 25mA
 *                              - 1 --> 40mA
 *                              - 2 --> 55mA
 *                              - 3 --> 70mA
 *                              - 4 --> 85mA
 *                              - 5 --> 100mA
 *                              - 6 --> 115mA
 *                              - 7 --> 130mA
 *                              - 8 --> 145mA
 *                              - 9 --> 160mA
 *                              - 10 --> 175mA
 *                              - 11 --> 190mA
 *                              - 12 --> 205mA
 *                              - 13 --> 220mA
 *                              - 14 --> 235mA
 *                              - 15 --> 250mA
 *
 * @return int16_t A 16-bit integer: 0 - success; < 0 - failure
 */
int16_t Adux1020DrvGetLedcurrent(uint8_t *pLedCurrent)
{
	/* Storing the error status to give feedback to called function */
	int16_t nRetCode = ADUXDrv_SUCCESS;

	/* Define the nMask variable to assign the mask bits */
	uint16_t nMask = 0U;

	/* Define the nReg variable to assign the register address */
	uint16_t nReg = 0U;

	/* Define the nBitPos variable for assign bit position value */
	uint16_t nBitPos = 0U;

	/* Define the nAduxRxData variable for store the register value */
	uint16_t nAduxRxData = 0U;

	/* Led current mask value assign to nMask variable */
	nMask = LED_CURRENT_MASK;

	/* Led current bit position assign to nBitPos variable*/
	nBitPos = LED_CURRENT_BITPOS;

	/* Led current register address assign to nReg variable */
	nReg = REG_LED_CURRENT;

	/* Read the current register value */
	if (Adux1020DrvRegRead(nReg, &nAduxRxData) != ADUXDrv_SUCCESS) {
		/* Update the error status */
		nRetCode = ADUXDrv_ERROR;
	}

	/* Assign the led current value to function output variable */
	*pLedCurrent = (uint8_t)((nAduxRxData & 
                                    (uint16_t)(~nMask)) >> nBitPos);

	return nRetCode;
}

/**
  * @brief Adux1020DrvSetLedOffset, Set Led Offset value to LED Offset Register
  * @param nLedOffset - Input to the function carrying the Offset value to be
  * set in the Register.
  * @param nMode - Specify the Mode register, whether going to read the
  * gesture/sample register (or) proximity register
  *                                - 0 --> For Gesture/Sample Mode
  *                                - 1 --> For Proximity Mode
  * @return int16_t A 16-bit integer: 0 - success; < 0 - failure
  */
int16_t Adux1020DrvSetLedOffset(uint8_t nLedOffset, uint16_t nMode)
{
	/* Storing the error status to give feedback to called function */
	uint16_t nRetCode = (uint16_t)ADUXDrv_SUCCESS;

	/* Define the nMask variable to assign the mask bits */
	uint16_t nMask = 0U;

	/* Define the nReg variable to assign the register address */
	uint16_t nReg = 0U;

	/* Define the nBitPos variable for assign bit position value */
	uint16_t nBitPos = 0U;

	/* Define the nAduxRxData variable for store the register value */
	uint16_t nAduxRxData = 0U;

	/* Define the nAduxTxData variable, carry the register value to be write
	   in the register
	*/
	uint16_t nAduxTxData= 0U;

	/* Led offset mask value assign to nMask variable */
	nMask = LED_OFFSET_MASK;

	/* Led offset bit position assign to nBitPos variable*/
	nBitPos = LED_OFFSET_BITPOS;

	/* Led offset register address assign to nReg variable */
	nReg = REG_GEST_LED_WIDTH + (nMode * 2U);
	/* Read the current register value */
	if (Adux1020DrvRegRead(nReg, &nAduxRxData) != ADUXDrv_SUCCESS) {
		/* Update the error status */
		nRetCode |= (uint16_t)ADUXDrv_ERROR;
	}

	/* Led offset value assign to nAduxTxData variable to be trasmit to
	   register address */
	nAduxTxData = ((uint16_t)((uint16_t)nAduxRxData & nMask)) |
	              (uint16_t)((uint16_t)nLedOffset << nBitPos);

	/* Write the current register */
	if (Adux1020DrvRegWrite(nReg, nAduxTxData) != ADUXDrv_SUCCESS) {
		/* Update the error status */
		nRetCode |= (uint16_t)ADUXDrv_ERROR;
	}

	return (int16_t)nRetCode;
}

/**
  * @brief Adux1020DrvGetLedOffset, Get the Led Offset value from the LED Offset
  * Register
  * @param nMode - Specify the Mode register, whether going to read the
  * gesture/sample register (or) proximity register
  *                                - 0 --> For Gesture/Sample Mode
  *                                - 1 --> For Proximity Mode
  * @param *pLedOffset - Pointer variable get the copy of LED Offset.
  * @return int16_t A 16-bit integer: 0 - success; < 0 - failure
*/
int16_t Adux1020DrvGetLedOffset(uint16_t nMode, uint8_t *pLedOffset)
{
	/* Storing the error status to give feedback to called function */
	int16_t nRetCode = ADUXDrv_SUCCESS;

	/* Define the nMask variable to assign the mask bits */
	uint16_t nMask = 0U;

	/* Define the nReg variable to assign the register address */
	uint16_t nReg = 0U;

	/* Define the nBitPos variable for assign bit position value */
	uint16_t nBitPos = 0U;

	/* Define the nAduxRxData variable for store the register value */
	uint16_t nAduxRxData = 0U;

	/* Led offset mask value assign to nMask variable */
	nMask = LED_OFFSET_MASK;

	/* Led offset bit position assign to nBitPos variable*/
	nBitPos = LED_OFFSET_BITPOS;

	/* Led offset register address assign to nReg variable */
	nReg = REG_GEST_LED_WIDTH + (nMode * 2U);

	/* Read the current register value */
	if (Adux1020DrvRegRead(nReg, &nAduxRxData) != ADUXDrv_SUCCESS) {
		/* Update the error status */
		nRetCode = ADUXDrv_ERROR;
	}

	/* Assign the led offset value to function output variable */
	*pLedOffset = (uint8_t)(((nAduxRxData) &
	                         (uint16_t)(~nMask)) >> nBitPos);

	return nRetCode;
}

/**
  * @brief Adux1020DrvSetLedWidth, Set Led Offset value to LED Width Register
  * @param nLedWidth - Input to the function carrying the Width value to be
  * set in the Register.
  * @param nMode - Specify the Mode register, whether going to read the
  * gesture/sample register (or) proximity register
  *                                - 0 --> For Gesture/Sample Mode
  *                                 -1 --> For Proximity Mode
  * @return int16_t A 16-bit integer: 0 - success; < 0 - failure
  */
int16_t Adux1020DrvSetLedWidth(uint8_t nLedWidth, uint16_t nMode)
{
	/* Storing the error status to give feedback to called function */
	uint16_t nRetCode = (uint16_t)ADUXDrv_SUCCESS;

	/* Define the nMask variable to assign the mask bits */
	uint16_t nMask = 0U;

	/* Define the nReg variable to assign the register address */
	uint16_t nReg = 0U;

	/* Define the nBitPos variable for assign bit position value */
	uint16_t nBitPos = 0U;

	/* Define the nAduxRxData variable for store the register value */
	uint16_t nAduxRxData = 0U;

	/* Define the nAduxTxData variable, carry the register value to be write
	   in the register
	*/
	uint16_t nAduxTxData = 0U;

	/* Led width mask value assign to nMask variable */
	nMask = LED_WIDTH_MASK;

	/* Led width bit position assign to nBitPos variable*/
	nBitPos = LED_WIDTH_BITPOS;

	/* Led width register address assign to nReg variable */
	nReg = REG_GEST_LED_WIDTH + (nMode * 2U);

	/* Read the current register value */
	if (Adux1020DrvRegRead(nReg, &nAduxRxData) != ADUXDrv_SUCCESS) {
		/* Update the error status */
		nRetCode |= (uint16_t)ADUXDrv_ERROR;
	}

	/* Led width value assign to nAduxTxData variable to be trasmit to
	   register address */
	nAduxTxData = (uint16_t)((uint16_t)(nAduxRxData & nMask) |
	                         (uint16_t)(((uint16_t)nLedWidth) << nBitPos));

	/* Write the current register */
	if (Adux1020DrvRegWrite(nReg, nAduxTxData) != ADUXDrv_SUCCESS) {
		/* Update the error status */
		nRetCode |= (uint16_t)ADUXDrv_ERROR;
	}

	return (int16_t)nRetCode;
}

/**
  * @brief Adux1020DrvGetLedWidth, Get the Led Width value from the LED Width
  * Register
  * @param nMode - Specify the Mode register, whether going to read the
  * gesture/sample register (or) proximity register
  *                                - 0 --> For Gesture/Sample Mode
  *                                - 1 --> For Proximity Mode
  * @param *pLedWidth - Pointer variable get the copy of LED Pulse.
  * @return int16_t A 16-bit integer: 0 - success; < 0 - failure
*/
int16_t Adux1020DrvGetLedWidth(uint16_t nMode, uint8_t *pLedWidth)
{
	/* Storing the error status to give feedback to called function */
	int16_t nRetCode = ADUXDrv_SUCCESS;

	/* Define the nMask variable to assign the mask bits */
	uint16_t nMask = 0U;

	/* Define the nReg variable to assign the register address */
	uint16_t nReg = 0U;

	/* Define the nBitPos variable for assign bit position value */
	uint16_t nBitPos = 0U;

	/* Define the nAduxRxData variable for store the register value */
	uint16_t nAduxRxData = 0U;

	/* Led width mask value assign to nMask variable */
	nMask = LED_WIDTH_MASK;

	/* Led width bit position assign to nBitPos variable*/
	nBitPos = LED_WIDTH_BITPOS;

	/* Led width register address assign to nReg variable */
	nReg = REG_GEST_LED_WIDTH + (nMode * 2U);

	/* Read the current register value */
	if (Adux1020DrvRegRead(nReg, &nAduxRxData) != ADUXDrv_SUCCESS) {
		/* Update the error status */
		nRetCode = ADUXDrv_ERROR;
	}

	/* Assign the led width value to function output variable */
	*pLedWidth = (uint8_t)((nAduxRxData & (uint16_t)(~nMask)) >> nBitPos);

	return nRetCode;
}

/**
  * @brief AduxDrvSetLedPeriod, Set Led Period value to LED Period Register
  * @param nLedPeriod - Input to the function carrying the Period value to be
  * set in the Register.
  * @param nMode - Specify the Mode register, whether going to read the
  * gesture/sample register (or) Proximity register
  *                                - 0 --> For Gesture/Sample Mode
  *                                - 1 --> For Proximity Mode
  * @return int16_t A 16-bit integer: 0 - success; < 0 - failure
*/
int16_t Adux1020DrvSetLedPeriod(uint8_t nLedPeriod, uint16_t nMode)
{
	/* Storing the error status to give feedback to called function */
	uint16_t nRetCode = (uint16_t)ADUXDrv_SUCCESS;

	/* Define the nMask variable to assign the mask bits */
	uint16_t nMask = 0U;

	/* Define the nReg variable to assign the register address */
	uint16_t nReg = 0U;

	/* Define the nBitPos variable for assign bit position value */
	uint16_t nBitPos = 0U;

	/* Define the nAduxRxData variable for store the register value */
	uint16_t nAduxRxData = 0U;

	/* Define the nAduxTxData variable, carry the register value to be write
	   in the register
	*/
	uint16_t nAduxTxData = 0U;

	/* Led period mask value assign to nMask variable */
	nMask = LED_PERIOD_MASK;

	/* Led period bit position assign to nBitPos variable*/
	nBitPos = LED_PERIOD_BITPOS;

	/* Led period register address assign to nReg variable */
	nReg = REG_GEST_LED_PERIOD + (nMode * 2U);

	/* Read the current register value */
	if (Adux1020DrvRegRead(nReg, &nAduxRxData) != ADUXDrv_SUCCESS) {
		/* Update the error status */
		nRetCode |= (uint16_t)ADUXDrv_ERROR;
	}

	/* Led period value assign to nAduxTxData variable to be trasmit to
	   register address */
	nAduxTxData = (uint16_t)((uint16_t)nAduxRxData & nMask) |
	              (uint16_t)((uint16_t)nLedPeriod << nBitPos);

	/* Write the current register */
	if (Adux1020DrvRegWrite(nReg, nAduxTxData) != ADUXDrv_SUCCESS) {
		/* Update the error status */
		nRetCode |= (uint16_t)ADUXDrv_ERROR;
	}

	return (int16_t)nRetCode;
}

/**
  * @brief AduxDrvGetLedPeriod, Get the Led period value from the LED Period
  * register
  * @param nMode - Specify the Mode register, whether going to read the
  * gesture/sample register (or) proximity register
  *                                - 0 --> For Gesture/Sample Mode
  *                                - 1 --> For Proximity Mode
  * @param *pLedPeriod - Pointer variable get the copy of LED Pulse.
  * @return int16_t A 16-bit integer: 0 - success; < 0 - failure
*/
int16_t Adux1020DrvGetLedPeriod(uint16_t nMode, uint8_t *pLedPeriod)
{
	/* Storing the error status to give feedback to called function */
	int16_t nRetCode = ADUXDrv_SUCCESS;

	/* Define the nMask variable to assign the mask bits */
	uint16_t nMask = 0u;

	/* Define the nReg variable to assign the register address */
	uint16_t nReg = 0U;

	/* Define the nBitPos variable for assign bit position value */
	uint16_t nBitPos = 0U;

	/* Define the nAduxRxData variable for store the register value */
	uint16_t nAduxRxData = 0U;

	/* Led period mask value assign to nMask variable */
	nMask = LED_PERIOD_MASK;

	/* Led period bit position assign to nBitPos variable*/
	nBitPos = LED_PERIOD_BITPOS;

	/* Led period register address assign to nReg variable */
	nReg = REG_GEST_LED_PERIOD + (nMode * 2U);

	/* Read the current register value */
	if (Adux1020DrvRegRead(nReg, &nAduxRxData) != ADUXDrv_SUCCESS) {
		/* Update the error status */
		nRetCode = ADUXDrv_ERROR;
	}

	/* Assign the led period value to function output variable */
	*pLedPeriod = (uint8_t)((nAduxRxData & (uint16_t)(~nMask)) >> nBitPos);

	return nRetCode;
}

/**
  * @brief AduxDrvSetLedPulse, Set Led Pulse value to LED Pulse Register
  * @param nLedPulse - Input to the function carrying the Period value to be
  * set in the Register.
  * @param nMode - Specify the Mode register, whether going to read the
  * gesture/sample register (or) Proximity register
  *                                - 0 --> For Gesture/Sample Mode
  *                                - 1 --> For Proximity Mode
  * @return int16_t A 16-bit integer: 0 - success; < 0 - failure
*/
int16_t Adux1020DrvSetLedPulse(uint8_t nLedPulse, uint16_t nMode)
{
	/* Storing the error status to give feedback to called function */
	uint16_t nRetCode = (uint16_t)ADUXDrv_SUCCESS;

	/* Define the nMask variable to assign the mask bits */
	uint16_t nMask = 0U;

	/* Define the nReg variable to assign the register address */
	uint16_t nReg = 0U;

	/* Define the nBitPos variable for assign bit position value */
	uint16_t nBitPos = 0U;

	/* Define the nAduxRxData variable for store the register value */
	uint16_t nAduxRxData = 0U;

	/* Define the nAduxTxData variable, carry the register value to be write
	   in the register
	*/
	uint16_t nAduxTxData = 0U;

	/* Led number pulse mask value assign to nMask variable */
	nMask = LED_NUMBER_PULSES_MASK;

	/* Led number pulse bit position assign to nBitPos variable*/
	nBitPos = LED_NUMBER_PULSES_BITPOS;

	/* Led number pulse register address assign to nReg variable */
	nReg = REG_GEST_LED_PERIOD + (nMode * 2U);

	/* Read the current register value */
	if (Adux1020DrvRegRead(nReg, &nAduxRxData) != ADUXDrv_SUCCESS) {
		/* Update the error status */
		nRetCode |= (uint16_t)ADUXDrv_ERROR;
	}

	/* Led number pulse value assign to nAduxTxData variable to be trasmit
	    to register address */
	nAduxTxData = (uint16_t)((uint16_t)nAduxRxData & nMask) |
	              (uint16_t)((uint16_t)nLedPulse << nBitPos);

	/* Write the current register */
	if (Adux1020DrvRegWrite(nReg, nAduxTxData) != ADUXDrv_SUCCESS) {
		/* Update the current register */
		nRetCode |= (uint16_t)ADUXDrv_ERROR;
	}

	return (int16_t)nRetCode;
}

/**
  * @brief AduxDrvGetLedPulse, Get the Led Pulse value from the LED Pulse
  * Register
  * @param nMode - Specify the Mode register, whether going to read the 
  * Gesture/Sample register (or) Proximity register
  *                                - 0 --> For Gesture/Sample Mode
  *                                - 1 --> For Proximity Mode
  * @param *nLedPulse - Pointer variable get the copy of LED Pulse.
  * @return int16_t A 16-bit integer: 0 - success; < 0 - failure
*/
int16_t Adux1020DrvGetLedPulse(uint16_t nMode, uint8_t *nLedPulse)
{
	/* Storing the error status to give feedback to called function */
	int16_t nRetCode = ADUXDrv_SUCCESS;

	/* Define the nMask variable to assign the mask bits */
	uint16_t nMask = 0U;

	/* Define the nReg variable to assign the register address */
	uint16_t nReg = 0U;

	/* Define the nBitPos variable for assign bit position value */
	uint16_t nBitPos = 0U;

	/* Define the nAduxRxData variable for store the register value */
	uint16_t nAduxRxData = 0U;

	/* Led pulse mask value assign to nMask variable */
	nMask = LED_NUMBER_PULSES_MASK;

	/* Led pulse bit position assign to nBitPos variable*/
	nBitPos = LED_NUMBER_PULSES_BITPOS;

	/* Led pulse register address assign to nReg variable */
	nReg = REG_GEST_LED_PERIOD + (nMode * 2U);

	/* Read the current register value */
	if (Adux1020DrvRegRead(nReg, &nAduxRxData) != ADUXDrv_SUCCESS) {
		/* Update the error status */
		nRetCode = ADUXDrv_ERROR;
	}

	/* Assign the led pulse value to function output variable */
	*nLedPulse = (uint8_t)((nAduxRxData & (uint16_t)(~nMask)) >> nBitPos);

	return nRetCode;
}

/* AFE CONTROL REGISTER FOR SAMPLING/GESTURE AND PROXIMITY MODE*/
/** @brief AduxDrvSetAfeWidth, set the AFE Clock Width for gesture/sample and
  * proximity mode
  * @param nAfeWidth - Input to the function, carrying the AFE Clock Width
  * value to be set in the register.
  * @param nMode - Specify the Mode register, whether going to Write the
  * Gesture/Sample register (or) Proximity register
  *                                - 0 --> For Gesture/Sample Mode
  *                                - 1 --> For Proximity Mode
  * @return int16_t A 16-bit integer: 0 - success; < 0 - failure
*/
int16_t Adux1020DrvSetAfeWidth(uint16_t nAfeWidth, uint16_t nMode)
{
	/* Storing the error status to give feedback to called function */
	uint16_t nRetCode = (uint16_t)ADUXDrv_SUCCESS;

	/* Define the nMask variable to assign the mask bits */
	uint16_t nMask = 0U;

	/* Define the nReg variable to assign the register address */
	uint16_t nReg = 0U;

	/* Define the nBitPos variable for assign bit position value */
	uint16_t nBitPos = 0U;

	/* Define the nAduxRxData variable for store the register value */
	uint16_t nAduxRxData = 0U;

	/* Define the nAduxTxData variable, carry the register value to be write
	   in the register
	*/
	uint16_t nAduxTxData = 0U;

	/* Afe width mask value assign to nMask variable */
	nMask = AFE_WIDTH_MASK;

	/* Afe width bit position assign to nBitPos variable*/
	nBitPos = AFE_WIDTH_BITPOS;

	/* Afe width register address assign to nReg variable */
	nReg = REG_GEST_AFE + nMode;

	/* Read the current register value */
	if (Adux1020DrvRegRead(nReg, &nAduxRxData) != ADUXDrv_SUCCESS) {
		/* Update the erroe status */
		nRetCode |= (uint16_t)ADUXDrv_ERROR;
	}

	/* Afe width value assign to nAduxTxData variable to be trasmit to
	   register address */
	nAduxTxData = (uint16_t)((uint16_t)nAduxRxData & nMask) |
	              (uint16_t)((uint16_t)nAfeWidth << nBitPos);

	/* Write the current register */
	if (Adux1020DrvRegWrite(nReg, nAduxTxData) != ADUXDrv_SUCCESS) {
		/* Update the error status */
		nRetCode |= (uint16_t)ADUXDrv_ERROR;
	}

	return (int16_t)nRetCode;
}

/** @brief AduxDrvGetAfeWidth, get the AFE Clock Width for gesture/sample and
  * proximity mode
  *
  * @param nMode - Specify the Mode register, whether going to Read the
  * Gesture/Sample register (or) Proximity register
  *                                - 0 --> For Gesture/Sample Mode
  *                                - 1 --> For Proximity Mode
  * @param *pAfeWidth - Pointer variable get the copy of AFE Clock Width.
  * @return int16_t A 16-bit integer: 0 - success; < 0 - failure
*/
int16_t Adux1020DrvGetAfeWidth( uint16_t nMode, uint8_t *pAfeWidth)
{
	/* Storing the error status to give feedback to called function */
	int16_t nRetCode = ADUXDrv_SUCCESS;

	/* Define the nMask variable to assign the mask bits */
	uint16_t nMask = 0U;

	/* Define the nReg variable to assign the register address */
	uint16_t nReg = 0U;

	/* Define the nBitPos variable for assign bit position value */
	uint16_t nBitPos = 0U;

	/* Define the nAduxRxData variable for store the register value */
	uint16_t nAduxRxData = 0U;

	/* Afe width mask value assign to nMask variable */
	nMask = AFE_WIDTH_MASK;

	/* Afe width bit position assign to nBitPos variable*/
	nBitPos = AFE_WIDTH_BITPOS;

	/* Afe width register address assign to nReg variable */
	nReg = REG_GEST_AFE + nMode;

	/* Read the current register value */
	if (Adux1020DrvRegRead(nReg, &nAduxRxData) != ADUXDrv_SUCCESS) {
		/* Update the error status */
		nRetCode = ADUXDrv_ERROR;
	}

	/* Assign the afe width value to function output variable */
	*pAfeWidth = (uint8_t)((nAduxRxData & (uint16_t)(~nMask)) >> nBitPos);

	return nRetCode;
}

/** @brief AduxDrvSetAfeOffset, set the AFE Clock coarse Offset for
  * gesture/sample and Proximity mode
  * @param nAfeOffset - Input to the function, carrying the Clock coarse Offset
  * value to be set in the Register.
  * @param nMode - Specify the Mode register, whether going to read the
  * Gesture/Sample register (or) Proximity register
  *                                - 0 --> For Gesture/Sample Mode
  *                                - 1 --> For Proximity Mode
  * @return int16_t A 16-bit integer: 0 - success; < 0 - failure
*/
int16_t Adux1020DrvSetAfeOffset(uint16_t nAfeOffset, uint16_t nMode)
{
	/* Storing the error status to give feedback to called function */
	uint16_t nRetCode = (uint16_t)ADUXDrv_SUCCESS;

	/* Define the nMask variable to assign the mask bits */
	uint16_t nMask = 0U;

	/* Define the nReg variable to assign the register address */
	uint16_t nReg = 0U;

	/* Define the nBitPos variable for assign bit position value */
	uint16_t nBitPos = 0U;

	/* Define the nAduxRxData variable for store the register value */
	uint16_t nAduxRxData = 0U;

	/* Define the nAduxTxData variable, carry the register value to be write
	   in the register
	*/
	uint16_t nAduxTxData= 0U;

	/* Afe offset mask value assign to nMask variable */
	nMask = AFE_OFFSET_MASK;

	/* Afe offset bit position assign to nBitPos variable*/
	nBitPos = AFE_OFFSET_BITPOS;

	/* Afe offset register address assign to nReg variable */
	nReg = REG_GEST_AFE + nMode;

	/* Read the current register value */
	if (Adux1020DrvRegRead(nReg, &nAduxRxData) != ADUXDrv_SUCCESS) {
		/* Update the error status */
		nRetCode |= (uint16_t)ADUXDrv_ERROR;
	}

	/* Afe offset value assign to nAduxTxData variable to be trasmit to
	   register address */
	nAduxTxData = (uint16_t)((uint16_t)nAduxRxData & nMask) |
	              (uint16_t)((uint16_t)nAfeOffset<< nBitPos);

	/* Write the current register */
	if (Adux1020DrvRegWrite(nReg, nAduxTxData) != ADUXDrv_SUCCESS) {
		/* Update the error status */
		nRetCode |= (uint16_t)ADUXDrv_ERROR;
	}
	return (int16_t)nRetCode;
}

/** @brief AduxDrvGetAfeOffset, get the AFE Clock coarse Offset for
  * Gesture/sample and Proximity mode
  * @param nMode - Specify the Mode register, whether going to read the
  * gesture/sample register (or) proximity register
  *                                - 0 --> For Gesture/Sample Mode
  *                                - 1 --> For Proximity Mode
  * @param *pAfeOffset - Pointer variable get the copy of AFE Clock coarse
  * Offset.
  * @return int16_t A 16-bit integer: 0 - success; < 0 - failure
*/
int16_t Adux1020DrvGetAfeOffset(uint16_t nMode, uint8_t *pAfeOffset)
{
	/* Storing the error status to give feedback to called function */
	int16_t nRetCode = ADUXDrv_SUCCESS;

	/* Define the nMask variable to assign the mask bits */
	uint16_t nMask = 0U;

	/* Define the nReg variable to assign the register address */
	uint16_t nReg = 0U;

	/* Define the nBitPos variable for assign bit position value */
	uint16_t nBitPos = 0U;

	/* Define the nAduxRxData variable for store the register value */
	uint16_t nAduxRxData = 0U;

	/* Afe offset mask value assign to nMask variable */
	nMask = AFE_OFFSET_MASK;

	/* Afe offset bit position assign to nBitPos variable*/
	nBitPos = AFE_OFFSET_BITPOS;

	/* Afe offset register address assign to nReg variable */
	nReg = REG_GEST_AFE + nMode;

	/* Read the current register value */
	if (Adux1020DrvRegRead(nReg, &nAduxRxData) != ADUXDrv_SUCCESS) {
		/* Update the error status */
		nRetCode = ADUXDrv_ERROR;
	}

	/* Assign the afe offset value to function output variable */
	*pAfeOffset = (uint8_t)((nAduxRxData & (uint16_t)(~nMask)) >> nBitPos);

	return nRetCode;
}

/** @brief AduxDrvSetAfeFineOffset, set the AFE Clock Fine Offset for
  * gesture/sample and proximity mode
  * @param nAfeFineOffset - Input to the function, carrying the AFE Clock Fine
  * Offset value to be set in the Register.
  * @param nMode - Specify the Mode register, whether going to read the
  * gesture/sample register (or) proximity register
  *                                - 0 --> For Gesture/Sample Mode
  *                                - 1 --> For Proximity Mode
  * @return int16_t A 16-bit integer: 0 - success; < 0 - failure
*/
int16_t Adux1020DrvSetAfeFineOffset(uint16_t nAfeFineOffset, uint16_t nMode)
{
	/* Storing the error status to give feedback to called function */
	uint16_t nRetCode = (uint16_t)ADUXDrv_SUCCESS;

	/* Define the nMask variable to assign the mask bits */
	uint16_t nMask = 0U;

	/* Define the nReg variable to assign the register address */
	uint16_t nReg = 0U;

	/* Define the nBitPos variable for assign bit position value */
	uint16_t nBitPos = 0U;

	/* Define the nAduxRxData variable for store the register value */
	uint16_t nAduxRxData = 0U;

	/* Define the nAduxTxData variable, carry the register value to be write
	   in the register
	*/
	uint16_t nAduxTxData = 0U;

	/* Afe fine offset mask value assign to nMask variable */
	nMask = AFE_FINE_OFFSET_MASK;

	/* Afe fine offset bit position assign to nBitPos variable*/
	nBitPos = AFE_FINE_OFFSET_BITPOS;

	/* Afe fine offset register address assign to nReg variable */
	nReg = REG_GEST_AFE + nMode;

	/* Read the current register value */
	if (Adux1020DrvRegRead(nReg, &nAduxRxData) != ADUXDrv_SUCCESS) {
		/* Update the error status */
		nRetCode |= (uint16_t)ADUXDrv_ERROR;
	}

	/* Afe fine offset value assign to nAduxTxData variable to be trasmit to
	   register address */
	nAduxTxData = (uint16_t)((uint16_t)nAduxRxData & nMask) |
	              (uint16_t)((uint16_t)nAfeFineOffset << nBitPos);

	/* Write the current register */
	if (Adux1020DrvRegWrite(nReg, nAduxTxData) != ADUXDrv_SUCCESS) {
		/* Update the error status */
          nRetCode |= (uint16_t)ADUXDrv_ERROR;
	}
	return (int16_t)nRetCode;
}

/** @brief AduxDrvGetAfeFineOffset, get the AFE Clock Fine Offset for
  * gesture/sample and proximity mode
  * @param nMode - Specify the Mode register, whether going to read the
  * gesture/sample register (or) proximity register
  *                                 0 --> For Gesture/Sample Mode
  *                                 1 --> For Proximity Mode
  * @param *pAfeFineOffset - Pointer variable get the copy of LED Pulse.
  * @return int16_t A 16-bit integer: 0 - success; < 0 - failure
*/
int16_t Adux1020DrvGetAfeFineOffset(uint16_t nMode, uint8_t *pAfeFineOffset)
{
	/* Storing the error status to give feedback to called function */
	int16_t nRetCode = ADUXDrv_SUCCESS;

	/* Define the nMask variable to assign the mask bits */
	uint16_t nMask = 0U;

	/* Define the nReg variable to assign the register address */
	uint16_t nReg = 0U;

	/* Define the nBitPos variable for assign bit position value */
	uint16_t nBitPos = 0U;

	/* Define the nAduxRxData variable for store the register value */
	uint16_t nAduxRxData = 0U;

	/* Afe fine offset mask value assign to nMask variable */
	nMask = AFE_FINE_OFFSET_MASK;

	/* Afe fine offset bit position assign to nBitPos variable*/
	nBitPos = AFE_FINE_OFFSET_BITPOS;

	/* Afe fine offset register address assign to nReg variable */
	nReg = REG_GEST_AFE + nMode;

	/* Read the current register value */
	if (Adux1020DrvRegRead(nReg, &nAduxRxData) != ADUXDrv_SUCCESS) {
		/* Update the error status */
		nRetCode = ADUXDrv_ERROR;
	}

	/* Assign the afe fine offset value to function output variable */
	*pAfeFineOffset = (uint8_t)((nAduxRxData &
	                             (uint16_t)(~nMask)) >> nBitPos);

	return nRetCode;
}

/**
 * @brief Set the sampling frequency rate
 *
 * @param        nSamplingRate - Specify the sample rate as Datasheet
 *                             - 0 --> 0.1 Hz
 *                             - 1 --> 0.2 Hz
 *                             - 2 --> 0.5 Hz
 *                             - 3 --> 1 Hz
 *                             - 4 --> 2 Hz
 *                             - 5 --> 5 Hz
 *                             - 6 --> 10 Hz
 *                             - 7 --> 20 Hz
 *                             - 8 --> 50 Hz
 *                             - 9 --> 100 Hz
 *                             - 10 --> 190 Hz
 *                             - 11 --> 450 Hz
 *                             - 12 --> 820 Hz
 *                             - 13 --> 1400 Hz
 *                             - 14 --> 1400 Hz
 *                             - 15 --> 1400 Hz
 *
 * @return int16_t A 16-bit integer: 0 - success; < 0 - failure
 */
int16_t Adux1020DrvSetSamplingFrequency(uint8_t nSamplingRate)
{
	/* Storing the error status to give feedback to called function */
	uint16_t nRetCode = (uint16_t)ADUXDrv_SUCCESS;

	/* Define the nMask variable to assign the mask bits */
	uint16_t nMask = 0U;

	/* Define the nReg variable to assign the register address */
	uint16_t nReg = 0U;

	/* Define the nBitPos variable for assign bit position value */
	uint16_t nBitPos = 0U;

	/* Define the nAduxRxData variable for store the register value */
	uint16_t nAduxRxData = 0U;

	/* Define the nAduxTxData variable, carry the register value to be write
	   in the register
	*/
	uint16_t nAduxTxData = 0U;

	/* Sampling frequency mask value assign to nMask variable */
	nMask = GESTURE_SAMPLING_FREQ_MASK;

	/* Sampling frequency bit position assign to nBitPos variable*/
	nBitPos = GESTURE_SAMPLING_FREQ_BITPOS;

	/* Sampling frequency register address assign to nReg variable */
	nReg = REG_FREQUENCY;

	/* Read the current register value */
	if(Adux1020DrvRegRead(nReg, &nAduxRxData) != ADUXDrv_SUCCESS) {
		/* Update the error status */
		nRetCode |= (uint16_t)ADUXDrv_ERROR;
	}

	/* Sampling frequency value assign to nAduxTxData variable to be trasmit
	 * to register address
	 */
	nAduxTxData = ((uint16_t)((uint16_t)nAduxRxData & nMask) |
	               (uint16_t)((uint16_t)nSamplingRate << nBitPos));

	/* Write the current register */
	if (Adux1020DrvRegWrite(nReg, nAduxTxData) != ADUXDrv_SUCCESS) {
		/* Update the error status */
		nRetCode |= (uint16_t)ADUXDrv_ERROR;
	}
	return (int16_t)nRetCode;
}

/**
  * @brief Adux1020DrvGetSamplingFrequency, get the sampling frequency of
  * gesture/sample mode
  * @param *pSamplingRate - Pointer variable to get the copy of sampling
  *                        frequency.
  * @return int16_t A 16-bit integer: 0 - success; < 0 - failure
  */
int16_t Adux1020DrvGetSamplingFrequency(uint8_t *pSamplingRate)
{
	/* Storing the error status to give feedback to called function */
	int16_t nRetCode = ADUXDrv_SUCCESS;

	/* Define the nMask variable to assign the mask bits */
	uint16_t nMask = 0U;

	/* Define the nReg variable to assign the register address */
	uint16_t nReg = 0U;

	/* Define the nBitPos variable for assign bit position value */
	uint16_t nBitPos = 0U;

	/* Define the nAduxRxData variable for store the register value */
	uint16_t nAduxRxData= 0U;

	/* Sampling frequency mask value assign to nMask variable */
	nMask = GESTURE_SAMPLING_FREQ_MASK;

	/* Sampling frequency bit position assign to nBitPos variable*/
	nBitPos = GESTURE_SAMPLING_FREQ_BITPOS;

	/* Sampling frequency register address assign to nReg variable */
	nReg = REG_FREQUENCY;

	/* Read the current register value */
	if(Adux1020DrvRegRead(nReg, &nAduxRxData) != ADUXDrv_SUCCESS) {
		/* Update the error status */
		nRetCode = ADUXDrv_ERROR;
	}

	/* Assign the sampling frequency value to function output variable */
	*pSamplingRate = (uint8_t)((nAduxRxData &
	                            (uint16_t)(~nMask)) >> nBitPos);

	return nRetCode;
}

/**
 * @brief Adux1020DrvSetProximitySamplingFrequency,Set the proximity sampling
 *        frequency rate.
 * @param    nProxSamplingRate - Specify the sample rate as Datasheet
 *                             - 0 --> 0.1 Hz
 *                             - 1 --> 0.2 Hz
 *                             - 2 --> 0.5 Hz
 *                             - 3 --> 1 Hz
 *                             - 4 --> 2 Hz
 *                             - 5 --> 5 Hz
 *                             - 6 --> 10 Hz
 *                             - 7 --> 20 Hz
 *                             - 8 --> 50 Hz
 *                             - 9 --> 100 Hz
 *                             - 10 --> 190 Hz
 *                             - 11 --> 450 Hz
 *                             - 12 --> 820 Hz
 *                             - 13 --> 1400 Hz
 *                             - 14 --> 1400 Hz
 *                             - 15 --> 1400 Hz
 *
 *
 * @return int16_t A 16-bit integer: 0 - success; < 0 - failure
 */
int16_t Adux1020DrvSetProximitySamplingFrequency(uint8_t nProxSamplingRate)
{
	/* Storing the error status to give feedback to called function */
	uint16_t nRetCode = (uint16_t)ADUXDrv_SUCCESS;

	/* Define the nMask variable to assign the mask bits */
	uint16_t nMask = 0U;

	/* Define the nReg variable to assign the register address */
	uint16_t nReg = 0U;

	/* Define the nBitPos variable for assign bit position value */
	uint16_t nBitPos = 0U;

	/* Define the nAduxRxData variable for store the register value */
	uint16_t nAduxRxData = 0U;

	/* Define the nAduxTxData variable, carry the register value to be write
	   in the register
	*/
	uint16_t nAduxTxData = 0U;

	/* Proximity sampling frequency mask value assign to nMask variable */
	nMask = PROXI_SAMPLING_FREQ_MASK;

	/* Proximity sampling frequency bit position assign to nBitPos
	 * variable
	 */
	nBitPos = PROXI_SAMPLING_FREQ_BITPOS;

	/* Proximity sampling frequency register address assign to nReg
	 * variable
	 */
	nReg = REG_FREQUENCY;

	/* Read the current register value */
	if(Adux1020DrvRegRead(nReg, &nAduxRxData) != ADUXDrv_SUCCESS) {
		/* Update the error status */
		nRetCode |= (uint16_t)ADUXDrv_ERROR;
	}

	/* Proximity sampling frequency value assign to nAduxTxData variable to
	 * be trasmit to register address
	 */
	nAduxTxData = (uint16_t)((uint16_t)nAduxRxData & nMask) |
	              (uint16_t)((uint16_t)nProxSamplingRate << nBitPos);

	/* Write the current register */
	if (Adux1020DrvRegWrite(nReg, nAduxTxData) != ADUXDrv_SUCCESS) {
		/* Update the error status */
		nRetCode |= (uint16_t)ADUXDrv_ERROR;
	}
	return (int16_t)nRetCode;
}

/**
  * @brief Adux1020DrvGetSamplingFrequency, get the sampling frequency of
  * proximity mode
  * @param *pProxSamplingRate - Pointer variable to get the copy of sampling
  *                        frequency.
  * @return int16_t A 16-bit integer: 0 - success; < 0 - failure
  */
int16_t Adux1020DrvGetProximitySamplingFrequency(uint8_t *pProxSamplingRate)
{
	/* Storing the error status to give feedback to called function */
	int16_t nRetCode = ADUXDrv_SUCCESS;

	/* Define the nMask variable to assign the mask bits */
	uint16_t nMask = 0U;

	/* Define the nReg variable to assign the register address */
	uint16_t nReg = 0U;

	/* Define the nBitPos variable for assign bit position value */
	uint16_t nBitPos = 0U;

	/* Define the nAduxRxData variable for store the register value */
	uint16_t nAduxRxData = 0U;

	/* Proximity sampling frequency mask value assign to nMask variable */
	nMask = PROXI_SAMPLING_FREQ_MASK;

	/* Proximity sampling frequency bit position assign to nBitPos
	 * variable
	 */
	nBitPos = PROXI_SAMPLING_FREQ_BITPOS;

	/* Proximity sampling frequency register address assign to nReg
	 * variable
	 */
	nReg = REG_FREQUENCY;
	/* Read the current register value */
	if(Adux1020DrvRegRead(nReg, &nAduxRxData) != ADUXDrv_SUCCESS) {
		/* Update the error status */
		nRetCode = ADUXDrv_ERROR;
	}

	/* Assign the proximity sampling frequency value to function output
	 * variable
	 */
	*pProxSamplingRate = (uint8_t)(nAduxRxData &
	                               (uint16_t)(~nMask)) >> nBitPos;
	return nRetCode;
}


/**
 * @brief Adux1020DrvSetDecimationRate,Set the sampling decimation rate.
 *
 * @param      nDecimationRate  - Specify the decimation as per Datasheet
 *                              - 0 --> 1
 *                              - 1 --> 2
 *                              - 2 --> 4
 *                              - 3 --> 8
 *                              - 4 --> 16
 *                              - 5 --> 32
 *
 *
 * @return int16_t A 16-bit integer: 0 - success; < 0 - failure
 */
int16_t Adux1020DrvSetDecimationRate(uint8_t nDecimationRate)
{
	/* Storing the error status to give feedback to called function */
	uint16_t nRetCode = (uint16_t)ADUXDrv_SUCCESS;

	/* Define the nMask variable to assign the mask bits */
	uint16_t nMask = 0U;

	/* Define the nReg variable to assign the register address */
	uint16_t nReg = 0U;

	/* Define the nBitPos variable for assign bit position value */
	uint16_t nBitPos = 0U;

	/* Define the nAduxRxData variable for store the register value */
	uint16_t nAduxRxData = 0U;

	/* Define the nAduxTxData variable, carry the register value to be write
	   in the register
	*/
	uint16_t nAduxTxData = 0U;

	/* Sampling decimation mask value assign to nMask variable */
	nMask = GESTURE_DECIMATION_MASK;

	/* Sampling decimation bit position assign to nBitPos variable*/
	nBitPos = GESTURE_DECIMATION_BITPOS;

	/* Sampling decimation register address assign to nReg variable */
	nReg = REG_DECIMATION;

	/* Read the current register value */
	if(Adux1020DrvRegRead(nReg, &nAduxRxData) != ADUXDrv_SUCCESS) {
		/* Update the error status */
		nRetCode |= (uint16_t)ADUXDrv_ERROR;
	}

	/* Sampling decimation value assign to nAduxTxData variable to
	 * be trasmit to register address
	 */
	nAduxTxData = (uint16_t)((uint16_t)nAduxRxData & nMask) |
	              (uint16_t)((uint16_t)nDecimationRate << nBitPos);

	/* Write the current register */
	if (Adux1020DrvRegWrite(nReg, nAduxTxData) != ADUXDrv_SUCCESS) {
		/* Update the error status */
		nRetCode |= (uint16_t)ADUXDrv_ERROR;
	}
	return (int16_t)nRetCode;
}

/**
  * @brief Adux1020DrvGetDecimationRate, get the decimation rate of
  * gesture/sample mode
  * @param *pDecimationRate - Pointer variable to get the copy of decimation
  *                        rate.
  * @return int16_t A 16-bit integer: 0 - success; < 0 - failure
  */
int16_t Adux1020DrvGetDecimationRate(uint8_t *pDecimationRate)
{
	/* Storing the error status to give feedback to called function */
	int16_t nRetCode = ADUXDrv_SUCCESS;

	/* Define the nMask variable to assign the mask bits */
	uint16_t nMask = 0U;

	/* Define the nReg variable to assign the register address */
	uint16_t nReg = 0U;

	/* Define the nBitPos variable for assign bit position value */
	uint16_t nBitPos = 0U;

	/* Define the nAduxRxData variable for store the register value */
	uint16_t nAduxRxData = 0U;

	/* Sampling decimation mask value assign to nMask variable */
	nMask = GESTURE_DECIMATION_MASK;

	/* Sampling decimation bit position assign to nBitPos variable*/
	nBitPos = GESTURE_DECIMATION_BITPOS;

	/* Sampling decimation register address assign to nReg variable */
	nReg = REG_DECIMATION;

	/* Read the current register value */
	if(Adux1020DrvRegRead(nReg, &nAduxRxData) != ADUXDrv_SUCCESS) {
		/* Update the error status */
		nRetCode = ADUXDrv_ERROR;
	}

	/* Assign the sampling decimation value to function output
	 * variable
	 */
	*pDecimationRate = (uint8_t)(nAduxRxData &
	                             (uint16_t)(~nMask)) >> nBitPos;
	return nRetCode;
}

/**
 * @brief Adux1020DrvSetProximityDecimationRate,Set the sampling decimation
 *        rate.
 * @param   nProxDecimationRate - Specify the decimation as per Datasheet
 *                              - 0 --> 1
 *                              - 1 --> 2
 *                              - 2 --> 4
 *                              - 3 --> 8
 *                              - 4 --> 16
 *                              - 5 --> 32
 *
 *
 * @return int16_t A 16-bit integer: 0 - success; < 0 - failure
 */
int16_t Adux1020DrvSetProximityDecimationRate(uint8_t nProxDecimationRate)
{
	/* Storing the error status to give feedback to called function */
	uint16_t nRetCode = (uint16_t)ADUXDrv_SUCCESS;

	/* Define the nMask variable to assign the mask bits */
	uint16_t nMask = 0U;

	/* Define the nReg variable to assign the register address */
	uint16_t nReg = 0U;

	/* Define the nBitPos variable for assign bit position value */
	uint16_t nBitPos = 0U;

	/* Define the nAduxRxData variable for store the register value */
	uint16_t nAduxRxData = 0U;

	/* Define the nAduxTxData variable, carry the register value to be write
	   in the register
	*/
	uint16_t nAduxTxData = 0U;

	/* Proximity decimation mask value assign to nMask variable */
	nMask = PROXI_DECIMATION_MASK;

	/* Proximity decimation bit position assign to nBitPos variable*/
	nBitPos = PROXI_DECIMATION_BITPOS;

	/* Proximity decimation register address assign to nReg variable */
	nReg = REG_DECIMATION;
	/* Read the current register value */
	if(Adux1020DrvRegRead(nReg, &nAduxRxData) != ADUXDrv_SUCCESS) {
		/* Update the error status */
		nRetCode |= (uint16_t)ADUXDrv_ERROR;
	}

	/* Proximity decimation value assign to nAduxTxData variable to
	 * be trasmit to register address
	 */
	nAduxTxData = (uint16_t)((uint16_t)nAduxRxData & nMask) |
	              (uint16_t)((uint16_t)nProxDecimationRate << nBitPos);

	/* Write the current register */
	if (Adux1020DrvRegWrite(nReg, nAduxTxData) != ADUXDrv_SUCCESS) {
		/* Update the error status */
		nRetCode |= (uint16_t)ADUXDrv_ERROR;
	}
	return (int16_t)nRetCode;
}

/**
  * @brief Adux1020DrvGetProximityDecimationRate, get the decimation rate of
  * proximity mode
  * @param *pProxDecimationRate - Pointer variable to get the copy of decimation
  *                        rate.
  * @return int16_t A 16-bit integer: 0 - success; < 0 - failure
  */
int16_t Adux1020DrvGetProximityDecimationRate(uint8_t *pProxDecimationRate)
{
	/* Storing the error status to give feedback to called function */
	int16_t nRetCode = ADUXDrv_SUCCESS;

	/* Define the nMask variable to assign the mask bits */
	uint16_t nMask = 0U;

	/* Define the nReg variable to assign the register address */
	uint16_t nReg = 0U;

	/* Define the nBitPos variable for assign bit position value */
	uint16_t nBitPos = 0U;

	/* Define the nAduxRxData variable for store the register value */
	uint16_t nAduxRxData = 0U;

	/* Proximity decimation mask value assign to nMask variable */
	nMask = PROXI_DECIMATION_MASK;

	/* Proximity decimation bit position assign to nBitPos variable*/
	nBitPos = PROXI_DECIMATION_BITPOS;

	/* Proximity decimation register address assign to nReg variable */
	nReg = REG_DECIMATION;

	/* Read the current register value */
	if(Adux1020DrvRegRead(nReg, &nAduxRxData) != ADUXDrv_SUCCESS) {
		/* Update the error status */
		nRetCode = ADUXDrv_ERROR;
	}

	/* Assign the prximity decimation value to function output
	 * variable
	 */
	*pProxDecimationRate = (uint8_t)(nAduxRxData &
	                                 (uint16_t)(~nMask)) >> nBitPos;
	return nRetCode;
}

/**
 * @brief Set the ADC offset
 *
 * @param        nAdcOffset - 0 to 0x3FFF -> Number to subtract from the
 *                                              raw ADC value
 * @param        nChannel -   1 to 4 (CH1 - CH4)
 *
 * @return int16_t A 16-bit integer: 0 - success; < 0 - failure
 */
int16_t Adux1020DrvSetAdcOffset(uint16_t nAdcOffset, uint16_t nChannel)
{
	/* Storing the error status to give feedback to called function */
	uint16_t nRetCode = (uint16_t)ADUXDrv_SUCCESS;

	/* Define the nMask variable to assign the mask bits */
	uint16_t nMask = 0U;

	/* Define the nReg variable to assign the register address */
	uint16_t nBitPos = 0U;

	/* Define the nBitPos variable for assign bit position value */
	uint16_t nReg = 0U;

	/* Define the nAduxRxData variable for store the register value */
	uint16_t nAduxRxData = 0U;

	/* Define the nAduxTxData variable, carry the register value to be write
	   in the register
	*/
	uint16_t nAduxTxData = 0U;

	/* ADC mask value assign to nMask variable */
	nMask = ADC_MASK;

	/* ADC bit position assign to nBitPos variable*/
	nBitPos = ADC_MASK_BITPOS;

	/* ADC register address assign to nReg variable */
	nReg = REG_CH1_OFFSET + nChannel;

	/* Verify the ADC channel list address is correct */
	if ((nReg < REG_CH1_OFFSET) || (nReg > REG_CH4_OFFSET)) {
		/* Update the error status */
		nRetCode |= (uint16_t)ADUXDrv_ERROR;
	}

	/* Read the current register value */
	if(Adux1020DrvRegRead(nReg, &nAduxRxData) != ADUXDrv_SUCCESS) {
		/* Update the error status */
		nRetCode |= (uint16_t)ADUXDrv_ERROR;
	}

	/* ADC value assign to nAduxTxData variable to be trasmit to register
	 * address
	 */
	nAduxTxData = (uint16_t)((uint16_t)nAduxRxData & nMask) |
	              (uint16_t)((uint16_t)nAdcOffset << nBitPos);

	/* Write the current register */
	if (Adux1020DrvRegWrite(nReg, nAduxTxData) != ADUXDrv_SUCCESS) {
		/* Update the error status */
		nRetCode |= (uint16_t)ADUXDrv_ERROR;
	}
	return (int16_t)nRetCode;
}


/**
 * @brief Get the ADC offset
 *
 * @param      *pAdcOffset - 0 to 0x3FFF -> Number to subtract from the
 *                                              raw ADC value
 * @param        nChannel -  1 to 4 (CH1 - CH4)
 *
 * @return int16_t A 16-bit integer: 0 - success; < 0 - failure
 */
int16_t Adux1020DrvGetAdcOffset(uint16_t *pAdcOffset, uint16_t nChannel)
{
	/* Storing the error status to give feedback to called function */
	int16_t nRetCode = ADUXDrv_SUCCESS;

	/* Define the nMask variable to assign the mask bits */
	uint16_t nMask = 0U;

	/* Define the nReg variable to assign the register address */
	uint16_t nBitPos = 0U;

	/* Define the nBitPos variable for assign bit position value */
	uint16_t nReg = 0U;

	/* Define the nAduxRxData variable for store the register value */
	uint16_t nAduxRxData = 0U;

	/* ADC mask value assign to nMask variable */
	nMask = ADC_MASK;

	/* ADC bit position assign to nBitPos variable*/
	nBitPos = ADC_MASK_BITPOS;

	/* ADC register address assign to nReg variable */
	nReg = REG_CH1_OFFSET + nChannel;

	/* Verify the ADC channel list address is correct */
	if ((nReg < REG_CH1_OFFSET) || (nReg > REG_CH4_OFFSET)) {
		/* Update the error status */
		nRetCode = ADUXDrv_ERROR;
	}

	/* Read the current register value */
	if (Adux1020DrvRegRead(nReg, &nAduxRxData) != ADUXDrv_SUCCESS) {
		/* Update the error status */
		nRetCode = ADUXDrv_ERROR;
	}

	/* Assign the ADC channel value to function output variable */
	*pAdcOffset = (nAduxRxData & (uint16_t)(~nMask)) >> nBitPos;
	return nRetCode;
}

/**
 * @brief Set the Proximity thershold level
 *
 * @param      nThreshold       - Thershold value
 *                              - 0 - 0xFFFF -> Number to set Threshold value
 * @param      nThresholdId     - Thershold Id for which trigger level to be set
 *                              - 1 --> ON1  Threshold Trigger Level
 *                              - 2 --> OFF1 Threshold Trigger Level
 *			        - 3 --> ON2  Threshold Trigger Level
 *			        - 4 --> OFF2 Threshold Trigger Level
 *
 * @return int16_t A 16-bit integer: 0 - success; < 0 - failure
*/
int16_t Adux1020DrvSetProximityOnOffThreshold(uint16_t nThreshold,
        uint8_t nThresholdId)
{
	/* Storing the error status to give feedback to called function */
	uint16_t nRetCode = (uint16_t)ADUXDrv_SUCCESS;

	/* Define the nBitPos variable for assign bit position value */
	uint16_t nReg = 0U;

	/* Define the nAduxTxData variable, carry the register value to be write
	    in the register
	 */
	uint16_t nAduxTxData = 0U;

	nReg = REG_PROX_TH_ON1 + ((uint16_t)nThresholdId - 1U);

	if ((nReg < REG_PROX_TH_ON1) || (nReg > REG_PROX_TH_OFF2)) {
		nRetCode |= (uint16_t)ADUXDrv_ERROR;
	}

	/* Threshold value assign to nAduxTxData variable to be trasmit to
	 * register address
	 */
	nAduxTxData = nThreshold;

	/* Write the current register */
	if (Adux1020DrvRegWrite(nReg, nAduxTxData) != ADUXDrv_SUCCESS) {
		/* Update the error status */
		nRetCode |= (uint16_t)ADUXDrv_ERROR;
	}
	return (int16_t)nRetCode;
}

/**
 * @brief Get the Proximity thershold level
 *
 * @param      nThresholdId     - Thershold Id for which trigger level to be set
 *                              - 1 --> ON1  Threshold Trigger Level
 *                              - 2 --> OFF1 Threshold Trigger Level
 *                              - 3 --> ON2  Threshold Trigger Level
 *                              - 4 --> OFF2 Threshold Trigger Level
 *
 * @param     *pThreshold - Get the copy of thershold value from register  
 *
 * @return int16_t A 16-bit integer: 0 - success; < 0 - failure
*/
int16_t Adux1020DrvGetProximityOnOffThreshold(uint8_t nThresholdId,
        uint16_t *pThreshold)
{
	/* Storing the error status to give feedback to called function */
	int16_t nRetCode = ADUXDrv_SUCCESS;

	/* Define the nBitPos variable for assign bit position value */
	uint16_t nReg = 0U;

	/* Define the nAduxRxData variable for store the register value */
	uint16_t nAduxRxData = 0U;

	nReg = REG_PROX_TH_ON1 + ((uint16_t)nThresholdId - 1U);

	if ((nReg < REG_PROX_TH_ON1) || (nReg > REG_PROX_TH_OFF2)) {
		nRetCode = ADUXDrv_ERROR;
	}


	/* Read the current register value */
	if (Adux1020DrvRegRead(nReg, &nAduxRxData) != ADUXDrv_SUCCESS) {
		/* Update the error status */
		nRetCode = ADUXDrv_ERROR;
	}

	/* Assign the proximity threshold value to function output variable */
	*pThreshold = nAduxRxData;
	return nRetCode;
}
#endif
