/**
    ***************************************************************************
      @file         example1020.c
      @author       ADI
      @version      V0.1
      @date         8-Nov-2016
      @brief        Sample application to use ADI ADUX1020 driver.
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
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <assert.h>
#include <time.h>
#include "Adux1020Drv.h"

/* Enum ---------------------------------------------------------------------*/
typedef enum {
  SAMPLE_MODE = 0,
  PROXIMITY_MODE
} Adux_Operation_Mode;

/* Macros -------------------------------------------------------------------*/

#define BUF_SIZE (256U)
#define debug(M, ...)  {_SBZ[0] = 0; \
                        snprintf(_SBZ, BUF_SIZE, "" M "", ##__VA_ARGS__); \
                          adi_printf("%s", _SBZ);}

char _SBZ[BUF_SIZE]; // used by 'debug'

/* Function Prototype */
static void HWGlobalInit();
void Adux1020FifoCallBack(void);
uint8_t LoadDefaultConfig(uint32_t *pnCfg);
void VerifyDefaultConfig(uint32_t *cfg);
void AduxDriverBringUp(Adux_Operation_Mode);

/* Private variables --------------------------------------------------------*/
uint8_t gnAduxDataReady = 0U;
uint32_t gnAduxTimeCurVal = 0;

/* Configuration file buffer */
uint32_t dcfg[] = {
  0x000c000f,
  0x00101010,
  0x0011004c,
  0x00125f0c,
  0x0013ada5,
  0x00140080,
  0x00150000,
  0x00160600,
  0x00170000,
  0x00182693,
  0x00190004,
  0x001a4280,
  0x001b0060,
  0x001c2094,
  0x001d0020,
  0x001e0001,
  0x001f0000,
  0x00200320,
  0x00210A13,
  0x00220320,
  0x00230113,
  0x00240000,
  0x00252412,
  0x00262412,
  0x00270022,
  0x00280000,
  0x00290300,
  0x002a0700,
  0x002b0600,
  0x002c6000,
  0x002d4000,
  0x002e0000,
  0x002f0000,
  0x00300000,
  0x00310000,
  0x00320040,
  0x00330008,
  0x0034E400,
  0x00388080,
  0x00398080,
  0x003a2000,
  0x003b1f00,
  0x003c2000,
  0x003d2000,
  0x003e0000,
  0x00408069,
  0x00411f2f,
  0x00424000,
  0x00430000,
  0x00440008,
  0x00460000,
  0x004800ef,
  0x00490000,
  0x00450000,
  0xFFFFFFFF
};

/**
*  @brief    Callback function.
*  @param    None
*  @retval   None
*/
void Adux1020FifoCallBack(void)
{
	/* Set gnAduxDataReady to 1 to indicate that the data and timestamp is ready */
	gnAduxDataReady = 1;
	/* Read the timestamp when the interrupt comes */
	gnAduxTimeCurVal = McuHalGetTick();
}

/**
  * Flow diagram of the code *
      ----------------------------
      | Hardware initializations |
      ----------------------------
      |
      |
      ----------------------------
      |  Data ready callback     |
      ----------------------------
      |
      |
      ----------------------------
      |Initialize the ADUX driver|
      ----------------------------
      |
      |
      ----------------------------
      | Load the default config  |
      | and verify it            |
      ----------------------------
      |
      |
      ----------------------------
      | Write standard value of  |
      | clock registers          |
      ----------------------------
      |
      |
      ----------------------------
  --->|     Driver bring up      |
  |   ----------------------------
  |               |
  |               |
  |---------------|
  */

void main(void)
{
	/* Hardware initializations */
	HWGlobalInit();
	/* Register data ready callback */
	Adux1020DrvDataReadyCallback(Adux1020FifoCallBack);
	/* Initialize the Adux1020 driver*/
	Adux1020DrvOpenDriver();
	/* Load default configuration parameters */
	LoadDefaultConfig(dcfg);
	/* Read default configuration parameters from the device registers and verify */
	VerifyDefaultConfig(dcfg);
	/**
           Driver bring up, pass the parameter from Adux_Operation_Mode Enum
            -> SAMPLE_MODE for Sample Mode
            -> PROXIMITY_MODE for Proximity Mode
         */
	AduxDriverBringUp(SAMPLE_MODE);
}

/* Private functions --------------------------------------------------------*/
/**
* @brief    Software Initialization.
* @retval   None
*/
static void SWGlobalInit()
{

}

/**
* @brief    Hardware Initialization.
* @retval   None
*/
static void HWGlobalInit()
{
	/**
	* HAL initializations such as enabling system tick and low level
	* hardware initialization.
	*/
	HAL_Init();
	/* Configure the system clock TO 16 MHz*/
	SystemClockConfig(168U);
	/* Initialize the GPIO. Should be called before I2C_Init() */
	GPIO_Init();
	/* Initialize the UART */
	UartInit();
	/* Initialize the I2C. Should be called after GPIO_Init() */
	I2CInit();
	/* Configure the voltage regulators in proper mode */
	I2C1Init();         //For communicating with ADP5258 and ADP5061
}

/**
* @brief    Load Adux1020 default configuration
* @note     This function will load the default configuration to the device.
* @param    pnCfg - Pointer to the configuration array
* @retval   None
*/
uint8_t LoadDefaultConfig(uint32_t *pnCfg)
{
	uint8_t nRegAddr, nIdx, nRet = -1;
	uint16_t nRegData;

	if (pnCfg == 0)
		return nRet;
	nIdx = 0;

	while (1) {
		/* Read the address and data from the config */
		nRegAddr = (uint8_t)(pnCfg[nIdx] >> 16);
		nRegData = (uint16_t)(pnCfg[nIdx]);
		nIdx++;

		if (nRegAddr == 0xFF)
			break;
		/* Load the data into the ADUX1020 registers */
		if (Adux1020DrvRegWrite(nRegAddr, nRegData) != ADUXDrv_SUCCESS) {
			debug(":FAIL. Set Reg%d=%d\r\n", (int)nRegAddr, (int)nRegData);
			return nRet;
		}
		nRegData = 0;
		Adux1020DrvRegRead(nRegAddr, &nRegData);
		debug("Config:, 0x%X ,  0x%X \r\n", nRegAddr, nRegData);
	}
	nRet = 0;
	debug(":PASS. Load Default Configure pass\r\n");
	return nRet;
}

/**
*  @brief    Read default configuration parameters to verify
*  @param    uint32_t *cfg
*  @retval   None
*/
void VerifyDefaultConfig(uint32_t *cfg)
{
	uint16_t def_val;
	uint8_t  i;
	uint8_t  regAddr;
	uint16_t regData;
	if (cfg == 0) {
		return;
	}
	i = 0;
	/* Read the address and data from the config */
	regAddr = (uint8_t)(cfg[0] >> 16);
	def_val = (uint16_t)(cfg[0]);
	/* Read the data from the ADUX registers and verify */
	while (regAddr != 0xFF) {
		if (Adux1020DrvRegRead(regAddr, &regData) != ADUXDrv_SUCCESS) {
			debug("DCFG: Read Error reg(%0.2x)\r\n", regAddr);
			return;
		} else if (regData != def_val) {
			debug("DCFG: Read mismatch reg(%0.2x) (%0.2x != %0.2x)\r\n",
			      regAddr, def_val, regData);
			return;
		}
		i++;
		regAddr = (uint8_t)(cfg[i] >> 16);
		def_val = (uint16_t)(cfg[i]);
	}
}

/**
  *  @brief  Adux1020 Driver bring up.
  *  @param  eModeOperation - Input to the function, specify which operation 
  *                           mode need to be set.
  *
  *  @retval None
*/
void AduxDriverBringUp(Adux_Operation_Mode eModeOperation)
{
  uint32_t LoopCnt;
  uint16_t nRetValue = 0;
  uint16_t nAduxFifoLevelSize = 0, nAduxDataSetSize = 8U, nIntStatus;
  uint8_t value[8] = {0U}, getFifoData = 0U;
  
  switch(eModeOperation) {
  case SAMPLE_MODE: 
      /* Set the fifo interrupt */
      Adux1020DrvSetInterrupt(FIFO_INT_ENABLE);
      /* Set Fifo level to trigger the interrupt */
      Adux1020DrvSetFifoLevel();
      /* Set the device operation to sample mode. The data can be collected now */
      Adux1020DrvSetOperationMode(ADUXDrv_MODE_SAMPLE);
      
      while (1) {
        /* Check if the data is ready */
        if(gnAduxDataReady)  {
          gnAduxDataReady = 0;
          /* Read the size of the data available in the FIFO */
          Adux1020DrvGetParameter(FIFO_LEVEL, &nAduxFifoLevelSize);
          /* Read the interrupt status */
          Adux1020DrvGetParameter(INT_STATUS, &nIntStatus);
          
          /* Enable the 32MHz Clock */
          if(nAduxFifoLevelSize >= nAduxDataSetSize) {
            Adux1020DrvEnable32MHzClock();
            getFifoData = 1;
          }
          /* Read the data from the FIFO and print them */
          while (nAduxFifoLevelSize >= nAduxDataSetSize) {
            nRetValue = Adux1020DrvReadFifoData(&value[0], nAduxDataSetSize);
            if (nRetValue == ADUXDrv_SUCCESS) {
              for (LoopCnt = 0; LoopCnt < 8; LoopCnt += 2)
                /* Byte swapping is needed to print Adux1020 data in proper format */
                debug("%u ", (value[LoopCnt] << 8) | value[LoopCnt + 1]);
              debug("%u\r\n", gnAduxTimeCurVal);
              //debug("Interrupt Status %04X \r\n", tmp);
              nAduxFifoLevelSize = nAduxFifoLevelSize - nAduxDataSetSize;
            }
          }
          /* Clear the interrupt status, inorder to occure next interrupt */
          if ((nIntStatus & IRQ_MASK_SAMPLE) != 0U) {
            Adux1020DrvRegWrite(REG_INT_STATUS, nIntStatus&IRQ_MASK_SAMPLE);
          }
          /* Disable the 32MHz Clock */
          if(getFifoData) {
            Adux1020DrvDisable32MHzClock();
            getFifoData = 0;
          }
          
        }
      }
      break;
  case PROXIMITY_MODE:
      /* Set the proximity interrupt */
      Adux1020DrvSetInterrupt(PROXIMITY_INT_ENABLE);
      /* Set the device operation to sample mode. The data can be collected now */
      Adux1020DrvSetOperationMode(ADUXDrv_MODE_PROXIMITY);
      while (1) {
        /* Check if the data is ready */
        if(gnAduxDataReady) {
          gnAduxDataReady = 0;
          /* Read the size of the data available in the FIFO */
          Adux1020DrvGetParameter(FIFO_LEVEL, &nAduxFifoLevelSize);
          /* Read the interrupt status */
          Adux1020DrvGetParameter(INT_STATUS, &nIntStatus);
          
          /* Enable the 32MHz Clock */
          if(nAduxFifoLevelSize >= nAduxDataSetSize) {
            Adux1020DrvEnable32MHzClock();
            getFifoData = 1;
          }
          /* Read the data from the FIFO and print them */
          while (nAduxFifoLevelSize >= nAduxDataSetSize) {
            nRetValue = Adux1020DrvReadFifoData(&value[0], nAduxDataSetSize);
            if (nRetValue == ADUXDrv_SUCCESS) {
              LoopCnt = 0;
              debug("%u ", (value[LoopCnt] << 8) | value[LoopCnt + 1]);
              debug("%u\r\n", gnAduxTimeCurVal);
              debug("Interrupt Status %04X \r\n", nIntStatus);
              nAduxFifoLevelSize = nAduxFifoLevelSize - nAduxDataSetSize;
            }
          }
          /* Clear the interrupt status, inorder to occure next interrupt */
          if ((nIntStatus & IRQ_MASK_PROXIMITY) != 0U) {
            Adux1020DrvRegWrite(REG_INT_STATUS, nIntStatus&IRQ_MASK_PROXIMITY);
          }
          /* Disable the 32MHz Clock */
          if(getFifoData) {
            Adux1020DrvDisable32MHzClock();
            getFifoData = 0;
          }
        }
      }
      break;
  default:
      break;
  }
}
