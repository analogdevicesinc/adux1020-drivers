#ifndef __AduxDrv_H__
#define __AduxDrv_H__

#include <stdint.h>
typedef enum {
	ADUXDrv_ERROR = -1,
	ADUXDrv_SUCCESS = 0
} AduxDrv_Error_Status_t;

typedef enum {
	ADUXDrv_MODE_OFF = 0,
	ADUXDrv_MODE_IDLE,
	ADUXDrv_MODE_SAMPLE,
	ADUXDrv_MODE_PROXIMITY
} AduxDrv_Operation_Mode_t;

typedef enum {
	FIFO_LEVEL = 0,
	INT_STATUS
} AduxDrvParameterList;

/* Register List */
#define REG_CHIP_ID                             0x08U
#define REG_OSC_CAL_OUT	                        0x0AU
#define REG_SW_RESET		                0x0FU

#define REG_OSCS_1            	                0x18U
#define REG_OSCS_3            	                0x1AU
#define REG_INT_ENABLE                          0x1CU
#define REG_INT_POLARITY                        0x1DU
#define	REG_I2C_1		                0x1EU
#define	REG_I2C_2		                0x1FU
#define REG_GEST_LED_WIDTH                      0x20U
#define REG_GEST_LED_PERIOD                     0x21U
#define REG_PROX_LED_WIDTH                      0x22U
#define REG_PROX_LED_PERIOD                     0x23U
#define REG_GEST_AFE                            0x25U
#define REG_PROX_AFE                            0x26U
#define	REG_GEST_DI_TH		                0x28U
#define REG_GEST_ORIEN_NPTS	                0x29U
#define REG_PROX_TH_ON1                         0x2AU
#define REG_PROX_TH_OFF1                        0x2BU
#define REG_PROX_TH_ON2                         0x2CU
#define REG_PROX_TH_OFF2                        0x2DU
#define	REG_PROX_TYPE		                0x2FU
#define	REG_TEST_MODES_1	                0x30U
#define	REG_TEST_MODES_3	                0x32U

#define	REG_FORCE_MODE_ADDR	                0x33U
#define REG_CH1_OFFSET                          0x3AU
#define REG_CH2_OFFSET                          0x3BU
#define REG_CH3_OFFSET                          0x3CU
#define REG_CH4_OFFSET                          0x3DU
#define	REG_FREQUENCY		                0x40U
#define REG_LED_CURRENT                         0x41U
#define	REG_OP_MODE_ADDR	                0x45U
#define	REG_DECIMATION		                0x46U
#define	REG_INT_MASK	                        0x48U
#define	REG_INT_STATUS	                        0x49U
#define	REG_DATA_BUFFER	                        0x60U

/* Gesture Sampling Frequency */
#define GESTURE_SAMPLING_FREQ_MASK		0xFFF0U
#define GESTURE_SAMPLING_FREQ_BITPOS	        0U

/* Proximity Sampling Frequency */
#define PROXI_SAMPLING_FREQ_MASK		0xFF0FU
#define PROXI_SAMPLING_FREQ_BITPOS		4U

/* Gesture Decimation */
#define GESTURE_DECIMATION_MASK			0xFF8FU
#define GESTURE_DECIMATION_BITPOS		4U

/* Proximity Decimation */
#define PROXI_DECIMATION_MASK			0xFFF8U
#define PROXI_DECIMATION_BITPOS			0U

/* MODE */
#define MODE_MASK                               0xFFFCU
#define MODE_BITPOS                             0U


/*LED Current */
#define LED_CURRENT_MASK                        0xFFF0U
#define LED_CURRENT_BITPOS                      0U

/* LED Scale Factor */
#define LED_SCALE_FACTOR_MASK                   0xDFFFU
#define LED_SCALE_FACTOR_BITPOS                 13U

/* LED Trim */
#define LED_1_TRIM_MASK                         0xFFE0U
#define LED_2_TRIM_MASK                         0xF83FU
#define LED_1_TRIM_BITPOS                       0U
#define LED_2_TRIM_BITPOS                       6U

/* LED Width */
#define LED_WIDTH_MASK                          0xE0FFU
#define LED_WIDTH_BITPOS                        8U

#define LED_WIDTH_LOW_POWER                     3U
#define LED_WIDTH_HIGH_POWER                    4U

/* LED OFFSET */
#define LED_OFFSET_MASK                         0xFF80U
#define LED_OFFSET_BITPOS                       0U

/* LED Number of Pulses */
#define LED_NUMBER_PULSES_MASK                  0x00FFU
#define LED_NUMBER_PULSES_BITPOS                8U

/* LED PERIOD */
#define LED_PERIOD_MASK                         0xFF00U
#define LED_PERIOD_BITPOS                       0U

/* AFE Width */
#define AFE_WIDTH_MASK                          0x07FFU
#define AFE_WIDTH_BITPOS                        11U


/*AFE FINE OFFSET */
#define AFE_FINE_OFFSET_MASK                    0xFFE0U
#define AFE_FINE_OFFSET_BITPOS                  0U

/*AFE OFFSET */
#define AFE_OFFSET_MASK                         0xF81FU
#define AFE_OFFSET_BITPOS                       5U

/* AFE TIA Gain*/
#define AFE_TIA_GAIN_MASK                       0xFFFCU
#define AFE_TIA_GAIN_BITPOS                     0U
#define AFE_TIA_GAIN_50K                        2U
#define AFE_TIA_GAIN_100K                       1U
#define AFE_TIA_GAIN_200K                       0U

/* AFE BPF */
#define AFE_TIA_BPF_MASK                        0xFFF3U
#define AFE_TIA_BPF_BITPOS                      2U

/* AFE  QR_DELAY */
#define AFE_QR_DELAY_MASK                       0xFFC0U
#define AFE_QR_DELAY_BITPOS                     0U


/*	OSC32K	*/
#define OSC32K_EN_MASK		                0xFF7FU
#define OSC32K_EN_BITPOS	                7U
#define OSC32K_TRIM_MASK	                0xFFC0U
#define OSC32K_TRIM_BITPOS	                0U

/* ADC OFFSET */
#define ADC_MASK                                0xC000U
#define ADC_MASK_BITPOS			        0U


/*	FIFO	*/
#define FIFO_LENGTH_THRESHOLD_MASK	        0x80FFU
#define FIFO_LENGTH_THRESHOLD_BITPOS	        0x08U
#define FIFO_CLR                                0x8000U
#define FIFO_INT_EN                             0xC07FU

#define	FORCE_CLOCK_ON			        0xF4FU
#define	FORCE_CLOCK_RESET		        0x40U

/* OPERATION */
#define MODE_OFF                                0x0000U
#define MODE_IDLE                               0x000FU
#define SAMPLE_MODE_GO                          0x0148U
#define PROXIMITY_MODE_GO                       0x0111U
#define FORCE_MODE                              0x000EU

#define TRIGGER_ON_CROSS_THRESHOLD              0x0000U
#define TRIGGER_ON_BOTH_THRESHOLD               0x8000U

#define PROXIMITY_INT_ENABLE                    0xF0U
#define FIFO_INT_ENABLE                         0x7FU
#define INT_DISABLE                             0xFFU
#define INT_CLEAR                               0x00FFU
#define FIFO_RESET                              0x8000U
#define ACTIVE_4_STATE                          0x0008U

/**
@brief Sample Interrupt mask value
*/
#define IRQ_MASK_SAMPLE                         0x00C0U
#define IRQ_MASK_PROXIMITY                      0x000FU

extern void Adux1020DrvDataReadyCallback(void (*pfAduxDataReady)(void));
extern int16_t Adux1020DrvOpenDriver(void);
extern int16_t Adux1020DrvCloseDriver(void);
extern int16_t Adux1020DrvRegWrite(uint16_t nAddr, uint16_t nRegValue);
extern int16_t Adux1020DrvRegRead(uint16_t nAddr, uint16_t *pnData);
extern int16_t Adux1020DrvSetOperationMode(uint8_t nOperationMode);
extern int16_t Adux1020DrvGetParameter(AduxDrvParameterList eCommands,
                                       uint16_t *pnValue);
extern int16_t Adux1020DrvSetInterrupt(uint16_t nIntMask);
extern int16_t Adux1020DrvSetFifoLevel(void);
extern void Adux1020DrvISR(void);
extern void Adux1020DrvSoftReset(void);
extern int16_t Adux1020DrvEnable32MHzClock(void);
extern int16_t Adux1020DrvDisable32MHzClock(void);
extern int16_t Adux1020DrvReadFifoData(uint8_t *pnData, uint16_t nDataSetSize);
extern int16_t Adux1020DrvSetLedcurrent(uint8_t nLedCurrent);
extern int16_t Adux1020DrvGetLedcurrent(uint8_t *pLedCcurrent);
extern int16_t Adux1020DrvSetLedOffset(uint8_t nLedOffset, uint16_t nMode);
extern int16_t Adux1020DrvGetLedOffset(uint16_t nMode, uint8_t *pLedOffset);
extern int16_t Adux1020DrvSetLedWidth(uint8_t nLedWidth, uint16_t nMode);
extern int16_t Adux1020DrvGetLedWidth(uint16_t nMode, uint8_t *pLedWidth);
extern int16_t Adux1020DrvSetLedPeriod(uint8_t nLedPeriod, uint16_t nMode);
extern int16_t Adux1020DrvGetLedPeriod(uint16_t nMode, uint8_t *pLedPeriod);
extern int16_t Adux1020DrvSetLedPulse(uint8_t nLedPulse, uint16_t nMode);
extern int16_t Adux1020DrvGetLedPulse(uint16_t nMode, uint8_t *nLedPulse);
extern int16_t Adux1020DrvSetAfeWidth(uint16_t nAfeWidth, uint16_t nMode);
extern int16_t Adux1020DrvGetAfeWidth(uint16_t nMode, uint8_t *pAfeWidth);
extern int16_t Adux1020DrvSetAfeOffset(uint16_t nAfeOffset, uint16_t nMode);
extern int16_t Adux1020DrvGetAfeOffset(uint16_t nMode, uint8_t *pAfeOffset);
extern int16_t Adux1020DrvSetAfeFineOffset(uint16_t nAfeFineOffset,
        uint16_t nMode);
extern int16_t Adux1020DrvGetAfeFineOffset(uint16_t nMode,
        uint8_t *pAfeFineOffset);
extern int16_t Adux1020DrvSetSamplingFrequency(uint8_t nSamplingRate);
extern int16_t Adux1020DrvGetSamplingFrequency(uint8_t *pSamplingRate);
extern int16_t Adux1020DrvSetProximitySamplingFrequency(
    uint8_t nProxSamplingRate);
extern int16_t Adux1020DrvGetProximitySamplingFrequency(
    uint8_t *pProxSamplingRate);
extern int16_t Adux1020DrvSetDecimationRate(uint8_t nDecimationRate);
extern int16_t Adux1020DrvGetDecimationRate(uint8_t *pDecimationRate);
extern int16_t Adux1020DrvSetProximityDecimationRate(uint8_t
        nProxDecimationRate);
extern int16_t Adux1020DrvGetProximityDecimationRate(uint8_t
        *pProxDecimationRate);
extern int16_t Adux1020DrvSetAdcOffset(uint16_t nAdcOffset, uint16_t nChannel);
extern int16_t Adux1020DrvGetAdcOffset(uint16_t *pAdcOffset, uint16_t nChannel);
extern int16_t Adux1020DrvSetProximityOnOffThreshold(uint16_t nThreshold,
        uint8_t nThresholdId);
extern int16_t Adux1020DrvGetProximityOnOffThreshold( uint8_t nThresholdId,
        uint16_t *pThreshold);
#endif
