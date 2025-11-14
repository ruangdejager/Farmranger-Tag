/*
 * Acc.c
 *
 *  Created on: Nov 14, 2025
 *      Author: Ruan de Jager
 */

#include "Acc.h"
#include "platform.h"

// SPI
#define SPI_READ_NWRITE_BP			7
#define SPI_MULTI_NSINGLE_BP		6

// Nominal registers setup (and suggested configuration sequence)
static const acc_reg_config_t AccRegConfig_P[] =
{
	{ ACC_CTRL_REG0,		0b00010000},		// default
	{ ACC_TEMP_CFG_REG,		0b00000000},		// default
	{ ACC_CTRL_REG1,		0b00110111},		// high res mode (low power mode disabled), output data rate = 25Hz, XYZ enabled
	{ ACC_CTRL_REG2,		0b00000000},		// default (HPF normal mode, HPF cutoff freq ?, internal filter bypassed, HPF disabled for click & AOIs)
	{ ACC_CTRL_REG3,		0b00000010},		// FIFO overrun enabled on INT1
	{ ACC_CTRL_REG4,		0b00000000},		// default (BDU continuous update, LSB at lower address, high-res disabled, self-test disabled, 4-wire SPI mode)
	{ ACC_CTRL_REG5,		0b01000000},		// FIFO enabled, int request not latched on INTx, 4D(s) disabled on INTx
	{ ACC_CTRL_REG6,		0b00000000},		// default (click & IAx & boot & ACT interrupts disabled, int polarity active high)
	{ ACC_REFERENCE,		0b00000000},		// default
	{ ACC_FIFO_CTRL_REG, 	0b10000000},		// FIFO mode = stream, event trigger on INT1
	{ ACC_INT1_CFG,			0b00000000},		// default
	{ ACC_INT1_THS,			0b00000000},		// default
	{ ACC_INT1_DUR,			0b00000000},		// default
	{ ACC_INT2_CFG,			0b00000000},		// default
	{ ACC_INT2_THS,			0b00000000},		// default
	{ ACC_INT2_DUR,			0b00000000},		// default
	{ ACC_CLICK_CFG,		0b00000000},		// default
	{ ACC_CLICK_THS,		0b00000000},		// default
	{ ACC_TIME_LIMIT,		0b00000000},		// default
	{ ACC_TIME_LATENCY,		0b00000000},		// default
	{ ACC_TIME_WINDOW,		0b00000000},		// default
	{ ACC_ACT_THS,			0b00000000},		// default
	{ ACC_ACT_DUR,			0b00000000},		// default
	{ ACC_CTRL_REG5,		0b01000000},		// Repeat, as per suggested startup procedure in AN3308
};

// Field-proven problematic registers setup
// This is a reg dump made from 767189 in May 2022 after it came back from the
// field with a sensor error. The uP was able to read the ID from the sensor,
// but the sensor did not stream any data. The reg dump revealed that CTRL_REG5
// (0x24) was set to an incorrect value, resulting in the FIFO disabled.

static const acc_reg_config_t AccRegConfigErr_P[] =
{
	{ ACC_CTRL_REG0,		0x10},		// OK
	{ ACC_TEMP_CFG_REG,		0x00},		// OK
	{ ACC_CTRL_REG1,		0x37},		// OK
	{ ACC_CTRL_REG2,		0x00},		// OK
	{ ACC_CTRL_REG3,		0x02},		// OK
	{ ACC_CTRL_REG4,		0x00},		// OK
	{ ACC_CTRL_REG5,		0x00},		// ERROR, should be 0x40 (i.e. FIFO is disabled and should be enabled
	{ ACC_CTRL_REG6,		0x00},		// OK
	{ ACC_REFERENCE,		0x00},		// OK
	{ ACC_FIFO_CTRL_REG, 	0x80},		// OK
	{ ACC_INT1_CFG,			0x00},		// OK
	{ ACC_INT1_THS,			0x00},		// OK
	{ ACC_INT1_DUR,			0x00},		// OK
	{ ACC_INT2_CFG,			0x00},		// OK
	{ ACC_INT2_THS,			0x00},		// OK
	{ ACC_INT2_DUR,			0x00},		// OK
	{ ACC_CLICK_CFG,		0x00},		// OK
	{ ACC_CLICK_THS,		0x00},		// OK
	{ ACC_TIME_LIMIT,		0x00},		// OK
	{ ACC_TIME_LATENCY,		0x00},		// OK
	{ ACC_TIME_WINDOW,		0x00},		// OK
	{ ACC_ACT_THS,			0x00},		// OK
	{ ACC_ACT_DUR,			0x00},		// OK
	{ ACC_CTRL_REG5,		0x00},		// ERROR (repeat), should be 0x40 (i.e. FIFO is disabled and should be enabled
};

// Local definitions
//uint8_t _u8DeviceReadReg(uint8_t u8RegAddr);
void _vDeviceReadRegArray(uint8_t u8RegAddrStart, uint8_t * pau8Buf, uint8_t u8NumBytes);
void _vDeviceWriteReg(uint8_t u8RegAddr, uint8_t u8RegData);
void _vDeviceRegsConfig(const acc_reg_config_t *pRegConfig_P, uint8_t u8RegCOnfigSize);

/**
 * Init the ACC device.
 *
 * Init the ACC device. The device is initialized by configuring the required
 * registers. Previously a "reboot memory content" operation was considered (BOOT
 * bit in CTRL_REG5), but this does not seem to be a method employed in design
 * examples.
 */
void ACC_vInit(void)
{
	// Only do reg config; do not issue boot command
	_vDeviceRegsConfig(AccRegConfig_P, sizeof(AccRegConfig_P)/sizeof(acc_reg_config_t));
}

/**
 * Read a number of sequential registers on the SPI bus.
 *
 */
void _vDeviceReadRegArray(uint8_t u8RegAddrStart, uint8_t * pau8Buf, uint8_t u8NumBytes)
{
	uint8_t u8Data;

	u8RegAddrStart &= ACC_SPI_ADDR_MASK;
	u8Data = (1 << SPI_READ_NWRITE_BP) | (1 << SPI_MULTI_NSINGLE_BP) | u8RegAddrStart;
	HAL_SPI_ACC_vSelect();
	HAL_SPI_ACC_vSpiWritePacket(&u8Data, 1);
	HAL_SPI_ACC_vSpiReadPacket(pau8Buf, u8NumBytes);
	HAL_SPI_ACC_vDeselect();
}

/**
 * Read a register on the SPI bus.
 *
 */
uint8_t _u8DeviceReadReg(uint8_t u8RegAddr)
{
	uint8_t u8Data;

	_vDeviceReadRegArray(u8RegAddr, &u8Data, 1);

	return u8Data;
}

/**
 * Write a register on the SPI bus.
 *
 */
void _vDeviceWriteReg(uint8_t u8RegAddr, uint8_t u8RegData)
{
	uint8_t u8Data;

	u8RegAddr &= ACC_SPI_ADDR_MASK;
	u8Data = (0 << SPI_READ_NWRITE_BP) | (0 << SPI_MULTI_NSINGLE_BP) |  u8RegAddr;

	HAL_SPI_ACC_vSelect();
	HAL_SPI_ACC_vSpiWritePacket(&u8Data, 1);
	HAL_SPI_ACC_vSpiWritePacket(&u8RegData, 1);
	HAL_SPI_ACC_vDeselect();

}

/**
 * Write an arbitrary register configuration.
 *
 * Write an arbitrary register configuration. The register values and write
 * sequence is defined by an array of structs (reg name-value pairs) in flash.
 *
 * @param	pRegConfig_P	Pointer to array of structs (reg name-value pairs) in flash
 * @param	u8RegCOnfigSize		Number of structs in the array
 */
void _vDeviceRegsConfig(const acc_reg_config_t *pRegConfig_P, uint8_t u8RegCOnfigSize)
{
	uint8_t i;

	for (i=0; i<u8RegCOnfigSize; i++) {
		_vDeviceWriteReg(pRegConfig_P[i].u8Name , pRegConfig_P[i].u8Value);
	}
}



/**
 * Read the device ID register.
 *
 */
uint8_t ACC_u8GetDeviceId(void)
{
	return _u8DeviceReadReg(ACC_WHO_AM_I);
}

/**
 * Check the device ID.
 *
 * Check the device ID. A affirmative response confirms that the device ID can
 * be read (and thus by extension correct SPI interface operation), and that the
 * device is in fact the ACC.
 */
bool ACC_bDeviceIdOk(void)
{
	return (ACC_u8GetDeviceId() == ACC_WHO_AM_I_VALUE);
}

/**
 * Get the number of samples available in the FIFO.
 *
 */
uint8_t ACC_u8NumSamplesInFifo(void)
{
	return (_u8DeviceReadReg(ACC_FIFO_SRC_REG) & 0x1F);
}

/**
 * Get the next available acceleration data (XYZ).
 *
 */
void ACC_vGetAccSample(acc_t * pAcc)
{
	_vDeviceReadRegArray(0x28, (uint8_t*)pAcc, 6);
}

/**
 * Configure the device with a known error configuration.
 *
 * Configure the device with a known error configuration. This is intended as a
 * test function, in order to confirm that the device can successfully recover
 * nominal operation.
 */
void ACC_vTestRegsConfigError(void)
{
	_vDeviceRegsConfig( AccRegConfigErr_P, sizeof(AccRegConfigErr_P)/sizeof(acc_reg_config_t) );
}
