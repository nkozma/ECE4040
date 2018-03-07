/*
 * All software, source code, included documentation, and any implied know-how
 * are property of Freescale Semiconductor and therefore considered
 * CONFIDENTIAL INFORMATION.
 * This confidential information is disclosed FOR DEMONSTRATION PURPOSES ONLY.
 *
 * All Confidential Information remains the property of Freescale Semiconductor
 * and will not be copied or reproduced without the express written permission
 * of the Discloser, except for copies that are absolutely necessary in order
 * to fulfill the Purpose.
 *
 * Services performed by FREESCALE in this matter are performed AS IS and
 * without any warranty.
 * CUSTOMER retains the final decision relative to the total design and
 * functionality of the end product.
 * FREESCALE neither guarantees nor will be held liable by CUSTOMER for the
 * success of this project.
 *
 * FREESCALE disclaims all warranties, express, implied or statutory
 * including, but not limited to, implied warranty of merchantability or
 * fitness for a particular purpose on any hardware, software ore advise
 * supplied to the project by FREESCALE, and or any product resulting from
 * FREESCALE services.
 * In no event shall FREESCALE be liable for incidental or consequential
 * damages arising out of this agreement. CUSTOMER agrees to hold FREESCALE
 * harmless against any and all claims demands or actions by anyone on account
 * of any damage,or injury, whether commercial, contractual, or tortuous,
 * rising directly or indirectly as a result of the advise or assistance
 * supplied CUSTOMER in connectionwith product, services or goods supplied
 * under this Agreement.
 */

#include "i2c.h"
#include "fsl_device_registers.h"

unsigned char MasterTransmission;

/*
 * I2C Initialization
 * Set Baud Rate and turn on I2C0
 */
void init_I2C(void)
{
	SIM_SCGC4 |= SIM_SCGC4_I2C1_MASK;	//Turn on clock to I2C` module

	/* configure GPIO for I2C0 function */
	PORTC_PCR10 |= PORT_PCR_MUX(2) | PORT_PCR_ODE_MASK;
	PORTC_PCR11 |= PORT_PCR_MUX(2) | PORT_PCR_ODE_MASK;

	I2C1_F |= 0xD;					/* Baud = 390 kHz */
	I2C1_C1 |= I2C_C1_IICEN_MASK;	/* enable I2C */
	i2c_DisableAck();
}

/*
 * Start I2C Transmision
 * @param Address is the 7 bit Slave Address
 * @param Mode sets Read or Write Mode
 */
void I2C_StartTransmission(uint8_t Address, uint8_t Mode)
{
	uint8_t SlaveID;

	if (Mode == MWSR) {
		/* set transmission mode */
		MasterTransmission = MWSR;
	} else {
		/* set transmission mode */
		MasterTransmission = MRSW;
	}

	/* shift ID in right position */
	SlaveID = Address << 1;

	/* Set R/W bit at end of Slave Address */
	SlaveID |= (uint8_t)MasterTransmission;

	/* send start signal */
	i2c_Start();

	/* send ID with W/R bit */
	i2c_write_byte(SlaveID);
}

/* Pause Routine */
void Pause(void)
{
	int n;
	for (n = 1; n < 50; n++) {
	}
}

/*
 * Read a register
 * @param SubAddress is Register Address
 * @return Data stored in Register
 */
void I2CReadRegister(uint8_t Address, uint8_t SubAddress, uint8_t *dest)
{
	unsigned char result;

	/* Send Slave Address */
	I2C_StartTransmission(Address, MWSR);
	i2c_Wait();

	/* Write Register Address */
	I2C1_D = SubAddress;
	i2c_Wait();

	/* Do a repeated start */
	I2C1_C1 |= I2C_C1_RSTA_MASK;

	/* Send Slave Address */
	I2C1_D = (Address << 1) | 0x01;	//read address
	i2c_Wait();

	/* Put in Rx Mode */
	I2C1_C1 &= (~I2C_C1_TX_MASK);

	/* Turn off ACK since this is second to last byte being read */
	I2C1_C1 |= I2C_C1_TXAK_MASK;

	/* Dummy read */
	*dest = I2C1_D;
	i2c_Wait();

	/* Send stop since about to read last byte */
	i2c_Stop();

	/* Read byte */
	*dest = I2C1_D;
}

/*
 * Write a byte of Data to specified register
 * @param SubAddress is Register Address
 * @param u8Data is Data to write
 */
void I2CWriteRegister(uint8_t Address, uint8_t SubAddress, uint8_t u8Data)
{
	/* send data to slave */
	I2C_StartTransmission(Address, MWSR);
	i2c_Wait();

	I2C1_D = SubAddress;
	i2c_Wait();

	I2C1_D = u8Data;
	i2c_Wait();

	i2c_Stop();

	Pause();
}

/*
 * Read multiple registers
 * @param SubAddress is Register Address
 * @return Data stored in Register
 */
void I2CReadMultiRegisters(uint8_t Address, uint8_t SubAddress, uint8_t bytes, uint8_t *dest)
{
	uint8_t n = bytes;
	int i;

	/* Send Slave Address */
	I2C_StartTransmission(Address, MWSR);
	i2c_Wait();

	/* Write Register Address */
	I2C1_D = SubAddress;
	i2c_Wait();

	/* Do a repeated start */
	I2C1_C1 |= I2C_C1_RSTA_MASK;

	/* Send Slave Address */
	I2C1_D = (Address << 1) | 0x01;	//read address
	i2c_Wait();

	/* Put in Rx Mode */
	I2C1_C1 &= (~I2C_C1_TX_MASK);

	/* Ensure TXAK bit is 0 */
	I2C1_C1 &= ~I2C_C1_TXAK_MASK;

	/* Dummy read */
	dest[0] = I2C1_D;
	i2c_Wait();

	for (i = 0; i < n - 2; i++) {
		/* Read first byte */
		dest[i] = I2C1_D;
		i2c_Wait();
	}

	/* Turn off ACK since this is second to last read */
	I2C1_C1 |= I2C_C1_TXAK_MASK;

	/* Read second to last byte */
	dest[i++] = I2C1_D;
	i2c_Wait();

	/* Send stop */
	i2c_Stop();

	/* Read last byte */
	dest[i++] = I2C1_D;
}
