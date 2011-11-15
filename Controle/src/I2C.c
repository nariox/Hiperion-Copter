/*
 * I2C.c
 *
 *  Created on: 25/10/2011
 *      Author: traysh
 */

#include "I2C.h"

extern volatile uint32_t I2CCount;
extern volatile uint32_t I2CMasterState;

void I2C_read(char sla_addr, char reg_addr, char * value) {

	/* Write SLA(W), address, SLA(R), and read one byte back. */
	I2CWriteLength = 2;
	I2CReadLength = 1;
	I2CMasterBuffer[0] = sla_addr;	// Slave I2C address
	I2CMasterBuffer[1] = reg_addr;	// Register address
	I2CMasterBuffer[2] = sla_addr | RD_BIT;
	I2CEngine();

	*value = I2CMasterBuffer[I2CWriteLength+2];
	
}

void I2C_write(char sla_addr, char reg_addr, char value) {

	I2CWriteLength = 3;
	I2CReadLength = 0;
	I2CMasterBuffer[0] = sla_addr;	// Slave I2C address
	I2CMasterBuffer[1] = reg_addr;	// Register address
	I2CMasterBuffer[2] = value;		// Data

	I2CEngine();

}
