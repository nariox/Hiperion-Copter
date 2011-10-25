/*
 * I2C.h
 *
 *  Created on: 25/10/2011
 *      Author: traysh
 */

#ifndef I2C_H_
#define I2C_H_

#include "driver_config.h"
#include "type.h"
#include "i2c.h"

extern volatile uint8_t I2CMasterBuffer[BUFSIZE];
extern volatile uint8_t I2CSlaveBuffer[BUFSIZE];
extern volatile uint32_t I2CReadLength, I2CWriteLength;

void i2c_inicializa();

#endif /* I2C_H_ */
