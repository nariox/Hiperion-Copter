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
#include "i2c_driver.h"

extern volatile uint8_t I2CMasterBuffer[I2C_BUFSIZE];
extern volatile uint8_t I2CSlaveBuffer[I2C_BUFSIZE];
extern volatile uint32_t I2CReadLength, I2CWriteLength;
void I2C_read(char sla_addr, char reg_addr, char * value);
void I2C_write(char sla_addr, char reg_addr, char value);

#endif /* I2C_H_ */
