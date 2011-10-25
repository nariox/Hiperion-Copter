/*
 * I2C.c
 *
 *  Created on: 25/10/2011
 *      Author: traysh
 */

#include "I2C.h"

extern volatile uint32_t I2CCount;
extern volatile uint32_t I2CMasterState;

void i2c_inicializa() {
	  if ( I2CInit( (uint32_t)I2CMASTER ) == FALSE )	{ /* initialize I2c */
		//TODO: Reportar erro
		while ( 1 );				/* Fatal error */
	  }

}

void i2c_master_read() {
	  uint32_t i;

	  for ( i = 0; i < BUFSIZE; i++ )
	  {
		I2CSlaveBuffer[i] = 0x00;
	  }
	  /* Write SLA(W), address, SLA(R), and read one byte back. */
	  I2CWriteLength = 2;
	  I2CReadLength = 4;
	  I2CMasterBuffer[0] = PCF8594_ADDR;
	  I2CMasterBuffer[1] = 0x00;		/* address */
	  I2CMasterBuffer[2] = PCF8594_ADDR | RD_BIT;
	  I2CEngine();
}

void i2c_master_write() {
	  uint32_t i;

	  /* In order to start the I2CEngine, the all the parameters
	  must be set in advance, including I2CWriteLength, I2CReadLength,
	  I2CCmd, and the I2cMasterBuffer which contains the stream
	  command/data to the I2c slave device.
	  (1) If it's a I2C write only, the number of bytes to be written is
	  I2CWriteLength, I2CReadLength is zero, the content will be filled
	  in the I2CMasterBuffer.
	  (2) If it's a I2C read only, the number of bytes to be read is
	  I2CReadLength, I2CWriteLength is 0, the read value will be filled
	  in the I2CMasterBuffer.
	  (3) If it's a I2C Write/Read with repeated start, specify the
	  I2CWriteLength, fill the content of bytes to be written in
	  I2CMasterBuffer, specify the I2CReadLength, after the repeated
	  start and the device address with RD bit set, the content of the
	  reading will be filled in I2CMasterBuffer index at
	  I2CMasterBuffer[I2CWriteLength+2].

	  e.g. Start, DevAddr(W), WRByte1...WRByteN, Repeated-Start, DevAddr(R),
	  RDByte1...RDByteN Stop. The content of the reading will be filled
	  after (I2CWriteLength + two devaddr) bytes. */

	  /* Write SLA(W), address and one data byte */
	  I2CWriteLength = 18; //Era 6
	  I2CReadLength = 0;
	  I2CMasterBuffer[0] = PCF8594_ADDR;
	  I2CMasterBuffer[1] = 0x00;		/* address */
	  I2CMasterBuffer[2] = 0x56;		/* Data0 */
	  I2CMasterBuffer[3] = 0xAB;		/* Data1 */
	  I2CMasterBuffer[4] = 0x13;		/* Data2 */
	  I2CMasterBuffer[5] = 0x35;		/* Data3 */
	  I2CMasterBuffer[6] = 0x56;		/* Data4 */
	  I2CMasterBuffer[7] = 0xAB;		/* Data5 */
	  I2CMasterBuffer[8] = 0x13;		/* Data6 */
	  I2CMasterBuffer[9] = 0x35;		/* Data7 */
	  I2CMasterBuffer[10] = 0x56;		/* Data8 */
	  I2CMasterBuffer[11] = 0xAB;		/* Data9 */
	  I2CMasterBuffer[12] = 0x13;		/* Data10 */
	  I2CMasterBuffer[13] = 0x35;		/* Data11 */
	  I2CMasterBuffer[14] = 0x56;		/* Data12 */
	  I2CMasterBuffer[15] = 0xAB;		/* Data13 */
	  I2CMasterBuffer[16] = 0x13;		/* Data14 */
	  I2CMasterBuffer[17] = 0x35;		/* Data15 */

	  I2CEngine();

	  /* Be careful with below fixed delay. From device to device, or
	  even same device with different write length, or various I2C clock,
	  below delay length may need to be changed accordingly. Having
	  a break point before Write/Read start will be helpful to isolate
	  the problem. */
	  for ( i = 0; i < 0x200000; i++ );	/* Delay after write */
}
