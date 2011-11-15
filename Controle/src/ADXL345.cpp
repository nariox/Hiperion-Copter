/*
	ADXL345 Library
	
	This libary contains functions to interact with the ADXL345 Triple Axis Digital Accelerometer from Analog Devices written for the ATmega168
	
	created 20 Aug 2009
	by Ryan Owens
	http://www.sparkfun.com
	
 
*/
#include "ADXL345.h"


#define GLOBALOBJECT

//Initialize the I2C communication and put the accelerometer in Measure mode
void Accel_Init()
{

	//Put the accelerometer in MEASURE mode
	Accel_Write(POWER_CTL, MEASURE);
	
	//Set the Range to +/- 4G
	Accel_Write(DATA_FORMAT, RANGE_0);
	
	//default ADXL345 rate is 100 Hz. Perfect!
}

//Read a register value from the ADXL345
//pre: register_addr is the register address to Accel_Read
//	   value is a pointer to an integer
//post: value contains the value of the register that was Accel_Read
//returns: 1-Success
//		   TWSR-Failure (Check out twi.h for TWSR error codes)
//usage: status = accelerometer.Accel_Read(DEVID, &value); //value is created as an 'int' in main.cpp
void Accel_Accel_Read(char register_addr, char * value){
	I2CWriteLength = 2;
	I2CReadLength = 1;
	I2CMasterBuffer[0] = ADXL_ADDR;
	I2CMasterBuffer[1] = register_addr;
	I2CMasterBuffer[2] = ADXL_ADDR | RD_BIT;
	I2CEngine();
	*value = (I2CMasterBuffer[I2CWriteLength+2]);
}

//Write a value to a register
//pre: register_addre is the register to Accel_Write to
//	   value is the value to place in the register
//returns: 1-Success
//		   TWSR- Failure
//usage status=accelerometer.Accel_Write(register_addr, value);
void Accel_Write(char register_addr, char value){
	I2CWriteLength = 2;
	I2CReadLength = 1;
	I2CMasterBuffer[0] = ADXL_ADDR;
	I2CMasterBuffer[1] = register_addr;
	I2CMasterBuffer[2] = value;
	I2CEngine();
}

//Reads the x,y and z registers and stores the contents into x,y and z variables
//returns 1
//usage: accelerometer.update();
//Note: update must be called before using the getX, getY or getZ functions in order
//      to obtain the most recent values from the accelerometer
void Accel_Update(void)
{
	char aux0=0,aux1=0;

	Accel_Read(DATAX0, &aux0);
	Accel_Read(DATAX1, &aux1);
	Accel_X = (aux1<<8)|aux0;
	
	Accel_Read(DATAY0, &aux0);
	Accel_Read(DATAY1, &aux1);
	Accel_Y = (aux1<<8)|aux0;

	Accel_Read(DATAZ0, &aux0);
	Accel_Read(DATAZ1, &aux1);
	Accel_Z = (aux1<<8)|aux0;
}

/*
get functions return the g value of the specified axis
The conversion is based on a +/-4G range.
If range is changed, make sure to update the scaling in the get functions
usage: printf("Xg = %1.3fg", (double)accelerometer.getX()
*/
void Accel_GetX(void)
{
	Accel_Xr = (float)Accel_X*0.0078;	
}

void Accel_GetY(void)
{
	Accel_Yr = (float)Accel_Y*0.0078;
}

void Accel_GetZ(void)
{
	Accel_Zr = (float)Accel_Z*0.0078;
}
