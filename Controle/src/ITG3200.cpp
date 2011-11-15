/*
	ITG3200 Library
	
	This libary contains functions to interact with the ITG3200 from Atmega168

	created 8/30/10
	by Ryan Owens
	http://www.sparkfun.com
 
*/

#include "ITG3200.h"
#include <avr/io.h>
#include <stdlib.h>

#include "driver_config.h"
#include "target_config.h"

#include "type.h"
#include "i2c.h"

extern volatile uint32_t I2CCount;
extern volatile uint8_t I2CMasterBuffer[BUFSIZE];
extern volatile uint8_t I2CSlaveBuffer[BUFSIZE];
extern volatile uint32_t I2CMasterState;
extern volatile uint32_t I2CReadLength, I2CWriteLength;

//Initialize the i2c communication and set the gyro to full scale range and 100 Hz update rate
void Gyro_Init()
{

	//twiInit(80000);			//Init. SCL speed to 50 kHz
	
	//Set internal clock to 1kHz with 42Hz LPF and Full Scale to 3 for proper operation
	Gyro_Write(DLPF_FS, DLPF_FS_SEL_0|DLPF_FS_SEL_1|DLPF_CFG_0);
	
	//Set sample rate divider for 100 Hz operation
	Gyro_Write(SMPLRT_DIV, 9);	//Fsample = Fint / (divider + 1) where Fint is 1kHz
	
	//Setup the interrupt to trigger when new data is ready.
	Gyro_Write(INT_CFG, INT_CFG_RAW_RDY_EN | INT_CFG_ITG_RDY_EN);
	
	//Select X gyro PLL for clock source
	Gyro_Write(PWR_MGM, PWR_MGM_CLK_SEL_0);
}

//Read a register value from the gyro
//pre: register_addr is the register address to read
//	   value is a pointer to an integer
//post: value contains the value of the register that was read
//returns: 1-Success
//		   TWSR-Failure (Check out twi.h for TWSR error codes)
//usage: status = gyro.read(DEVID, &value); //value is created as an 'int' in main.cpp
void Gyro_Read(char register_addr, char * value){
	I2CWriteLength = 2;
	I2CReadLength = 1;
	I2CMasterBuffer[0] = GYRO_ADDR;
	I2CMasterBuffer[1] = register_addr;
	I2CMasterBuffer[2] = GYRO_ADDR | RD_BIT;
	I2CEngine();
	*value = (I2CMasterBuffer[I2CWriteLength+2]);
}

//Write a value to a register
//pre: register_addre is the register to write to
//	   value is the value to place in the register
//returns: 1-Success
//		   TWSR- Failure
//usage status=gyro.write(register_addr, value);
void Gyro_Write(char register_addr, char value){
	I2CWriteLength = 6;
	I2CReadLength = 0;
	I2CMasterBuffer[0] = GYRO_ADDR;
	I2CMasterBuffer[1] = register_addr;
	I2CMasterBuffer[2] = value;
	I2CEngine();
}

//Reads the x,y and z registers and stores the contents into x,y and z variables
//returns 1
//usage: gyro.update();
//Note: update must be called before using the getX, getY or getZ functions in order
//      to obtain the most recent values from the gyro
void Gyro_Update()
{
	char aux0=0,aux1=0;

	Gyro_Read(GYRO_XOUT_H, &aux1);
	Gyro_Read(GYRO_XOUT_L, &aux0);
	Gyro_X = (aux1<<8)|aux0;
	
	Gyro_Read(GYRO_YOUT_H, &aux1);
	Gyro_Read(GYRO_YOUT_L, &aux0);
	Gyro_Y = (aux1<<8)|aux0;

	Gyro_Read(GYRO_ZOUT_H, &aux1);
	Gyro_Read(GYRO_ZOUT_L, &aux0);
	Gyro_Z = (aux1<<8)|aux0;

	Gyro_Read(TEMP_OUT_H, &aux1);
	Gyro_Read(TEMP_OUT_L, &aux0);
	Gyro_Temp = (aux1<<8)|aux0;
}

/*
get functions return the g value of the specified axis
usage: printf("Xg = %1.3fg", (double)gyro.getX()
*/

void Gyro_GetX()
{
	Gyro_Xr = (float)Gyro_X/14.375;
}

void Gyro_GetY()
{
	Gyro_Yr = (float)Gyro_Y/14.375;
}

void Gyro_GetZ()
{
	Gyro_Zr = (float)Gyro_Z/14.375;
}

void Gyro_GetTemp()
{
	Gyro_Temp = -13200-Gyro_Temp;	//Get the offset temp
	Gyro_Tempr = (float)Gyro_Temp/280;	//Convert the offset to degree C
	Gyro_Tempr += 35;	//Add 35 degrees C to compensate for the offset
}
