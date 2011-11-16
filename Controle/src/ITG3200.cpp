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
	I2C_write(ITG_ADDR, DLPF_FS, DLPF_FS_SEL_0|DLPF_FS_SEL_1|DLPF_CFG_0);
	
	//Set sample rate divider for 100 Hz operation
	I2C_write(ITG_ADDR, SMPLRT_DIV, 9);	//Fsample = Fint / (divider + 1) where Fint is 1kHz
	
	//Setup the interrupt to trigger when new data is ready.
	I2C_write(ITG_ADDR, INT_CFG, INT_CFG_RAW_RDY_EN | INT_CFG_ITG_RDY_EN);
	
	//Select X gyro PLL for clock source
	I2C_write(ITG_ADDR, PWR_MGM, PWR_MGM_CLK_SEL_0);
}

//Reads the x,y and z registers and stores the contents into x,y and z variables
//returns 1
//usage: gyro.update();
//Note: update must be called before using the getX, getY or getZ functions in order
//      to obtain the most recent values from the gyro
void Gyro_Update(gyro_data_t gyro_data)
{
	char aux0=0,aux1=0;

	I2C_read(ITG_ADDR, GYRO_XOUT_H, &aux1);
	I2C_read(ITG_ADDR, GYRO_XOUT_L, &aux0);
	gyro_data.x = (aux1<<8)|aux0;
	
	I2C_read(ITG_ADDR, GYRO_YOUT_H, &aux1);
	I2C_read(ITG_ADDR, GYRO_YOUT_L, &aux0);
	gyro_data.y = (aux1<<8)|aux0;

	I2C_read(ITG_ADDR, GYRO_ZOUT_H, &aux1);
	I2C_read(ITG_ADDR, GYRO_ZOUT_L, &aux0);
	gyro_data.z = (aux1<<8)|aux0;

	I2C_read(ITG_ADDR, TEMP_OUT_H, &aux1);
	I2C_read(ITG_ADDR, TEMP_OUT_L, &aux0);
	gyro_data.temp = (aux1<<8)|aux0;
}

/*
get functions return the g value of the specified axis
usage: printf("Xg = %1.3fg", (double)gyro.getX()
*/

/* TODO: remover
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
*/

float Gyro_GetTemp(gyro_data_t gyro_data)
{
    int Gyro_Temp = -13200-gyro_data.temp;    //Get the offset temp
    float Gyro_Tempr = (float)Gyro_Temp/280;    //Convert the offset to degree C
	return Gyro_Tempr + 35;    //Add 35 degrees C to compensate for the offset
}
