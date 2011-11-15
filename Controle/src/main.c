/*
===============================================================================
 Name        : main.c
 Author      : Danilo Luvizotto
 Version     : 0.1
 Copyright   : Copyright (C) 
 Description : main definition
===============================================================================
*/

#include <cr_section_macros.h>
#include <NXP/crp.h>

// Variable to store CRP value in. Will be placed automatically
// by the linker when "Enable Code Read Protect" selected.
// See crp.h header for more information
__CRP const unsigned int CRP_WORD = CRP_NO_CRP ;

/////////////////////////////////////////////////////////////////

#include <stdlib.h>

//Configs
#include "driver_config.h"
#include "target_config.h"

// Drivers
#include "timer16.h"
#include "gpio.h"
#include "type.h"
#include "debug.h"

// Libraries
#include "PID.h"
#include "I2C.h"
#include "ITG3200.h"
#include "ADXL345.h"
#include "SSP.h"

#define GPIOMaskAddress(port,bits) ((volatile uint16_t * const) (LPC_GPIO_BASE + (0x10000*(port)) + (4*(bits))))
#define ToggleGPIOBits(port,bits) ( *GPIOMaskAddress((port),(bits)) = ~(*GPIOMaskAddress((port),(bits))) )
#define ToggleGPIOBit(port,bit) ToggleGPIOBits((port), 1<<(bit))
#define SetGPIOBits(port,bits) ( *GPIOMaskAddress((port),(bits)) = (bits) )
#define SetGPIOBit(port,bit) SetGPIOBits((port), 1<<(bit))
#define ClrGPIOBits(port,bits) ( *GPIOMaskAddress((port),(bits)) = 0 )
#define ClrGPIOBit(port,bit) ClrGPIOBits((port), 1<<(bit))

#define Tamostragem 100             // O tempo de amostragem em ms

/*typedef struct IMU_sigs_t {
    int accel_x, accel_y, accel_z;  // Sinal tridimensional do acelerômetro
    int giro_x, giro_y, giro_z;     // Sinal tridimensional da IMU
}* IMU_sigs_t; */

typedef struct nav_params_t {
	int pitch, roll, yaw;  // Ângulos de Euler, variando de -128 a 127
	int throttle;          // Potência total dos motores, variando de 0 a 255
}* nav_params_t;

//IMU_sigs_t IMU_sigs;
nav_params_t nav_params;

pid_data_t pid_angles[3];
pid_data_t pid_pitch;
pid_data_t pid_roll;
pid_data_t pid_yaw;

//IMU_sigs_t init_IMU_sigs() { return malloc(sizeof(struct IMU_sigs_t)); }
nav_params_t init_nav_params() { return malloc(sizeof(struct nav_params_t)); }

uint8_t still_running;

void inicializa() {
	uint8_t i;

	I2CInit();
	Accel_Init();
	Gyro_Init();

	IMU_sigs = init_IMU_sigs();
	nav_params = init_nav_params();

	for(i = 0; i < 3; ++i) {
		pid_angles[i] = malloc(sizeof(struct pid_data_t));
		pid_angles[i]->SampleTime = Tamostragem;            //default Controller Sample Time is 0.1 seconds
	}
	
	pid_pitch = pid_angles[0];
	pid_roll  = pid_angles[1];
	pid_yaw   = pid_angles[2];

	//Setup timer
	/* Initialize 16-bit timer 0. TIME_INTERVAL is defined as 10mS */
	init_timer16(0, Tamostragem*TIME_INTERVALmS_KHZ_CLOCK);
	/* Enable the TIMER0 Interrupt */
	NVIC_EnableIRQ(TIMER_16_0_IRQn);
	/* Enable timer 0. */
	enable_timer16(0);
	/* Initialize GPIO (sets up clock) */
	GPIOInit();
	/* Set LED port pin to output */
	GPIOSetDir( LED_PORT, LED_BIT, 1 );
}

void le_IMU(IMU_sigs_t sigs)
{
	Gyro_Update();
	Accel_Update();
}

void le_nav(nav_params_t params)
{
    //TODO: implementar
}

int main(void)
{
	uint8_t i;

	inicializa();

	//Loop principal
	while(1) {
	still_running = TRUE;
	//Lê sinais dos sensores
	le_IMU();
	Accel_GetX();
	Accel_GetY();
	Accel_GetZ();
	Gyro_GetX();
	Gyro_GetY();
	Gyro_GetZ();
	
	debug_puts("IMU GX: %1.3f GY: %1.3f GZ: %1.3f\n",Gyro_Xr,Gyro_Yr,Gyro_Zr);
	debug_puts("IMU AX: %1.3f AY: %1.3f AZ: %1.3f\n",Accel_Xr,Accel_Yr,Accel_Zr);

	//Lê parâmetros de navegação
	//le_nav();

	//Realiza o processamento PID
	for(i = 0; i < 3; ++i) {
		pid_compute(pid_angles[i]);
	}

	//Envia as rotações dos motores aos ESCs
	//envia_rotacoes();

	still_running = FALSE;
	/* Go to sleep to save power between timer interrupts */
		__WFI();
	}

	// Enter an infinite loop, just incrementing a counter
	return 0;
}

void TIMER16_0_IRQHandler(void)
{
    if(still_running) { // O laço principal ainda está rodando e não deveria. Isso é um erro.
        NVIC_DisableIRQ(TIMER_16_0_IRQn); // Desabilita a interrupção do timer,
        delayMs(0, 2000);                 // espera...
        ClrGPIOBit( LED_PORT, LED_BIT );  // Mantém o LED aceso para indicar o erro.
    }
    if ( LPC_TMR16B0->IR & 0x01 )
        LPC_TMR16B0->IR = 1;          /* clear interrupt flag */

    ToggleGPIOBit( LED_PORT, LED_BIT );
    return;
}
