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

typedef struct nav_params_t {
	int pitch, roll, yaw;  // Ângulos de Euler, variando de -128 a 127
	int throttle;          // Potência total dos motores, variando de 0 a 255
}* nav_params_t;

nav_params_t nav_params;
gyro_data_t gyro_data;
accel_data_t accel_data;

pid_data_t pid_angles[3];
pid_data_t pid_pitch;
pid_data_t pid_roll;
pid_data_t pid_yaw;

nav_params_t init_nav_params() { return malloc(sizeof(struct nav_params_t)); }
gyro_data_t init_gyro_data() { return malloc(sizeof(struct gyro_data_t)); }
accel_data_t init_accel_data() { return malloc(sizeof(struct accel_data_t)); }
pid_data_t init_pid_data() { return malloc(sizeof(struct pid_data_t)); }

uint8_t still_running;

void inicializa() {
	uint8_t i;

	if ( I2CInit( (uint32_t)I2CMASTER ) == FALSE )    /* initialize I2c */ {
	    //TODO: sinalizar o erro de alguma forma no futuro
	    while ( 1 );                /* Fatal error */
	}
	Gyro_Init();     //Inicializa o giroscópio
	Accel_Init();    //Inicializa o acelerômetro

	//Inicializa a estrutura de dados dos parâmetros de navegação
	nav_params = init_nav_params();

	//Inicializa a estrutura de dados do giroscópio
	gyro_data = init_gyro_data();

	//Inicializa a estrutura de dados do acelerômetro
	accel_data = init_accel_data();

	//Inicializa as estruturas de dados dos PIDs
	for(i = 0; i < 3; ++i) {
		pid_angles[i] = init_pid_data();
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

/*void le_IMU()
{
	Gyro_Update();
	Accel_Update();
}*/

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
	Gyro_Update(gyro_data);
	Accel_Update(accel_data);

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
