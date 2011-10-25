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

// Drivers
#include "type.h"

#include "driver_config.h"
#include "PID.h"
#include "I2C.h"

typedef struct IMU_sigs_t {
    int accel_x, accel_y, accel_z;  // Sinal tridimensional do acelerômetro
    int giro_x, giro_y, giro_z;     // Sinal tridimensional da IMU
}* IMU_sigs_t;

typedef struct nav_params_t {
	int pitch, roll, yaw;  // Ângulos de Euler, variando de -128 a 127
	int throttle;          // Potência total dos motores, variando de 0 a 255
}* nav_params_t;

IMU_sigs_t IMU_sigs;
nav_params_t nav_params;

pid_data_t pid_angles[3];
pid_data_t pid_pitch;
pid_data_t pid_roll;
pid_data_t pid_yaw;

IMU_sigs_t init_IMU_sigs() { return malloc(sizeof(struct IMU_sigs_t)); }
nav_params_t init_nav_params() { return malloc(sizeof(struct nav_params_t)); }

void inicializa() {
    uint8_t i;

    i2c_inicializa();

	IMU_sigs = init_IMU_sigs();
	nav_params = init_nav_params();

    for(i = 0; i < 3; ++i) {
        pid_angles[i] = malloc(sizeof(struct pid_data_t));
        pid_angles[i]->SampleTime = 100;            //default Controller Sample Time is 0.1 seconds
    }
    pid_pitch = pid_angles[0];
	pid_roll  = pid_angles[1];
	pid_yaw   = pid_angles[2];
}

void le_IMU(IMU_sigs_t sigs)
{
    //TODO: implementar
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
		//Lê sinais dos sensores
		//le_IMU();

		//Lê parâmetros de navegação
		//le_nav();

	    //Realiza o processamento PID
	    for(i = 0; i < 3; ++i) {
	        pid_compute(pid_angles[i]);
	    }

	    //Envia as rotações dos motores aos ESCs
	    //envia_rotacoes();
	}

	// Enter an infinite loop, just incrementing a counter
	return 0;
}
