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
#include "timer32.h"
#include "gpio.h"
#include "type.h"

// Libraries
#include "PID.h"
#include "I2C.h"
#include "ITG3200.h"
#include "ADXL345.h"

#define GPIOMaskAddress(port,bits) ((volatile uint16_t * const) (LPC_GPIO_BASE + (0x10000*(port)) + (4*(bits))))
#define ToggleGPIOBits(port,bits) ( *GPIOMaskAddress((port),(bits)) = ~(*GPIOMaskAddress((port),(bits))) )
#define ToggleGPIOBit(port,bit) ToggleGPIOBits((port), 1<<(bit))
#define SetGPIOBits(port,bits) ( *GPIOMaskAddress((port),(bits)) = (bits) )
#define SetGPIOBit(port,bit) SetGPIOBits((port), 1<<(bit))
#define ClrGPIOBits(port,bits) ( *GPIOMaskAddress((port),(bits)) = 0 )
#define ClrGPIOBit(port,bit) ClrGPIOBits((port), 1<<(bit))

#define Tamostragem 100                              // O tempo de amostragem em ms.
#define pwm_periodo 60000                            // PWM de 50Hz. Ver o ajuste do prescaler no driver.
volatile uint16_t throttle_min = 95*(pwm_periodo+1)/100; // O throttle mínimo ocorre quando o dutty cicle está
volatile uint16_t throttle_max = 90*(pwm_periodo+1)/100; // em 95% e o máximo quando o dutty cicle está em 90%

// Parâmetros do controle
float Kp = 1.000;
float Kd = 0.001;

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
void throttle(uint8_t timer_num, uint8_t match, float percent);

uint8_t still_running;

void inicializa() {
	uint8_t i;

	if ( I2CInit( (uint32_t)I2CMASTER ) == FALSE )    /* initialize I2c */ {
	    //TODO: sinalizar o erro de alguma forma no futuro
	    while ( 1 );                /* Fatal error */
	}

	//GPIOInit();
	/* Set LED port pin to output */
	GPIOSetDir( LED_PORT, LED_BIT, 1 );

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
		pid_setTunings(pid_angles[i], 1.0/128, 0, 0);
		pid_angles[i]->outMax = 4;
		pid_angles[i]->outMin = -4;
	}
	
	pid_pitch = pid_angles[0];
	pid_roll  = pid_angles[1];
	pid_yaw   = pid_angles[2];

	//O timer 1 será usado apenas para gerar atrasos
	//init_timer16(1, 1*Tamostragem*TIME_INTERVALmS_KHZ_CLOCK);
	//enable_timer16(1);

	//Inicialização do PWM do timer32 0, ativando as saídas PWMs 0 e 1.
	//init_timer32PWM(1, period, MATCH0);
	init_timer16PWM(0, pwm_periodo, MATCH0 | MATCH1, FALSE);
	throttle(0, 0, 0);
	throttle(0, 1, 0);
	enable_timer16(0);

	//Inicialização do PWM do timer32 1, ativando as saídas PWMs 0 e 1.
	init_timer32PWM(1, pwm_periodo, MATCH0 | MATCH1 );
	throttle(1, 0, 0);
	throttle(1, 1, 0);
	enable_timer32(1);

	//delayMs(1, 2000); //Atraso para que os periféricos e componentes externos inicializem

	//O timer 1 será usado para gerar o intervalo de 100ms, que é o tempo de amostragem.
	/* Initialize 16-bit timer 1. TIME_INTERVAL is defined as 1mS */
	init_timer16(1, Tamostragem*TIME_INTERVALmS_KHZ_CLOCK);
	/* Enable the TIMER1 Interrupt */
	NVIC_EnableIRQ(TIMER_16_1_IRQn);
	/* Enable timer 0. */
	enable_timer16(1);
}

void le_nav(nav_params_t params)
{
    //TODO: implementar
}

float temp=0;

void throttle(uint8_t timer_num, uint8_t match, float percent) {
    if (timer_num == 1)
        setMatch_timer32PWM (1, match, throttle_min - (throttle_min - throttle_max)*percent);
    else
        setMatch_timer16PWM (0, match, throttle_min - (throttle_min - throttle_max)*percent);
}
float percent = 0.5;

uint32_t apague = 0;

int main(void)
{
	uint8_t i;

	inicializa();

	//Loop principal
	while(1) {
		//apague = LPC_TMR16B1->TC;
        still_running = TRUE;

        //Lê sinais dos sensores
        Gyro_Update(gyro_data);
        Accel_Update(accel_data);
        temp = Gyro_GetTemp(gyro_data);

        //Lê parâmetros de navegação
        //le_nav();

        //Realiza o processamento PID
        for(i = 0; i < 3; ++i) {
        	pid_update_data(pid_angles[i], accel_data->x, 0);
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

void TIMER16_1_IRQHandler(void)
{
    if(still_running) { // O laço principal ainda está rodando e não deveria. Isso é um erro.
        //NVIC_DisableIRQ(TIMER_16_0_IRQn); // Desabilita a interrupção do timer,
        //delayMs(0, 2000);                 // espera...
        //ClrGPIOBit( LED_PORT, LED_BIT );  // Mantém o LED aceso para indicar o erro.
        //disable_timer16(0);               // Desliga o timer 0.
    }

    if ( LPC_TMR16B1->IR & 0x1 )
    	LPC_TMR16B1->IR = 1;			/* clear interrupt flag */

    ToggleGPIOBit( LED_PORT, LED_BIT );
    return;
}
