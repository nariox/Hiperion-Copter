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
#include <math.h>

//Configs
#include "driver_config.h"
#include "target_config.h"

// Drivers
#include "timer16.h"
#include "timer32.h"
#include "gpio.h"
#include "type.h"
#include "uart.h"

// Libraries
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

#define M_PI 3.14159265358979323846                  // pi
#define NAV_MES_SIZE 8                               // Tamanho da mensagem recebida da navegação.
#define Tamostragem 100                              // O tempo de amostragem em ms.
#define pwm_periodo 59999                            // PWM de 50Hz. Ver o ajuste do prescaler no driver.
volatile uint16_t throttle_min = 95.0*(pwm_periodo+1)/100; // O throttle mínimo ocorre quando o dutty cicle está
volatile uint16_t throttle_max = 90.0*(pwm_periodo+1)/100; // em 95% e o máximo quando o dutty cicle está em 90%
volatile float accel_scale = 1.0/128;                // fator de escala do giroscópio. //TODO: explicar
volatile float gyro_scale = 2*M_PI/(14.375*360);     // fator de escala do giroscópio. //TODO: explicar
volatile float nav_scale = 2*M_PI/256.0;             // fator de escala dos dados vindos da navegação. //TODO: explicar
extern volatile uint32_t UARTCount;
extern volatile uint8_t UARTBuffer[UART_BUFSIZE];


typedef struct nav_params_t {
	int pitch, roll, yaw;  // Ângulos de Euler, variando de -128 a 127
	int throttle;          // Potência total dos motores, variando de 0 a 255
}* nav_params_t;

nav_params_t nav_params;
gyro_data_t gyro_data;
accel_data_t accel_data;
float temp=0;

float kp = 0;      //Constante proporcional do controlador PD
float kd = 0;      //Constante derivativa do controlador PD
float kd_yaw = 0;  //Constante derivativa do yaw do controlador PD
float A, B, C, D;      //Ver "Controle - ortogonalização" na monografia associada a esse código.
float throttles[4];    //Valor de rotação dos motores. Varia entre 0 e 255.

int UARTBuffer_p = 0;

nav_params_t init_nav_params() { return malloc(sizeof(struct nav_params_t)); }
gyro_data_t init_gyro_data() { return malloc(sizeof(struct gyro_data_t)); }
accel_data_t init_accel_data() { return malloc(sizeof(struct accel_data_t)); }
void throttle(uint8_t timer_num, uint8_t match, float percent);
void processa();

void inicializa() {
    // Inicializa o I2C
	if ( I2CInit( (uint32_t)I2CMASTER ) == FALSE ) {
	    //TODO: sinalizar o erro de alguma forma no futuro
	    while ( 1 );                // Fatal error
	}

	//GPIOInit();
	//Set LED port pin to output
	GPIOSetDir( LED_PORT, LED_BIT, 1 );

	Gyro_Init();     //Inicializa o giroscópio
	Accel_Init();    //Inicializa o acelerômetro

	//Inicializa a estrutura de dados dos parâmetros de navegação
	nav_params = init_nav_params();

	//Inicializa a estrutura de dados do giroscópio
	gyro_data = init_gyro_data();

	//Inicializa a estrutura de dados do acelerômetro
	accel_data = init_accel_data();

	//O timer 0 será usado apenas para gerar atrasos
	init_timer32(0, 1*Tamostragem*TIME_INTERVALmS_KHZ_CLOCK);
	enable_timer32(0);

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

	delay32Ms(0, 8000); //Atraso para que os periféricos e componentes externos inicializem

	//Inicializa o UART
	UARTInit(UART_BAUD);

	//O timer 1 será usado para checar se o intervalo de 100ms está sendo respeitado.
	// Initialize 16-bit timer 1. TIME_INTERVAL is defined as 1mS
	init_timer16(1, Tamostragem*TIME_INTERVALmS_KHZ_CLOCK);
}

int nav_m_p(int deslocamento) // ponteiro para o início da mensagem da navegação
{
	if (UARTBuffer_p == 0)
		return deslocamento;

    deslocamento += UARTBuffer_p; // 'deslocamento' nunca deve ser mais que UART_BUFSIZE
    return (deslocamento > UART_BUFSIZE -1) ? deslocamento - UART_BUFSIZE : deslocamento;
}

void le_nav()
{
	//Faz o UARTBuffer_p apontar para o início da mensagem da navegação
	while(UARTBuffer[UARTBuffer_p] != 'R')
		UARTBuffer_p = (UARTBuffer_p == UART_BUFSIZE -1) ? 0 : UARTBuffer_p + 1;

	nav_params->roll  = (UARTBuffer[nav_m_p(1)] - '0')*100;
	nav_params->roll += (UARTBuffer[nav_m_p(2)] - '0')*10;
	nav_params->roll +=  UARTBuffer[nav_m_p(3)] - '0';
	nav_params->pitch  = (UARTBuffer[nav_m_p(5)] - '0')*100;
	nav_params->pitch += (UARTBuffer[nav_m_p(6)] - '0')*10;
    nav_params->pitch +=  UARTBuffer[nav_m_p(7)] - '0';
    nav_params->yaw  = (UARTBuffer[nav_m_p(9)]  - '0')*100;
    nav_params->yaw += (UARTBuffer[nav_m_p(10)] - '0')*10;
    nav_params->yaw +=  UARTBuffer[nav_m_p(11)] - '0';
    nav_params->throttle  = (UARTBuffer[nav_m_p(13)] - '0')*100;
    nav_params->throttle += (UARTBuffer[nav_m_p(14)] - '0')*10;
    nav_params->throttle +=  UARTBuffer[nav_m_p(15)] - '0';
}

void throttle(uint8_t timer_num, uint8_t match, float percent) {
    if (timer_num == 1)
        setMatch_timer32PWM (1, match, throttle_min - (throttle_min - throttle_max)*percent);
    else
        setMatch_timer16PWM (0, match, throttle_min - (throttle_min - throttle_max)*percent);
}

void processa()
{
	int32_t accel_x_med = (accel_data->x = accel_data->x_ant)/2;
	int32_t accel_y_med = (accel_data->y = accel_data->y_ant)/2;

    A = kp*( (nav_params->pitch -127)*nav_scale - asin(accel_x_med*accel_scale) )  + kd*gyro_data->y*gyro_scale;
    B = kp*( (nav_params->roll -127)*nav_scale - asin(accel_y_med*accel_scale) )  - kd*gyro_data->x*gyro_scale;
    C = kd_yaw*( (nav_params->yaw -127)*nav_scale - kd_yaw*gyro_data->z*gyro_scale);
    //D = nav_params->throttle/64.0;
    D = 1;
}

void envia_rotacoes()
{
	int i;

	throttles[0] = (-A -B +C +D)/4;
	throttles[1] = (-A +B -C +D)/4;
	throttles[2] = ( A +B +C +D)/4;
	throttles[3] = ( A -B -C +D)/4;

	for(i = 0; i < 4; ++i) {
		if (throttles[i] > 1)
		    throttles[i] = 1;
		else if (throttles[i] < 0)
			throttles[i] = 0;
	}

	//TODO: APAGUE! RISCO DE ACIDENTE! APENAS PARA TESTE!!!
	if(nav_params->roll != 127 ||
	   nav_params->pitch != 127 ||
	   nav_params->yaw != 127) {
	    throttle(0, 0, 0);
		throttle(0, 1, 0);
		throttle(1, 0, 0);
		throttle(1, 1, 0);
		while(1);
	}

    throttle(0, 0, throttles[0]);
	throttle(0, 1, throttles[1]);
	throttle(1, 0, throttles[2]);
	throttle(1, 1, throttles[3]);
}

int main(void)
{

	inicializa();

	kp = 3.0/(2*M_PI);      // Constante proporcional do controlador PD
	kd = 0.0/(2*M_PI);      // Constante derivativa do controlador PD
	kd_yaw = 0/(2*M_PI);    // Constante derivativa do yaw do controlador PD
	D = 0;

	//Loop principal
	while(1) {
        // Go to sleep to save power between timer interrupts
        __WFI();
        init_timer16(1, Tamostragem*TIME_INTERVALmS_KHZ_CLOCK);
        enable_timer16(1);

        //Lê sinais dos sensores
        Gyro_Update(gyro_data);
        Accel_Update(accel_data);
        temp = Gyro_GetTemp(gyro_data);

        //Lê parâmetros de navegação
	    le_nav();

        //Realiza o processamento PID
        processa();

        //Envia as rotações dos motores aos ESCs
        envia_rotacoes();

        //Inverte o estado do led
        ToggleGPIOBit( LED_PORT, LED_BIT );

        if (LPC_TMR16B1->TC == 0) { //O processamento demorou mais que o tempo de amostragem.
        	ClrGPIOBit( LED_PORT, LED_BIT );    // Mantém o LED aceso para indicar o erro.
        	while(1); //Erro
        }
    }

    // Enter an infinite loop, just incrementing a counter
    return 0;
}

void atualiza_PWMs() {

}

void TIMER16_1_IRQHandler(void)
{
    //if(still_running) { // O laço principal ainda está rodando e não deveria. Isso é um erro.
        //NVIC_DisableIRQ(TIMER_16_1_IRQn);   // Desabilita a interrupção do timer,
        //delay32Ms(0, 2000);                 // espera...
        //ClrGPIOBit( LED_PORT, LED_BIT );    // Mantém o LED aceso para indicar o erro.
        //disable_timer16(1);                 // Desliga o timer.
    //}

    if ( LPC_TMR16B1->IR & 0x1 )
    	LPC_TMR16B1->IR = 1;			/* clear interrupt flag */

    return;
}
