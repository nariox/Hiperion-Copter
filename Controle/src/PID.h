/*
 * PID.h
 *
 *  Created on: 19/10/2011
 *      Author: Danilo Luvizotto
 */

//TODO: verificar o significado da licença e atribuir os créditos
/**********************************************************************************************
 * Arduino PID Library - Version 1
 * by Brett Beauregard <br3ttb@gmail.com> brettbeauregard.com
 *
 * This Code is licensed under a Creative Commons Attribution-ShareAlike 3.0 Unported License.
 **********************************************************************************************/

#ifndef PID_h
#define PID_h
//TODO: verificar utilidade desse define no projeto
//#define LIBRARY_VERSION	1.0.0

//Constants used in some of the functions below
#define PID_AUTOMATIC	1
#define PID_MANUAL	0
#define PID_DIRECT  0
#define PID_REVERSE  1

typedef struct pid_data_t {
    float Input;
    float Output;
    float Setpoint;
    float kp;                      // * (P)roportional Tuning Parameter
    float ki;                      // * (I)ntegral Tuning Parameter
    float kd;                      // * (D)erivative Tuning Parameter
    float dispKp;				   // * we'll hold on to the tuning parameters in user-entered
    float dispKi;				   //   format for display purposes
    float dispKd;				   //
    int   controllerDirection;
    float ITerm, lastInput;
    int   SampleTime;
    float outMin, outMax;
    int   inAuto;
}* pid_data_t;

//commonly used functions **************************************************************************
void pid_update_data(pid_data_t, float Input, float Setpoint);
void pid_setMode(pid_data_t, int);                   // * sets PID to either PID_MANUAL (0) or PID_AUTOMATIC (non-0)
void pid_compute(pid_data_t pid_data);               // * performs the PID calculation.
void pid_setOutputLimits(pid_data_t, float, float);  // * sets the output to a specific range. 0-255 by default


//available but not commonly used functions ********************************************************
void pid_setTunings(pid_data_t,float, float, float);    // pid_data, Kp, Ki, Kd
void pid_setControllerDirection(pid_data_t, int);	    // pid_data, PID_DIRECT ou PID_REVERSE
void pid_setSampleTime(pid_data_t, int);                // pid_data, NewSampleTime
void pid_initialize(pid_data_t);

//Display functions ****************************************************************
float pid_getKp(pid_data_t);						  // These functions query the pid for interal values.
float pid_getKi(pid_data_t);						  //  they were created mainly for the pid front-end,
float pid_getKd(pid_data_t);						  // where it's important to know what is actually
int pid_getMode(pid_data_t);						  //  inside the PID.
int pid_getDirection(pid_data_t);					  //

#endif

