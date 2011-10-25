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
    int   ControllerDirection;
    float ITerm, lastInput;
    int   SampleTime;
    float outMin, outMax;
    int   inAuto;
}* pid_data_t;

//commonly used functions **************************************************************************
pid_data_t config_data(float* Input, float* Output, float* Setpoint,
                       float Kp, float Ki, float Kd, int ControllerDirection);
void pid_setMode(int);                   // * sets PID to either PID_MANUAL (0) or PID_AUTOMATIC (non-0)
void pid_compute();                      // * performs the PID calculation.
void pid_setOutputLimits(float, float);  // * sets the output to a specific range. 0-255 by default


//available but not commonly used functions ********************************************************
void pid_setTunings(float, float, float);   //  Kp, Ki, Kd
void pid_setControllerDirection(int);	    // * PID_DIRECT ou PID_REVERSE
void pid_setSampleTime(int);                // * NewSampleTime
void initialize();

//Display functions ****************************************************************
float pid_getKp();						  // These functions query the pid for interal values.
float pid_getKi();						  //  they were created mainly for the pid front-end,
float pid_getKd();						  // where it's important to know what is actually
int pid_getMode();						  //  inside the PID.
int pid_getDirection();					  //

#endif

