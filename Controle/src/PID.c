/*
 * PID.c
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

#include "PID.h"
#define FALSE 0

void initialize(float* Input, float* Output, float* Setpoint,
                float Kp, float Ki, float Kd, int ControllerDirection);
void reinitialize();

float dispKp;				// * we'll hold on to the tuning parameters in user-entered
float dispKi;				//   format for display purposes
float dispKd;				//

float kp;                  // * (P)roportional Tuning Parameter
float ki;                  // * (I)ntegral Tuning Parameter
float kd;                  // * (D)erivative Tuning Parameter

int controllerDirection;

float *myInput;              // * Pointers to the Input, Output, and Setpoint variables
float *myOutput;             //   This creates a hard link between the variables and the
float *mySetpoint;           //   PID, freeing the user from having to constantly tell us
                             //   what these values are.  with pointers we'll just know.

float ITerm, lastInput;

int SampleTime;
float outMin, outMax;
int inAuto;


/*Constructor (...)*********************************************************
 *    The parameters specified here are those for for which we can't set up
 *    reliable defaults, so we need to have the user set them.
 ***************************************************************************/
void initialize(float* Input, float* Output, float* Setpoint,
                float Kp, float Ki, float Kd, int ControllerDirection)
{
	pid_setOutputLimits(0, 255);				//default output limit corresponds to
												//the arduino pwm limits

    SampleTime = 100;							//default Controller Sample Time is 0.1 seconds

    pid_setControllerDirection(ControllerDirection);
    pid_setTunings(Kp, Ki, Kd);

    inAuto = FALSE;
    myOutput = Output;
    myInput = Input;
    mySetpoint = Setpoint;
}//initialize


/* pid_compute() **********************************************************************
 *     This, as they say, is where the magic happens.  this function should be called
 *   every time "void loop()" executes.  the function will decide for itself whether a new
 *   pid Output needs to be computed
 **********************************************************************************/
void pid_compute()
{
	if(!inAuto) return; //não está no modo automático, ou seja, o PID está desabilitado

	/*Compute all the working error variables*/
	float input = *myInput;
	float error = *mySetpoint - input;
	ITerm+= (ki * error);
	if(ITerm > outMax) ITerm= outMax;
	else if(ITerm < outMin) ITerm= outMin;
	float dInput = (input - lastInput);

	/*Compute PID Output*/
	float output = kp * error + ITerm- kd * dInput;

	if(output > outMax) output = outMax;
	else if(output < outMin) output = outMin;
	*myOutput = output;

	/*Remember some variables for next time*/
	lastInput = input;
}//pid_compute


/* pid_setTunings(...)*************************************************************
 * This function allows the controller's dynamic performance to be adjusted.
 * it's called automatically from the constructor, but tunings can also
 * be adjusted on the fly during normal operation
 ******************************************************************************/
void pid_setTunings(float Kp, float Ki, float Kd)
{
   if (Kp<0 || Ki<0 || Kd<0) return;

   dispKp = Kp; dispKi = Ki; dispKd = Kd;

   float SampleTimeInSec = ((float)SampleTime)/1000;
   kp = Kp;
   ki = Ki * SampleTimeInSec;
   kd = Kd / SampleTimeInSec;

  if(controllerDirection == PID_REVERSE)
   {
      kp = (0 - kp);
      ki = (0 - ki);
      kd = (0 - kd);
   }
}//pid_setTunings

/* pid_setTunings(...) *********************************************************
 * sets the period, in Milliseconds, at which the calculation is performed
 ******************************************************************************/
void pid_setSampleTime(int NewSampleTime)
{
   if (NewSampleTime > 0)
   {
      float ratio  = (float)NewSampleTime
                      / (float)SampleTime;
      ki *= ratio;
      kd /= ratio;
      SampleTime = (unsigned long)NewSampleTime;
   }
}//pid_setSampleTime

/* pid_setOutputLimits(...)****************************************************
 *     This function will be used far more often than SetInputLimits.  while
 *  the input to the controller will generally be in the 0-1023 range (which is
 *  the default already,)  the output will be a little different.  maybe they'll
 *  be doing a time window and will need 0-8000 or something.  or maybe they'll
 *  want to clamp it from 0-125.  who knows.  at any rate, that can all be done
 *  here.
 **************************************************************************/
void pid_setOutputLimits(float Min, float Max)
{
   if(Min >= Max) return;
   outMin = Min;
   outMax = Max;

   if(inAuto)
   {
	   if(*myOutput > outMax) *myOutput = outMax;
	   else if(*myOutput < outMin) *myOutput = outMin;

	   if(ITerm > outMax) ITerm= outMax;
	   else if(ITerm < outMin) ITerm= outMin;
   }
}//pid_setOutputLimits

/* pid_setMode(...)****************************************************************
 * Allows the controller Mode to be set to manual (0) or Automatic (non-zero)
 * when the transition from manual to auto occurs, the controller is
 * automatically initialized
 ******************************************************************************/
void pid_setMode(int Mode)
{
    int newAuto = (Mode == PID_AUTOMATIC);
    if(newAuto == !inAuto)
    {  /*we just went from manual to auto*/
    	reinitialize();
    }
    inAuto = newAuto;
}//pid_setMode

/* reinitialize()****************************************************************
 *	does all the things that need to happen to ensure a bumpless transfer
 *  from manual to automatic mode.
 ******************************************************************************/
void reinitialize()
{
   ITerm = *myOutput;
   lastInput = *myInput;
   if(ITerm > outMax) ITerm = outMax;
   else if(ITerm < outMin) ITerm = outMin;
}//reinitialize

/* pid_setControllerDirection(...)*************************************************
 * The PID will either be connected to a DIRECT acting process (+Output leads
 * to +Input) or a REVERSE acting process(+Output leads to -Input.)  we need to
 * know which one, because otherwise we may increase the output when we should
 * be decreasing.  This is called from the constructor.
 ******************************************************************************/
void pid_setControllerDirection(int Direction)
{
   if(inAuto && Direction !=controllerDirection)
   {
	  kp = (0 - kp);
      ki = (0 - ki);
      kd = (0 - kd);
   }
   controllerDirection = Direction;
}//pid_setControllerDirection

/* Status Funcions*************************************************************
 * Just because you set the Kp=-1 doesn't mean it actually happened.  these
 * functions query the internal state of the PID.  they're here for display
 * purposes.  this are the functions the PID Front-end uses for example
 ******************************************************************************/
float pid_getKp(){ return  dispKp; }
float pid_getKi(){ return  dispKi;}
float pid_getKd(){ return  dispKd;}
int pid_getMode(){ return  inAuto ? PID_AUTOMATIC : PID_MANUAL;}
int pid_getDirection(){ return controllerDirection;}

