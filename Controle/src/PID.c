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

#include <stdlib.h>

#include "PID.h"

#define FALSE 0

/*config_data (...)*********************************************************
 *    The parameters specified here are those for for which we can't set up
 *    reliable defaults, so we need to have the user set them.
 ***************************************************************************/
void pid_update_data(pid_data_t pid_data, float Input, float Setpoint)
{
	pid_data->Input    = Input;
	pid_data->Setpoint = Setpoint;

}//pid_update_data


/* pid_compute() **********************************************************************
 *     This, as they say, is where the magic happens.  this function should be called
 *   every time "void loop()" executes.  the function will decide for itself whether a new
 *   pid Output needs to be computed
 **********************************************************************************/
void pid_compute(pid_data_t pid_data)
{
	if(!pid_data->inAuto) return; //não está no modo automático, ou seja, o PID está desabilitado

	/*Compute all the working error variables*/
	float error = pid_data->Setpoint - pid_data->Input;
	pid_data->ITerm += (pid_data->ki * error);
	if(pid_data->ITerm > pid_data->outMax)
		pid_data->ITerm = pid_data->outMax;
	else if(pid_data->ITerm < pid_data->outMin)
		pid_data->ITerm = pid_data->outMin;
	float dInput = (pid_data->Input - pid_data->lastInput);

	/*Compute PID Output*/
	float output = pid_data->kp * error + pid_data->ITerm- pid_data->kd * dInput;

	if(output > pid_data->outMax) output = pid_data->outMax;
	else if(output < pid_data->outMin) output = pid_data->outMin;
	pid_data->Output = output;

	/*Remember some variables for next time*/
	pid_data->lastInput = pid_data->Input;
}//pid_compute


/* pid_setTunings(...)*************************************************************
 * This function allows the controller's dynamic performance to be adjusted.
 * it's called automatically from the constructor, but tunings can also
 * be adjusted on the fly during normal operation
 ******************************************************************************/
void pid_setTunings(pid_data_t pid_data, float Kp, float Ki, float Kd)
{
   if (Kp<0 || Ki<0 || Kd<0) return;

   pid_data->dispKp = Kp; pid_data->dispKi = Ki; pid_data->dispKd = Kd;

   float SampleTimeInSec = ((float)pid_data->SampleTime)/1000;
   pid_data->kp = Kp;
   pid_data->ki = Ki * SampleTimeInSec;
   pid_data->kd = Kd / SampleTimeInSec;

  if(pid_data->controllerDirection == PID_REVERSE)
   {
	  pid_data->kp = (0 - pid_data->kp);
	  pid_data->ki = (0 - pid_data->ki);
	  pid_data->kd = (0 - pid_data->kd);
   }
}//pid_setTunings

/* pid_setSampleTime(...) *********************************************************
 * sets the period, in Milliseconds, at which the calculation is performed
 ******************************************************************************/
void pid_setSampleTime(pid_data_t pid_data, int NewSampleTime)
{
   if (NewSampleTime > 0)
   {
      float ratio  = (float)NewSampleTime
                      / (float)pid_data->SampleTime;
      pid_data->ki *= ratio;
      pid_data->kd /= ratio;
      pid_data->SampleTime = (unsigned long)NewSampleTime;
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
void pid_setOutputLimits(pid_data_t pid_data, float Min, float Max)
{
   if(Min >= Max) return;        //TODO: reportar erro
   pid_data->outMin = Min;
   pid_data->outMax = Max;

   if(pid_data->inAuto)
   {
	   if(pid_data->Output > pid_data->outMax)
		   pid_data->Output = pid_data->outMax;
	   else if(pid_data->Output < pid_data->outMin)
		   pid_data->Output = pid_data->outMin;

	   if(pid_data->ITerm > pid_data->outMax)
		   pid_data->ITerm = pid_data->outMax;
	   else if(pid_data->ITerm < pid_data->outMin)
		   pid_data->ITerm = pid_data->outMin;
   }
}//pid_setOutputLimits

/* pid_setMode(...)****************************************************************
 * Allows the controller Mode to be set to manual (0) or Automatic (non-zero)
 * when the transition from manual to auto occurs, the controller is
 * automatically initialized
 ******************************************************************************/
void pid_setMode(pid_data_t pid_data, int Mode)
{
    int newAuto = (Mode == PID_AUTOMATIC);
    if(newAuto == !pid_data->inAuto)
    {  /*we just went from manual to auto*/
    	pid_initialize(pid_data);
    }
    pid_data->inAuto = newAuto;
}//pid_setMode

/* initialize()****************************************************************
 *	does all the things that need to happen to ensure a bumpless transfer
 *  from manual to automatic mode.
 ******************************************************************************/
void pid_initialize(pid_data_t pid_data)
{
   pid_data->ITerm = pid_data->Output;
   pid_data->lastInput = pid_data->Input;
   if(pid_data->ITerm > pid_data->outMax)
	   pid_data->ITerm = pid_data->outMax;
   else if(pid_data->ITerm < pid_data->outMin)
	   pid_data->ITerm = pid_data->outMin;
}//initialize

/* pid_setControllerDirection(...)*************************************************
 * The PID will either be connected to a DIRECT acting process (+Output leads
 * to +Input) or a REVERSE acting process(+Output leads to -Input.)  we need to
 * know which one, because otherwise we may increase the output when we should
 * be decreasing.  This is called from the constructor.
 ******************************************************************************/
void pid_setControllerDirection(pid_data_t pid_data, int Direction)
{
   if(pid_data->inAuto && Direction != pid_data->controllerDirection)
   {
	  pid_data-> kp = (0 - pid_data->kp);
	  pid_data-> ki = (0 - pid_data->ki);
	  pid_data-> kd = (0 - pid_data->kd);
   }
   pid_data->controllerDirection = Direction;
}//pid_setControllerDirection

/* Status Funcions*************************************************************
 * Just because you set the Kp=-1 doesn't mean it actually happened.  these
 * functions query the internal state of the PID.  they're here for display
 * purposes.  this are the functions the PID Front-end uses for example
 ******************************************************************************/
float pid_getKp(pid_data_t pid_data){ return  pid_data->dispKp; }
float pid_getKi(pid_data_t pid_data){ return  pid_data->dispKi;}
float pid_getKd(pid_data_t pid_data){ return  pid_data->dispKd;}
int pid_getMode(pid_data_t pid_data){ return  pid_data->inAuto ? PID_AUTOMATIC : PID_MANUAL;}
int pid_getDirection(pid_data_t pid_data){ return pid_data->controllerDirection;}

