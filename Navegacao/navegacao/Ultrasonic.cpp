/*
  Ultrasonic.h - Library for HR-SC04 Ultrasonic Ranging Module.
  Created by ITead studio. Alex, Apr 20, 2010.
  iteadstudio.com
  Adapted for Hiperion - Dez, 2011.
*/


#include "Ultrasonic.h"
#define TMAX 35000 // em micro segundos

Ultrasonic::Ultrasonic(int TP, int EP)
{
   pinMode(TP,OUTPUT);
   pinMode(EP,INPUT);
   Trig_pin=TP;
   Echo_pin=EP;
}

long Ultrasonic::Timing()
{
  digitalWrite(Trig_pin, LOW);
  delayMicroseconds(2);
  digitalWrite(Trig_pin, HIGH);
  delayMicroseconds(10);
  digitalWrite(Trig_pin, LOW);
  duration = pulseIn(Echo_pin,HIGH, TMAX);
  return duration;
}

long Ultrasonic::Ranging(int sys)
{
  Timing();
  distacne_cm = duration /29 / 2 ;
  distance_inc = duration / 74 / 2;
  if (sys)
  return distacne_cm;
  else
  return distance_inc;
}

long Ultrasonic::Distancia(int trigger)
{
  //seta o pino trigger com um pulso baixo "LOW" (ou desligado ou ainda 0)
  digitalWrite(trigger, LOW);
  // delay de 2 microssegundos
  delayMicroseconds(2);
  //seta o pino trigger com pulso alto "HIGH" (ou ligado ou ainda 1)
  digitalWrite(trigger, HIGH);
  //delay de 10 microssegundos
  delayMicroseconds(10);
  //seta o pino trigger com pulso baixo novamente
  digitalWrite(trigger, LOW);
  // função Ranging, faz a conversão do tempo de SENSOR DISTANCIA resposta do echo em centimetros
  return (Ranging(CM));
}
