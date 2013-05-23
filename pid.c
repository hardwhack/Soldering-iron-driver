/*
 * File:   pid.c
 * Author: Bogdan Kecman <bogdan.kecman@crsn.rs>
 *
 * Created on January 08, 2012, 04:15am
 * Site: http://sid.crsndoo.com/ (hosted on SourceForge.NET)
 * Licence: Creative Commons Attribution License
 *
 * $Id: pid.c 32 2012-05-23 12:41:22Z bogdan-kecman $
 * $Rev: 32 $
 */

#include "hardware.h"

float Pk = 0;
float Ik = 0;
float Dk = 0;

float PTerm = 0;
float ITerm = 0;
float DTerm = 0;

volatile signed int pwmValue = 0;


extern float measuredTemperature;
extern int targetTemperature;
extern float oldTemperature;
extern volatile unsigned long int millis;

void pidCompute(void);
void pidInit(void);
void pidSetKoefficients(float p, float i, float d);


void pidCompute(){
   float error;
   float dCurrent;

   unsigned long int now;
   signed int outPWM;
   static unsigned long int oldTime = 0;

   now = millis;

   if( (now - oldTime) >= 1000){

      error = targetTemperature - measuredTemperature;
      ITerm += (Ik * error);
      if (ITerm > maxPWM){
         ITerm = maxPWM;
      } else if(ITerm < minPWM){
           ITerm= minPWM;
      }

      // negative input derivative instead of
      // positive error derivative works better when changing target
      dCurrent = measuredTemperature - oldTemperature;

      PTerm = Pk * error;
      DTerm = -Dk * dCurrent;

      outPWM = PTerm + ITerm + DTerm;
      if(outPWM > maxPWM){
         outPWM = maxPWM;
      } else if(outPWM < minPWM){
         outPWM = minPWM;
      }

      oldTemperature = measuredTemperature;
      oldTime = now;
      pwmValue = outPWM;
   }
   ClrWdt();
}

void pidInit(){
   oldTemperature = measuredTemperature;
   ITerm = pwmValue;
   if(ITerm > maxPWM) ITerm= maxPWM;
   else if(ITerm < minPWM) ITerm= minPWM;
}

void pidSetKoefficients(float p, float i, float d){
  //koefficients should be positive
  //one could add a check here but it's waste of flash
  //since we know what we should set :D

  // sampling at 1sec makes everything simpler :)
  Pk = p;
  Ik = i;   // * sample time in seconds
  Dk = d;   // / sample time in seconds
  pidInit();
}
