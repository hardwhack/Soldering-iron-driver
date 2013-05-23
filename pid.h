/*
 * File:   pid.h
 * Author: Bogdan Kecman <bogdan.kecman@crsn.rs>
 *
 * Created on January 08, 2012, 04:15am
 * Site: http://sid.crsndoo.com/ (hosted on SourceForge.NET)
 * Licence: Creative Commons Attribution License
 *
 * $Id: pid.h 32 2012-05-23 12:41:22Z bogdan-kecman $
 * $Rev: 32 $
 */

#ifndef __PID_H__
#define __PID_H__

extern float Pk;
extern float Ik;
extern float Dk;

extern float PTerm;
extern float ITerm;
extern float DTerm;

extern volatile signed int pwmValue;


void pidCompute(void);
void pidInit(void);
void pidSetKoefficients(float p, float i, float d);


#endif
