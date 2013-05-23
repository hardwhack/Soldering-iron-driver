/* 
 * File:   calibration.c
 * Author: arhimed
 *
 * Created on January 6, 2012, 4:20 PM
 * Site: http://sid.crsndoo.com/ (hosted on SourceForge.NET)
 * Licence: Creative Commons Attribution License
 *
 * $Id: calibration.c 32 2012-05-23 12:41:22Z bogdan-kecman $
 * $Rev: 32 $
 */

#include "ee.h"
#include "pid.h"
#include "hardware.h"

#include <stdio.h>
#include <stdlib.h>

//from MAIN
float adc2temp(float x);
float readTemperature(unsigned char x);
float ntc2temp(float x);
extern float measuredTemperature;
extern int targetTemperature;
extern volatile signed int encoder;
extern signed int encoderMAX;
extern signed int encoderMIN;
extern volatile unsigned char menu;

void customCalibration(void);



float slope;
float offset;

float newSlope;
float newOffset;


void customCalibration(void){
    unsigned int T190C;
    unsigned int T260C;
    int oldTarget;
    int lp, rp;
    
    oldTarget = targetTemperature;


    menu = 10;
    encoder = 0;
    encoderMAX = 1023;
    encoderMIN = 0;
    fprintf(_H_USER,  (const far rom char*)"\fStep1: 190C"); //Sn:Pb = 60:40 melting point
    menu = 10;
    while(menu == 10){
        targetTemperature = encoder;
        measuredTemperature = readTemperature(1);
        fprintf(_H_USER,  (const far rom char*)"\nA%04i M%04i N%03i", (int)targetTemperature, (int) measuredTemperature, (int) ntc2temp(readTemperature(0)));
        pidCompute();
        Delay10KTCYx(100);
        ClrWdt();
    }
    T190C = targetTemperature; //ADC VALUE FOR 190C

     fprintf(_H_USER,  (const far rom char*)"\fStep2: 260C"); //no idea what's the best way to detect 260C other then measuring it :(, using low value as most thermometers can go only up to 260C
    menu = 11;
    while(menu == 11){
        targetTemperature = encoder;
        measuredTemperature = readTemperature(1);
        fprintf(_H_USER,  (const far rom char*)"\nA%04i M%04i N%03i", (int)targetTemperature, (int) measuredTemperature, (int) ntc2temp(readTemperature(0)));
        pidCompute();
        Delay10KTCYx(100);
        ClrWdt();
    }
    T260C = targetTemperature; //ADC VALUE FOR 260C

    targetTemperature = oldTarget;
    measuredTemperature = 1;
    pwmValue = 0;
    heaterOFF();

    /*
     * Now calculate offset and slope for this 2 points
     * 
     * T = offset + slope * ADCVALUE
     * 
     * SLOPE  = (T2-T1) / (ADC2-ADC1)
     * OFFSET = T1 - SLOPE * ADC1
     * 
     * 
     */
    newSlope = 70.0 / (T260C-T190C);
    newOffset = 190 - newSlope * T190C;

    lp = (int)((float)newOffset);
    rp = (int)((float)newOffset*10000)-lp*10000;
    if (rp < 0) rp = -rp;
    fprintf(_H_USER,  (const far rom char*)"\fOffset %04i.%04i",lp,rp);

    lp = (int)((float)newSlope);
    rp = (int)((float)newSlope*10000)-lp*10000;
    fprintf(_H_USER,  (const far rom char*)"\nSlope  %04i.%04i",lp,rp);
    menu = 12;
    while(menu == 12){
        Delay10KTCYx(100);
        ClrWdt();
    }

    menu = 8;
}

void calibration(void) {
    unsigned char j;

    encoderMIN = 0;
    encoderMAX = 4;
    encoder = 0;
    fprintf(_H_USER,  (const far rom char*)"\fSelect SENSOR");
    menu = 7;
    while(menu == 7){
        ClrWdt();
        switch (encoder){
            case 0:
                fprintf(_H_USER,  (const far rom char*)"\nCUSTOM    ");
                newSlope = slope;
                newOffset = offset;
                break;
            case 1:
                fprintf(_H_USER,  (const far rom char*)"\nTC K-Type "); //511C MAX with the 240x gain on the TC input
                newSlope  =    0.50;
                newOffset =    0.00;
                break;
            case 2:
                fprintf(_H_USER,  (const far rom char*)"\nTC J-Type "); //383C MAX with the 240x gain on the TC input
                newSlope  =    0.375;
                newOffset =    0.000;
                break;
            case 3:
                fprintf(_H_USER,  (const far rom char*)"\nRTD HAKKO ");
                newSlope  =    0.75;
                newOffset = -207.00;
                break;
            case 4:
                fprintf(_H_USER,  (const far rom char*)"\nRTD GORDAK"); 
                newSlope  =    3.0434;
                newOffset =  -32.2508;
                break;
            case 5:
                fprintf(_H_USER,  (const far rom char*)"\nRTD QUICK "); //NOT TESTED
                newSlope  =    0.00;
                newOffset =    0.00;
                break;
            case 6:
                fprintf(_H_USER,  (const far rom char*)"\nRTD BAKKU "); //NOT TESTED
                newSlope  =    0.00;
                newOffset =    0.00;
                break;
        }
    }
    if (encoder == 0) customCalibration();
    fprintf(_H_USER,  (const far rom char*)"\fAPPLY CHANGES?");
    encoderMIN = 0;
    encoderMAX = 1;
    encoder = 0;
    menu = 8;
    while(menu == 8){
        ClrWdt();
        switch (encoder){
            case 0:
                fprintf(_H_USER,  (const far rom char*)"\n     NO ");
                break;
            case 1:
                fprintf(_H_USER,  (const far rom char*)"\n     YES");
                break;
        }        
    }
    if (encoder == 1){
        slope = newSlope;
        offset = newOffset;
    }

    fprintf(_H_USER,  (const far rom char*)"\fSAVE CHANGES?");
    encoderMIN = 0;
    encoderMAX = 1;
    encoder = 0;
    menu = 9;
    while(menu == 9){
        ClrWdt();
        switch (encoder){
            case 0:
                fprintf(_H_USER,  (const far rom char*)"\n     NO ");
                break;
            case 1:
                fprintf(_H_USER,  (const far rom char*)"\n     YES");
                break;
        }
    }
    if (encoder == 1){
      for (j = 0; j<4; j++)  writeEEProm( ((char*)(&newOffset))[j], j+0x18);
      for (j = 0; j<4; j++)  writeEEProm( ((char*)(&newSlope ))[j], j+0x14);
    }
    ClrWdt();
    menu = 8;
}

