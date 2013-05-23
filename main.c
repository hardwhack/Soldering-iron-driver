/*
 * File:   main.c
 * Author: Bogdan Kecman <bogdan.kecman@crsn.rs>
 *
 * Created on November 23, 2011, 4:24 PM
 * Site: http://sid.crsndoo.com/ (hosted on SourceForge.NET)
 * Licence: Creative Commons Attribution License
 *
 * $Id$
 * $Rev$
 */

//Not really relevant if bootloader sets the fuse bits  ..

#pragma config PLLDIV   = 5         // 20 MHz
#pragma config CPUDIV   = OSC1_PLL2
#pragma config USBDIV   = 2         // Clock source from 96MHz PLL/2
#pragma config FOSC     = HSPLL_HS
#pragma config FCMEN    = OFF
#pragma config IESO     = OFF
#pragma config PWRT     = OFF
#pragma config BOR      = OFF
#pragma config BORV     = 3
#pragma config VREGEN   = ON		//USB Voltage Regulator
#pragma config WDT      = ON
#pragma config WDTPS    = 1024
#pragma config MCLRE    = ON
#pragma config LPT1OSC  = OFF
#pragma config PBADEN   = OFF
#pragma config CCP2MX   = OFF       // CCP2 on RB3
#pragma config STVREN   = ON
#pragma config LVP      = OFF
//#pragma config ICPRT    = OFF     // Dedicated In-Circuit Debug/Programming
#pragma config XINST    = OFF       // Extended Instruction Set
#pragma config CP0      = OFF
#pragma config CP1      = OFF
//#pragma config CP2      = OFF
//#pragma config CP3      = OFF
#pragma config CPB      = OFF
//#pragma config CPD      = OFF
#pragma config WRT0     = OFF
#pragma config WRT1     = OFF
//#pragma config WRT2     = OFF
//#pragma config WRT3     = OFF
#pragma config WRTB     = OFF       // Boot Block Write Protection
#pragma config WRTC     = OFF
//#pragma config WRTD     = OFF
#pragma config EBTR0    = OFF
#pragma config EBTR1    = OFF
//#pragma config EBTR2    = OFF
//#pragma config EBTR3    = OFF
#pragma config EBTRB    = OFF



///////////////////////////////
//EEPROM DATA//////////////////
///////////////////////////////

//sizeof(float)  == 4
//sizeof(int)    == 2

#pragma romdata pid_values=0xf00000
rom float pid_values[] = {
  3.500,                                 //Initial P value
  1.875,                                 //Initial I value
  3.250                                  //Initial D value
};

#pragma romdata startup_temp=0xf0000C
rom signed int startup_temp[] = { 20 };       // Initial start temp value (/5)

#pragma romdata sleep_temp=0xf0000E
rom signed int sleep_temp[] = { 300 };        // Sleep temp value (times 2)

#pragma romdata boost_temp=0xf00010
rom signed int boost_temp[] = { 800 };        // Boost temp value (times 2)

#pragma romdata backglight_value=0xf00012
rom signed int backglight_value[] = { 50 };   // backglight value 0-100

#pragma romdata calibration_data=0xf00014
rom float calibration_data[] = {
       0.75,                                  // SLOPE  (HAKKO ORIGINAL)
    -207.00                                   // OFFSET (HAKKO ORIGINAL)
};


#include "hardware.h"
#include "lcd.h"
#include "calibration.h"
#include "pid.h"
#include "ee.h"

#include <adc.h>
#include <stdio.h>
#include <usart.h>
#include <pwm.h>
#include <timers.h>

void low_isr(void);
void high_isr(void);
void setupPorts(void);
void setupInterrupts(void);
void loadDefaults(void);
float readTemperature(unsigned char x);
float adc2temp(float x);
void pidInit(void);
void pidCompute(void);


void runMenu(void);
void goToSleep(void);
void goToBoost(void);

void sendFloat(float x);
void savePID(float p, float i, float d);




//204-GT thermistor with 10K in voltage divider
const static unsigned int NTC_TABLE[] = {
    0x7FFF, 0x03E7, 0x03D8, 0x03C2, 0x03A3, 0x0378, 0x0342, 0x02FF, 0x02B3, 0x0261,     // 0-90 //0==1009
    0x020E, 0x01BE, 0x0174, 0x0133, 252,    206,    168,    136,    111,    91,         // 100-190
    75,     62,     51,     43,     36,     30,     26,     22,     18,     14,         // 200-290
    1};

volatile unsigned int heaterPos = 0;
volatile unsigned long int millis = 0;
volatile unsigned char changed = 0;
volatile unsigned char menu = 0;
volatile signed int encoder = 0;
signed int encoderMAX = 1023;
signed int encoderMIN = 0;

float measuredTemperature = 0;
int targetTemperature = 0;
float oldTemperature = 0;

unsigned char starting=1;



#if defined(BOOTLOADER)
extern void _startup (void);
#pragma code REMAPPED_RESET_VECTOR = REMAPPED_RESET_VECTOR_ADDRESS
void _reset (void){
    _asm goto _startup _endasm
}
#endif

#pragma code REMAPPED_HIGH_INTERRUPT_VECTOR = REMAPPED_HIGH_INTERRUPT_VECTOR_ADDRESS
void Remapped_High_ISR (void){
    _asm goto high_isr _endasm
}

#pragma code REMAPPED_LOW_INTERRUPT_VECTOR = REMAPPED_LOW_INTERRUPT_VECTOR_ADDRESS
void Remapped_Low_ISR (void){
	     _asm goto low_isr _endasm
}

#if defined(BOOTLOADER)
#pragma code HIGH_INTERRUPT_VECTOR = 0x08
void High_ISR (void){
    _asm goto REMAPPED_HIGH_INTERRUPT_VECTOR_ADDRESS _endasm
}

#pragma code LOW_INTERRUPT_VECTOR = 0x18
void Low_ISR (void){
    _asm goto REMAPPED_LOW_INTERRUPT_VECTOR_ADDRESS _endasm
}
#endif

#pragma code

#pragma interruptlow low_isr
void low_isr(void) {
    static unsigned long int debounce = 0;
    unsigned long int isr_started = millis;
    static unsigned int integB = 0, integ1 = 0, integ2 = 0;
    static unsigned long int currB = 0, curr1 = 0, curr2 = 0;
    long unsigned int time_diff;

    if (INTCON3bits.INT1IF) { //ENC1 rising edge
        if (isr_started - debounce > DEBOUNCE_TIME) {
            if (PIN_E2) {
                if (encoder > encoderMIN) encoder--;
            } else {
                if (encoder < encoderMAX) encoder++;
            }
            changed = 1;
        }
        debounce = isr_started;
        INTCON3bits.INT1IF = 0;
    } else if (INTCON3bits.INT2IF) { //ENC2 rising edge
        if (isr_started - debounce > DEBOUNCE_TIME) {

#ifndef DETENT_DOUBLE
            if (PIN_E1) {
                if (encoder < encoderMAX) encoder++;
            } else {
                if (encoder > encoderMIN) encoder--;
            }
            changed = 1;
#endif
        }
        debounce = isr_started;
        INTCON3bits.INT2IF = 0;
    } else if (PIR1bits.TMR2IF) {   //q 2ms timer routine
#ifndef USE_ENCODER        //debounce and autorepeat all rolled into one!
                            // NB when using buttons, INT0, INT1, INT2 are all turned off so
                            // should never go into above interupt routines or high_isr below
                            // uses Kenneth Kuhn?s debounce algorithm found on Hackaday
        if (PIN_EB){
            if (integB >0)                  //ie not pushed
                integB--;
        } else if (integB <DEBOUNCE_COUNT)  //ie pushed
            integB++;

        if (PIN_E1){
            if (integ1 >0)                  //ie pinE1 not pushed
                integ1--;
        } else if (integ1 <DEBOUNCE_COUNT)  //ie pinE1 pushed
            integ1++;

        if (PIN_E2){
            if (integ2 >0)                  //ie pinE2 not pushed
                integ2--;
        } else if (integ2 <DEBOUNCE_COUNT)  //ie pinE2 pushed
            integ2++;

        if (integB == 0)                    //proccess integ values
            currB = 0;
        else if (integB >= DEBOUNCE_COUNT) {
            if (currB == 0){
                currB = 1;
                menu++;
            }
            integB = DEBOUNCE_COUNT;  /* defensive code if integrator got corrupted */
        }

        if (integ2 == 0)
            curr2 = 0;
        else if (integ2 >= DEBOUNCE_COUNT) {
            if (curr2 == 0){
                debounce = millis;
            }
            curr2++;
            time_diff = millis - debounce;
            if (time_diff == 0 || ( time_diff > AUTOREPEAT_DELAY && ( time_diff%AUTOREPEAT_PERIOD == 0)) ) {
                if (encoder < encoderMAX) {
                    encoder++;
                    changed = 1;
                }
            }
            integ2 = DEBOUNCE_COUNT;  /* defensive code if integrator got corrupted */

        }

//        if (!starting) {
//            lcd_gotoxy(1, 2);
//            fprintf(_H_USER,  (const far rom char*)"i %3i,curr1 %4li", integ1, curr1);
//        }
        if (integ1 == 0)
            curr1 = 0;
        else if (integ1 >= DEBOUNCE_COUNT) {
            if (curr1 == 0){
                debounce = millis;
            }
            curr1++;
            time_diff = millis - debounce;
            if (time_diff == 0 || ( time_diff > AUTOREPEAT_DELAY && ( time_diff%AUTOREPEAT_PERIOD == 0)) ) {
                if (encoder > encoderMIN) {
                    encoder--;
                    changed = 1;
                }
            }
            integ1 = DEBOUNCE_COUNT;  /* defensive code if integrator got corrupted */

        }
#endif
        millis += 2;
        if (((menu < 10) || (menu > 12)) && (starting != 1) && (measuredTemperature > TOOHOT)) while (1) heaterOFF();
        if (++heaterPos > maxHeaterPos) heaterPos = 0;
        if (heaterPos >= pwmValue) {
            heaterOFF();
        } else {
            heaterON();
        }
        PIR1bits.TMR2IF = 0;
    }

}

#pragma interrupt high_isr
void high_isr(void) {
    static unsigned long int debounce = 0;
    unsigned long int isr_started = millis;

    if (INTCONbits.INT0IF) {
        if (isr_started - debounce > DEBOUNCE_TIME) menu++;
        INTCONbits.INT0IF = 0;
    }
    debounce = isr_started;
}


#pragma code


void setupPorts(){

  ClrWdt();
  TRIS_HEATER         = OUTPUT;
  TRIS_LCD_D4         = OUTPUT;
  TRIS_LCD_D5         = OUTPUT;
  TRIS_LCD_D6         = OUTPUT;
  TRIS_LCD_D7         = OUTPUT;
  TRIS_LCD_E          = OUTPUT;
  TRIS_LCD_RS         = OUTPUT;
  TRIS_LCD_BRIGHTNESS = OUTPUT;

  TRIS_TEMPERATURE    = INPUT;
  TRIS_NTC            = INPUT;
  TRIS_E1             = INPUT;
  TRIS_E2             = INPUT;
  TRIS_EB             = INPUT;
  TRIS_B1             = INPUT;
  TRIS_B2             = INPUT;
  TRIS_USBDETECT      = INPUT;

  CMCON               = 0x07; // comparators off

  heaterOFF();

  lcd_init();

  OpenTEMP();
  setupLCDBackLight();
  ClrWdt();
}

void setupInterrupts(){
    RCONbits.IPEN = 1;

    //PIE1bits.SPPIE  = 0; //does not exist on 28pin devices like 18F2550
    PIE1bits.ADIE   = 0;
    PIE1bits.RCIE   = 0;
    PIE1bits.TXIE   = 0;
    PIE1bits.SSPIE  = 0;
    PIE1bits.CCP1IE = 0;
    PIE1bits.TMR2IE = 1;
    PIE1bits.TMR1IE = 0;

    PIE2bits.OSCFIE = 0; //osc fail
    PIE2bits.CMIE   = 0; //comparator
    PIE2bits.USBIE  = 0; //usb
    PIE2bits.EEIE   = 0; //eeprom write
    PIE2bits.BCLIE  = 0; //bus collision
    PIE2bits.HLVDIE = 0; //high low voltage detect
    PIE2bits.TMR3IE = 0;
    PIE2bits.CCP2IE = 1;

    PIR1 = 0;
    PIR2 = 0;

    //IPR1bits.SPPIP = 0; //does not exist on 28pin devices
    IPR1bits.ADIP   = 0;
    IPR1bits.RCIP   = 0;
    IPR1bits.TXIP   = 0;
    IPR1bits.SSPIP  = 0;
    IPR1bits.CCP1IP = 0;
    IPR1bits.TMR2IP = 0; //low priority
    IPR1bits.TMR1IP = 0; //low priority

    IPR2bits.OSCFIP = 0;
    IPR2bits.CMIP   = 0;
    IPR2bits.USBIP  = 0;
    IPR2bits.EEIP   = 0;
    IPR2bits.BCLIP  = 0;
    IPR2bits.HLVDIP = 0;
    IPR2bits.TMR3IP = 0; //low priority
    IPR2bits.CCP2IP = 0; //low priority


    INTCON2bits.RBPU    = 1; //disable portb pull-up's

    INTCON2bits.INTEDG0 = 1; //INT0 on rising edge
    INTCON2bits.INTEDG1 = 1; //INT1 on rising edge
    INTCON2bits.INTEDG2 = 1; //INT2 on rising edge

    INTCON2bits.TMR0IP  = 1; //TMR0 set to high priority
    INTCON2bits.RBIP    = 0; //port B change interrupt set to low priority

    INTCON3bits.INT2IP  = 0; //INT2 low priority
    INTCON3bits.INT1IP  = 0; //INT1 low priority
#ifdef USE_ENCODER
    INTCON3bits.INT2IE  = 1; //INT2 enable
    INTCON3bits.INT1E   = 1; //INT1 enable
    INTCONbits.INT0IE = 1;   //INT0 enable (always high priority)
#else
    INTCON3bits.INT2IE  = 0; //INT2 enable
    INTCON3bits.INT1E   = 0; //INT1 enable
    INTCONbits.INT0IE = 0;   //INT0 enable (always high priority)
#endif

    INTCON3bits.INT2IF  = 0;
    INTCON3bits.INT1IF  = 0;

    INTCONbits.GIEH   = 1;
    INTCONbits.GIEL   = 1;
    INTCONbits.TMR0IE = 0;
    INTCONbits.RBIE   = 0;
    INTCONbits.TMR0IF = 0;
    INTCONbits.RBIF   = 0;
    INTCONbits.INT0IF = 0;



    TRISBbits.TRISB6 =0;

    //OpenTimer1(T1_SOURCE_INT & T1_PS_1_1 & T1_OSC1EN_ON & T1_SYNC_EXT_OFF & T1_8BIT_RW);
    //WriteTimer1(0);


    //input = FOsc/4 (48M / 4 = 12MHz)
    //T2 INT = 16*6*250 / 12000000 = 0.002000000sec = 2ms
    OpenTimer2(  T2_PS_1_16 &
                 T2_POST_1_6);
    PR2 = 250;
    WriteTimer2(0);

}



void loadDefaults(){
    float p,i,d;
    unsigned char j;

    heaterPos = 0;
    pwmValue = 0;

    encoder = readEEProm(0x12);
    setLCDBackLight(encoder * 10);

    ((unsigned char*) (&encoder))[0] = readEEProm(12);
    ((unsigned char*) (&encoder))[1] = readEEProm(13);

    encoderMAX = 120;
    encoderMIN = 20;

    targetTemperature = encoder * 5;

    for (j = 0; j<4; j++)  ((unsigned char*)(&p))[j] = readEEProm(j);        //0,1,2,3   P
    for (j = 0; j<4; j++)  ((unsigned char*)(&i))[j] = readEEProm(j+4);      //4,5,6,7   I
    for (j = 0; j<4; j++)  ((unsigned char*)(&d))[j] = readEEProm(j+8);      //8,9,10,11 D
    pidSetKoefficients(p, i, d);

    for (j = 0; j<4; j++)  ((unsigned char*)(&slope))[j]  = readEEProm(j+0x14);
    for (j = 0; j<4; j++)  ((unsigned char*)(&offset))[j] = readEEProm(j+0x18);

    measuredTemperature = adc2temp(readTemperature(1));
}

float readTemperature(unsigned char x){
    unsigned long int rawADC;
    int i;

    rawADC = 0;
    for (i = 0; i< 8; i++){
        ClrWdt();
        if (x) {
            OpenTEMP();
        } else {
            OpenNTC();
        }
        Delay10TCYx( 100 );
        ConvertADC();         // Start conversion
        while( BusyADC() );   // Wait for completion
        rawADC += ReadADC();   // Read result
    }

    return (rawADC / 8.0);
}

float adc2temp(float x){
    static float t=-10;
    if (t !=x ){
        changed = 1;
        t = x;
    }
    return offset + slope * x;
}

void measureAndCompute(unsigned char x){
    int lPart;
    int rPart;

    measuredTemperature = (measuredTemperature + adc2temp(readTemperature(1))) / 2.0;
    if (measuredTemperature > TOOHOT) while(1) heaterOFF();
    if (changed == 1 && menu == 0) {
        if (x == 0) targetTemperature = encoder * 5;

        fprintf(_H_USER,  (const far rom char*)"\fTarget:  %3i \n", targetTemperature);
        lPart = (int)((float)measuredTemperature);
        rPart = (int)((float)measuredTemperature*100)-lPart*100;
        fprintf(_H_USER,  (const far rom char*)"Current: %3i.%02i", lPart,rPart);

        changed = 0;
    }
    pidCompute();
}


void savePID(float p, float i, float d){
    unsigned int j;
    //sizeof(float) == 4
    for (j = 0; j<4; j++)  writeEEProm( ((char*)(&p))[j], j);        //0,1,2,3   P
    for (j = 0; j<4; j++)  writeEEProm( ((char*)(&i))[j], j+4);      //4,5,6,7   I
    for (j = 0; j<4; j++)  writeEEProm( ((char*)(&d))[j], j+8);      //8,9,10,11 D
}



void runMenu(){
   signed int oldEncoder;
   float p,i,d;
   unsigned long int startTime;

   oldEncoder = encoder;

   pwmValue = 0;
   heaterOFF();

   while (menu){
       Delay10KTCYx(50);
       ClrWdt();
       switch (menu){
           case 1:
               encoder = Pk*1000;
               encoderMIN = 0;
               encoderMAX = 32000;
               startTime = millis;
               changed = 0;
               fprintf(_H_USER,  (const far rom char*)"\fPage 1 of 6");
               while(menu == 1){
                 if (encoder < 0) encoder = 0;
                 fprintf(_H_USER,  (const far rom char*)"\nPk: %3i.%03i", encoder/1000, encoder - ((int)(encoder/1000)) * 1000);
                 p = encoder / 1000.0;
                 Delay10KTCYx(50);
                 ClrWdt();
                 if (changed){
                     changed  = 0;
                     startTime = millis;
                 }
                 if (millis - startTime > menuTIMEOUT) {
                     menu = 0;
                     break;
                 }
               }
               break;
           case 2:
               encoder = Ik*1000;
               encoderMIN = 0;
               encoderMAX = 32000;
               startTime = millis;
               changed = 0;
               fprintf(_H_USER,  (const far rom char*)"\fPage 2 of 6");
               while(menu == 2){
                 if (encoder < 0) encoder = 0;
                 fprintf(_H_USER,  (const far rom char*)"\nIk: %3i.%03i", encoder/1000, encoder - ((int)(encoder/1000)) * 1000);
                 i = encoder / 1000.0;
                 Delay10KTCYx(50);
                 ClrWdt();
                 if (changed){
                     changed  = 0;
                     startTime = millis;
                 }
                 if (millis - startTime > menuTIMEOUT) {
                     menu = 0;
                     break;
                 }
               }
               break;
           case 3:
               encoder = Dk*1000;
               encoderMIN = 0;
               encoderMAX = 32000;
               startTime = millis;
               changed = 0;
               fprintf(_H_USER,  (const far rom char*)"\fPage 3 of 6");
               while(menu == 3){
                 if (encoder < 0) encoder = 0;
                 fprintf(_H_USER,  (const far rom char*)"\nDk: %3i.%03i", encoder/1000, encoder - ((int)(encoder/1000)) * 1000);
                 d = encoder / 1000.0;
                 Delay10KTCYx(50);
                 ClrWdt();
                 if (changed){
                     changed  = 0;
                     startTime = millis;
                 }
                 if (millis - startTime > menuTIMEOUT) {
                     menu = 0;
                     break;
                 }
               }
               pidSetKoefficients(p,i,d);
               savePID(p,i,d);
               break;
           case 4:
               encoderMIN = 20;
               encoderMAX = 120;
               startTime = millis;
               changed = 0;
               ((unsigned char*) (&encoder))[0] = readEEProm(12);
               ((unsigned char*) (&encoder))[1] = readEEProm(13);
               fprintf(_H_USER,  (const far rom char*)"\fPage 4 of 6");
               while(menu == 4){
                 fprintf(_H_USER,  (const far rom char*)"\nInit Temp: %3i", encoder*5);
                 Delay10KTCYx(50);
                 ClrWdt();
                 if (changed){
                     changed  = 0;
                     startTime = millis;
                 }
                 if (millis - startTime > menuTIMEOUT) {
                     menu = 0;
                     break;
                 }
               }
               writeEEProm( ((char*)(&encoder))[0], 12);
               writeEEProm( ((char*)(&encoder))[1], 13);
               break;
            case 5:
               encoderMIN = 0;
               encoderMAX = 100;
               encoder = readEEProm(0x12);
               startTime = millis;
               changed = 0;
               fprintf(_H_USER,  (const far rom char*)"\fPage 5 of 6");
               while(menu == 5){
                 fprintf(_H_USER,  (const far rom char*)"\nBacklight: %3i", encoder);
                 setLCDBackLight(encoder*10);
                 Delay10KTCYx(50);
                 ClrWdt();
                 if (changed){
                     changed  = 0;
                     startTime = millis;
                 }
                 if (millis - startTime > menuTIMEOUT) {
                     menu = 0;
                     break;
                 }
               }
               setLCDBackLight(encoder*10);
               writeEEProm( encoder & 0xFF, 0x12);
               break;
           case 6:
               encoderMIN = 0;
               encoderMAX = 1;
               encoder = 0;
               startTime = millis;
               changed = 0;
               fprintf(_H_USER,  (const far rom char*)"\f6/6: CALIBRATION");
               while(menu == 6){
                 if(encoder == 1){
                     fprintf(_H_USER,  (const far rom char*)"\n     ENTER", encoder);
                 }else{
                     fprintf(_H_USER,  (const far rom char*)"\n     SKIP ", encoder);
                 }
                 Delay10KTCYx(50);
                 ClrWdt();
                 if (changed){
                     changed  = 0;
                     startTime = millis;
                 }
                 if (millis - startTime > menuTIMEOUT) {
                     menu = 0;
                     break;
                 }
               }
               break;
           case 7:
               if (encoder == 1){
                 calibration();
                 measuredTemperature = adc2temp(readTemperature(1));
               }
               menu = 8;
               break;

           default:
               fprintf(_H_USER,  (const far rom char*)"\fdone");
               menu = 0;
               startTime = millis;
               changed = 0;
               while (menu == 0){
                 Delay10KTCYx(50);
                 ClrWdt();
                 if (changed){
                     changed  = 0;
                     startTime = millis;
                 }
                 if (millis - startTime > menuTIMEOUT) {
                     menu = 0;
                     break;
                 }
               }
               menu = 0;
       }
   }

   menu = 0;
   encoderMAX = 120;
   encoderMIN = 20;
   encoder = oldEncoder;
}

void goToSleep(){
/*
    float oldTarget;
    oldTarget = targetTemperature;
    targetTemperature = 150;
    changed = 1;
    while (PIN_B1){
        measureAndCompute(1);
        ClrWdt();
    }
    targetTemperature = oldTarget;
    changed = 1;
*/
}

void goToBoost(){
/*
    float oldTarget;
    oldTarget = targetTemperature;
    targetTemperature = 480;
    changed = 1;
    while (PIN_B2){
        measureAndCompute(1);
        ClrWdt();
    }
    targetTemperature = oldTarget;
    changed = 1;
 */
}

void sendFloat(float x){
  long int lPart;
  long int rPart;
  lPart = (long int) x;
  rPart = (long int)( x * 10000.0)-lPart*10000;
  fprintf(_H_USART,  (const far rom char*)"%li.%04li,", lPart,rPart);
}

float ntc2temp(float x){

  unsigned int tempVal;
  unsigned int lowValue;
  unsigned int highValue;
  unsigned int dataValue;
  unsigned int range;
  unsigned char j;

  for(j=1; j<32; ++j){
    tempVal = NTC_TABLE[j];
    if (x > tempVal){
      lowValue = tempVal;
      highValue = NTC_TABLE[j-1];
      range = highValue - lowValue;
      dataValue = x - lowValue;

      return  (j * 10.0 - (dataValue * 10.0 / range));
    }
  }
  return (310);

}

void main(void) {
    volatile int i;
    float f;

    if (RCONbits.TO == 0){ //we had WatchDog timeout, this should never happen
      setupPorts();
      SetDCPWM2(1023);
      heaterOFF();
      fprintf( _H_USER,  (const far rom char*)"\fERROR: WD0001");
      i = 0;
      while (1){
          setLCDBackLight(i++);
          if (i > 1000) i = 0;
          Delay10KTCYx(1);
          ClrWdt();
      }
    }

    //normal startup
    starting = 1;

    setupPorts();
    setupInterrupts();
    fprintf( _H_USER,  (const far rom char*)"\fSID  $Rev$\nelco.crsndoo.com");
    for (i=0;i<10;i++) {Delay10KTCYx(0);ClrWdt();}

    loadDefaults();

    //CHECK CALIBRATION VALUES
    while ((measuredTemperature > TOOHOT) || (measuredTemperature < 0)){
        //Calibration is wrong or no hand piece is attached
        //so start calibration function before we continue
        menu = 7;
        encoder = 1;
        runMenu(); //start calibration procedure
        targetTemperature = 100;
    }
    starting = 0;


    OpenUSART( USART_TX_INT_OFF  &
              USART_RX_INT_OFF  &
              USART_ASYNCH_MODE &
              USART_EIGHT_BIT   &
              USART_CONT_RX     &
              USART_BRGH_HIGH,
              51                ); //  FOSC / (16 * (spbrg + 1)) = 57692.31
    fprintf (_H_USART,  (const far rom char*)"\r\n\r\n\r\n\r\n");

    while(1) {
      measureAndCompute(0);
      if (measuredTemperature > TOOHOT) while(1) heaterOFF();

      Delay10KTCYx(200);
      ClrWdt();


      if (menu){
        runMenu();
      }

      if (PIN_B1){
        goToSleep();
      }

      if (PIN_B2){
        goToBoost();
      }

      //SEND DATA TO USART
      f = ntc2temp(readTemperature(0));
      sendFloat(millis/1000.0);                                                 //SEND TIME
      sendFloat(targetTemperature);                                             //SEND TARGET
      sendFloat(measuredTemperature);                                           //SEND MEASURED HEATER TEMP
      sendFloat(f);                                                             //SEND NTC CALCULATED TIP TEMP
      sendFloat(Pk);                                                            //P coefficient
      sendFloat(Ik);                                                            //I coefficient
      sendFloat(Dk);                                                            //D coefficient
      sendFloat(PTerm);                                                         //P term of the PID formula
      sendFloat(ITerm);                                                         //I term of the PID formula
      sendFloat(DTerm);                                                         //D term of the PID formula
      fprintf ((FILE *)_H_USART, (const far rom char*) "%i\r\n",pwmValue);      //PWM value

    }
}
