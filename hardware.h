/*
 * File:   hardware.h
 * Author: Bogdan Kecman <bogdan.kecman@crsn.rs>
 *
 * Created on November 23, 2011, 4:24 PM
 * Site: http://sid.crsndoo.com/ (hosted on SourceForge.NET)
 * Licence: Creative Commons Attribution License
 * 
 * $Id: hardware.h 35 2012-05-26 05:07:18Z bogdan-kecman $
 * $Rev: 35 $
 */

#ifndef __SID_GEN4_HARDWARE_H__
#define __SID_GEN4_HARDWARE_H__

#include <p18cxxx.h>
#include <delays.h>

//defined if detent is after both A and B rising edge 
//comment if detent is both after A and after B edge
#define DETENT_DOUBLE

#define DEBOUNCE_TIME 200
#define DEBOUNCE_COUNT 5    //pjk integrator threshold for debounce
#define AUTOREPEAT_DELAY 150 //pjk initial delay of 150 cycles * 2ms = 0.3s
#define AUTOREPEAT_PERIOD 80 //pjk autorepeat period 80 cycles * 2ms = 160ms ~ 6Hz

//defined if you use encoder+button
//commented out if you use 3 buttons
//#define USE_ENCODER

//LCD pins (RW is tied to the ground)
#define PIN_LCD_D4          LATAbits.LATA2
#define PIN_LCD_D5          LATAbits.LATA3
#define PIN_LCD_D6          LATAbits.LATA4
#define PIN_LCD_D7          LATAbits.LATA5

#define PIN_LCD_E           LATBbits.LATB4
#define PIN_LCD_RS          LATBbits.LATB5
#define PIN_LCD_BRIGHTNESS  LATBbits.LATB3

#define TRIS_LCD_D4         TRISAbits.RA2
#define TRIS_LCD_D5         TRISAbits.RA3
#define TRIS_LCD_D6         TRISAbits.RA4
#define TRIS_LCD_D7         TRISAbits.RA5
#define TRIS_LCD_E          TRISBbits.RB4
#define TRIS_LCD_RS         TRISBbits.RB5
#define TRIS_LCD_BRIGHTNESS TRISBbits.RB3

#define PIN_TEMPERATURE     PORTAbits.AN0
#define TRIS_TEMPERATURE    TRISAbits.RA0

#define PIN_NTC             PORTAbits.AN1
#define TRIS_NTC            TRISAbits.RA1

#define PIN_EB              PORTBbits.RB0
#define PIN_E1              PORTBbits.RB1
#define PIN_E2              PORTBbits.RB2
#define TRIS_EB             TRISBbits.RB0
#define TRIS_E1             TRISBbits.RB1
#define TRIS_E2             TRISBbits.RB2

#define PIN_B1              PORTCbits.RC1
#define TRIS_B1             TRISCbits.RC1
#define PIN_B2              PORTCbits.RC0
#define TRIS_B2             TRISCbits.RC0

#define PIN_USBDETECT       PORTBbits.RB7
#define TRIS_USBDETECT      TRISBbits.RB7

#define PIN_HEATER          PORTCbits.RC2
#define TRIS_HEATER         TRISCbits.RC2

#define heaterON()          {PIN_HEATER = 0;}
#define heaterOFF()         {PIN_HEATER = 1;}

#define INPUT  1
#define OUTPUT 0

//select appropriate configuration rather then
//uncommenting a define here
//#define PROGRAMMABLE_WITH_MICROCHIP_USB_HID_BOOTLOADER
//#define PROGRAMMABLE_WITH_USB_DIOLAN_BOOTLOADER

#if defined(PROGRAMMABLE_WITH_MICROCHIP_USB_HID_BOOTLOADER)
    #define BOOTLOADER
    #define REMAPPED_RESET_VECTOR_ADDRESS		0x1000
    #define REMAPPED_HIGH_INTERRUPT_VECTOR_ADDRESS	0x1008
    #define REMAPPED_LOW_INTERRUPT_VECTOR_ADDRESS	0x1018
#elif defined(PROGRAMMABLE_WITH_USB_DIOLAN_BOOTLOADER)
    #define BOOTLOADER
    #define REMAPPED_RESET_VECTOR_ADDRESS		0x800
    #define REMAPPED_HIGH_INTERRUPT_VECTOR_ADDRESS	0x808
    #define REMAPPED_LOW_INTERRUPT_VECTOR_ADDRESS	0x818
#else
    #define REMAPPED_RESET_VECTOR_ADDRESS		0x00
    #define REMAPPED_HIGH_INTERRUPT_VECTOR_ADDRESS	0x08
    #define REMAPPED_LOW_INTERRUPT_VECTOR_ADDRESS	0x18
#endif



#define OpenTEMP()   OpenADC(\
          ADC_FOSC_64 & ADC_RIGHT_JUST & ADC_20_TAD,\
          ADC_CH0 & ADC_INT_OFF & ADC_VREFPLUS_VDD & ADC_VREFMINUS_VSS,\
          13)


#define OpenNTC()   OpenADC(\
          ADC_FOSC_64 & ADC_RIGHT_JUST & ADC_20_TAD,\
          ADC_CH1 & ADC_INT_OFF & ADC_VREFPLUS_VDD & ADC_VREFMINUS_VSS,\
          13)

#define setLCDBackLight(x)  {ClrWdt();SetDCPWM2(x);}
#define setupLCDBackLight() {ClrWdt();OpenPWM2(0xFF);SetDCPWM2(512);}

#define TOOHOT 550
#define maxHeaterPos 100
#define maxPWM 100
#define minPWM 0

#define menuTIMEOUT 5000

#endif