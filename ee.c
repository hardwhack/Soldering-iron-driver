/*
 * File:   ee.c
 * Author: Bogdan Kecman <bogdan.kecman@crsn.rs>
 *
 * Created on January 6, 2012, 4:20 PM
 * Site: http://sid.crsndoo.com/ (hosted on SourceForge.NET)
 * Licence: Creative Commons Attribution License
 *
 * $Id: ee.c 32 2012-05-23 12:41:22Z bogdan-kecman $
 * $Rev: 32 $
 */
#include "hardware.h"

void writeEEProm(unsigned char data, unsigned char address ){

    EEADR  = address;
    EEDATA = data;
    EECON1bits.EEPGD = 0;
    EECON1bits.CFGS = 0;
    EECON1bits.WREN = 1;

    INTCONbits.GIEH   = 0;
    INTCONbits.GIEL   = 0;

    _asm
    MOVLW 0x55
    MOVWF EECON2,0
    MOVLW 0xAA
    MOVWF EECON2,0
    _endasm
    EECON1bits.WR=1;

    INTCONbits.GIEH   = 1;
    INTCONbits.GIEL   = 1;

    while (EECON1bits.WR == 1);
    EECON1bits.WREN = 0;
}

unsigned char readEEProm(unsigned char address){
    EEADR = address;
    EECON1bits.EEPGD = 0;
    EECON1bits.CFGS = 0;
    EECON1bits.RD = 1;
    return EEDATA;
}
