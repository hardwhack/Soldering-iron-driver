/*
 * File:   lcd.c
 * Author: Bogdan Kecman <bogdan.kecman@crsn.rs>
 *
 * Created on November 23, 2011, 6:32 PM
 * Site: http://sid.crsndoo.com/ (hosted on SourceForge.NET)
 * Licence: Creative Commons Attribution License
 *
 * $Id: lcd.c 32 2012-05-23 12:41:22Z bogdan-kecman $
 * $Rev: 32 $
 */

#include "hardware.h"
#include <delays.h>

//OSC = 48M, 1T = 1/12M = .00000008333333sec
// 2us ~= 48T
// 2ms = 48000T

/* SAFE
#define LCD_DELAY_LONG()  Delay10KTCYx(48)  //delay_ms(40)
#define LCD_DELAY()       Delay10KTCYx(19)  //delay_ms(16)
#define LCD_DELAY_F()     Delay10KTCYx(5)   //delay_ms(4)
#define LCD_DELAY_SHORT() Delay10TCYx(5)    //delay_us(4)
*/

#define LCD_DELAY_LONG()  Delay10KTCYx(48)
#define LCD_DELAY()       Delay10KTCYx(1)
#define LCD_DELAY_F()     Delay10KTCYx(20)
#define LCD_DELAY_SHORT() Delay10TCYx(5)  

//safe E pulse length min 1000ns
#define LCD_STROBE_EN()   { ClrWdt(); PIN_LCD_E = 1;  Delay10TCYx(1);ClrWdt(); PIN_LCD_E = 0; Delay10TCYx(1); ClrWdt(); }
#define lcd_line_two 0x40
#define lcd_type 2

void lcd_send_nibble( unsigned char n );
void lcd_init(void);
void lcd_send_byte( unsigned char address, unsigned char n );
void lcd_gotoxy(unsigned char x, unsigned char y);

void lcd_send_nibble( unsigned char n ) {
  PIN_LCD_D4 = (n & 1) ? 1 : 0;
  PIN_LCD_D5 = (n & 2) ? 1 : 0;
  PIN_LCD_D6 = (n & 4) ? 1 : 0;
  PIN_LCD_D7 = (n & 8) ? 1 : 0;
  LCD_STROBE_EN();
}

void lcd_init(){
  PIN_LCD_E  = 0;
  PIN_LCD_RS = 0;
  PIN_LCD_D4 = 0;
  PIN_LCD_D5 = 0;
  PIN_LCD_D6 = 0;
  PIN_LCD_D7 = 0;

  LCD_DELAY_LONG();

  lcd_send_nibble(3);
  Delay10KTCYx(6); //5ms

  lcd_send_nibble(3);
  Delay100TCYx(25); //200us

  lcd_send_nibble(3);
  Delay100TCYx(25); //200us

  lcd_send_nibble(2);
  Delay10KTCYx(6); //5ms

  lcd_send_nibble(2);
  lcd_send_nibble(8);      //4bits 2lines
  Delay10KTCYx(6); //5ms

  lcd_send_nibble(0);
  lcd_send_nibble(8);      //no shift, hide cursor
  Delay10KTCYx(6); //5ms

  lcd_send_nibble(0);
  lcd_send_nibble(1);      //clear
  Delay10KTCYx(6); //5ms

  lcd_send_nibble(0);
  lcd_send_nibble(6);      //left to right
  Delay10KTCYx(6); //5ms

  lcd_send_nibble(0);
  lcd_send_nibble(0xC);    //turn on
  Delay10KTCYx(6); //5ms

  LCD_DELAY_LONG();

}


void lcd_send_byte( unsigned char address, unsigned char n ) {
   PIN_LCD_RS = 0;
   LCD_DELAY();
   PIN_LCD_RS = address;
   ClrWdt();
   PIN_LCD_E = 0;
   lcd_send_nibble(n >> 4);
   lcd_send_nibble(n & 0xf);
}

void lcd_gotoxy(unsigned char x, unsigned char y){
   unsigned char address;

   if(y!=1)
      address = lcd_line_two;
   else
      address = 0;

   address += x-1;
   lcd_send_byte(0,0x80|address);
}

int _user_putc (char c){
   switch (c)
   {
      case '\a'   :  lcd_gotoxy(1,1);     break;
      case '\f'   :  
                     lcd_send_byte(0,1);
                     LCD_DELAY_F();
                     break;
      case '\n'   : lcd_gotoxy(1,2);        break;
      case '\b'   : lcd_send_byte(0,0x10);  break;
      default     : lcd_send_byte(1,c);     break;
   }
   return c;
}