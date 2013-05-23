/*
 * File:   lcd.h
 * Author: Bogdan Kecman <bogdan.kecman@crsn.rs>
 *
 * Created on November 23, 2011, 4:24 PM
 * Site: http://sid.crsndoo.com/ (hosted on SourceForge.NET)
 * Licence: Creative Commons Attribution License
 *
 * $Id: lcd.h 32 2012-05-23 12:41:22Z bogdan-kecman $
 * $Rev: 32 $
 */

#ifndef __LCD_H__
#define __LCD_H__

void lcd_init(void);
void lcd_gotoxy(unsigned char x, unsigned char y);
int _user_putc (char c);

#endif