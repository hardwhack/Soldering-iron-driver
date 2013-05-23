/*
 * File:   ee.h
 * Author: Bogdan Kecman <bogdan.kecman@crsn.rs>
 *
 * Created on January 6, 2012, 4:20 PM
 * Site: http://sid.crsndoo.com/ (hosted on SourceForge.NET)
 * Licence: Creative Commons Attribution License
 *
 * $Id: ee.h 32 2012-05-23 12:41:22Z bogdan-kecman $
 * $Rev: 32 $
 */

#ifndef __EE_H_
#define __EE_H_

void writeEEProm(unsigned char data, unsigned char address );
unsigned char readEEProm(unsigned char address);

#endif
