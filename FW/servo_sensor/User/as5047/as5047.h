#ifndef __AS5047_H
#define __AS5047_H

#include "main.h"

/** volatile **/
#define AS4047_NOP 				0x0000
#define AS4047_ERRFL 			0x0001
#define AS4047_PROG 			0x0003
#define AS4047_DIAAGC 		0x3FFC
#define AS4047_CORDICMAG 	0x3FFD
#define AS4047_ANGLEUNC 	0x3FFE
#define AS4047_ANGLECOM 	0x3FFF


/** non-volatile **/
#define AS4047_ZPOSM 			0x0016
#define AS4047_ZPOSL 			0x0017
#define AS4047_SETTINGS1 	0x0018
#define AS4047_SETTINGS2 	0x0019

#define AS4047_RD 				0x4000    // bit 14 = "1" is Read + parity even
#define AS4047_WR 				0x3FFF    // bit 14 = "0" is Write

void AS5047_Init(void);
float AS5047_Get_Angle(void);

#endif


