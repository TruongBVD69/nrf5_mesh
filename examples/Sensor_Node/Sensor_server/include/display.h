/**
 * @file display.h
 * @author 
 * @brief 
 * @version 1.0
 * @date 2023-02-09
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#ifndef _DISPLAY_H_
#define _DISPLAY_H_

#include "string.h"
#include "stdint.h"

#define DIG0_PIN 2
#define DIG1_PIN 3
#define DIG2_PIN 4
#define DIG3_PIN 5


extern uint8_t digIndex;

void displayInit(void);
void displayDigit(uint8_t dig);

void displayInt(int i);
void displayFloat(float f);

void displayStop(void);
// void displayResume(void);



#endif  //_DISPLAY_H_