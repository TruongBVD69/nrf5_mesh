/*
 * 74HC595.h
 *
 *  Created on: Feb 6, 2023
 *      Author: Admin
 */

#ifndef INC_74HC595_H_
#define INC_74HC595_H_

#include "stdint.h"

#define LATCH_PIN 12
#define CLOCK_PIN 29
#define DATA_PIN  25

void HC595_Init(void);
void HC595_SendByte(uint8_t byte);

#endif /* INC_74HC595_H_ */