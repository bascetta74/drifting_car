/*
 * encoder.h
 *
 *  Created on: Aug 10, 2017
 *      Author: Luca Bascetta
 */

#include <Arduino.h>

#ifndef ENCODER_H_
#define ENCODER_H_

void init_encoderIO();
void init_encoderInterrupts();

unsigned int get_wheelspeed_dx_ms();
unsigned int get_wheelspeed_sx_ms();
byte is_wheel_dx_ccw();
byte is_wheel_sx_ccw();

#endif /* ENCODER_H_ */
