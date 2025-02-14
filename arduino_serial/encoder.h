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

void set_encoderConfiguration(int fl_config, int fr_config);

int get_wheelcount_dx();
int get_wheelcount_sx();

#endif /* ENCODER_H_ */
