/*
 * receiver.h
 *
 *  Created on: Aug 9, 2017
 *      Author: Luca Bascetta
 */

#include <Arduino.h>

#ifndef RECEIVER_H_
#define RECEIVER_H_

void init_receiverIO();
void init_receiverInterrupts();

unsigned int get_steer_value_ms();
unsigned int get_speed_value_ms();
unsigned int get_button_value_ms();
byte get_steer_state_ms();
byte get_speed_state_ms();
byte get_button_state_ms();

#endif /* RECEIVER_H_ */
