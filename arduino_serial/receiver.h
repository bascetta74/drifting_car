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

unsigned int get_steer_value_us();
unsigned int get_speed_value_us();
unsigned int get_button_value_us();
byte get_steer_state_us();
byte get_speed_state_us();
byte get_button_state_us();
byte is_receiver_button_pressed();

#endif /* RECEIVER_H_ */
