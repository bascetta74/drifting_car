/*
 * telemetry.h
 *
 *  Created on: Aug 11, 2017
 *      Author: Luca Bascetta
 */

#include <Arduino.h>
#include "constant.h"


#ifndef TELEMETRY_H_
#define TELEMETRY_H_

typedef struct
{
	unsigned int steer_cmd;
	unsigned int speed_cmd;
	unsigned int wheel_dx_speed;
	bool wheel_dx_ccw;
	unsigned int wheel_sx_speed;
	bool wheel_sx_ccw;
	unsigned char arduino_state;
	unsigned char arduino_state_info;
	unsigned long arduino_time;
} telemetry_message;

void encode_odroid_message(char encoded_message[MESSAGE_SIZE], const telemetry_message* message);
unsigned int decode_odroid_message(const char message[MESSAGE_SIZE], telemetry_message* decoded_message);

void initialize_odroid_message(telemetry_message* message);
void set_odroid_message(telemetry_message* message, unsigned int steer_cmd, unsigned int speed_cmd,
		unsigned int wheel_dx_speed, bool wheel_dx_ccw, unsigned int wheel_sx_speed, bool wheel_sx_ccw,
		unsigned char arduino_state, unsigned char arduino_state_info, unsigned long arduino_time);
void get_odroid_message(const telemetry_message* message, unsigned int* steer_cmd, unsigned int* speed_cmd,
		unsigned int* wheel_dx_speed, bool* wheel_dx_ccw, unsigned int* wheel_sx_speed, bool* wheel_sx_ccw,
		unsigned char* arduino_state, unsigned char* arduino_state_info, unsigned long* arduino_time);

#endif /* TELEMETRY_H_ */
