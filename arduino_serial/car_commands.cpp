/*
 * car_commands.cpp
 *
 *  Created on: Aug 11, 2017
 *      Author: Luca Bascetta
 */

#include "car_commands.h"
#include "constant.h"

#include <Servo.h>
#include <Arduino.h>


// Internal variables
Servo steer, speed;


void init_carcommandsIO()
{
	// Set pin 2, 3 as steer, speed commands
	steer.attach(STEER_SERVO_PIN);
	speed.attach(SPEED_SERVO_PIN);

	// Set the initial servo values to neutral speed and steer
	steer.writeMicroseconds(STEER_ZERO);
	speed.writeMicroseconds(SPEED_ZERO);
}

void set_steer(unsigned int steer_cmd)
{
	// Saturate steer command and execute it
	steer.writeMicroseconds(constrain(steer_cmd, STEER_MIN, STEER_MAX));
}

void set_speed(unsigned int speed_cmd)
{
	// Saturate speed command and execute it
	speed.writeMicroseconds(constrain(speed_cmd, SPEED_MIN, SPEED_MAX));
}
