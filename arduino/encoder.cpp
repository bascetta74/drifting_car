/*
 * encoder.cpp
 *
 *  Created on: Aug 10, 2017
 *      Author: Luca Bascetta
 */

#include "receiver.h"
#include "constant.h"

#include <avr/interrupt.h>


typedef struct
{
	byte state;						// Channel state (high/low)
	unsigned int pulse_duration;	// Duration in ms of last pulse
	unsigned long edge_time;		// Clock time of last rising/falling edge
} encoder_channel;


// Internal variables
volatile encoder_channel encoder_sx_A, encoder_sx_B, encoder_dx_A, encoder_dx_B;


void init_encoderIO()
{
	// Set pin 8, 9, 10, 11 as input
	pinMode(ENC_FRWHEEL_A_PIN, INPUT);
	pinMode(ENC_FRWHEEL_B_PIN, INPUT);
	pinMode(ENC_FLWHEEL_A_PIN, INPUT);
	pinMode(ENC_FLWHEEL_B_PIN, INPUT);

	// Initialize encoder_channel variables
	encoder_sx_A.state = encoder_sx_B.state = encoder_dx_A.state = encoder_dx_B.state = 0;
	encoder_sx_A.pulse_duration = encoder_sx_B.pulse_duration = encoder_dx_A.pulse_duration = encoder_dx_B.pulse_duration = 0;
	encoder_sx_A.edge_time = encoder_sx_B.edge_time = encoder_dx_A.edge_time = encoder_dx_B.edge_time = 0;
}

void init_encoderInterrupts()
{
	// Switch interrupt off
	cli();

	// Enable PCMSK0
	PCICR |= (1<<PCIE0);

	// Front right wheel encoder -> 8, 9
	PCMSK0 = (1<<PB0);
	PCMSK0 |= (1<<PB1);

	// Front left wheel encoder -> 10, 11
	PCMSK0 |= (1<<PB2);
	PCMSK0 |= (1<<PB3);

	// Switch interrupt on again
	sei();
}

unsigned int get_wheelspeed_dx_ms()
{
	// TBD
	return 0;
}

unsigned int get_wheelspeed_sx_ms()
{
	// TBD
	return 0;
}

byte is_wheel_dx_ccw()
{
	// TBD
	return 0x01;
}

byte is_wheel_sx_ccw()
{
	// TBD
	return 0x01;
}


// Internal functions
ISR (PCINT0_vect)
{
	// write ISR routine
}
