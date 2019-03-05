/*
 * receiver.cpp
 *
 *  Created on: Aug 9, 2017
 *      Author: Luca Bascetta
 */

#include "receiver.h"
#include "constant.h"

#include <avr/interrupt.h>


typedef struct
{
	byte pulse_state;				// Channel state (high/low)
	unsigned int pulse_duration;	// Duration in ms of last pulse
	unsigned long edge_time;		// Clock time of last rising/falling edge
} receiver_command_channel;

typedef struct
{
	byte pulse_state;				// Channel state (high/low)
	unsigned int pulse_duration;	// Duration in ms of last pulse
	unsigned long edge_time;		// Clock time of last rising/falling edge
	byte button_state;				// Button state (for channels that handle a radio button)
} receiver_button_channel;


// Internal variables
unsigned long receiver_current_time;
volatile receiver_command_channel receiver_steer, receiver_speed;
volatile receiver_button_channel receiver_button;


void init_receiverIO()
{
	// Set pin A0, A1, A2 as inputs
	pinMode(CH1_RECEIVER_PIN, INPUT);
	pinMode(CH2_RECEIVER_PIN, INPUT);
	pinMode(CH3_RECEIVER_PIN, INPUT);

	// Initialize receiver_channel variables
	receiver_steer.pulse_state = receiver_speed.pulse_state = receiver_button.pulse_state = 0;
	receiver_steer.pulse_duration = receiver_speed.pulse_duration = receiver_button.pulse_duration = 0;
	receiver_steer.edge_time = receiver_speed.edge_time = receiver_button.edge_time = 0;
	receiver_button.button_state = 0;
}

void init_receiverInterrupts()
{
	// Switch interrupt off
	cli();

	// Enable PCMSK1
	PCICR |= (1<<PCIE1);

	// CH1 -> A0
	PCMSK1 = (1<<PC0);

	// CH2 -> A1
	PCMSK1 |= (1<<PC1);

	// CH3 -> A2
	PCMSK1 |= (1<<PC2);

	// Switch interrupt on again
	sei();
}

unsigned int get_steer_value_us()
{
	return receiver_steer.pulse_duration;
}

unsigned int get_speed_value_us()
{
	return receiver_speed.pulse_duration;
}

unsigned int get_button_value_us()
{
	return receiver_button.pulse_duration;
}

byte get_steer_state_us()
{
	return receiver_steer.pulse_state;
}

byte get_speed_state_us()
{
	return receiver_speed.pulse_state;
}

byte get_button_state_us()
{
	return receiver_button.pulse_state;
}

byte is_receiver_button_pressed()
{
	if (receiver_button.button_state)
	{
		receiver_button.button_state = 0;
		return 1;
	}
	else
		return 0;
}

// Internal functions
ISR (PCINT1_vect)
{
	// Get current time
	receiver_current_time = micros();

	/////////////////////////////////////////////////////////////////////////////////////////////////////
	// Receiver CH1 - steer                                                                            //
	/////////////////////////////////////////////////////////////////////////////////////////////////////
	// Is input A0 HIGH?
	if (PINC & B00000001) {
		// Input A0 changed from LOW to HIGH
		if (receiver_steer.pulse_state == 0) {
			// Update channel state
			receiver_steer.pulse_state = 1;
			receiver_steer.edge_time = receiver_current_time;
		}
	}
	// Input A0 changed from HIGH to LOW
	else if (receiver_steer.pulse_state == 1) {
		// Update channel state
		receiver_steer.pulse_state = 0;
		receiver_steer.pulse_duration = receiver_current_time-receiver_steer.edge_time;
	}

	/////////////////////////////////////////////////////////////////////////////////////////////////////
	// Receiver CH2 - speed                                                                            //
	/////////////////////////////////////////////////////////////////////////////////////////////////////
	// Is input A1 HIGH?
	if (PINC & B00000010) {
		// Input A1 changed from LOW to HIGH
		if (receiver_speed.pulse_state == 0) {
			// Update channel state
			receiver_speed.pulse_state = 1;
			receiver_speed.edge_time = receiver_current_time;
		}
	}
	// Input A1 changed from HIGH to LOW
	else if (receiver_speed.pulse_state == 1) {
		// Update channel state
		receiver_speed.pulse_state = 0;
		receiver_speed.pulse_duration = receiver_current_time-receiver_speed.edge_time;
	}

	/////////////////////////////////////////////////////////////////////////////////////////////////////
	// Receiver CH3 - button                                                                            //
	/////////////////////////////////////////////////////////////////////////////////////////////////////
	// Is input A2 HIGH?
	if (PINC & B00000100) {
		// Input A2 changed from LOW to HIGH
		if (receiver_button.pulse_state == 0) {
			// Update channel state
			receiver_button.pulse_state = 1;
			receiver_button.edge_time = receiver_current_time;
		}
	}
	// Input A2 changed from HIGH to LOW
	else if (receiver_button.pulse_state == 1) {
		// Update channel state
		receiver_button.pulse_state = 0;

		// Update button state
		if (receiver_button.pulse_duration < BUTTON_DEAD_ZONE)	// Introduce to handle initialization
			receiver_button.button_state = 0;
		else if ((receiver_button.pulse_duration>(receiver_current_time-receiver_button.edge_time+BUTTON_DEAD_ZONE)) ||
				(receiver_button.pulse_duration<(receiver_current_time-receiver_button.edge_time-BUTTON_DEAD_ZONE)))
			receiver_button.button_state = 1;

		receiver_button.pulse_duration = receiver_current_time-receiver_button.edge_time;
	}
}
