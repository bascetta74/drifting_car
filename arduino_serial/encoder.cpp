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
	int count;			    // Encoder position in counts
	bool ccw;	            // True if the encoder is turning counterclockwise, false otherwise
	bool reset_counter;		// True if the counter has to be set to zero at next interrupt, false otherwise
} encoder_channel;


// Internal variables
volatile encoder_channel encoder_sx, encoder_dx;


void init_encoderIO()
{
	// Set pin 8, 9, 10, 11 as input
	pinMode(ENC_FRWHEEL_A_PIN, INPUT);
	pinMode(ENC_FRWHEEL_B_PIN, INPUT);
	pinMode(ENC_FLWHEEL_A_PIN, INPUT);
	pinMode(ENC_FLWHEEL_B_PIN, INPUT);

	// Initialize encoder_channel variables
	encoder_sx.count = encoder_dx.count = 0;
	encoder_sx.ccw = encoder_dx.ccw = false;
	encoder_sx.reset_counter = encoder_dx.reset_counter = false;
}

void init_encoderInterrupts()
{
	// Switch interrupt off
	cli();

	// Enable PCMSK0
	PCICR |= (1<<PCIE0);

	// Front right wheel encoder -> 8, 9 (set interrupt only on channel A)
	PCMSK0 = (1<<PB0);
	//PCMSK0 |= (1<<PB1);

	// Front left wheel encoder -> 10, 11 (set interrupt only on channel A)
	PCMSK0 |= (1<<PB2);
	//PCMSK0 |= (1<<PB3);

	// Switch interrupt on again
	sei();
}

unsigned int get_wheelcount_dx()
{
	encoder_dx.reset_counter = true;

	return encoder_dx.count;
}

unsigned int get_wheelcount_sx()
{
	encoder_sx.reset_counter = true;

	return encoder_sx.count;
}


// Internal functions
ISR (PCINT0_vect)
{

	/////////////////////////////////////////////////////////////////////////////////////////////////////
	// Resetting counters                                                                              //
	/////////////////////////////////////////////////////////////////////////////////////////////////////
	if (encoder_sx.reset_counter) {
		encoder_sx.count = 0;
		encoder_sx.reset_counter = false;
	}

	if (encoder_dx.reset_counter) {
		encoder_dx.count = 0;
		encoder_dx.reset_counter = false;
	}

	/////////////////////////////////////////////////////////////////////////////////////////////////////
	// Right encoder (8, 9)                                                                            //
	/////////////////////////////////////////////////////////////////////////////////////////////////////
	// Is input 8 HIGH (channel A)?
	if (PINB & B00000001) {			// Input 8 changed from LOW to HIGH
		// Is input 9 HIGH (channel B)?
		if (PINB & B00000010) {
			encoder_sx.ccw = false;
		} else {
			encoder_sx.ccw = true;
		}
	} else {						// Input 8 changed from HIGH to LOW
		// Do nothing
	}

	if (encoder_sx.ccw) {
		encoder_sx.count = encoder_sx.count-1;
	} else {
		encoder_sx.count = encoder_sx.count+1;
	}
	/////////////////////////////////////////////////////////////////////////////////////////////////////
	// Left encoder (10, 11)                                                                            //
	/////////////////////////////////////////////////////////////////////////////////////////////////////
	// Is input 10 HIGH (channel A)?
	if (PINB & B00000100) {			// Input 10 changed from LOW to HIGH
		// Is input 11 HIGH (channel B)?
		if (PINB & B00001000) {
			encoder_dx.ccw = false;
		} else {
			encoder_dx.ccw = true;
		}
	} else {						// Input 10 changed from HIGH to LOW
		// Do nothing
	}

	if (encoder_dx.ccw) {
		encoder_dx.count = encoder_dx.count-1;
	} else {
		encoder_dx.count = encoder_dx.count+1;
	}
}
