/*
 * encoder.cpp
 *
 *  Created on: Aug 10, 2017
 *      Author: Luca Bascetta
 */

#include "encoder.h"
#include "constant.h"

#include <avr/interrupt.h>


typedef struct
{
	int count;			    // Encoder position in counts
	bool ccw;	            // True if the encoder is turning counterclockwise, false otherwise
	bool reset_counter;		// True if the counter has to be set to zero at next interrupt, false otherwise
} encoder_channel;


// Internal variables
volatile encoder_channel encoder_fl, encoder_fr;
bool enc_frwheel_A_state, enc_frwheel_B_state,
	 enc_flwheel_A_state, enc_flwheel_B_state;
int enc_frwheel_conf, enc_flwheel_conf;

void init_encoderIO()
{
	// Set pin 8, 9, 10, 11 as input
	pinMode(ENC_FRWHEEL_A_PIN, INPUT);
	pinMode(ENC_FRWHEEL_B_PIN, INPUT);
	pinMode(ENC_FLWHEEL_A_PIN, INPUT);
	pinMode(ENC_FLWHEEL_B_PIN, INPUT);

	// Initialize encoder_channel variables
	encoder_fl.count = encoder_fr.count = 0;
	encoder_fl.ccw = encoder_fr.ccw = false;
	encoder_fl.reset_counter = encoder_fr.reset_counter = false;

	enc_frwheel_A_state = enc_frwheel_B_state = enc_flwheel_A_state = enc_flwheel_B_state = false;

	enc_frwheel_conf = enc_flwheel_conf = ENC_A_RISING;
}

void init_encoderInterrupts()
{
	// Switch interrupt off
	cli();

	// Enable PCMSK0 (detect any change on PCINT 0-7)
	PCICR |= (1<<PCIE0);

	// Front right wheel encoder -> 8, 9 (channels A & B)
	PCMSK0 = (1<<PB0);
	PCMSK0 |= (1<<PB1);

	// Front left wheel encoder -> 10, 11 (channels A & B)
	PCMSK0 |= (1<<PB2);
	PCMSK0 |= (1<<PB3);

	// Switch interrupt on again
	sei();
}

void set_encoderConfiguration(int fl_config, int fr_config)
{
	enc_frwheel_conf = fr_config;
	enc_flwheel_conf = fl_config;
}

int get_wheelcount_dx()
{
	encoder_fr.reset_counter = true;

	return encoder_fr.count;
}

int get_wheelcount_sx()
{
	encoder_fl.reset_counter = true;

	return encoder_fl.count;
}


// Internal functions
ISR (PCINT0_vect)
{	// One channel (A) interrupt service routine

	/////////////////////////////////////////////////////////////////////////////////////////////////////
	// Resetting counters                                                                              //
	/////////////////////////////////////////////////////////////////////////////////////////////////////
	if (encoder_fl.reset_counter) {
		encoder_fl.count = 0;
		encoder_fl.reset_counter = false;
	}

	if (encoder_fr.reset_counter) {
		encoder_fr.count = 0;
		encoder_fr.reset_counter = false;
	}

	/////////////////////////////////////////////////////////////////////////////////////////////////////
	// Front right encoder (8, 9)                                                                      //
	/////////////////////////////////////////////////////////////////////////////////////////////////////
	// Is input 8 HIGH (channel A)?
	if (PINB & B00000001) {			// Input 8 HIGH
		if (!enc_frwheel_A_state) {			// Input 8 changed from LOW to HIGH
			enc_frwheel_A_state = true;

			// Update counter
			encoder_fr.count++;
		}
		else {								// Input 8 stay HIGH
			enc_frwheel_A_state = true;
			// Do nothing
		}
	} else {						// Input 8 LOW
		if (!enc_frwheel_A_state) {			// Input 8 stay LOW
			enc_frwheel_A_state = false;
			// Do nothing
		}
		else {								// Input 8 changed from HIGH to LOW
			enc_frwheel_A_state = false;

			if (enc_frwheel_conf==ENC_A_BOTH)
				encoder_fr.count++;
		}
	}

	// Is input 9 HIGH (channel B)?
	if (PINB & B00000010) {			// Input 9 HIGH
		if (!enc_frwheel_B_state) {			// Input 9 changed from LOW to HIGH
			enc_frwheel_B_state = true;

			if ((enc_frwheel_conf==ENC_AB_RISING) || (enc_frwheel_conf==ENC_AB_BOTH))
				encoder_fr.count++;
		}
		else {								// Input 9 stay HIGH
			enc_frwheel_B_state = true;
			// Do nothing
		}
	} else {						// Input 9 LOW
		if (!enc_frwheel_B_state) {			// Input 9 stay LOW
			enc_frwheel_B_state = false;
			// Do nothing
		}
		else {								// Input 9 changed from HIGH to LOW
			enc_frwheel_B_state = false;

			if (enc_frwheel_conf==ENC_AB_BOTH)
				encoder_fr.count++;
		}
	}

	/////////////////////////////////////////////////////////////////////////////////////////////////////
	// Front left encoder (10, 11)                                                                     //
	/////////////////////////////////////////////////////////////////////////////////////////////////////
	// Is input 10 HIGH (channel A)?
	if (PINB & B00000100) {			// Input 10 HIGH
		if (!enc_flwheel_A_state) {			// Input 10 changed from LOW to HIGH
			enc_flwheel_A_state = true;

			// Update counter
			encoder_fl.count++;
		}
		else {								// Input 10 stay HIGH
			enc_flwheel_A_state = true;
			// Do nothing
		}
	} else {						// Input 10 LOW
		if (!enc_flwheel_A_state) {			// Input 10 stay LOW
			enc_flwheel_A_state = false;
			// Do nothing
		}
		else {								// Input 10 changed from HIGH to LOW
			enc_flwheel_A_state = false;

			if (enc_flwheel_conf==ENC_A_BOTH)
				encoder_fl.count++;
		}
	}

	// Is input 11 HIGH (channel B)?
	if (PINB & B00001000) {			// Input 11 HIGH
		if (!enc_flwheel_B_state) {			// Input 11 changed from LOW to HIGH
			enc_flwheel_B_state = true;

			if ((enc_flwheel_conf==ENC_AB_RISING) || (enc_flwheel_conf==ENC_AB_BOTH))
				encoder_fl.count++;
		}
		else {								// Input 11 stay HIGH
			enc_flwheel_B_state = true;
			// Do nothing
		}
	} else {						// Input 11 LOW
		if (!enc_flwheel_B_state) {			// Input 11 stay LOW
			enc_flwheel_B_state = false;
			// Do nothing
		}
		else {								// Input 11 changed from HIGH to LOW
			enc_flwheel_B_state = false;

			if (enc_flwheel_conf==ENC_AB_BOTH)
				encoder_fl.count++;
		}
	}
}


/* Previous version */
/*
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
*/
