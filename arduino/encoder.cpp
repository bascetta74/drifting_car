/*
 * encoder.cpp
 *
 *  Created on: Aug 10, 2017
 *      Author: Luca Bascetta
 */

#include "receiver.h"
#include "constant.h"

#include <avr/interrupt.h>
#include <avr/wdt.h>


typedef struct
{
	byte pulse_state_A;				// Channel A state (high/low)
	byte pulse_state_B;				// Channel A state (high/low)
	unsigned long edge_time;		// Clock time of last rising/falling edge
	unsigned int delta_t;			// Distance in ms between last two edges
} encoder_channel;


// Internal variables
unsigned long encoder_current_time;
volatile encoder_channel encoder_sx, encoder_dx;


void init_encoderIO()
{
	// Set pin 8, 9, 10, 11 as input
	pinMode(ENC_FRWHEEL_A_PIN, INPUT);
	pinMode(ENC_FRWHEEL_B_PIN, INPUT);
	pinMode(ENC_FLWHEEL_A_PIN, INPUT);
	pinMode(ENC_FLWHEEL_B_PIN, INPUT);

	// Initialize encoder_channel variables
	encoder_sx.pulse_state_A = encoder_sx.pulse_state_B = encoder_dx.pulse_state_A = encoder_dx.pulse_state_B = 0;
	encoder_sx.edge_time = encoder_dx.edge_time = 0;
	encoder_sx.delta_t = encoder_dx.delta_t = 0;
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

void init_encoderWDT()
{
	// Switch interrupt off
	cli();

    // Reset status register flags
	MCUSR = 0;

	// Put timer in interrupt-only mode:
	// Set WDCE (5th from left) and WDE (4th from left) to enter config mode,
	// using bitwise OR assignment (leaves other bits unchanged)
	WDTCSR |= 0b00011000;

	// Set WDIE: interrupt enabled
	// Clr WDE: reset disabled
	// and set delay interval (right side of bar) to 16 milliseconds
	WDTCSR =  0b01000000 | 0b000000;

	// Switch interrupt on again
	sei();

	// reminder of the definitions for the time before firing
	// delay interval patterns:
	//  16 ms:     0b000000
	//  500 ms:    0b000101
	//  1 second:  0b000110
	//  2 seconds: 0b000111
	//  4 seconds: 0b100000
	//  8 seconds: 0b100001
}

unsigned int get_wheelspeed_dx_us()
{
	return encoder_dx.delta_t;
}

unsigned int get_wheelspeed_sx_us()
{
	return encoder_sx.delta_t;
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
	// Get current time
	encoder_current_time = micros();

	// Reset the watchdog
	wdt_reset();

	/////////////////////////////////////////////////////////////////////////////////////////////////////
	// Right encoder (8, 9)                                                                            //
	/////////////////////////////////////////////////////////////////////////////////////////////////////
	// Is input 8 HIGH?
	if (PINB & B00000001) {
		// Input 8 changed from LOW to HIGH
		if (encoder_dx.pulse_state_A == 0) {
			// Update channel state
			encoder_dx.pulse_state_A = 1;
			encoder_dx.delta_t = encoder_current_time-encoder_dx.edge_time;
			encoder_dx.edge_time = encoder_current_time;
		}
	}
	// Input 8 changed from HIGH to LOW
	else if (encoder_dx.pulse_state_A == 1) {
		// Update channel state
		encoder_dx.pulse_state_A = 0;
	}

	// Is input 9 HIGH?
	if (PINB & B00000010) {
		// Input 9 changed from LOW to HIGH
		if (encoder_dx.pulse_state_B == 0) {
			// Update channel state
			encoder_dx.pulse_state_B = 1;
			encoder_dx.delta_t = encoder_current_time-encoder_dx.edge_time;
			encoder_dx.edge_time = encoder_current_time;
		}
	}
	// Input 9 changed from HIGH to LOW
	else if (encoder_dx.pulse_state_B == 1) {
		// Update channel state
		encoder_dx.pulse_state_B = 0;
	}

	/////////////////////////////////////////////////////////////////////////////////////////////////////
	// Left encoder (10, 11)                                                                            //
	/////////////////////////////////////////////////////////////////////////////////////////////////////
	// Is input 10 HIGH?
	if (PINB & B00000100) {
		// Input 10 changed from LOW to HIGH
		if (encoder_sx.pulse_state_A == 0) {
			// Update channel state
			encoder_sx.pulse_state_A = 1;
			encoder_sx.delta_t = encoder_current_time-encoder_sx.edge_time;
			encoder_sx.edge_time = encoder_current_time;
		}
	}
	// Input 10 changed from HIGH to LOW
	else if (encoder_sx.pulse_state_A == 1) {
		// Update channel state
		encoder_sx.pulse_state_A = 0;
	}

	// Is input 11 HIGH?
	if (PINB & B00001000) {
		// Input 11 changed from LOW to HIGH
		if (encoder_sx.pulse_state_B == 0) {
			// Update channel state
			encoder_sx.pulse_state_B = 1;
			encoder_sx.delta_t = encoder_current_time-encoder_sx.edge_time;
			encoder_sx.edge_time = encoder_current_time;
		}
	}
	// Input 11 changed from HIGH to LOW
	else if (encoder_sx.pulse_state_B == 1) {
		// Update channel state
		encoder_sx.pulse_state_B = 0;
	}
}

/*
ISR (WDT_vect)
{
	// Set wheel velocity to zero
	encoder_sx.delta_t = encoder_dx.delta_t = 0;

	// Reset the watchdog
	wdt_reset();
}
*/
