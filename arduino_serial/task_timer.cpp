/*
 * task_timer.cpp
 *
 *  Created on: Aug 11, 2017
 *      Author: Luca Bascetta
 */

#include "task_timer.h"

#include <avr/interrupt.h>


// Internal variables
volatile bool can_start;

void init_taskTimer(int frequency)
{
	// Switch interrupt off
	cli();

	// Set entire TCCR2A and TCCR2B registers to 0
	TCCR2A = 0;
	TCCR2B = 0;

	// Initialize counter value to 0
	TCNT2  = 0;

	// Set compare match register for 1 kHz increments
	OCR2A = (16*10^6) / (frequency*64);

	// Turn on CTC mode
	TCCR2A |= (1 << WGM21);

	// Set CS22 for 64 prescaler (maximum value)
	TCCR2B |= (1 << CS22);

	// Enable timer compare interrupt
	TIMSK2 |= (1 << OCIE2A);

	// Switch interrupt on again
	sei();

	// Initialize start variable
	can_start = false;
}

bool canStart()
{
	if (can_start)
	{
		can_start = false;
		return true;
	}

	return false;
}


// Internal functions
ISR(TIMER2_COMPA_vect)
{
	can_start = true;
}
