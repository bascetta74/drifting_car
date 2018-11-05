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
unsigned int loop_lost_cnt;


void init_taskTimer(int frequency)
{
	// Switch interrupt off
	cli();

	// Make sure TCCR1A is zero (after Arduino set it in init function)
	TCCR1A = 0;

	// Initialize counter value to 0
	TCNT1  = 0;

	// Set compare match register for frequency Hz increments
	OCR1A = 16000000 / (frequency*64) - 1;

	// Turn on CTC mode
	TCCR1B = (1 << WGM12);

	// Set for 64 prescaler
	TCCR1B |= (1 << CS11) | (1 << CS10);

	// Enable timer compare interrupt
	TIMSK1 = (1 << OCIE1A);

	// Switch interrupt on again
	sei();

	// Initialize start variable
	can_start = false;

	// Reset loop lost counter
	loop_lost_cnt = 0;
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

unsigned int get_lost_loops()
{
	return loop_lost_cnt;
}

void reset_lost_loops_counter()
{
	loop_lost_cnt = 0;
}


// Internal functions
ISR(TIMER1_COMPA_vect)
{
	if (!can_start) {
		// If the loop section has been executed set can_start to true and reset the loop lost counter
		can_start = true;
	}
	else {
		// If the loop section has not been executed increase the loop lost counter
		loop_lost_cnt++;
	}
}
