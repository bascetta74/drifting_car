/*
 * task_timer.h
 *
 *  Created on: Aug 11, 2017
 *      Author: Luca Bascetta
 */

#ifndef TASK_TIMER_H_
#define TASK_TIMER_H_

void init_taskTimer(int frequency);	// Tested for frequencies between 10 and 500 Hz

bool canStart();
unsigned int get_lost_loops();
void reset_lost_loops_counter();

#endif /* TASK_TIMER_H_ */
