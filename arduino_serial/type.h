/*
 * type.h
 *
 *  Created on: Aug 24, 2017
 *      Author: Luca Bascetta
 */

#ifndef TYPE_H_
#define TYPE_H_

// State machine states and state_info structure
typedef enum States { SAFE=0, MANUAL=1, AUTOMATIC=2, HALT=3 } State;
typedef struct
{
	State state;
	unsigned int info;
} state_info;

#endif /* TYPE_H_ */
