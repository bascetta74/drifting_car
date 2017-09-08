/*
 * FIR_filter.h
 *
 *  Created on: Jul 5, 2017
 *      Author: Luca Bascetta
 */

#ifndef FIR_FILTER_H_
#define FIR_FILTER_H_


#include "stdint.h"
#include "constant.h"


class FIRfilter
{
	uint8_t order;
	uint8_t scaling;
	int state[MAX_FIR_ORDER];
	int coefficient[MAX_FIR_ORDER+1];

public:
	FIRfilter(uint8_t order, int coefficient[], uint8_t scaling);

	bool evaluate(int u, int& y);
};


#endif /* FIR_FILTER_H_ */
