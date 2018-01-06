/*
 * FIR_filter.cpp
 *
 *  Created on: Jul 5, 2017
 *      Author: Luca Bascetta
 */


#include "FIR_filter.h"

#include <math.h>
#include <string.h>


FIRfilter::FIRfilter(uint8_t order, int coefficient[], uint8_t scaling)
{
    // Check the consistency of order parameter
    if ((order>MAX_FIR_ORDER) || (order==0))
    {
        this->order = 0;
        return;
    }

    /* Set filter order */
	this->order = order;

    /* Set filter order */
	this->scaling = scaling;

	/* Set filter coefficients */
	memset(this->coefficient, 0, (MAX_FIR_ORDER+1)*sizeof(int));
	memcpy(this->coefficient, coefficient, (this->order+1)*sizeof(int));

	/* Reset filter state */
	memset(this->state, 0, MAX_FIR_ORDER*sizeof(int));
}

bool FIRfilter::evaluate(int u, int& y)
{
	/* Check that the filter has been correctly initialized */
	if (!(this->order>0))
		return false;
	
    /* Evaluate the filter output */
    y = (signed int) ((this->coefficient[0]*u + (1 << (this->scaling) >> 1)) >> this->scaling);
    for (uint8_t k=1; k<=this->order; k++)
        y += (signed int) ((this->coefficient[k]*this->state[k-1] + (1 << (this->scaling) >> 1)) >> this->scaling);
    
    /* Update the filter state */
    int tmp_state[this->order-1];
    memcpy(tmp_state, this->state, (this->order-1)*sizeof(int));
    memcpy(&(this->state[1]), tmp_state, (this->order-1)*sizeof(int));
    this->state[0] = u;

    return true;
}

