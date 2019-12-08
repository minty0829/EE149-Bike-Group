/*
 * states.h
 *
 *  Created on: Sep 22, 2018
 *      Author: shromonaghosh
 */

#ifndef STATES_H_
#define STATES_H_

#include <stdio.h>

typedef enum {
    OFF=0,
    ACTIVE,
    LEFT,
    RIGHT
} states;

typedef enum {
	BRAKE, 
	NOT_BRAKE
} braking_states;

#endif /* STATES_H_ */
