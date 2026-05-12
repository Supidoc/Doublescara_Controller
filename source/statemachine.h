/*
 * statemachine.h
 *
 *  Created on: 21.04.2026
 *      Author: BSc
 */

#ifndef STATEMACHINE_H_
#define STATEMACHINE_H_

typedef enum {
    IDLE,
	EXECUTE,
	DONE,
	ERROR
} State;

void Statemachine_task(void *pvParameters);



#endif /* STATEMACHINE_H_ */
