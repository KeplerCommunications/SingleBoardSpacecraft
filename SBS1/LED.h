/*
	Author: Keenan Burnett
	(Originally taken from Digi-Key Design Support)

	***********************************************************************
	*	FILE NAME:		LED.h
	*
	*	PURPOSE:	This file contains constant definitions and prototypes for LED.c
	*
	*	FILE REFERENCES:	None.
	*
	*	EXTERNAL VARIABLES:	
	*
	*	EXTERNAL REFERENCES:	Same a File References.
	*
	*	ABORNOMAL TERMINATION CONDITIONS, ERROR AND WARNING MESSAGES: None yet.
	*
	*	ASSUMPTIONS, CONSTRAINTS, CONDITIONS:	None
	*
	*	NOTES:	
	*
	*	REQUIREMENTS/ FUNCTIONAL SPECIFICATION REFERENCES:
	*	None so far.
	*
	*	DEVELOPMENT HISTORY:
	*	01/02/2015		Created.
	*
	*	02/06/2015		Edited the header.
	*
*/
#ifndef LED_H
#define LED_H

/* LED Pin Mapping 
 * 
 *   LED1 PORTB6
 *   LED2 PORTD0
 */
#define LED1 1
#define LED2 2

void led_set(uint8_t LED);
void led_clr(uint8_t LED);
void led_toggle(uint8_t LED);
void led_init();

#endif /* LED_H_ */

