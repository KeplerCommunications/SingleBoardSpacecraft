/*
	Author: Keenan Burnett
	(Originally taken from Digi-Key Design Support)

	***********************************************************************
	*	FILE NAME:		LED.c
	*
	*	PURPOSE:	This program contains the basic API for setting LEDs.
	*
	*	FILE REFERENCES:	io.h, LED.h
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
#include <avr/io.h>
#include "led.h"

void led_init(){
	DDRB |= (1<<6);
	DDRD |= 1;
	DDRC |= 1;
}

void led_clr(uint8_t LED) {
	switch(LED){
		case 1: PORTB &= ~(1<<PB6); break;
		case 2: PORTD &= ~(1<<PD0); break;
		case 3: PORTC &= ~(1<<PC0); break;
	}
}

void led_set(uint8_t LED) {
	switch(LED){
		case 1: PORTB |= 1<<PB6;  break;
		case 2: PORTD |= 1<<PD0;  break;
		case 3: PORTC |= 1<<PC0;  break;
	}
}

void led_toggle(uint8_t LED) {
	switch(LED){
		case 1: PORTB ^= 1<<PB6; break;
		case 2: PORTD ^= 1<<PD0; break;
		case 3: PORTC ^= 1<<PC0; break;
	}
}

