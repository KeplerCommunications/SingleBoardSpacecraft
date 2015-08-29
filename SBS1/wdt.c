#include "wdt.h"
volatile int long count16ms = 0;


void wdt_init(){
	cli();
	count16ms = 0;
	//reset watchdog
	wdt_reset();
	//Start watchdog timer with 16ms
	WDTCSR = (1<<WDIE);
	sei();
}

//Watchdog timeout ISR
ISR(WDT_vect){
	count16ms++;
}
