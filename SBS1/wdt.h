#ifndef WDT_H

#include <avr/wdt.h>
#include <avr/io.h>
#include <avr/interrupt.h>

extern volatile int long count16ms;
#define millis() (count16ms<<4)

void wdt_init();
#endif