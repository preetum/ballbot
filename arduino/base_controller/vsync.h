#ifndef __vsync_h
#define __vsync_h

#include "TimerOne.h"

// Pins for TimerOne must be 9 or 10
// If we can't use these, 
#define VSYNC_PIN 9
#define NOP asm("nop")
void vsync_initialize(void);
void vsync_pulse(void);

#endif
