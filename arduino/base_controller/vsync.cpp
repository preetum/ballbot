// Made by Steven Rhodes
// For syncing the Playstation Eye Cameras
// Using TimerOne.cpp

#include "WProgram.h"

#include "vsync.h"


void vsync_initialize(void){
  pinMode(VSYNC_PIN, OUTPUT);
  Timer1.initialize(33333);
  Timer1.pwm(VSYNC_PIN, 9, 33333); // 33.333ms, 9/1024 duty cycle is 292us
}

