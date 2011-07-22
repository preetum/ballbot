// -*- mode: c++; indent-tabs-mode: nil -*-
/* 
 * smc.cpp
 *
 * AUTHOR: John Wang
 * VERSION: 0.1  (21 Jul 2011)
 *
 * DESCRIPTION:
 * Pololu Simple Motor Controller driver for Arduino using SoftwareSerial.
 */

#include <WProgram.h>

#include "smc.h"

/* Broken! Do not use! */
void serialWrite(SoftwareSerial &s, unsigned char *buf, unsigned int len) {
  while (--len >= 0)
    s.print(*buf++);
}

SimpleMotorController::SimpleMotorController(int txPin)
  : serial(13, txPin) {
  pinMode(txPin, OUTPUT);
  serial.begin(9600);
}

void SimpleMotorController::initialize(void) {
  // Send baud rate detection byte
  serial.print((unsigned char)0xAA);

  exitSafeStart();
}

void SimpleMotorController::exitSafeStart(void) {
  serial.print((unsigned char)0x83);
}

/* speed should be between -3200 and 3200 */
void SimpleMotorController::setSpeed(int speed) {
  unsigned char buffer[3];

  if (speed < 0) {  // reverse
    buffer[0] = 0x86;
    speed = -speed;
  } else {
    buffer[0] = 0x85;
  }
  buffer[1] = speed & 0x1F;
  buffer[2] = speed >> 5;
  serial.print(buffer[0]);
  serial.print(buffer[1]);
  serial.print(buffer[2]);
}

/* brake should be between 0 to 32, where 0 = coast and 32 = brake */
void SimpleMotorController::setBrake(unsigned char brake) {
  if (brake > 32)
    brake = 32;
  serial.print((unsigned char)0x92);
  serial.print(brake);
}
