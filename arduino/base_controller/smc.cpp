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
void serialWrite(SoftwareSerial s, unsigned char *buf, unsigned int len) {
  while (--len >= 0)
    s.print(*buf++);
}

SimpleMotorController::SimpleMotorController(int txPin)
  : serial(13, txPin) {
  pinMode(txPin, OUTPUT);
  serial.begin(9600);
}

void SimpleMotorController::initialize(void) {
  // Simple Motor Controller must be running for at least 1ms
  // before receiving serial data
  delay(2);

  // Send baud rate detection byte
  serial.print((char)0xAA);

  exitSafeStart();
}

void SimpleMotorController::exitSafeStart(void) {
  serial.print((char)0x83);
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
