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

void serialWrite(SoftwareSerial &s, unsigned char *buf, unsigned int len) {
  while (len-- > 0)
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

/* Sets the motor PWM duty cycle and direction.
 *
 * pwm is a value from -3200 to 3200 
 */
void SimpleMotorController::setPWM(int pwm) {
  unsigned char buffer[3];

  if (pwm < 0) {   // reverse
    buffer[0] = 0x86;
    pwm = -pwm;
  } else {
    buffer[0] = 0x85;
  }

  if (pwm > 3200)  // bounds check
    pwm = 3200;

  buffer[1] = pwm & 0x1F;
  buffer[2] = pwm >> 5;

  cli();
  serialWrite(serial, buffer, 3);
  sei();
}

/* Sets the braking mode of the motor controller (either
 * coast or brake) during the "off" phase of PWM.
 *
 * brake is a value from 0-32, where 0 = coast and 32 = brake
 */
void SimpleMotorController::setBrake(unsigned char brake) {
  if (brake > 32)
    brake = 32;
  serial.print((unsigned char)0x92);
  serial.print(brake);
}
