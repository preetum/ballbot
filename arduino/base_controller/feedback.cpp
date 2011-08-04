// -*- mode: c++; indent-tabs-mode: nil -*-
/* 
 * feedback.cpp
 *
 * AUTHOR: John Wang
 * VERSION: 0.1  (2 Aug 2011)
 *
 * DESCRIPTION:
 * PID feedback control for velocity control and steering.
 */

#include <WProgram.h>

#include "base_controller.h"
#include "encoder.h"
#include "feedback.h"

int sampleRateHz = 10;

double velocityInput = 0,
  velocityOutput = 0,
  velocitySetpoint = 0;
PID pidVelocity(&velocityInput, &velocityOutput, &velocitySetpoint,
	     40, 10, 0, DIRECT);

void feedback_initialize(void) {
  pidVelocity.SetOutputLimits(-3200, 3200);
  pidVelocity.SetSampleTime(1000/sampleRateHz);
  pidVelocity.SetMode(AUTOMATIC);  // enable PID
}

void feedback_update(void) {
  static long lastEncoderCount = 0L;

  // Compute change in encoder count (discrete velocity estimate)
  long tmpEncoderCount = encoder_getCount();
  velocityInput = (double)(tmpEncoderCount - lastEncoderCount);
  lastEncoderCount = tmpEncoderCount;

  double lastVelocityOutput = velocityOutput;
  pidVelocity.Compute();   // computes only if sample time is reached
  if (velocityOutput != lastVelocityOutput) {  // save some SoftwareSerial time
    driveMotor.setPWM((int)velocityOutput);
  }
}

/* Set the velocity to VEL cm/s. For practical reasons,
 *  velocity is limited to -300 <= vel <= 300.
 */
void feedback_setVelocity(int vel) {
  if (vel > 300) vel = 300;
  else if (vel < -300) vel = -300;

  // counts/s = velocity * 2 counts/cm / 10 Hz
  velocitySetpoint = vel * 2 / (double)sampleRateHz;
}
