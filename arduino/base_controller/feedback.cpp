// -*- mode: c++; indent-tabs-mode: nil -*-
/* 
 * feedback.cpp
 *
 * AUTHOR: John Wang
 * VERSION: 0.1  (21 Jul 2011)
 *
 * DESCRIPTION:
 * PID feedback control for velocity control and steering.
 */

#include "base_controller.h"
#include "feedback.h"

int sampleRateHz = 10;

double velocityInput = 0,
  velocityOutput = 0,
  velocitySetpoint = 0;
PID pidVelocity(&velocityInput, &velocityOutput, &velocitySetpoint,
	     5, 0.0, 0.0, DIRECT);

void feedback_initialize(void) {
  pidVelocity.SetOutputLimits(-3200, 3200);
  pidVelocity.SetSampleTime(1000/sampleRateHz);
  pidVelocity.SetMode(AUTOMATIC);  // enable PID
}

void feedback_update(void) {
  double lastVelocityOutput = velocityOutput;
  pidVelocity.Compute();   // computes only if sample time is reached
  if (velocityOutput != lastVelocityOutput)  // save some SoftwareSerial time
    driveMotor.setPWM((int)velocityOutput);
}

/* Set the velocity to VEL cm/s. For practical reasons,
 *  velocity is limited to -300 <= vel <= 300.
 */
void feedback_setVelocity(double vel) {
  if (vel > 300) vel = 300;
  else if (vel < -300) vel = -300;

  // counts/s = velocity * 2 counts/cm / 10 Hz
  velocitySetpoint = vel * 2 / sampleRateHz;
}
