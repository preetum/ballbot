#ifndef __feedback_h
#define __feedback_h

#include <PID_v1.h>

void feedback_initialize(void);
void feedback_update(void);
void feedback_setVelocity(int vel);

extern PID pidVelocity;

// For debug only
extern double velocityInput,
  velocityOutput,
  velocitySetpoint;

#endif
