#ifndef __smc_h
#define __smc_h

#include <SoftwareSerial.h>

class SimpleMotorController {
  SoftwareSerial serial;

 public:
  SimpleMotorController(int pin);
  void initialize(void);
  void setPWM(int pwm);
  void setBrake(unsigned char brake);
  void exitSafeStart(void);
};

#endif
