#ifndef __pickup_h
#define __pickup_h

#include <wiring.h>

#define PICKUP_ENABLE_PIN    8
#define PICKUP_REVERSE_PIN   9

#define pickup_start()    digitalWrite(PICKUP_ENABLE_PIN, HIGH)
#define pickup_stop()     digitalWrite(PICKUP_ENABLE_PIN, LOW)

inline void pickup_initialize(void) {
  pinMode(PICKUP_ENABLE_PIN, OUTPUT);
  pinMode(PICKUP_REVERSE_PIN, OUTPUT);
}

inline void pickup_forward(void) {
  digitalWrite(PICKUP_REVERSE_PIN, LOW);
  pickup_start();
}

inline void pickup_reverse(void) {
  digitalWrite(PICKUP_REVERSE_PIN, HIGH);
  pickup_start();
}

#endif
