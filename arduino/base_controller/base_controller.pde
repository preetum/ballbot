// Hey, emacs: -*- mode: c++; indent-tabs-mode: nil -*-
/*
 * base_controller.pde
 *
 * AUTHOR: John Wang
 * VERSION: 0.3  (2 Aug 2011)
 *
 * DESCRIPTION:
 * Main entry point for application. Contains routines for processing
 * serial commands.
 *
 * NOTES:
 * We were pushing the limits of the AVR's 2kB RAM with rosserial, so
 * we switched back to simple serial packets.
 * If weird behavior results, then we've probably exceeded the RAM limits.
 * Note that string constants are also stored in RAM (more details at
 * http://www.avrfreaks.net/index.php?name=PNphpBB2&file=viewtopic&t=38003)
 */

#include <Servo.h>

#include "base_controller.h"
#include "encoder.h"
#include "feedback.h"
#include "imu.h"
#include "pickup.h"

// Globals
Servo steering, panServo, tiltServo;
SimpleMotorController driveMotor(5);
IMU imu(Serial1);

Packet packet(Serial3);

bool watchdogFlag = false;
bool sendOdometry = false;
//unsigned long last_setvelocity = 0;

void setSteering(int angle) {
  angle = -angle + SERVO_CENTER;
  if (angle > SERVO_LEFT)
    angle = SERVO_LEFT;
  else if (angle < SERVO_RIGHT)
    angle = SERVO_RIGHT;
  steering.write((unsigned char)angle);
}

/* Note: this reuses the global packet object and is not thread-safe!
 * Only call in the same thread as packet.receive()
 */
void writeOdometry(void) {
  static long lastEncoderCount = 0L;
    
  // Compute change in encoder count (discrete velocity estimate)
  long tmpEncoderCount = encoder_getCount();
  long delta = tmpEncoderCount - lastEncoderCount;
  unsigned long time = millis();
  lastEncoderCount = tmpEncoderCount;

  packet.length = 15;
  packet.data[0] = CMD_SYNC_ODOMETRY;
  reverse_memcpy(packet.data+1, &tmpEncoderCount, 4);
  reverse_memcpy(packet.data+5, &delta, 4);
  reverse_memcpy(packet.data+9, &imu.yaw, 2);
  reverse_memcpy(packet.data+11, &time, 4);
  packet.send();
}

void packetReceived (void) {
  switch (packet.data[0]) {
  case CMD_SET_RAW: {
    unsigned char steerVal = packet.data[1];
    unsigned int motorVal = packet.data[2] << 8 | packet.data[3];
    steering.write(steerVal);
    driveMotor.setPWM(motorVal);
    break;
  }

  case CMD_SET_VELOCITY: {
    int linear = packet.data[1] << 8 | packet.data[2];
    int angular = packet.data[3] << 8 | packet.data[4];
    feedback_setVelocity(linear);
    setSteering(angular);
    // TODO steering PID

    // Clear watchdog (timeout) flag
    watchdogFlag = false;
    break;
  }

  case CMD_SET_PICKUP: {
    signed char value = packet.data[1];
    if (value == 0)
      pickup_stop();
    else if (value == 1) {
      pickup_forward();
    } else if (value == -1) {
      pickup_reverse();
    }
    break;
  }

  case CMD_SET_VELOCITY_PID: 
  case CMD_SET_STEERING_PID: {
    int kp = packet.data[1] << 8 | packet.data[2];
    int ki = packet.data[3] << 8 | packet.data[4];
    int kd = packet.data[5] << 8 | packet.data[6];

    if (packet.data[0] == CMD_SET_VELOCITY_PID)
      pidVelocity.SetTunings(kp/100.0, ki/100.0, kd/100.0);
    // TODO steering PID
    break;
  }
        
  case CMD_SET_ODOMETRY:
    sendOdometry = (packet.data[1] != false);
    break;

  case CMD_SET_PANTILT: {
    unsigned char pan = packet.data[1],
      tilt = packet.data[2];

    panServo.write(pan);
    tiltServo.write(tilt);
  }

  } // switch

  // Toggle the LED on a valid packet
  digitalWrite(13, 1-digitalRead(13));
}


void setup() {
#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
  Serial3.begin(115200);
#endif
  Serial.begin(115200);

  // Initialize servos
  steering.attach(4);
  steering.write(SERVO_CENTER);
  panServo.attach(6);
  panServo.write(95);
  tiltServo.attach(7);
  tiltServo.write(95);

  // Initialize the drive motor
  driveMotor.initialize();
  
  encoder_initialize();
  feedback_initialize();
  pickup_initialize();

  // Use external Vref (3.3V)
  analogReference(EXTERNAL);

  pinMode(13, OUTPUT); // enable LED pin
  //last_setvelocity = 0;
}

void loop()  {
  static unsigned long nextUpdate = 0L;
  static unsigned char feedbackCount = 0,
    outputCount = 0,
    timeoutCount = 0;

  // Process serial buffer
#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
  while (Serial3.available())
    // If full packet received, process the packet
    if (packet.receive(Serial3.read())) {
      packetReceived();
      break;
    }
#else
  while (Serial.available())
    // If full packet received, process the packet
    if (packet.receive(Serial.read())) {
      packetReceived();
      break;
    }
#endif

  imu.update();

  unsigned long time = millis();

  if (time > nextUpdate) {
    nextUpdate += 20;  // Run this outer loop every 20 ms

    // Run feedback loop every 100ms
    if (feedbackCount == 0) {
      feedbackCount = 4;
      feedback_update();
    } else {
      feedbackCount -= 1;
    }

    // Write odometry every 40 ms
    if (sendOdometry) {
      if (outputCount == 0) {
        outputCount = 1;
        writeOdometry();
      } else {
        outputCount -= 1;
      }
    }

    // Check the timeout flag every 500 ms
    if (timeoutCount == 0) {
      timeoutCount = 24;
      // If flag hasn't been cleared in last 500 ms, stop the motors
      if (watchdogFlag) {
        feedback_setVelocity(0);
      }
      // Reset the flag
      watchdogFlag = true;
    } else {
      timeoutCount -= 1;
    }
  }
}
