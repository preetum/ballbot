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

// Globals
Servo steering;
SimpleMotorController driveMotor(5);
IMU imu(7, 10);

Packet packet;

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
  lastEncoderCount = tmpEncoderCount;

  packet.length = 13;
  packet.data[0] = CMD_GET_ODOMETRY;
  memcpy(packet.data+1, &tmpEncoderCount, 4);
  memcpy(packet.data+5, &delta, 4);
  memset(packet.data+9, 0, 2); // TODO angle
  memset(packet.data+11, 0, 2);
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
    // TODO
    break;
  }

  case CMD_SET_PICKUP: {
    signed char value = packet.data[1];
    // TODO
    break;
  }

  case CMD_SET_PID: {
    int kp = packet.data[1] << 8 | packet.data[2];
    int ki = packet.data[3] << 8 | packet.data[4];
    int kd = packet.data[5] << 8 | packet.data[6];

    pidVelocity.SetTunings(kp/100.0, ki/100.0, kd/100.0);
  }
        
  case CMD_GET_ODOMETRY:
    writeOdometry();
    break;

  } // switch

  // Toggle the LED on a valid packet
  digitalWrite(13, 1-digitalRead(13));
}


void setup() {
  Serial.begin(115200);

  // Initialize servos
  steering.attach(4);
  steering.write(SERVO_CENTER);

  // Initialize the drive motor
  driveMotor.initialize();
  
  encoder_initialize();
  feedback_initialize();

  // Use external Vref (3.3V)
  analogReference(EXTERNAL);

  pinMode(13, OUTPUT); // enable LED pin
}

void loop()  {
  static unsigned long nextUpdate = 0L;

  // Process serial buffer
  while (Serial.available())
    // If full packet received, process the packet
    if (packet.receive(Serial.read())) {
      packetReceived();
      break;
    }

  // Run feedback loop every 100ms
  if (millis() > nextUpdate) {
    nextUpdate += 100;
    feedback_update();
    //imu.update();

    // Debug
    /*
    Serial.print((int)velocityInput);
    Serial.print('\t');
    Serial.print((int)velocitySetpoint);
    Serial.println();
    */
  }
}
