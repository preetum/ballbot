// -*- mode: c++; indent-tabs-mode: nil -*-
/* 
 * imu.cpp
 *
 * AUTHOR: John Wang
 * VERSION: 0.1  (2 Aug 2011)
 *
 * DESCRIPTION:
 * Communicates with SparkFun Razor IMU via SoftwareSerial.
 *
 * Be cautious of calling update() too often since it is slow. This code is
 * quite whack. We may have to find a better way to interface the IMU.
 */

#include "imu.h"
#include "WProgram.h"

IMU::IMU(int rxPin, int txPin)
  : serial(rxPin, txPin) {
  pinMode(txPin, OUTPUT);
  pinMode(rxPin, INPUT);
  digitalWrite(rxPin, HIGH);
  serial.begin(9600);

  roll = 0.0;
  pitch = 0.0;
  yaw = 0.0;
}

void IMU::update(void) {
  unsigned char buf[IMU_PACKET_LENGTH];

  // Ping the IMU
  serial.print('a');

  // Read the bytes
  cli();
  for (unsigned char i = 0; i < IMU_PACKET_LENGTH; i += 1) {
    buf[i] = serial.read();
  }
  sei();

  // Process packet
  for (unsigned char i = 0; i < IMU_PACKET_LENGTH; i += 1)
    if (packet.receive(buf[i])) {
      // Read 4 big-endian bytes and convert bytewise to float32
      float *tmp = (float*)packet.data;
      yaw = *tmp;
    }
}
