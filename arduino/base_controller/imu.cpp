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

IMU::IMU(HardwareSerial serial)
  : mySerial(serial) {
  mySerial.begin(115200);

  roll = 0.0;
  pitch = 0.0;
  yaw = 0.0;
}

/* Call in main loop to process the IMU serial buffer.
 * Returns true iff values were updated.
 */
unsigned char IMU::update(void) {
  // Read as many bytes are available
  while (mySerial.available()) {
    // If full packet received, process the packet
    if (packet.receive(mySerial.read())) {
      if (packet.length == IMU_PACKET_LENGTH) {
        roll = ((int)packet.data[0] << 8) | packet.data[1];
        pitch = ((int)packet.data[2] << 8) | packet.data[3];
        yaw = ((int)packet.data[4] << 8) | packet.data[5];

        return true;
      }
    }
  }
  return false;
}
