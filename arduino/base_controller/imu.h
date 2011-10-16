#ifndef __imu_h
#define __imu_h

#include <WProgram.h>
#include "packet.h"

// Packet format:
// int16 rollBAMS
// int16 pitchBAMS
// int16 yawBAMS
// int32 timestamp
#define IMU_PACKET_LENGTH  10

class IMU {
  HardwareSerial mySerial;
  Packet packet;

 public:
  int roll, pitch, yaw;

  IMU(HardwareSerial serial);
  unsigned char update(void);
};

#endif
