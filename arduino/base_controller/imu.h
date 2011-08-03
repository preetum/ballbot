#ifndef __imu_h
#define __imu_h

#include <SoftwareSerial.h>
#include "packet.h"

#define IMU_PACKET_LENGTH  7

class IMU {
  SoftwareSerial serial;
  Packet packet;

 public:
  float roll, pitch, yaw;

  IMU(int rxPin, int txPin);
  void update(void);
};

#endif
