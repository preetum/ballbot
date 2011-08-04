#ifndef __base_controller_h
#define __base_controller_h

#include "smc.h"

// Packet data formats
// Note: SET commands begin at 0x41 ('A')
//  and GET commands begin at 0x61 ('a')

// SET_RAW (4 bytes)
//  uint8    id = 0x41
//  uint8    servo
//  uint16   motor
#define CMD_SET_RAW       0x41

// SET_VELOCITY (5 bytes)
//  uint8    id = 0x42
//  int16   linear velocity (cm/s)
//  int16   angular velocity (binary angle/s)
#define CMD_SET_VELOCITY  0x42

// SET_PICKUP (2 bytes)
//  uint8    id = 0x43
//  int8     status (0 = off, 1 = fwd, -1 = rev)
#define CMD_SET_PICKUP    0x43

// SET_PID    (7 bytes)
//  uint8    id = 0x44
//  int16    Kp * 100
//  int16    Ki * 100
//  int16    Kd * 100
#define CMD_SET_PID       0x44

// GET_ODOMETRY (1 byte)
//  uint8    id = 0x61
// Response is in packet format (13 bytes):
//  uint8    id = 0x61
//  int32    counts
//  int32    counts_delta
//  int16    angle (16-bit binary angle)
//  int16    angular velocity (16-bit binary angle / s)
#define CMD_GET_ODOMETRY  0x61

#define SERVO_RIGHT  57
#define SERVO_CENTER 92
#define SERVO_LEFT   127

extern SimpleMotorController driveMotor;

#endif
