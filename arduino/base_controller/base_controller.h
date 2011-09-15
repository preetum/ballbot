#ifndef __base_controller_h
#define __base_controller_h

#include "smc.h"

// Configure servo limits
#define SERVO_RIGHT  57
#define SERVO_CENTER 92
#define SERVO_LEFT   127


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
//  uint8    id = 0x44 or 0x45
//  int16    Kp * 100
//  int16    Ki * 100
//  int16    Kd * 100
#define CMD_SET_VELOCITY_PID       0x44
#define CMD_SET_STEERING_PID       0x45

// SYNC_ODOMETRY (3 bytes)
//  uint8    id = 0x61
//  int16    heading (16-bit binary angle)
// Response is in packet format (17 bytes):
//  uint8    id = 0x61
//  int32    counts
//  int32    counts_delta
//  int16    angle (16-bit binary angle)
//  int16    angular velocity (16-bit binary angle / s)
//  int32    timestamp (milliseconds)
#define CMD_SYNC_ODOMETRY  0x61


extern SimpleMotorController driveMotor;

#endif
