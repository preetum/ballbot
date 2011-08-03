// Hey, emacs: -*- mode: c++; indent-tabs-mode: nil -*-
#include <Servo.h>
#include <ros.h>
#include <bb_msgs/OdometryRaw.h>
#include <bb_msgs/Velocity1D.h>
#include <bb_msgs/PID.h>

#include "base_controller.h"
#include "encoder.h"
#include "feedback.h"
#include "imu.h"

// Globals
Servo steering;
SimpleMotorController driveMotor(5);
IMU imu(7, 10);

// rosserial globals
ros::NodeHandle nh;

// publishers
bb_msgs::OdometryRaw odomMsg;
ros::Publisher odometry("base/odometry", &odomMsg);

// subscribers
bb_msgs::PID pidMsg;
bb_msgs::Velocity1D velMsg;

void pidMsgCallback(unsigned char *data) {
  pidMsg.deserialize(data);
  pidVelocity.SetTunings(pidMsg.kp, pidMsg.ki, pidMsg.kd);
}
ros::Subscriber pidSub("base/pid", &pidMsg, &pidMsgCallback);

void velMsgCallback(unsigned char *data) {
  velMsg.deserialize(data);
  feedback_setVelocity(velMsg.linear);
}
ros::Subscriber velSub("base/velocity", &velMsg, &velMsgCallback);

void setup() {
  // Initialize ROS node
  nh.initNode();
  nh.advertise(odometry);
  odomMsg.counts = 0L;
  nh.subscribe(pidSub);
  nh.subscribe(velSub);

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

  delay(50);
}

void loop()  {
  static unsigned long nextUpdate = 0L;

  // Handle any serial data waiting in the buffer
  nh.spinOnce();

  // Run feedback loop as often as possible
  // (only does computations when sample time is reached)
  feedback_update();

  // Publish odometry message every 33 ms
  if (millis() > nextUpdate) {
    nextUpdate += 33;

    long tmpEncoderCount = encoder_getCount();
    odomMsg.counts_delta = tmpEncoderCount - odomMsg.counts;
    odomMsg.counts = tmpEncoderCount;
    
    //imu.update();
    odomMsg.angle = imu.yaw;
    
    odometry.publish(&odomMsg);
  }
}
