// Hey, emacs: -*- mode: c++; indent-tabs-mode: nil -*-
#include <Servo.h>
#include <PID_v1.h>
#include <MsTimer2.h>
#include <ros.h>
#include <bb_msgs/OdometryRaw.h>
#include <bb_msgs/Velocity1D.h>

#include "base_controller.h"
#include "encoder.h"
#include "imu.h"
#include "smc.h"

// Globals
Servo steering;
SimpleMotorController driveMotor(5);
IMU imu(7, 10);

ros::NodeHandle nh;

bb_msgs::OdometryRaw odomMsg;
ros::Publisher odometry("base/odometry", &odomMsg);

//--------- PID declarations ---------------------
//Encoder PID:
double encoder_Input, encoder_Output, encoder_Setpoint;
PID pid_dist(&encoder_Input, &encoder_Output, &encoder_Setpoint,
	     0.8, 0.0, 0.001, DIRECT);
//---------------x-x-x----------------------------

/* Sets the setpoint of the PID velocity controller.
   vel is given in cm/s
 */
void set_speed(int vel)
{
  // (velocity in m) * (81.45 ticks / cm) / (10 Hz sample rate)
  encoder_Setpoint = vel * 8.145;
}


/* Called every time a VALID packet is received
 * from the main processor.
 *
void packetReceived (Packet& packet) {
  switch (packet.data[0]) {
  case CMD_VALUES: {
    unsigned int steerVal = packet.data[1] << 8 | packet.data[2];  //---->>need to change this: we get heading from the ROS: convert it to the angular velocity we expect.
    unsigned int motorVal = packet.data[3] << 8 | packet.data[4];
                  
    steering.write(steerVal);
    //set_speed(motorVal);
    cli();
    driveMotor.setPWM(motorVal);
    sei();

    break;
  }
        
  case DATA_REQUESTED:
    writeOdometry();
    break;
  
  } // switch
}
*/

/* MStimer2 callback function used for timed update for: 
 *  1. angle | 2. Encoder PID loop | 3. Steering PID Loop
 */
void timer_callback() {
  static long lastEncoderCount = 0L;

  // Compute change in encoder count (discrete velocity estimate)
  long tmpEncoderCount = encoder_getCount();
  long delta = tmpEncoderCount - lastEncoderCount;
  lastEncoderCount = tmpEncoderCount;

  encoder_Input =  (double) delta;
  pid_dist.Compute(); //give the PID the opportunity to compute if needed
}


void setup() {
  // Initialize servos
  steering.attach(4);
  steering.write(SERVO_CENTER);

  // Initialize the drive motor
  driveMotor.initialize();
  
  encoder_initialize();

  analogReference(EXTERNAL);  // Use external Vref (3.3V)
  pinMode(13, OUTPUT); // enable LED pin

  // PID Encoder Stuff:
  pid_dist.SetOutputLimits(0,180); //tell the PID the bounds on the output
  encoder_Output = 0;
  pid_dist.SetMode(AUTOMATIC); //turn on the PID
  pid_dist.SetSampleTime(100); //delay in the loop
  
  nh.initNode();
  nh.advertise(odometry);
}



void loop()  {
  static long lastEncoderCount = 0L;

  imu.update();

  odomMsg.counts = encoder_getCount();
  odomMsg.counts_delta = odomMsg.counts - lastEncoderCount;
  odomMsg.angle = imu.yaw;
  lastEncoderCount = odomMsg.counts;

  odometry.publish(&odomMsg);
  nh.spinOnce();
  
  delay(100);
}
