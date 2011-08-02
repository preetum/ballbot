// Hey, emacs: -*- mode: c++; indent-tabs-mode: nil -*-
#include <Servo.h>
#include <PID_v1.h>
#include <MsTimer2.h>

#include "base_controller.h"
#include "encoder.h"
#include "packet.h"
#include "smc.h"


/*
CMD_VALUES packet data field (5 bytes)
  uint8  command type (0x42)
  uint16 steering value
  uint16 motor value
*/
#define CMD_VALUES 0x42
#define DATA_REQUESTED 0x21


// Globals
Servo steering;
SimpleMotorController driveMotor(5);


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

void writeInt(unsigned int i) {
  Serial.print((byte)(i >> 8));
  Serial.print((byte)i);
}

/* Writes odometry info to the serial port in the following order:
 *
 * int16 encoder delta    (in ticks)
 * int16 angular position (as a 16-bit binary angle)
 * int16 angular velocity (as a 16-bit binary angle/s)
 *
 * Notes: ints are sent big-endian
 */
void writeOdometry() {
  static long lastEncoderCount = 0L;
    
  // Compute change in encoder count (discrete velocity estimate)
  long tmpEncoderCount = encoder_getCount();
  int delta = (int)(tmpEncoderCount - lastEncoderCount);
  lastEncoderCount = tmpEncoderCount;
  
  Serial.print(0xff,BYTE);                // send init byte
  writeInt(delta);
  writeInt(0);
  writeInt(0);
}


/* Called every time a VALID packet is received
 * from the main processor.
 */
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
  // Initialize servo objects
  steering.attach(4);

  // Initialize the drive motor
  driveMotor.initialize();
  
  // Center the steering servo
  steering.write(SERVO_CENTER);
  
  encoder_initialize();

  analogReference(EXTERNAL);  // Use external Vref (3.3V)
  pinMode(13, OUTPUT); // enable LED pin

  // PID Encoder Stuff:
  pid_dist.SetOutputLimits(0,180); //tell the PID the bounds on the output
  encoder_Output = 0;
  pid_dist.SetMode(AUTOMATIC); //turn on the PID
  pid_dist.SetSampleTime(100); //delay in the loop
  
  Serial.begin(115200);
  
  //Initialize interrupt timer2 - for gyro update
  MsTimer2::set(100, timer_callback); // 100ms period
  MsTimer2::start();
  
  // Initialize packet callback
  packet_initialize(packetReceived);
}



void loop() 
{ 
  // Main command-processing state machine
  while (Serial.available())
    packet_byteReceived(Serial.read());
}
