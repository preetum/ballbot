// Hey, emacs: -*- mode: c++; indent-tabs-mode: nil -*-
#include <Servo.h>
#include <PID_v1.h>
#include <MsTimer2.h>

#include "encoder.h"
#include "packet.h"

#define SERVO_LEFT   57
#define SERVO_CENTER 92
#define SERVO_RIGHT  127

// Motor deadband: 91-99
#define MOTOR_FULL_FORWARD 0
#define MOTOR_MIN_FORWARD  91
#define MOTOR_NEUTRAL      95
#define MOTOR_MIN_REVERSE  99
#define MOTOR_FULL_REVERSE 180

// Serial state machine states
enum {
  WAIT,
  READ_LENGTH,
  READ_DATA,
  READ_CHECKSUM
};

// Drive motor states
enum {
  STATE_NORMAL,
  STATE_DELAY1, // 100ms delay to go from forward to reverse (treated as brake)
  STATE_DELAY2 // 100ms delay to go from reverse to neutral
};


// Globals
Servo steering, motor;
Packet packet;

//--------------- Gyro declarations -------------------------
int gyroPin = 0;                 //Gyro is connected to analog pin 0
float gyroVoltage = 3.3;         //Gyro is running at 3.3V

float gyroZeroVoltage = 1.215;   //Gyro is zeroed at 1.23V - given in the datasheet
float gyroSensitivity = .01;     // Gyro senstivity for 4 times amplified output is 10mV/deg/sec

float rotationThreshold = 0.3;   //Minimum deg/sec to keep track of - helps with gyro drifting
//----------------------x-x-x---------------------------------

// TODO store angles in binary angle representation
volatile float currentAngle = 0;          //Keep track of our current angle
volatile float currentAngularVelocity = 0;

//--------- PID declarations ---------------------
//Encoder PID:
double encoder_Input, encoder_Output, encoder_Setpoint;
PID pid_dist(&encoder_Input, &encoder_Output, &encoder_Setpoint,
	     0.8, 0.0, 0.001, DIRECT);
//---------------x-x-x----------------------------

unsigned int driveMotorTarget = MOTOR_NEUTRAL;

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
  long tmpEncoderCount = getEncoderCount();
  int delta = (int)(tmpEncoderCount - lastEncoderCount);
  lastEncoderCount = tmpEncoderCount;
  
  Serial.print(0xff,BYTE);                // send init byte
  writeInt(delta);
  writeInt((int)(currentAngle/180*32768));
  writeInt((int)(currentAngularVelocity/180*32768));
}


/*
 * Sets the drive motor to a value from [0,1023], while taking into account the
 * braking behavior of the hobby motor controller.
 */
void setDriveMotor(unsigned int val) {
  driveMotorTarget = val;
}


/*
 * Called every time a byte is received.
 * Decodes packets and calls packetReceived() when a full valid packet arrives.
 */
void byteReceived (unsigned char byte) {
  static unsigned char state = WAIT;
  static unsigned char i = 0;
  static unsigned char checksum = 0;

  switch (state) {
  case WAIT:
    if (byte == START_BYTE)
      state = READ_LENGTH;
    break;

  case READ_LENGTH:
    packet.length = byte;
    i = 0;
    checksum = byte;
    state = READ_DATA;
    break;

  case READ_DATA:
    if (i < packet.length) {
      packet.data[i++] = byte;
      checksum = checksum ^ byte;
    }

    if (i >= packet.length)
      state = READ_CHECKSUM;
    break;

  case READ_CHECKSUM:
    packet.checksum = byte;
    if (byte == checksum) {
      packetReceived();
    } else {
      // Long blink for bad packet
      //digitalWrite(13, HIGH);
      //delay(200);
      //digitalWrite(13, LOW);
    }
    state = WAIT;
    break;

  default:
    state = WAIT;
    break;
  }
}

/* Called every time a VALID packet is received
 * from the main processor.
 */
void packetReceived () {
  switch (packet.data[0]) {
  case CMD_VALUES: {
    unsigned int steerVal = packet.data[1] << 8 | packet.data[2];  //---->>need to change this: we get heading from the ROS: convert it to the angular velocity we expect.
    unsigned int motorVal = packet.data[3] << 8 | packet.data[4];
                  
    steering.write(steerVal);
    set_speed(motorVal);

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

  // read from Gyro and find the current angle of the car
  int raw_gyro_read = analogRead(gyroPin);

  //Angle Update
  float gyroRate = (raw_gyro_read * gyroVoltage) / 1024;
  gyroRate -= gyroZeroVoltage;
  gyroRate /= gyroSensitivity;
  gyroRate /= 10; // we divide by 10 as gyro updates every 100ms

  if (gyroRate >= rotationThreshold || gyroRate <= -rotationThreshold) {
    currentAngle += gyroRate;
    currentAngularVelocity = gyroRate;
  } else {
    currentAngularVelocity = 0;
  }

  
  /*
  if(steer_input_from_ROS == 0)
    {
      double steer_err = -( raw_gyro_read - steer_Setpoint), angle_err ( angle_Setpoint-currentAngle );
      //cap the steer_err:
      if(steer_err < 3 && steer_err > -3)
	steer_err = 0;
    
      double desi_pid_output = (steer_kP*steer_err +steer_kI*-angle_err);
      if (desi_pid_output < -40)
	desi_pid_output = -40;
      else if (desi_pid_output > 40)
	desi_pid_output = 40;
      steering.write(desi_pid_output+SERVO_CENTER);
      Serial.print("Steering error = "); Serial.println(steer_err);
      Serial.print("Angle = "); Serial.println(currentAngle);
      //    Serial.println(desi_pid_output);
    }
  */
    
  // Compute change in encoder count (discrete velocity estimate)
  long tmpEncoderCount = getEncoderCount();
  long delta = tmpEncoderCount - lastEncoderCount;
  lastEncoderCount = tmpEncoderCount;

  encoder_Input =  (double) delta;
  pid_dist.Compute(); //give the PID the opportunity to compute if needed
}


void setup() {
  // Initialize servo objects
  steering.attach(4);
  motor.attach(5);
  
  // Center the steering servo
  steering.write(SERVO_CENTER);
  motor.write(MOTOR_NEUTRAL);
  
  encoderSetup();

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
  
  
}



void loop() 
{ 
  long curTime;

  // Main command-processing state machine
  while (Serial.available())
    byteReceived(Serial.read());
   
  // Drive motor direction fixing
  static unsigned char lastDirFwd = 1;
  static unsigned char driveMotorState = STATE_NORMAL;
  static unsigned long waitTime = 0; 
  
  // Refresh drive motor values, handling the drive motor braking behavior
  // Need to reverse right after driving forward:
  // 1. send a reverse pulse (treated as a brake signal)
  // 2. send a neutral pulse
  // 3. send a reverse pulse (now treated as a reverse signal)

  switch (driveMotorState) {
  case STATE_NORMAL:
    // In case of a reverse after driving forward
    if (driveMotorTarget >= MOTOR_MIN_REVERSE && lastDirFwd) {
      motor.write(MOTOR_MIN_REVERSE);
      driveMotorState = STATE_DELAY1;
      waitTime = millis() + 100;
        
      // Normal operation
    } else {
      // Deadband
      if (driveMotorTarget > MOTOR_MIN_FORWARD &&
	  driveMotorTarget < MOTOR_MIN_REVERSE)
	motor.write(MOTOR_NEUTRAL);
      else
	motor.write(driveMotorTarget);
        
      // Update the last direction flag
      if (driveMotorTarget <= MOTOR_MIN_FORWARD)
	lastDirFwd = 1;
    }
    break;
      
  case STATE_DELAY1:
    curTime = millis();
    if (curTime >= waitTime) {
      motor.write(MOTOR_NEUTRAL);
      driveMotorState = STATE_DELAY2;
      waitTime = curTime + 100;
    } else if (driveMotorTarget < MOTOR_MIN_REVERSE) {
      driveMotorState = STATE_NORMAL;
    }
    break;
    
  case STATE_DELAY2:
    curTime = millis();
    if (curTime >= waitTime) {
      motor.write(driveMotorTarget);
      driveMotorState = STATE_NORMAL;
      lastDirFwd = 0;
    } else if (driveMotorTarget < MOTOR_MIN_REVERSE) {
      driveMotorState = STATE_NORMAL;
    }
    break;
  } // switch
  
  
} // loop()
