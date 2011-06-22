#include <Servo.h>
#include "packet.h"
#include <PID_Beta6.h>  // PID library
#include <MsTimer2.h>

#define SERVO_LEFT   60
#define SERVO_CENTER 100
#define SERVO_RIGHT  140

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


//--------------- Gyro declerations -------------------------
int gyroPin = 0;                 //Gyro is connected to analog pin 0
float gyroVoltage = 3.3;         //Gyro is running at 3.3V
float gyroZeroVoltage = 1.215;   //Gyro is zeroed at 1.23V - given in the datasheet
float gyroSensitivity = .01;     // Gyro senstivity for 4 times amplified output is 10mV/deg/sec

float rotationThreshold = 2.0;   //Minimum deg/sec to keep track of - helps with gyro drifting
//----------------------x-x-x---------------------------------


long cummulative_count = 0;
long distance_limit = 0;
int vel_m = 0;

float currentAngle = 0;          //Keep track of our current angle

//--------- PID declerations ---------------------
double Input, Output, Setpoint;
PID pid_dist(&Input, &Output, &Setpoint,0.8,0.00000,0.001);  //pid(,,, kP, kI, kd)
//---------------x-x-x----------------------------

int encoder_counter = 0;
unsigned int driveMotorTarget = MOTOR_NEUTRAL;

// fix the setpoint for PID control of speed
void  set_speed(float vel_m)
{
Setpoint =  (long) (((vel_m*3.2808399*2.48) - 1.61)*2.89435601);   //vel_m is speed in meters/second, we convert it into feet/s->multiply by ticks/second and then scale the setpoint.
}

// Write (encodercount, currentangle) to the serial port. This is actually (distance,dtheta) from last sample

void writeOscilloscope(int value_x, int value_y) {
  
  Serial.print(0xff);                // send init byte

  Serial.print( (value_x >> 8) & 0xff); // send first part
  Serial.print( value_x & 0xff);        // send second part
  
  Serial.print( value_y & 0xff, BYTE );        // send second part
  Serial.print( (value_y >> 8) & 0xff, BYTE ); // send first part

  //Serial.print("value_y");
  //Serial.println(value_y);

}

void encoder_tick()
{
  encoder_counter += 1;
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
                  unsigned int steerVal = packet.data[1] << 8 | packet.data[2],
                    motorVal = packet.data[3] << 8 | packet.data[4];
                  
steering.write(steerVal);
set_speed(motorVal / 100.0);

break;
        }
    }
}

void setup() {
  
  
  // Initialize servo objects
  steering.attach(4);
  motor.attach(5);
  
  // Center the steering servo
  steering.write(SERVO_CENTER);
  motor.write(MOTOR_NEUTRAL);
  
  attachInterrupt(0, encoder_tick, CHANGE); //interrupt to count encoder ticks  

  analogReference(EXTERNAL);  //Tell the  gyro to use external Vref
  pinMode(13, OUTPUT); // enable LED pin

  // PID Stuff
  pid_dist.SetOutputLimits(0,150); //tell the PID the bounds on the output
  pid_dist.SetInputLimits(0,60); //number of ticks in 150 milliseconds
  Output = 0;
  Setpoint = 0;
  pid_dist.SetMode(AUTO); //turn on the PID
  pid_dist.SetSampleTime(100); //delay in the loop
  
  Serial.begin(115200);
  
  //Initialize interrupt timer2 - for gyro update
  MsTimer2::set(100, angle_update); // 100ms period
  MsTimer2::start();
}


void angle_update()
{
  // read from Gyro and find the current angle of the car
    float gyroRate = (analogRead(gyroPin) * gyroVoltage) / 1024;
    gyroRate -= gyroZeroVoltage;
    gyroRate /= gyroSensitivity;

   if (gyroRate >= rotationThreshold || gyroRate <= -rotationThreshold) {
      gyroRate /= 10; // we divide by 10 as gyro updates every 100ms
      currentAngle += gyroRate;
    } 
}




void loop() 
{
  static unsigned long pidLoopCount = 0;
  long curTime;

  // Run PID loop and output odometry every 100ms
  curTime = millis();
  if (curTime >= pidLoopCount) {
    pidLoopCount = curTime + 100;

   /* // read from Gyro and find the current angle of the car
    float gyroRate = (analogRead(gyroPin) * gyroVoltage) / 1024;
    gyroRate -= gyroZeroVoltage;
    gyroRate /= gyroSensitivity;

   if (gyroRate >= rotationThreshold || gyroRate <= -rotationThreshold) {
      gyroRate /= 10;
      currentAngle += gyroRate;
    } */

    int tmpEncoderCount = encoder_counter;	// save encoder value
    encoder_counter = 0;
    cummulative_count += tmpEncoderCount;
    // rescale the angle by 90/145; multiply by 10 for extra decimal precision
    writeOscilloscope(tmpEncoderCount, (int) currentAngle); //send for visual output
    Input =  (double)tmpEncoderCount;
    pid_dist.Compute(); //give the PID the opportunity to compute if needed
  
    setDriveMotor(MOTOR_NEUTRAL - Output*180/1024);
  }
    
  // Check the serial port for new command byte
  if (Serial.available())
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
