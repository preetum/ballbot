//#include "oscilloscope.h"
//---------- oscilloscope.h --------------------------
#ifndef OSCILLOSCOPE_INC
#define OSCILLOSCOPE_INC

#include "WProgram.h"

void writeOscilloscope(int value_x, int value_y) {
  Serial.print( 0xff, BYTE );                // send init byte
  Serial.print( (value_x >> 8) & 0xff, BYTE ); // send first part
  Serial.print( value_x & 0xff, BYTE );        // send second part
  
  Serial.print( (value_y >> 8) & 0xff, BYTE ); // send first part
  Serial.print( value_y & 0xff, BYTE );        // send second part
}

#endif
////----------------------------------------------------


//--------------- Gyro declerations -------------------------
int gyroPin = 0;               //Gyro is connected to analog pin 0
float gyroVoltage = 3.3;         //Gyro is running at 3.3V
float gyroZeroVoltage = 1.23;   //Gyro is zeroed at 1.23V - given in the datasheet
float gyroSensitivity = .007;  //Our example gyro is 7mV/deg/sec
float rotationThreshold = 1.3;   //Minimum deg/sec to keep track of - helps with gyro drifting
//----------------------x-x-x---------------------------------
int encoder_counter = 0;
long cummulative_count = 0;
long distance_limit = 0;
int vel_m = 0;


float Aimed_steerval=590;
float steerval = 590;
float prev_angle = 0;
float currentAngle = 0;          //Keep track of our current angle


#include "hwservo.h"    // For motor control
#include <PID_Beta6.h>  // PID library




//--------- PID declerations ---------------------
double Input, Output, Setpoint;
PID pid_dist(&Input, &Output, &Setpoint,0.8,0.00000,0.001);  //pid(,,, kP, kI, kd)
//---------------x-x-x----------------------------



Servo steering, motor;  // Motor objects


void  set_dist(float dist_m, boolean flag_break)  // first parameter is the intended distance in meter | second para: flag whether you want to account for breaking
                                                  //                                                                  if yes, then we decrease the encoder ticks we want to register, before we spin the motors backwards
{
 if (flag_break)
  distance_limit =  (long) ((dist_m*100)*(124/(5*12*2.54)) - (45/19*Setpoint));
 else
  distance_limit =  (long) (dist_m*100)*(124/(5*12*2.54));

}

void  set_speed(float vel_m)
{
Setpoint =  (long) (((vel_m*3.2808399*2.48) - 1.61)*2.89435601);   //vel_m is speed in meters/second, we convert it into feet/s->multiply by ticks/second and then scale the setpoint.
}


void set_heading(double degree)
{
 Aimed_steerval = 590+(degree*200/90);
}

void setup() 
{
  Serial.begin(9600);
  attachInterrupt(0, encoder_tick, CHANGE); //interrupt to count encoder ticks
  analogReference(EXTERNAL);  //Tell the  (for gyro) to use external Vref
  
  //------------- PID stuff-------------------------
  steering.attach(9);
  motor.attach(10);
   
  motor.write10(510);
  steering.write10(590);
   
  pid_dist.SetOutputLimits(0,150); //tell the PID the bounds on the output
  pid_dist.SetInputLimits(0,60); //number of ticks in 150 milliseconds
  Output = 50; //start the output at its max and let the PID adjust it from there
  pid_dist.SetMode(AUTO); //turn on the PID
  pid_dist.SetSampleTime(100); //delay in the loop
 
  set_speed(1);    // set the desired speed here in meters/second (can be a float value)
  set_dist (1.3, true);    // set distance to be covered in meters (can be a float value)
                                /***********************IMP Note ***************************
            set_speed MUST come before set_dist (as the set_speed changs the veclocity which affects the braking distance.)
                                ************************************************************/
  set_heading(90);
}


void loop() {
  
  // if (cummulative_count < distance_limit )  ///80 encoder counts correspond to 1 meter, we substract 45 for breaking (it might not be a constant should depend on speed)
  if (currentAngle < 90)
  {float gyroRate = (analogRead(gyroPin) * gyroVoltage) / 1023;
  gyroRate -= gyroZeroVoltage;
  gyroRate /= gyroSensitivity;

 if (gyroRate >= rotationThreshold || gyroRate <= -rotationThreshold) {
    gyroRate /= 10;
    currentAngle += gyroRate;
  }

  writeOscilloscope(encoder_counter, (int)currentAngle); //send for visual output
  cummulative_count += encoder_counter;
  
   //Serial.print("counter after update");
   //Serial.println(Sample_Counter());
   //Serial.print (pid.bias);
   //Serial.print (Output);
   
  Input =  (double) encoder_counter;
  pid_dist.Compute(); //give the PID the opportunity to compute if needed
   
  motor.write10(550-Output);
   //digitalWrite(LEDpin, state);
  encoder_counter = 0;
  
  
 //Serial.println(currentAngle);
//  prev_angle = currentAngle;
 // currentAngle = 0;
  
  
 ////////////////////////////////////////// 
 steerval = Aimed_steerval-(currentAngle*200/90);
  if(steerval < 390)
    steerval = 390;
  else if (steerval > 790)
    steerval = 790;
 Serial.println(steerval);
 Serial.println(currentAngle);
 steering.write10(steerval);
 ////////////////////////////////////////// 
  delay(100);
  
}
 else motor.write10(800);
} 


void encoder_tick()
{
encoder_counter += 1;
}



