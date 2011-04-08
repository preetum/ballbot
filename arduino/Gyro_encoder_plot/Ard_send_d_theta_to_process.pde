int gyroPin = 0;               //Gyro is connected to analog pin 0
float gyroVoltage = 3.3;         //Gyro is running at 3.3V
float gyroZeroVoltage = 1.23;   //Gyro is zeroed at 1.23V - given in the datasheet
float gyroSensitivity = .007;  //Our example gyro is 7mV/deg/sec
float rotationThreshold = 1.3;   //Minimum deg/sec to keep track of - helps with gyro drifting

float prev_angle = 0;
float currentAngle = 0;          //Keep track of our current angle

void setup() {
  Serial.begin(9600);
  analogReference(EXTERNAL);  //Tell the ADC to use external Vref
}

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
void loop() {
  
  float gyroRate = (analogRead(gyroPin) * gyroVoltage) / 1023;
  gyroRate -= gyroZeroVoltage;
  gyroRate /= gyroSensitivity;

 if (gyroRate >= rotationThreshold || gyroRate <= -rotationThreshold) {
    gyroRate /= 100;
    currentAngle += gyroRate;
  }

 writeOscilloscope(0, (int) currentAngle);
 //Serial.println((int)currentAngle);
  

  
  delay(10);
} 




