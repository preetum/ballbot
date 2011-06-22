int gyroPin = 0;               //Gyro is connected to analog pin 0
float gyroVoltage = 3.3;         //Gyro is running at 3.3V
float gyroZeroVoltage =1.211;   //Gyro is zeroed at 1.23V - given in the datasheet
float gyroSensitivity = 0.00609;  //Our gyro is 2.5mV/deg/sec from the datasheet
float rotationThreshold = 10.0;   //Minimum deg/sec to keep track of - helps with gyro drifting
float currentAngle = 0;          //Keep track of our current angle



void setup() {
  Serial.begin(9600);
  analogReference(EXTERNAL);  //Tell the ADC to use external Vref
}

void loop() {
  
 
 float gyroRate = (analogRead(gyroPin) * gyroVoltage) / 1024;
 gyroRate -= gyroZeroVoltage;
 gyroRate /= gyroSensitivity;
 if (gyroRate >= rotationThreshold || gyroRate <= -rotationThreshold) {
    gyroRate /= 100;
    currentAngle += gyroRate;
  }


/*
float gyroRate = analogRead(gyroPin);
gyroRate -= 381.48;
gyroRate /= gyroSensitivity*1024/3.3;


if (gyroRate >= rotationThreshold || gyroRate <= -rotationThreshold) {
  gyroRate /= 100;
  currentAngle += gyroRate;
}

*/
  Serial.println(currentAngle);
  delay(10);
 
 
} 
