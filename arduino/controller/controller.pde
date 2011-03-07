//#include <AFMotor.h>
#include <Servo.h>
#include "packet.h"

#define SERVO_LEFT    60  //430
#define SERVO_CENTER  100 //580
#define SERVO_RIGHT   140 //730

// Motor deadband: 520-560
#define MOTOR_FULL_FORWARD 0
#define MOTOR_MIN_FORWARD  91
#define MOTOR_NEUTRAL      95
#define MOTOR_MIN_REVERSE  99
#define MOTOR_FULL_REVERSE 180

// Globals
Servo steering, motor;
//AF_DCMotor sweeper(1, MOTOR12_64KHZ), hopper(2, MOTOR12_64KHZ);
Packet packet;

int encoder_counter = 0;
unsigned int driveMotorTarget = MOTOR_NEUTRAL;

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
    STATE_DELAY1,  // 100ms delay to go from forward to reverse (treated as brake)
    STATE_DELAY2   // 100ms delay to go from reverse to neutral
};

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
 * Sets a motor shield motor to a value from [0,255].
void setMotorShield(AF_DCMotor *motor, unsigned char val) {
  if (val > 127) {
    motor->setSpeed((val-127) * 2);
    motor->run(FORWARD);
  } else if (val == 127) {
    motor->run(RELEASE);
  } else {
    motor->setSpeed((127-val) * 2);
    motor->run(BACKWARD);
  }
}
*/

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
		  setDriveMotor(motorVal);
                  //setMotorShield(&sweeper, packet.data[5]);
                  //setMotorShield(&hopper, packet.data[6]);
/*
                  // short blink for packet received
                  digitalWrite(13, HIGH);
                  delay(50);
                  digitalWrite(13, LOW);
*/
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
  
  pinMode(13, OUTPUT);  // enable LED pin
  
  Serial.begin(115200);
}

void loop() {
  static unsigned char lastDirFwd = 1;
  static unsigned char driveMotorState = STATE_NORMAL;
  static unsigned long waitTime = 0;
  
  // Check the serial port for new command byte
  if (Serial.available())
    byteReceived(Serial.read());

  // Refresh drive motor values, handling the drive motor braking behavior
  // Need to reverse right after driving forward:
  //  1. send a reverse pulse (treated as a brake signal)
  //  2. send a neutral pulse
  //  3. send a reverse pulse (now treated as a reverse signal)
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
      if (millis() >= waitTime) {
        motor.write(MOTOR_NEUTRAL);
        driveMotorState = STATE_DELAY2;
        waitTime = millis() + 100;
      } else if (driveMotorTarget < MOTOR_MIN_REVERSE) {
        driveMotorState = STATE_NORMAL; 
      }
      break;
    
    case STATE_DELAY2:
      if (millis() >= waitTime) {
        motor.write(driveMotorTarget);
        driveMotorState = STATE_NORMAL;
        lastDirFwd = 0;
      } else if (driveMotorTarget < MOTOR_MIN_REVERSE) {
        driveMotorState = STATE_NORMAL; 
      }
      break;
  } // switch
} // loop()


