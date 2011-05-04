#include <AFMotor.h>
#include "hwservo.h"
#include "packet.h"

#define SERVO_LEFT    430
#define SERVO_CENTER  580
#define SERVO_RIGHT   730

// Motor deadband: 520-560
#define MOTOR_FULL_FORWARD 0
#define MOTOR_MIN_FORWARD  510
#define MOTOR_NEUTRAL      540
#define MOTOR_MIN_REVERSE  570
#define MOTOR_FULL_REVERSE 1023

// Globals
Servo steering, motor;
AF_DCMotor sweeper(1, MOTOR12_64KHZ), hopper(2, MOTOR12_64KHZ);
Packet packet;

int encoder_counter = 0;

// State machine states
enum {
	WAIT,
	READ_LENGTH,
	READ_DATA,
	READ_CHECKSUM
};

/*
 * Sets the drive motor to a value from [0,1023], while taking into account the
 * braking behavior of the hobby motor controller.
*/
void setDriveMotor(unsigned int val) {
  static signed char lastDirFwd = 1;
  
  // Need to reverse right after driving forward:
  //  1. send a reverse pulse (treated as a brake signal)
  //  2. send a neutral pulse
  //  3. send a reverse pulse (now treated as a reverse signal)
  if (val >= MOTOR_MIN_REVERSE && lastDirFwd) {
    motor.write10(MOTOR_MIN_REVERSE);
    delay(100);  // wait for servo refresh time
    motor.write10(MOTOR_NEUTRAL);
    delay(100);  // wait for servo refresh time
    lastDirFwd = 0;
  
  // Update the last direction flag
  } else if (val <= MOTOR_MIN_FORWARD) {
    lastDirFwd = 1;
  }
  
  motor.write10(val);
}

/*
 * Sets a motor shield motor to a value from [0,255].
*/
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
                  
		  steering.write10(steerVal);
		  setDriveMotor(motorVal);
                  setMotorShield(&sweeper, packet.data[5]);
                  setMotorShield(&hopper, packet.data[6]);
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
  steering.attach(9);
  motor.attach(10);
  
  // Center the steering servo
  steering.write10(SERVO_CENTER);
  
  attachInterrupt(0, encoder_tick, CHANGE); //interrupt to count encoder ticks
  
  pinMode(13, OUTPUT);  // enable LED pin
  
  Serial.begin(115200);
}

void loop() {
  // Check the serial port for new command byte
  if (Serial.available())
    byteReceived(Serial.read());
}

void encoder_tick()
{
  encoder_counter += 1;
}


