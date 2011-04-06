#include <AFMotor.h>
#include "hwservo.h"
#include "packet.h"

// Globals
Servo steering, motor;
AF_DCMotor sweeper(1, MOTOR12_64KHZ), hopper(2, MOTOR12_64KHZ);
Packet packet;

// State machine states
enum {
	WAIT,
	READ_LENGTH,
	READ_DATA,
	READ_CHECKSUM
};

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
                          /*
                          // Long blink for bad packet
                          digitalWrite(13, HIGH);
                          delay(200);
                          digitalWrite(13, LOW);
                          */
                        }
			state = WAIT;
		break;
		
		default:
			state = WAIT;
		break;
	}
}

void setMotor(AF_DCMotor *motor, unsigned char val) {
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

/* Called every time a VALID packet is received
 * from the main processor.
 */
void packetReceived () {
	switch (packet.data[0]) {
		case CMD_VALUES: {
                  unsigned int steerVal = packet.data[1] << 8 | packet.data[2],
                    motorVal = packet.data[3] << 8 | packet.data[4];
                  
		  steering.write10(steerVal);
		  motor.write10(motorVal);
                  setMotor(&sweeper, packet.data[5]);
                  setMotor(&hopper, packet.data[6]);
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
  steering.attach(9);
  motor.attach(10);
  
  pinMode(13, OUTPUT);  // enable LED pin
  
  Serial.begin(115200);
}

void loop() {
  // Check the serial port for new command byte
  if (Serial.available())
    byteReceived(Serial.read());
}

