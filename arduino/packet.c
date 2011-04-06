#include "packet.h"

// State machine states
enum {
	WAIT,
	READ_LENGTH,
	READ_DATA,
	READ_CHECKSUM
};

// Global packet
Packet packet;

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
			checksum = checksum ^ byte;
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
			if (byte == checksum)
				packetReceived();
			state = WAIT;
		break;
		
		default:
			state = WAIT;
		break;
	}
}

