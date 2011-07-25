/* packet.h
 *
 * AUTHOR: John Wang
 * VERSION: 0.2  (6 Sep 2010)
 *
 * DESCRIPTION:
 * State machine for AVR processor. Receives command packets from
 * the main processor via UART.
 */
 
#ifndef __packet_h
#define __packet_h

/* Packet format
  
  START   (1 byte  =  0xFF)
  LENGTH  (1 byte)
  DATA    (LENGTH bytes)
  CHECKSUM(1 byte  = XOR of COMMAND, LENGTH, and DATA bits)
 */
#define START_BYTE 0xFF

// Packet structure
typedef struct packet {
	unsigned char length;	// length of data
	unsigned char data[255];
	unsigned char checksum;
} Packet;

// Serial state machine states
enum {
  WAIT,
  READ_LENGTH,
  READ_DATA,
  READ_CHECKSUM
};


/* Callback for every time a valid packet is received. */
void packet_initialize(void (*callback)(Packet&));
//void packetReceived ();

/* Call this every time a byte is received from the main processor. */
void packet_byteReceived (unsigned char byte);

#endif
