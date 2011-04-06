/* packet.c
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


/*
CMD_VALUES packet data field (5 bytes)

Digital out byte order
(MSB)                (LSB)
D7 D6 D5 D4 D3 D2 D1 D0

  0x42  (command type)
  A1    (8-bit analog)
  A2
  A3
  A4
*/
#define CMD_VALUES 0x42


// Packet structure
typedef struct packet {
	unsigned char length;	// length of data
	unsigned char data[255];
	unsigned char checksum;
} Packet;

extern Packet packet;


/* Callback for every time a valid packet is received. */
void packetReceived ();

/* Call this every time a byte is received from the main processor. */
void byteReceived (unsigned char byte);

#endif
