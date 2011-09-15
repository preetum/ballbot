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

#include <stddef.h>

/* Packet format
  
  START   (1 byte  =  0xFF)
  LENGTH  (1 byte)
  DATA    (LENGTH bytes)
  CHECKSUM(1 byte  = XOR of LENGTH and DATA bits)
 */
#define START_BYTE 0xFF
#define MAX_PACKET_LENGTH  20

// Serial state machine states
enum {
  WAIT,
  READ_LENGTH,
  READ_DATA,
  READ_CHECKSUM
};

// Packet structure
class Packet {
  unsigned char state;  // receive state machine state
  unsigned char index;  // receive data index

 public:
  unsigned char length;	// length of data
  unsigned char data[MAX_PACKET_LENGTH];
  unsigned char checksum;

  Packet(void) {
    state = WAIT;
    index = 0;
  }

  void send(void);
  unsigned char receive(unsigned char byte);
};

#endif
