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
  
  START   (1 byte  =  0xAF)
  LENGTH  (1 byte)
  DATA    (LENGTH bytes)
  CHECKSUM(1 byte  = XOR of LENGTH and DATA bits)
 */
#define START_BYTE         0xAF
#define ESCAPE_BYTE        0xAE

#define MAX_PACKET_LENGTH  16

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
  unsigned char escaped; // receive state (if last char was an escape)
  unsigned char index;  // receive data index

 public:
  unsigned char length;	// length of data
  unsigned char data[MAX_PACKET_LENGTH];
  unsigned char checksum;

  Packet(void) {
    state = WAIT;
    escaped = false;
    index = 0;
  }

  void send(void);
  unsigned char receive(unsigned char byte);
};

// Function prototypes
void reverse_memcpy(void *destination, const void *source, size_t len);

#endif
