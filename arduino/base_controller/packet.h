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
  CHECKSUM(1 byte  = XOR of COMMAND, LENGTH, and DATA bits)
 */
#define START_BYTE 0xFF

// Packet structure
class Packet {
  // Callback when full packet is received
  void (*packetReceived)(Packet&);

 public:
  unsigned char length;	// length of data
  unsigned char data[10];
  unsigned char checksum;

  Packet(void) {
    packetReceived = NULL;
  }
  void setCallback(void (*callback)(Packet&));
  unsigned char byteReceived(unsigned char byte);
};

// Serial state machine states
enum {
  WAIT,
  READ_LENGTH,
  READ_DATA,
  READ_CHECKSUM
};

#endif
