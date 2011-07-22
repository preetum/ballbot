// -*- mode: c++; indent-tabs-mode: nil -*-
/* packet.cpp
 *
 * AUTHOR: John Wang
 * VERSION: 0.2  (6 Sep 2010)
 *
 * DESCRIPTION:
 * State machine for AVR processor. Receives command packets from
 * the main processor via UART.
 */
#include <stddef.h>
#include "packet.h"

// Global Packet struct
Packet packet;
// Callback when full packet is received
void (*packetReceived)(Packet&) = NULL;

/* Registers the packetReceived callback.
 */
void packet_initialize(void (*callback)(Packet&)) {
  packetReceived = callback;
}

/*
 * Called every time a byte is received.
 * Decodes packets and calls packetReceived() when a full valid packet arrives.
 */
void packet_byteReceived (unsigned char byte) {
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
    if (byte == checksum)
      packetReceived(packet);
    state = WAIT;
    break;

  default:
    state = WAIT;
    break;
  }
}

