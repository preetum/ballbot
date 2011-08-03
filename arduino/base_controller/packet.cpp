// -*- mode: c++; indent-tabs-mode: nil -*-
/* cpp
 *
 * AUTHOR: John Wang
 * VERSION: 0.2  (6 Sep 2010)
 *
 * DESCRIPTION:
 * State machine for AVR processor. Receives command packets from
 * the main processor via UART.
 */
#include "packet.h"

/* Registers the packetReceived callback.
 */
void Packet::setCallback(void (*callback)(Packet&)) {
  packetReceived = callback;
}

/*
 * Called every time a byte is received.
 * Decodes packets and calls packetReceived() when a full valid packet arrives.
 * Returns true if byte was the last byte of a full valid packet.
 */
unsigned char Packet::byteReceived (unsigned char byte) {
  static unsigned char state = WAIT;
  static unsigned char i = 0;
  static unsigned char checksum = 0;

  unsigned char rv = false;

  switch (state) {
  case WAIT:
    if (byte == START_BYTE)
      state = READ_LENGTH;
    break;

  case READ_LENGTH:
    length = byte;
    i = 0;
    checksum = byte;
    state = READ_DATA;
    break;

  case READ_DATA:
    if (i < length) {
      data[i++] = byte;
      checksum = checksum ^ byte;
    }

    if (i >= length)
      state = READ_CHECKSUM;
    break;

  case READ_CHECKSUM:
    checksum = byte;
    if (byte == checksum && packetReceived != NULL)
      packetReceived(*this);
    state = WAIT;
    rv = true;
    break;

  default:
    state = WAIT;
    break;
  }

  return rv;
}

