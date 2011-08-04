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
#include <WProgram.h>
#include "packet.h"

void Packet::send(void) {
  if (length > MAX_PACKET_LENGTH)
    length = MAX_PACKET_LENGTH;

  // Print packet header
  Serial.print((unsigned char)START_BYTE);
  Serial.print(length);

  // Print data
  checksum = 0;  // compute checksum as we go
  for (unsigned char i = 0; i < length; i += 1) {
    Serial.print(data[i]);
    checksum ^= data[i];
  }

  // Print checksum
  Serial.print(checksum);
}

/*
 * Called every time a byte is received.
 * Decodes packets and calls packetReceived() when a full valid packet arrives.
 * Returns true if byte was the last byte of a full valid packet.
 */
unsigned char Packet::receive (unsigned char byte) {
  unsigned char rv = false;

  switch (state) {
  case WAIT:
    if (byte == START_BYTE)
      state = READ_LENGTH;
    break;

  case READ_LENGTH:
    length = byte;
    index = 0;
    checksum = byte;
    state = READ_DATA;
    break;

  case READ_DATA:
    if (index < length) {
      data[index++] = byte;
      checksum = checksum ^ byte;
    }

    if (index >= length)
      state = READ_CHECKSUM;
    break;

  case READ_CHECKSUM:
    rv = (byte == checksum);
    state = WAIT;
    break;

  default:
    state = WAIT;
    break;
  }

  return rv;
}

