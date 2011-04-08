#define ANALOG_IN 0
//#include "oscilloscope.h"
//---------- oscilloscope.h --------------------------
#ifndef OSCILLOSCOPE_INC
#define OSCILLOSCOPE_INC

#include "WProgram.h"

void writeOscilloscope(int value_x, int value_y) {
  Serial.print( 0xff, BYTE );                // send init byte
  Serial.print( (value_x >> 8) & 0xff, BYTE ); // send first part
  Serial.print( value_x & 0xff, BYTE );        // send second part
  
  Serial.print( (value_y >> 8) & 0xff, BYTE ); // send first part
  Serial.print( value_y & 0xff, BYTE );        // send second part
}

#endif
////----------------------------------------------------

void setup() {
  Serial.begin(9600);
  analogReference(EXTERNAL); 
}

int i = 1;

void loop() {
writeOscilloscope(20+i, 20+i);
i++;
}


