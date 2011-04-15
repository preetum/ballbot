//Aim: arduino sends encoder and gyro values
// processing should calculate x,y and plot them!


import processing.serial.*;

int valx, valy;      // Data received from the serial port
Serial port;  // Create object from Serial class
float theta = 0;
//int x =50, y = 400; 
float x1 = 320, y1 = 450; //start at lower-left corner

void setup() 
{
  size(750, 480);
  // Open the port that the board is connected to and use the same speed (9600 bps)
  String portName = Serial.list()[0];
  port = new Serial(this, portName, 9600);
  ellipseMode(RADIUS);
  smooth();
}


void draw()
{
  while (port.available() >= 5) {
    if (port.read() == 0xff) {
      valx = (port.read() << 8) | (port.read());
      valy = (port.read() << 8) | (port.read());
    }
  
  
 
//theta += (valy/100.0); 
 
y1 = y1 - valx*cos(valy*(3.14159265/180))/5;
x1 = x1 + valx*sin(valy*(3.14159265/180))/5;

  
ellipse(x1, y1, 1, 1); // ellipse(x,y,width,height)
//dD = valy;
}
}

