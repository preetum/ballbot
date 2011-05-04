import serial
import sys, re, struct, time

camera_params = {'height': 33.5,  # in cm
  'offset_x': -4.5,  # offset from center of front axle, in cm
  'offset_y': -10,
  'tilt':  1.228,   # in radians
  'resolution': (640, 480),
  'radians_per_px': 0.0016
}

class Robot:
  '''
  Encapsulates the hardware functionality of the robot.
  Communicates with the Arduino and any other peripherals.
  '''
  START_BYTE = 0xFF
  COMMAND_BYTE = 0x42
  
  SERVO_LEFT = 430
  SERVO_CENTER = 580
  SERVO_RIGHT = 730
  
  # Motor deadband: 520-560
  MOTOR_FULL_FORWARD = 0
  MOTOR_MIN_FORWARD = 510
  MOTOR_NEUTRAL = 540
  MOTOR_MIN_REVERSE = 570
  MOTOR_FULL_REVERSE = 1023

  def __init__(self, port='/dev/ttyUSB0'):
  
    # Initialize Arduino COM port
    try:
      self.serial = serial.Serial(port, baudrate=115200)
    except serial.serialutil.SerialException:
      print "No Arduino connected."
    
    # Initialize Arduino command packet
    # Command packet format: 
    # > = big endian
    # B = unsigned byte
    # H = unsigned short (2 bytes)
    self.cmd_packet_format = struct.Struct('>BHHBB')
    self.cmd_packet = [Robot.COMMAND_BYTE, 0, 0, 0, 0]
    
    # Wait for Arduino to initialize, then set initial values
    time.sleep(2)
    self.reset()

  def reset(self):
    self.set_steering(0)
    self.set_drive(0)
    self.set_hopper(0)
    self.set_sweeper(0)
    self.send_arduino_packet()
    
  def send_arduino_packet(self):
    '''
    Arduino packet format:
      START   (1 byte  =  0xFF)
      LENGTH  (1 byte)
      DATA    (LENGTH bytes)
      CHECKSUM(1 byte  = XOR of LENGTH and DATA bits)
    
    Data format:
    
    byte  description
    0     command byte = 0x42
    1:2   10-bit steering value (big endian, byte 1 is most significant byte)
    3:4   10-bit drive motor value
    5     8-bit sweeper motor value
    6     8-bit hopper motor value
    '''
    # Generate checksum
    data_string = self.cmd_packet_format.pack(*self.cmd_packet)
    length = self.cmd_packet_format.size
    checksum = length ^ reduce(lambda x,y: x^y, bytearray(data_string), 0)
    
    # Construct full packet
    string = chr(Robot.START_BYTE) + chr(length) + \
              data_string + chr(checksum)
    
    self.serial.write(string)
    self.serial.flushInput()
    self.serial.flushOutput()
  
  def set_steering(self, angle):
    '''
    Set the steering servo to the given angle in degrees.
    0 is center, a positive angle is right, and a negative angle is left.
    
    NOTE: actual angle => servo values have not been calibrated!
    '''
    value = angle * (Robot.SERVO_RIGHT - Robot.SERVO_LEFT) / 60 + \
              Robot.SERVO_CENTER
    
    # check that output value is in valid range
    if value > Robot.SERVO_RIGHT:
      value = Robot.SERVO_RIGHT
    elif value < Robot.SERVO_LEFT:
      value = Robot.SERVO_LEFT
    
    self.cmd_packet[1] = int(value)
    
  def set_drive(self, duty_cycle):
    '''
    Set the drive motor to the given % duty cycle, ranging from -100 to 100.
    0 is stop, a positive value is forward, and a negative value is reverse
    '''
    if duty_cycle == 0:
      value = Robot.MOTOR_NEUTRAL
    else:
      if duty_cycle > 0:
        value = duty_cycle * (Robot.MOTOR_FULL_FORWARD -
                  Robot.MOTOR_MIN_FORWARD) / 100 + Robot.MOTOR_MIN_FORWARD
      else:   # duty_cycle < 0
        value = -duty_cycle * (Robot.MOTOR_FULL_REVERSE - 
                  Robot.MOTOR_MIN_REVERSE) / 100 + Robot.MOTOR_MIN_REVERSE
                  
        # check that output value is in valid range
        if value < 0:
          value = 0
        elif value > 1023:
          value = 1023
    
    self.cmd_packet[2] = int(value)
    print 'drive: %d' % value
  
  def set_hopper(self, duty_cycle):
    '''
    Set the hopper motor to the given % duty cycle, ranging from -100 to 100.
    '''
    pass
  
  def set_sweeper(self, duty_cycle):
    '''
    Set the hopper motor to the given % duty cycle, ranging from -100 to 100.
    '''
    pass

  '''
  def setAnalog(self, ch, val):
    """
    Set the value of an analog channel
    ch in range [0,4]
    val in range [0, 255]
    """
    val = int(val)
    lb = val % 256
    hb = (val >> 8) % 256
    if ch == 0 or ch == 1:
      self.packet[ch*2] = hb
      self.packet[ch*2+1] = lb
    else:
      self.packet[ch+2] = lb
      '''


def main():
  '''
  Test stub: takes a comma separated list of values
  steering, motor, sweeper, hopper

  Optional argument: com port
  '''
  if len(sys.argv) > 1:
    robot = Robot(sys.argv[1])
  else:
    robot = Robot()

  try:
    while (True):
      robot.send_arduino_packet()
          
      line = sys.stdin.readline()
      
      values = [float(v) for v in line.split(',')]
      for i in range(min(len(values), 4)):
        if i == 0: robot.set_steering(values[i])
        elif i == 1: robot.set_drive(values[i])
  except:
    # In case of error or Ctrl-C, zero motor values
    robot.reset()
    raise
    
if __name__ == '__main__':
  main()
