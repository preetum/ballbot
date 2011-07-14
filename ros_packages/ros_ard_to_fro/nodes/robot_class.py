import struct
import time
import serial



'''******************************************************************************
                        ROBOT Class
******************************************************************************'''

class Robot:
  '''
  Encapsulates the hardware functionality of the robot.
  Communicates with the Arduino and any other peripherals.
  '''
  START_BYTE = 0xFF
  COMMAND_BYTE = 0x42
  REQUEST_DATA = 0x21

  SERVO_LEFT = 60
  SERVO_CENTER = 100
  SERVO_RIGHT = 140
  
  # Motor deadband: 91-99
  MOTOR_FULL_FORWARD = 0
  MOTOR_MIN_FORWARD = 91
  MOTOR_NEUTRAL = 95
  MOTOR_MIN_REVERSE = 99
  MOTOR_FULL_REVERSE = 180

  MAX_VELOCITY = 300
  ser = serial.Serial()

  def __init__(self):
    
    try:
        self.ser.port = "/dev/ttyUSB0"
        self.ser.baudrate = 115200
        self.ser.timeout = 0.005
        self.ser.open()
    except serial.serialutil.SerialException:
        try:
            self.ser.port = "/dev/ttyUSB1"
            self.ser.open()
        except serial.serialutil.SerialException:
            print "no arduino connected!"
    
    
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
    print "resetting"
    self.set_steering(0)
    self.set_drive(0)
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
    #global ser
    self.ser.write(string)
    self.ser.flushInput()
    self.ser.flushOutput()

  def send_getpacket(self):
      self.cmd_packet[0] = Robot.REQUEST_DATA
      self.send_arduino_packet()
  
  
  def set_steering(self, angle):
    '''
    Set the steering servo to the given angle in degrees.
    0 is center, a positive angle is right, and a negative angle is left.
    
    NOTE: actual angle => servo values have not been calibrated!
    '''
    value = angle + Robot.SERVO_CENTER
    
    # check that output value is in valid range
    if value > Robot.SERVO_RIGHT:
      value = Robot.SERVO_RIGHT
    elif value < Robot.SERVO_LEFT:
      value = Robot.SERVO_LEFT
    
    self.cmd_packet[0] = Robot.COMMAND_BYTE
    self.cmd_packet[1] = int(value)

  def set_velocity(self, velocity):
    '''
    Set the velocity setpoint to the given value in cm/s.
    '''
    if velocity < 0:
	velocity = 0
    elif velocity > Robot.MAX_VELOCITY:
	velocity = Robot.MAX_VELOCITY
    self.cmd_packet[0] = Robot.COMMAND_BYTE
    self.cmd_packet[2] = int(velocity)

  def set_drive(self, duty_cycle):
    pass
#*******************************************************