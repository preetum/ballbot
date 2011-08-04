import serial
import sys, re, struct, time, thread

class BaseController:
  '''
  Encapsulates the hardware functionality of the robot.
  Communicates with the Arduino and any other peripherals.
  '''
  START_BYTE = 0xFF
  CMD_SET_RAW = 0x41
  CMD_SET_VELOCITY = 0x42
  CMD_SET_PICKUP = 0x43
  CMD_SET_PID = 0x44
  CMD_GET_ODOMETRY = 0x61
  
  SERVO_LEFT = 57
  SERVO_CENTER = 92
  SERVO_RIGHT = 127
  
  MAX_VELOCITY = 3200

  def __init__(self, port='/dev/ttyACM0'):
  
    # Initialize Arduino COM port
    try:
      self.serial = serial.Serial(port, baudrate=115200, timeout=0.01)
    except serial.serialutil.SerialException:
      print "No Arduino connected."
    
    # Initialize Arduino command packet
    # Command packet format:
    # > = big endian
    # B = unsigned byte
    # H = unsigned short (2 bytes)
    self.cmd_packet_format = struct.Struct('>BBH')
    self.cmd_packet = [BaseController.CMD_SET_RAW, 0, 0]
    
    # Wait for Arduino to initialize, then set initial values
    time.sleep(2)
    self.reset()

  def reset(self):
    self.set_steering(0)
    self.set_drive(0)
    self.refresh()

  def set_velocity(self, linear, angular):
    data_string = struct.pack('>Bhh', BaseController.CMD_SET_VELOCITY, linear, angular)
    self.send_arduino_packet(data_string)

  def set_pid(self, kp, ki, kd):
    kp = int(kp * 100)
    ki = int(ki * 100)
    kd = int(kd * 100)
    data_string = struct.pack('>Bhhh', BaseController.CMD_SET_PID, kp, ki, kd)
    self.send_arduino_packet(data_string)

  def refresh(self):
    # Send data commands
    data_string = self.cmd_packet_format.pack(*self.cmd_packet)
    self.send_arduino_packet(data_string)
    
  def send_arduino_packet(self, data_string):
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
    length = len(data_string)
    checksum = length ^ reduce(lambda x,y: x^y, bytearray(data_string), 0)
    
    # Construct full packet
    string = chr(BaseController.START_BYTE) + chr(length) + \
              data_string + chr(checksum)
    
    self.serial.write(string)
    self.serial.flush()
  
  def set_steering(self, angle):
    '''
    Set the steering servo to the given angle in degrees.
    0 is center, a positive angle is right, and a negative angle is left.
    
    NOTE: actual angle => servo values have not been calibrated!
    '''
    value = angle + BaseController.SERVO_CENTER
    
    # check that output value is in valid range
    if value > BaseController.SERVO_RIGHT:
      value = BaseController.SERVO_RIGHT
    elif value < BaseController.SERVO_LEFT:
      value = BaseController.SERVO_LEFT
    
    self.cmd_packet[1] = int(value)

  def set_drive(self, velocity):
    '''
    Set the velocity setpoint to the given value in cm/s.
    '''
    if velocity < 0:
	velocity = 0
    elif velocity > BaseController.MAX_VELOCITY:
	velocity = BaseController.MAX_VELOCITY
    self.cmd_packet[2] = int(velocity)

def update_thread(serial):
  # Print from serial terminal
  while True:
    b = serial.read(100)
    if b:
      print b,
    time.sleep(0.5)

def main():
  '''
  Test stub: takes a comma separated list of values
  steering, motor, sweeper, hopper

  Optional argument: com port
  '''
  if len(sys.argv) > 1:
    robot = BaseController(sys.argv[1])
  else:
    robot = BaseController()

  velocity_values = [0, 0]
  pid_values = [0, 0, 0]

  thread.start_new_thread(update_thread, (robot.serial,))

  try:
    while (True):
      line = sys.stdin.readline().strip()
      if len(line) > 0:
        if re.match('vel', line):
          line = line[3:]
          values = [int(v) for v in line.split(',')]
          for i in range(min(len(values), 2)):
            velocity_values[i] = values[i]
            robot.set_velocity(*velocity_values)
          print 'velocity:', velocity_values
        elif re.match('pid', line):
          line = line[3:]
          values = [float(v) for v in line.split(',')]
          for i in range(min(len(values), 3)):
            pid_values[i] = values[i]
            robot.set_pid(*pid_values)
          print 'pid constants:', pid_values
          
  except:
    # In case of error or Ctrl-C, zero motor values
    robot.reset()
    raise
    
if __name__ == '__main__':
  main()
