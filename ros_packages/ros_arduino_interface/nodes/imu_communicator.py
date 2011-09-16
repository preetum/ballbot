import serial
import sys, re, struct, time, thread
from robot import Packet
import rospy

class IMU:
  '''
  Communicates with the IMU
   Initialization parameters:
   1. port: port name for the IMU (Default is /dev/ttyUSB0)
   2. ask_rate = rate at which to ask the IMU for data
  '''
  def __init__(self, port='/dev/ttyUSB0', ask_rate = 30): 
  
    # Initialize IMU COM port
    try:
      self.serial = serial.Serial(port, baudrate=57600, timeout=0.01)
    except serial.serialutil.SerialException:
      print "No IMU connected."
       
    # Wait for IMU to initialize, then set initial values
    time.sleep(1)

    self.rate = ask_rate
    self.headingBAMS = 0
    self.gyro_z = 0

    # Start serial read thread
    thread.start_new_thread(self.serial_to_fro_thread, ())

  def serial_read_callback(self, packet):
    '''
    Called whenever a full packet is received back from the arduino
    '''
    try:
      if(len(packet.data)==24):
        rollBAMS, pitchBAMS, yawBAMS, \
            gyro_x_raw, gyro_y_raw, gyro_z_raw, \
            accelero_x_raw, accelero_y_raw, accelero_z_raw, \
            magneto_x_raw, magneto_y_raw, magneto_z_raw \
            = struct.unpack('<hhhhhhhhhhhh', packet.data)
      print float(yawBAMS)*180.0/32768.0
      self.headingBAMS = yawBAMS
      self.gyro_z = gyro_z_raw

    except:
      print "Read error...Retrying"
      pass
      
  def serial_to_fro_thread(self):
    recv_packet = Packet(self.serial_read_callback)
    poller =  rospy.Rate(self.rate)
    while True:
      try:
        # Request data from IMU
        self.request_data()
        # Read as much bytes as possible
        data_string = self.serial.read(100)
        # Fill the packet object
        recv_packet.read(data_string)
        poller.sleep()
      except OSError:
        print "OSError...Retrying"
        pass
      except serial.serialutil.SerialException:
        print "Serial Exception..Retrying"
        pass


  def request_data(self):
    '''
    Sends 'ab' which signal the IMU to send the 
    data packet
    ''' 
    self.serial.write('ab')
    self.serial.flush()
  
  
def main():
  '''
  Test stub: Initializes IMU, lets it to print roll, pitch yaw
  '''
  rospy.spin()
        
if __name__ == '__main__':
  main()
