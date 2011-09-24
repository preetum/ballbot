import serial
import sys, re, struct, time, thread, math
from robot import Packet

class IMU:
  '''
  Communicates with the IMU
   Initialization parameters:
   1. port: port name for the IMU (Default is /dev/ttyUSB0)
   2. ask_rate = rate at which to ask the IMU for data
  '''
  def __init__(self, port='/dev/ttyUSB0', ask_rate=30):
  
    # Initialize IMU COM port
    try:
      self.serial = serial.Serial(port, baudrate=115200, timeout=0.01)
    except serial.serialutil.SerialException:
      print "No IMU connected."
       
    # Wait for IMU to initialize, then set initial values
    time.sleep(1)

    self.rate = ask_rate
    self.headingBAMS = 0
    self.gyro_z = 0
    self.canvas = None

    # Start serial read thread
    thread.start_new_thread(self.serial_to_fro_thread, ())

  def start_gui(self):
    from Tkinter import Tk, Canvas

    root = Tk()
    root.title('Compass Heading')
    canvas = Canvas(root, width=300, height=300)
    canvas.pack()

    canvas.create_oval(0, 0, 300, 300, width=2)

    self.canvas = canvas
    self.compassLine = canvas.create_line(150, 150, 150, 10)
    root.mainloop()

  def serial_read_callback(self, packet):
    '''
    Called whenever a full packet is received back from the arduino
    '''
    try:
      if packet.length == 10:
        rollBAMS, pitchBAMS, yawBAMS, timestamp = \
            struct.unpack('>hhhI', packet.data)
      self.headingBAMS = yawBAMS
      #print float(yawBAMS)*180.0/32768.0,
      #print timestamp

      # Refresh GUI if running in standalone mode
      if self.canvas is not None:
        theta = math.pi/2 - (yawBAMS * math.pi / 32768)
        x, y = 140 * math.cos(theta), 140 * math.sin(theta)
        self.canvas.coords(self.compassLine, 150, 150, 150+x, 150-y)

    except:
      print "Read error...Retrying"
      pass
      
  def serial_to_fro_thread(self):
    print 'IMU: Listening'
    recv_packet = Packet(self.serial_read_callback)
    while True:
      try:
        # Read as much bytes as possible
        data_string = self.serial.read(100)
        # Fill the packet object
        if len(data_string) > 0:
          recv_packet.read(data_string)
      except OSError:
        print "OSError...Retrying"
        pass
      except serial.serialutil.SerialException:
        print "Serial Exception..Retrying"
        pass


  def request_data(self):
    '''
    Sends 'a' which signal the IMU to send the 
    data packet
    ''' 
    self.serial.write('a\n')
    self.serial.flush()

def update_loop():
  while True:
    sys.stdin.readline()
    imu.request_data()

  
def main():
  '''
  Test stub: Initializes IMU, lets it to print roll, pitch yaw
  '''
  imu = IMU()
  imu.start_gui()

        
if __name__ == '__main__':
  main()
