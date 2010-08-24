import serial
import sys, re

class Robot:
    START_BYTE = 0xFF
    COMMAND_BYTE = 0x42

    def __init__(self):
        try:
            self.serial = serial.Serial('/dev/ttyUSB0', baudrate=115200)
        except serial.serialutil.SerialException:
            print "No Arduino connected."
        
        self.packet = [
            # Initialize analog bytes (4)
            0, 0, 0, 0, 127, 127
            ]
        self.setAnalog(0, 600)
        self.setAnalog(1, 600)

    def sendPacket(self):
        length = len(self.packet) + 1
        string = chr(Robot.START_BYTE) + chr(length) + chr(Robot.COMMAND_BYTE)
        checksum = length ^ Robot.COMMAND_BYTE
        for byte in self.packet:
            string += chr(byte)
            checksum ^= byte
        string += chr(checksum)
        
        self.serial.write(string)
        self.serial.flushInput()
        self.serial.flushOutput()

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


def main():
    '''
    Takes a comma separated list of values
    steering, motor, sweeper, hopper
    '''
    robot = Robot()

    try:
        while (True):
            robot.sendPacket()
            
            line = sys.stdin.readline()
            values = [int(v) for v in line.split(',')]
            for i in range(min(len(values), 4)):
                robot.setAnalog(i, values[i])
    except KeyboardInterrupt:
        # In case of Ctrl-C, zero motor values
        robot.setAnalog(0, 600)
        robot.setAnalog(1, 600)
        robot.setAnalog(2, 127)
        robot.setAnalog(3, 127)

if __name__ == '__main__':
    main()
