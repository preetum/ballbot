import serial
import sys, re
import math
import time

Moves = [-1,-1,-1,-1]
forwardspeed = 0.1
turnspeed = 0.4
scale = 0.5
groundspeed = 0.0348

curve_number = -1
CHANNEL_DRIVE  = 1
CHANNEL_STEER  = 0
ROBOT_RADIUS   = 61  #in cm
ROBOT_SPEED    = 38  #in cm/s
			 
def fmodr(x,y):
    return x - y*math.floor(x/y)

def mod2pi(theta):
    return fmodr(theta,2*math.pi)

def dubins(alpha,d,beta):
    # find distance between start and end points
    lengths = [-1.0,-1.0,-1.0,-1.0,-1.0,-1.0]
    t       = [-1.0,-1.0,-1.0,-1.0,-1.0,-1.0]
    p       = [-1.0,-1.0,-1.0,-1.0,-1.0,-1.0]
    q       = [-1.0,-1.0,-1.0,-1.0,-1.0,-1.0]
    sin_alpha = math.sin(alpha)
    cos_alpha = math.cos(alpha)
    sin_beta = math.sin(beta)
    cos_beta = math.cos(beta)
    cos_alphaminusbeta = math.cos(alpha - beta)
    

    # find length of curve 1: Left Straight Left                     [Lq(Sp(Lt(0,0,alpha))) = (d,0,beta)]
    tmp0 = d+sin_alpha-sin_beta;
    tmp2 = 2 + (d*d) - (2*cos_alphaminusbeta) + (2*d*(sin_alpha - sin_beta))
    if(tmp2 >= 0) and (tmp0 >= 0):
        tmp1 = math.atan((cos_beta - cos_alpha)/tmp0)
        t[0]    = mod2pi(-alpha + tmp1)
        p[0]    = math.sqrt(tmp2)
        q[0]    = mod2pi(beta - tmp1)
        lengths[0] = t[0] + p[0] + q[0]

    # find length of curve 2: Right Straight Right                    [Rq(Sp(Rt(0,0,alpha))) = (d,0,beta)]
    tmp0 = d - sin_alpha + sin_beta
    tmp2 = 2 + (d*d) - 2*cos_alphaminusbeta + (2*d*(sin_beta - sin_alpha))
    if(tmp2 >= 0 and tmp0 >= 0):
        tmp1 = math.atan((cos_alpha - cos_beta)/tmp0)
        t[1]    = mod2pi(alpha - tmp1)
        p[1]    = math.sqrt(tmp2)
        q[1]    = mod2pi(-beta + tmp1)
        lengths[1] = t[1] + p[1] + q[1]

    # find length of curve 3: Left Straight Right                     [Rq(Sp(Lt(0,0,alpha))) = (d,0,beta)]
    tmp1 = -2 + (d*d) + (2*cos_alphaminusbeta) + (2*d*(sin_alpha + sin_beta))
    if(tmp1 >= 0):
        p[2] = math.sqrt(tmp1)
        tmp2 = math.atan((-cos_alpha-cos_beta)/(d+sin_alpha+sin_beta)) - math.atan(-2.0/p[2])
        t[2] = mod2pi(-alpha + tmp2)
        q[2] = mod2pi( - mod2pi(beta) + tmp2)
        lengths[2] = t[2] + p[2] + q[2]
    
    # find length of curve 4: Right Straight Left                      [Lq(Sp(Rt(0,0,alpha))) = (d,0,beta)]
    tmp1 = (d*d) - 2.0 + (2*cos_alphaminusbeta) - (2*d*(sin_alpha + sin_beta))
    if(tmp1 > 0):
        p[3] = math.sqrt(tmp1)
        tmp2 = math.atan((cos_alpha + cos_beta)/(d - sin_alpha - sin_beta)) - math.atan(2.0/p[3])
        t[3] = mod2pi(alpha - tmp2)
        q[3] = mod2pi(beta - tmp2)
        lengths[3] = t[3] + p[3] + q[3]
    
    # find length of curve 5: Right Left Right                         [Rq(Lp(Rt(0,0,alpha))) = (d,0,beta)]
    tmp_rlr = (6.0 - d*d + 2.0*cos_alphaminusbeta + 2.0 * d * (sin_alpha - sin_beta))/8.0
    if(math.fabs(tmp_rlr) < 1):
        p[4] = math.acos(tmp_rlr)
        t[4] = mod2pi(alpha - math.atan((cos_alpha - cos_beta)/(d-sin_alpha + sin_beta)) + mod2pi(p[4]/2.0))
        q[4] = mod2pi(alpha - beta - t[4] + mod2pi(p[4]))
        lengths[4] = t[4] + p[4] + q[4]
    
    # find length of curve 6: Left Right Left                          [Lq(Rp(Lt(0,0,alpha))) = (d,0,beta)]
    tmp_lrl = (6.0 - d*d + 2*cos_alphaminusbeta + 2*d*(-sin_alpha + sin_beta))/8.0
    if(math.fabs(tmp_lrl) < 1):
        p[5] = mod2pi(math.acos(tmp_lrl))
        t[5] = mod2pi(-alpha - math.atan2((cos_alpha - cos_beta),(d + sin_alpha - sin_beta)) + p[5]/2.0)
        q[5] = mod2pi(mod2pi(beta) - alpha - t[5] + mod2pi(p[5]))
        lengths[5] = t[5] + p[5] + q[5]

    # find curve with minimum length
    min_length = lengths[0]
    curve_number = -1
    i = 0
    for length in lengths:
        if((length <= min_length) and (length >= 0)):
            min_length = length
            curve_number = i
        i=i+1                         
    
    # record curve number and move lengths
	#Moves[0] = curve_number
	#Moves[1] = t[curve_number]
    #Moves[2] = p[curve_number]
    #Moves[3] = q[curve_number]
	Moves[0] = curve_number
	Moves[1] = ROBOT_RADIUS * t[curve_number]/ROBOT_SPEED
	Moves[2] = ROBOT_RADIUS * p[curve_number]/ROBOT_SPEED
	Moves[3] = ROBOT_RADIUS * q[curve_number]/ROBOT_SPEED

    print "Shortest path is curve #",curve_number," ",Moves[1]," ",Moves[2]," ",Moves[3]


class Robot:
    START_BYTE = 0xFF
    COMMAND_BYTE = 0x42
    PLANNER_BYTE = 0x08
    #CHANNEL_DRIVE  = 1
    #CHANNEL_STEER  = 0
    #ROBOT_RADIUS   = 61  #in cm
    #ROBOT_SPEED    = 38  #in cm/s

    def __init__(self):
        try:
            self.serial = serial.Serial('/dev/ttyUSB0', baudrate=115200)
        except serial.serialutil.SerialException:
            print "No Arduino connected."
        
        self.packet = [
            # Initialize analog bytes (4)
            0, 0, 0, 0, 127, 127
            ]
        self.plannerpacket = [0,0,0,0,0,0,0,0,0]
        theta = 45
        distance = 100
        self.setAnalog(CHANNEL_DRIVE,510)
        self.sendPacket()
        #self.findTarget(theta,distance)
        self.setMotion('S',1500)
        self.sendMotionPacket()
		
    def findTarget(self,theta,distance):
        """
        calculate shortest Dubins curve and follow it. 
        arguments: theta    =    angle to target (in degrees). Angle is measured in a range [-90,90] with 
                                 negative angles to the left of the robot and positive angles to the right
                   distance  =   straight line distance to target (in cm)
        """

        # Calculate current orientation of robot, taking line to target as the x-axis
        if(theta <0):
            alpha = theta + 360
        else:
            alpha = theta
            
        # Scale distances such that turn radius = 1, for Dubins curves calculation
        d = distance/ROBOT_RADIUS
        
        # final orientation is arbitrarily set to 45 degrees
        beta = 45

        # calculate dubins curves
        dubins(alpha,d,beta)
        time.sleep(2)
        # scale up path segment lengths and convert to time
        #Moves[0] = curve_number
        #Moves[1] = ROBOT_RADIUS * t[curve_number]/ROBOT_SPEED
        #Moves[2] = ROBOT_RADIUS * p[curve_number]/ROBOT_SPEED
        #Moves[3] = ROBOT_RADIUS * q[curve_number]/ROBOT_SPEED
        
        # First action for Moves[1] seconds
        if((Moves[0] == 0) or (Moves[0] == 2) or (Moves[0] == 5)): #LSL,LSR,LRL
            # turn left
            self.turnLeft()
        elif((Moves[0] == 1) or (Moves[0] == 3) or (Moves[0] == 4)): #RSR,RSL,RLR
            # turn right
            self.turnRight()

        time.sleep(Moves[1])

        # Second action for Moves[2] seconds
        if(Moves[0] <= 3): #LSL,RSR,LSR,RSL
            # go forward
            self.driveStraight()
        elif(Moves[0] == 4): #RLR
            # turn left
            self.turnLeft()
        elif(Moves[0] == 5): #LRL
            # turn right
            self.turnRight()

        time.sleep(Moves[2])
        
        # Third action for Moves[3] seconds
        if((Moves[0] == 0) or (Moves[0] == 3) or (Moves[0] == 5)):
            # turn left
            self.turnLeft()
        elif((Moves[0] == 1) or (Moves[0] == 2) or (Moves[0] == 5)):
            # turn right
            self.turnRight()
        
        time.sleep(Moves[3])
        
        self.Stop()

    def driveStraight(self):
        # drive Robot straight at motorspeed = 510 (speed = 38cm/s)
        self.setAnalog(CHANNEL_STEER,590)
        self.setAnalog(CHANNEL_DRIVE,510)
        self.sendPacket()

    def turnRight(self):
        # turn Robot to the right at steering = 790 (radius of turn = 58.4 cm). Drive speed remains the same.
        self.setAnalog(CHANNEL_STEER,790)
        self.sendPacket()

    def turnLeft(self):
        # turn Robot to the left at steering = 390 (radius of turn = 58.4 cm). Drive speed remains the same.
        self.setAnalog(CHANNEL_STEER,390)
        self.sendPacket()

    def Stop(self):
        # stop the Robot
        self.setAnalog(CHANNEL_DRIVE,550)
        self.sendPacket()

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

	def sendMotionPacket(self):
		length = len(self.plannerpacket) + 1
		string = chr(Robot.START_BYTE) + chr(length) + chr(Robot.PLANNER_BYTE)
		checksum = length ^ Robot.PLANNER_BYTE
		for byte in self.plannerpacket:
			string += chr(byte)
			checksum ^= byte
		string += chr(checksum)

		self.serial.write(string)
		self.serial.flushInput()
		self.serial.flushOutput()

	def setMotion(self,action1,val1):
		"""
		Set motion packet
		"""
		val1 = int(val1)
		lb   = val1 % 256
		hb   = (val1 >> 8) % 256
		self.plannerpacket[0] = action1
		self.plannerpacket[1] = hb
		self.plannerpacket[2] = lb
																	

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

    """
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
        """
if __name__ == '__main__':
    main()
