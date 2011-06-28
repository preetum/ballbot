"""
For first run on Ycon
This program computes the Dubins curve between two points.
Reads encoder and gyro readings from Arduino over serial.
Sends steering commands to Arduino based on these readings.
Logs expected positions and encoder readings.
"""

import serial
import sys, re
import math
import time
import struct
from ballbotmotion_plot import *
from Tkinter import *

sys.path.append("/home/karthik/ballbotcode/ballbot/importfiles")
#from robot import *


Moves = [-1,-1,-1,-1]
forwardspeed = 0.1
turnspeed = 0.4
scale = 0.5
groundspeed = 0.0348

curve_number = -1
CHANNEL_DRIVE  = 1
CHANNEL_STEER  = 0
ROBOT_RADIUS   = 69.6  #in cm
ROBOT_SPEED    = 38.0  #in cm/s
			 
def fmodr(x,y):
    return x - y*math.floor(x/y)

def mod2pi(theta):
    return fmodr(theta,2*math.pi)

def dubins(alpha,beta,d):
    # find distance between start and end points
    #print "dubins d ",d," alpha ",alpha, " beta ", beta
    lengths = [-1.0,-1.0,-1.0,-1.0,-1.0,-1.0]
    t       = [-1.0,-1.0,-1.0,-1.0,-1.0,-1.0]
    p       = [-1.0,-1.0,-1.0,-1.0,-1.0,-1.0]
    q       = [-1.0,-1.0,-1.0,-1.0,-1.0,-1.0]
    sin_alpha = math.sin(alpha)
    cos_alpha = math.cos(alpha)
    sin_beta = math.sin(beta)
    cos_beta = math.cos(beta)
    cos_alphaminusbeta = math.cos(alpha - beta)
    
    # find length of curve 0: Left Straight Left                     [Lq(Sp(Lt(0,0,alpha))) = (d,0,beta)]
    tmp0 = d+sin_alpha-sin_beta;
    tmp2 = 2 + (d*d) - (2*cos_alphaminusbeta) + (2*d*(sin_alpha - sin_beta))
    if(tmp2 >= 0) and (tmp0 > 0):
        tmp1 = math.atan((cos_beta - cos_alpha)/tmp0)
        t[0]    = mod2pi(-alpha + tmp1)
        p[0]    = math.sqrt(tmp2)
        q[0]    = mod2pi(beta - tmp1)
        lengths[0] = t[0] + p[0] + q[0]
    #print "LSL ",lengths[0]
    #print "t = ",t[0]," p = ",p[0]," q = ",q[0]

 # find length of curve 1: Right Straight Right                    [Rq(Sp(Rt(0,0,alpha))) = (d,0,beta)]
    tmp0 = d - sin_alpha + sin_beta
    tmp2 = 2 + (d*d) - 2*cos_alphaminusbeta + (2*d*(sin_beta - sin_alpha))
    if(tmp2 >= 0 and tmp0 > 0):
        tmp1 = math.atan((cos_alpha - cos_beta)/tmp0)
        t[1]    = mod2pi(alpha - tmp1)
        p[1]    = math.sqrt(tmp2)
        q[1]    = mod2pi(-beta + tmp1)
        lengths[1] = t[1] + p[1] + q[1]
    #print "RSR ",lengths[1]
    #print "t = ",t[1]," p = ",p[1]," q = ",q[1]

    # find length of curve 2: Left Straight Right                     [Rq(Sp(Lt(0,0,alpha))) = (d,0,beta)]
    tmp1 = -2 + (d*d) + (2*cos_alphaminusbeta) + (2*d*(sin_alpha + sin_beta))
    if(tmp1 >= 0):
        p[2] = math.sqrt(tmp1)
        tmp2 = math.atan((-cos_alpha-cos_beta)/(d+sin_alpha+sin_beta)) - math.atan(-2.0/p[2])
        t[2] = mod2pi(-alpha + tmp2)
        q[2] = mod2pi( - mod2pi(beta) + tmp2)
        lengths[2] = t[2] + p[2] + q[2]
    #print "LSR ",lengths[2]
    #print "t = ",t[2]," p = ",p[2]," q = ",q[2]

     # find length of curve 3: Right Straight Left                      [Lq(Sp(Rt(0,0,alpha))) = (d,0,beta)]
    tmp1 = (d*d) - 2.0 + (2*cos_alphaminusbeta) - (2*d*(sin_alpha + sin_beta))
    if(tmp1 > 0):
        p[3] = math.sqrt(tmp1)
        tmp2 = math.atan((cos_alpha + cos_beta)/(d - sin_alpha - sin_beta)) - math.atan(2.0/p[3])
        t[3] = mod2pi(alpha - tmp2)
        q[3] = mod2pi(beta - tmp2)
        lengths[3] = t[3] + p[3] + q[3]
    #print "RSL ",lengths[3]
    #print "t = ",t[3]," p = ",p[3]," q = ",q[3]

    # find length of curve 4: Right Left Right                         [Rq(Lp(Rt(0,0,alpha))) = (d,0,beta)]
    tmp_rlr = (6.0 - d*d + 2.0*cos_alphaminusbeta + 2.0 * d * (sin_alpha - sin_beta))/8.0
    if(math.fabs(tmp_rlr) < 1):
        p[4] = math.acos(tmp_rlr)
        t[4] = mod2pi(alpha - math.atan2((cos_alpha - cos_beta),(d-sin_alpha + sin_beta)) + mod2pi(p[4]/2.0))
        q[4] = mod2pi(alpha - beta - t[4] + mod2pi(p[4]))
        lengths[4] = t[4] + p[4] + q[4]

    # find length of curve 5: Left Right Left                          [Lq(Rp(Lt(0,0,alpha))) = (d,0,beta)]
    tmp_lrl = (6.0 - d*d + 2*cos_alphaminusbeta + 2*d*(-sin_alpha + sin_beta))/8.0
    if(math.fabs(tmp_lrl) < 1):
        p[5] = mod2pi(math.acos(tmp_lrl))
        t[5] = mod2pi(-alpha - math.atan2((cos_alpha - cos_beta),(d + sin_alpha - sin_beta)) + p[5]/2.0)
        q[5] = mod2pi(mod2pi(beta) - alpha - t[5] + mod2pi(p[5]))
        lengths[5] = t[5] + p[5] + q[5]
    #print "LRL ",lengths[5]
    #print "t = ",t[5]," p = ",p[5]," q = ",q[5]

    # find curve with minimum length
    i = 0
    for length in lengths:
        if(length >= 0): 
            min_length = length
            curve_number = i

    i = 0
    for length in lengths:
        if((length <= min_length) and (length >= 0)):
            min_length = length
            curve_number = i
        i=i+1                                
   
    Moves[0] = curve_number
    Moves[1] = ROBOT_RADIUS * t[curve_number]
    Moves[2] = ROBOT_RADIUS * p[curve_number]
    Moves[3] = ROBOT_RADIUS * q[curve_number]

    #print "Shortest path is curve #",curve_number," ",Moves[1]," ",Moves[2]," ",Moves[3]

def inputcommands(robot):
    """
    responsible for creating a canvas and firing the correct path generator
    """
    root = Tk()
    root.title(' Ballbot Motion ')
    canvas = Canvas(root,width = 500, height = 500,bg = 'white')
    canvas.pack()
    canvas.create_rectangle(50,50,450,450)

    cmd = IntVar()
    Radiobutton(root,text = "Line",variable = cmd,value = 1).pack(anchor = W, side = LEFT)
    Radiobutton(root,text = "Circle",variable = cmd,value = 2).pack(anchor = W, side = LEFT)
    Radiobutton(root,text = "Dubins",variable = cmd,value = 3).pack(anchor = W, side = LEFT)
    
    # distance
    entryLabel_d = Label(root)
    entryLabel_d["text"] = "    distance (cm) "
    entryLabel_d.pack(side = LEFT)
    entryWidget_d = Entry(root)
    entryWidget_d["width"] = 5
    entryWidget_d.pack(side = LEFT)

    # theta goal. This is in degrees, in the range [-90,90]
    entryLabel_th = Label(root)
    entryLabel_th["text"] = "TH (deg) "
    entryLabel_th.pack(side = LEFT)
    entryWidget_th = Entry(root)
    entryWidget_th["width"] = 5
    entryWidget_th.pack(side = LEFT)

    
    b = Button(root,text = "Go",command = lambda: startPlanner(canvas,robot,cmd,entryWidget_d.get(),entryWidget_th.get()))
    b.pack(side = RIGHT)

      
    root.mainloop()

def drawCanvas(canvas):
    canvas.create_rectangle(50,50,450,450)
    canvas.create_oval(322 + 50 - 2,500 - (200+50-2),322 + 50 +2, 500 - (200 + 50 + 2),width=1,outline = 'blue',fill = 'blue')
	
  
def startPlanner(canvas,robot,cmd,distance,theta):
    """
    first plot the expected path. 
    Then call the correct function to drive the robot along calculated path
    """
    global Moves
    d = float(distance)
    distance = float(distance)/ROBOT_RADIUS
    theta = float(theta)
    cmd = cmd.get()
    canvas.delete(ALL)
    drawCanvas(canvas)
    if(cmd == 1): # Straight line
        path = return_path_straightline(distance)
        drawpath(canvas,path)
        robot.drive_straight(d,canvas)
    elif(cmd == 2): # Circle
        path = return_path_circle(distance)
        drawpath(canvas,path)
        robot.drive_circle(d,canvas)
    elif(cmd == 3): # Dubins curve
        # Calculate current orientation of robot, taking line to target as the x-axis
        if(theta <0):
            alpha = theta + 360
        else:
            alpha = theta
        
        # calculate dubins curves

        dubins(math.radians(alpha),math.radians(45),distance)
        path = return_path_dubins(Moves[0],Moves[1],Moves[2],Moves[3])
        min_dubins = [Moves[0],Moves[1],Moves[2],Moves[3]]
        min_length = Moves[1] + Moves[2] + Moves[3]
        """
        for beta in range(0,360,10):
            # calculate dubins curves for a variety of approach angles. choose the shortest path
            dubins(math.radians(alpha),math.radians(beta),distance)
            length = Moves[1] + Moves[2] + Moves[3]
            if (length < min_length):
                min_length = length
                min_dubins = [Moves[0],Moves[1],Moves[2],Moves[3]]            
        
        Moves = min_dubins
        path = []
        path = return_path_dubins(Moves[0],Moves[1],Moves[2],Moves[3])
        """
        drawpath(canvas,path)

        """ Draw all dubins curves
        for beta in range(0,360,20):
            dubins(math.radians(alpha),math.radians(beta),distance)
            path = []
            path = return_path_dubins(Moves[0],Moves[1],Moves[2],Moves[3])          
            drawpath(canvas,path)
            print beta
            time.sleep(1)
            canvas.delete(ALL)
        """    
        #robot.drive_dubins(canvas)

def drawpath(canvas,path):
    """
    Draws calculated path on the canvas
    """
    for point in path:
        x = point[0]
        y = point[1]
        #print "Drawing calc ",x," ",y
        canvas.create_oval(x + 50 - 1,500 - (y+50-1),x + 50 +1, 500 - (y + 50 + 1),width=1,outline = 'green',fill = 'green')
    canvas.update_idletasks()

def encoderticks_to_distance(encoderticks):
    """
    Given encoderticks, returns corresponding distance in cm
    """
    return encoderticks*100.0/75.0
        
def drawpoint(canvas,point):
    """
    Given a point (x,y,theta) with x,y in cm, plot it on the canvas
    """
    x = point[0]
    y = point[1]
    canvas.create_oval(x + 50 - 1,500 - (y+50-1),x + 50 +1, 500 - (y + 50 + 1),width=1,outline = 'red',fill = 'red')
    canvas.update_idletasks()

#class Ballbot(Robot):
class Ballbot():
    START_BYTE = 0xFF
    COMMAND_BYTE = 0x42
    PLANNER_BYTE = 0x08
    #CHANNEL_DRIVE  = 1
    #CHANNEL_STEER  = 0
    #ROBOT_RADIUS   = 61  #in cm
    #ROBOT_SPEED    = 38  #in cm/s

    def __init__(self):
	#Robot.__init__(self)
        self.distancetravelled = 0
        self.position = (200,200,math.pi/2)
        inputcommands(self)
        
    def drive_straight(self,d,canvas):
        """
        drive straight for d cm
        """
        self.distancetravelled = 0
	self.tickssofar = 0
	started = 0 #so that we send a drive command only once
	
	self.initangle = 0.0
	initangle_registered = 0

        while(1):
	    #print "reading serial"	
      	    serialIn = self.serial.read()
            if(ord(serialIn) == 0xFF): # initial byte for sensor data from arduino
                #print "saw sensor packet"
                ticks = (ord(self.serial.read()) << 8) | ord(self.serial.read())
		distance = encoderticks_to_distance(ticks)
                #distance = (int(self.serial.read()) << 8)| (int(self.serial.read()))
                #angle    = (int(self.serial.read()) << 8) | int(self.serial.read())
                angle = self.serial.read(2)
                angle = struct.unpack("<h",angle)
                angle = angle[0]
		angle = float(angle)/10.0   # gyro sends angle*10, so we divide to get the actual angle
		if(initangle_registered == 0):
		    initangle_registered = 1
		    self.initangle = angle

		angle = angle - self.initangle
                #print "distance ",distance," angle ",angle," distance travelled ", self.distancetravelled, " goal ", d
                # update position based on sensor readings
                y = self.position[1] + distance*math.cos(math.radians(angle))
                x = self.position[0] + distance*math.sin(math.radians(angle))
                theta = self.position[2] - math.radians(angle)
		
                if(theta < 0):
                    theta = theta + 2*math.pi
                elif(theta > 2*math.pi):
                    theta = theta - 2*math.pi
                self.position = (x,y,theta)
                
                # draw position on canvas
		#print "drawing x: ",x," y:", y
                drawpoint(canvas,self.position)
                canvas.update_idletasks()
                self.distancetravelled = self.distancetravelled + distance
		self.tickssofar = self.tickssofar + ticks
            #print "travelled ",self.distancetravelled, " ticks ", self.tickssofar
            if(encoderticks_to_distance(self.tickssofar) < d):
		if(started == 0):
		    self.driveStraight()
		    started = 1
		else:		
                    continue
            else:
                self.Stop()
                print "stopping"
		break

    def drive_circle(self,d,canvas):
        """
        turn right for d cm
        """
        self.distancetravelled = 0
	self.tickssofar = 0
	self.initangle = 0
	initangle_registered = 0
	started = 0

        while(1):
        
            serialIn = self.serial.read();      
            if(ord(serialIn) == 0xff): # initial byte for sensor data from arduino
                
                ticks = (ord(self.serial.read()) << 8) | ord(self.serial.read())
                distance = encoderticks_to_distance(ticks)
                #distance = (ord(self.serial.read()) << 8) | ord(self.serial.read());
                angle = self.serial.read(2)
                angle = struct.unpack("<h",angle)
                angle = angle[0]

                        #angle = float(angle)/10.0   # gyro sends angle*10, so we divide to get the actual angle
                
                print "ticks so far",self.tickssofar,"distance ",distance," angle ",angle," distance travelled ", self.distancetravelled, " goal ", d
                
        
                        # update position based on sensor readings
                y = self.position[1] + distance*math.cos(math.radians(angle));
                x = self.position[0] + distance*math.sin(math.radians(angle));
                theta = self.position[2] - math.radians(angle)
                
                if(theta < 0):
                    theta = theta + 2*math.pi
                elif(theta > 2*math.pi):
                    theta = theta - 2*math.pi
                    
                self.position = (x,y,theta)
                
                # draw position on canvas
                drawpoint(canvas,self.position)
                self.distancetravelled = self.distancetravelled + distance
                self.tickssofar = self.tickssofar + ticks

            if(encoderticks_to_distance(self.tickssofar) < d):
                if(started == 0):
                    started = 1     
                    self.turnRight()
            else:
                self.Stop()
                print "stopping"
                break
	

    def drive_dubins(self,canvas):
        """
        follow calculated dubins curve, stored in Moves[]            
        """
        
        # drive through planned route
        self.distancetravelled = 0
	self.tickssofar = 0
	self.initangle = 0
	initangle_registered = 0
        #print "driving dubins"
        segment = 0 # stores segment of dubins curve that car is executing to avoid repeating commands over serial
        while(1):
            # Read serial port for sensor inputs from Arduino
            serialIn = self.serial.read();
            if(ord(serialIn) == 0xFF): # initial byte for sensor data from arduino
                ticks = (ord(self.serial.read()) << 8) | ord(self.serial.read())
                distance = encoderticks_to_distance(ticks)
		angle = self.serial.read(2)
                angle = struct.unpack("<h",angle)
                angle = angle[0]
		angle = float(angle)/10.0   # gyro sends angle*10, so we divide to get the actual angle
		#print "distance ",distance," angle ",angle," distance travelled ", self.distancetravelled, " goal ", d
		if(initangle_registered == 0):
                    initangle_registered = 1
                    self.initangle = angle

                angle = angle - self.initangle

             # update position based on sensor readings
                y = self.position[1] + distance*math.cos(math.radians(angle))
                x = self.position[0] + distance*math.sin(math.radians(angle))
                theta = self.position[2] - math.radians(angle)

                if(theta < 0):
                    theta = theta + 2*math.pi
                elif(theta > 2*math.pi):
                    theta = theta - 2*math.pi
                self.position = (x,y,theta)
                
                # draw position on canvas
                drawpoint(canvas,self.position)
                self.distancetravelled = self.distancetravelled + distance
            	self.tickssofar = self.tickssofar + ticks
            # First action until self.distancetravelled >= Moves[1]
            if(encoderticks_to_distance(self.tickssofar) < Moves[1]):
                #print "seg 1"
		if(segment == 1):
                    continue
                if((Moves[0] == 0) or (Moves[0] == 2) or (Moves[0] == 5)): #LSL,LSR,LRL
                    # turn left
                    self.turnLeft()
                elif((Moves[0] == 1) or (Moves[0] == 3) or (Moves[0] == 4)): #RSR,RSL,RLR
                    # turn right
                    self.turnRight()
                segment = 1
    
            # Second action until distancetravelled >= Moves[1] + Moves[2]
            elif(encoderticks_to_distance(self.tickssofar) < Moves[1] + Moves[2]):
		#print "seg 2"
                if(segment == 2):
                    continue
                if(Moves[0] <= 3): #LSL,RSR,LSR,RSL
                    # go forward
                    self.driveStraight()
                elif(Moves[0] == 4): #RLR
                    # turn left
                    self.turnLeft()
                elif(Moves[0] == 5): #LRL
                    # turn right
                    self.turnRight()
                segment = 2

            # Third action until distancetravelled >= Moves[1] + Moves[2] + Moves[3]
            elif(encoderticks_to_distance(self.tickssofar) < Moves[1] + Moves[2] + Moves[3]):
		#print "seg 3"
                if(segment == 3):
                    continue
                if((Moves[0] == 0) or (Moves[0] == 3) or (Moves[0] == 5)):
                    # turn left
                    self.turnLeft()
                elif((Moves[0] == 1) or (Moves[0] == 2) or (Moves[0] == 5)):
                    # turn right
                    self.turnRight()
                segment = 3

            else:
                self.Stop()
                break

    def driveStraight(self):
        # drive Robot straight at speed = 100cm/s)
	self.set_steering(0)
        self.set_velocity(250)
	#print "driving straight"
	self.send_arduino_packet()

    def turnRight(self):
        # turn Robot to the right at steering = 40 degrees (radius of turn = 58.4 cm). Drive speed = 100 cm/s
        self.set_steering(40)
	self.set_velocity(250)
	self.send_arduino_packet()

    def turnLeft(self):
        # turn Robot to the left at steering = -40 (radius of turn = 58.4 cm). Drive speed = 100 cm/s
        self.set_steering(-40)
	self.set_velocity(250)
	self.send_arduino_packet()
 
    def Stop(self):
        # stop the Robot
        self.set_velocity(0)
	self.send_arduino_packet()
																	
		
def main():
    '''
    Takes a comma separated list of values
    steering, motor, sweeper, hopper
    '''
        
    robot = Ballbot()
    #inputcommands()


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
