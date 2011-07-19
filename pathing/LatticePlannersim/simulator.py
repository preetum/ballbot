"""
Simulates the motion of the robot
"""
import graphics
import math
import time
import util
import threading

# GRAPHICS OBJECTS
CAR = None

# ODOMETRY
ROBOT_X = 0.0    # x coordinate of center in global frame
ROBOT_Y = 0.0    # y coordinate of center in global frame
ROBOT_TH = 0.0   # heading of robot in global frame
ROBOT_D  = 0.0   # distance travelled by robot since CLOCK_START

# CONTROL
ROBOT_SPEED = 0.0 # speed in cm per second
ROBOT_STEER = 0.0 # steering angle in degrees
ROBOT_ACTION = None  # action that the robot is currently executing

# TIME
CLOCK_START = 0.0
CLOCK_NOW   = 0.0

PATH = None

# THREADS
thread_control = None
thread_odom = None
event = None

def init_simulator(startNode,path):
    global ROBOT_X,ROBOT_Y,ROBOT_TH,ROBOT_SPEED,ROBOT_STEER,CLOCK_START,PATH,thread_control,thread_odom
    (ROBOT_X,ROBOT_Y,ROBOT_TH,v) = startNode.get_stateparams()       
    ROBOT_SPEED = 0.0
    ROBOT_STEER = 0.0
    ROBOT_D = 0.0
    CLOCK_START = time.time()
    PATH = path
    
    thread_control = threading.Thread(target = drive_path,name = "drivepath")
    thread_odom = threading.Thread(target = odometry,name = "odometry")
    thread_control.start()
    thread_odom.start()
    
    event = threading.Event()

def drive_path():
    """
    Given a path, set speed and steering
    """    
    global ROBOT_SPEED,ROBOT_STEER,ROBOT_ACTION,CLOCK_START,CLOCK_NOW
    path = PATH
    for action in path:        
        print action
        ROBOT_ACTION = action        
        ROBOT_D_atstart = ROBOT_D

        if action == "R_f":
            ROBOT_SPEED = util.ROBOT_SPEED_MAX
            ROBOT_STEER = 70 # should be the steering angle that gives radius = 70            
            while((ROBOT_D - ROBOT_D_atstart) <= math.pi*util.ROBOT_RADIUS_MIN/2):
                continue
                
        elif action == "R_b":
            ROBOT_SPEED = -util.ROBOT_SPEED_MAX
            ROBOT_STEER = 70
            while ((ROBOT_D - ROBOT_D_atstart) <= math.pi*util.ROBOT_RADIUS_MIN/2):
                continue

        elif action == "L_f":
            ROBOT_SPEED = util.ROBOT_SPEED_MAX
            ROBOT_STEER = -70
            while ((ROBOT_D - ROBOT_D_atstart) <= math.pi*util.ROBOT_RADIUS_MIN/2):
                continue

        elif action == "L_b":
            ROBOT_SPEED = -util.ROBOT_SPEED_MAX
            ROBOT_STEER = -70
            while ((ROBOT_D - ROBOT_D_atstart) <= math.pi*util.ROBOT_RADIUS_MIN/2):
                continue

        elif action == "F":
            ROBOT_SPEED = util.ROBOT_SPEED_MAX
            ROBOT_STEER = 0
            while ((ROBOT_D - ROBOT_D_atstart) <= util.CELL_SIZE):
                continue
        
        elif action == "F3":
            ROBOT_SPEED = util.ROBOT_SPEED_MAX
            ROBOT_STEER = 0
            while ((ROBOT_D - ROBOT_D_atstart) <= 3*util.CELL_SIZE):
                continue
        
        elif action == "B":
            ROBOT_SPEED = -util.ROBOT_SPEED_MAX
            ROBOT_STEER = 0
            while ((ROBOT_D - ROBOT_D_atstart) <= util.CELL_SIZE):
                continue

        elif action == "SL_f":
            ROBOT_SPEED = util.ROBOT_SPEED_MAX
            ROBOT_STEER = 0
            while ((ROBOT_D - ROBOT_D_atstart) <= 20.5/70.0 * util.ROBOT_RADIUS_MIN):
                continue
            ROBOT_STEER = -70
            while((ROBOT_D - ROBOT_D_atstart) <= ((20.5/70.0 * util.ROBOT_RADIUS_MIN) + util.ROBOT_RADIUS_2*math.pi/4)):
                continue

        elif action == "SR_f":
            ROBOT_SPEED = util.ROBOT_SPEED_MAX
            ROBOT_STEER = 0
            while ((ROBOT_D - ROBOT_D_atstart) <= 20.5/70.0 * util.ROBOT_RADIUS_MIN): 
                #print "in control"
                time.sleep(0.1)
                continue
            ROBOT_STEER = 70
            while((ROBOT_D - ROBOT_D_atstart) <= ((20.5/70.0 * util.ROBOT_RADIUS_MIN) + util.ROBOT_RADIUS_2*math.pi/4)):
                continue

        elif action == "F_diag":
            ROBOT_SPEED = util.ROBOT_SPEED_MAX
            ROBOT_STEER = 0
            while ((ROBOT_D - ROBOT_D_atstart) <= util.CELL_SIZE*math.sqrt(2)):                
                continue

        elif action == "F_diag3":
            ROBOT_SPEED = util.ROBOT_SPEED_MAX
            ROBOT_STEER = 0
            while ((ROBOT_D - ROBOT_D_atstart) <= 3*util.CELL_SIZE*math.sqrt(2)):
                continue              

        elif action == "B_diag":
            ROBOT_SPEED = -util.ROBOT_SPEED_MAX
            ROBOT_STEER = 0
            while ((ROBOT_D - ROBOT_D_atstart) <= util.CELL_SIZE*math.sqrt(2)):
                continue
        print "thread_control"
        if(not thread_odom.isAlive()):
            return
        
    ROBOT_ACTION = None
    ROBOT_SPEED = 0

def odometry():
    """
    Draw car, and update position
    """
    global CAR,ROBOT_X,ROBOT_Y,ROBOT_TH,ROBOT_D
    v = 0
    t1 = 0
    looptime = 0
    while(True):        
        loopstart = time.time()
        action = ROBOT_ACTION               
        d = ROBOT_SPEED*(looptime) 
        print d, ROBOT_SPEED,"steer",ROBOT_STEER
        ROBOT_D += d    
        if action in ("R_f","L_f","F","F3","SR_f","SL_f","LS_f","RS_f","F_diag","F_diag3"):
            direction = "f"
        else:
            direction = "b"
    
        if action in ("R_f","R_b"): # turning right
            (ROBOT_X,ROBOT_Y,ROBOT_TH,v2) = util.turn_Right((ROBOT_X,ROBOT_Y,ROBOT_TH,v),direction,d,util.ROBOT_RADIUS_MIN)        

        elif action in ("L_f","L_b"): # turning left
            (ROBOT_X,ROBOT_Y,ROBOT_TH,v2) = util.turn_Left((ROBOT_X,ROBOT_Y,ROBOT_TH,v),direction,d,util.ROBOT_RADIUS_MIN)
    
        elif action in ("F","B"):
            (ROBOT_X,ROBOT_Y,ROBOT_TH,v2) = util.go_Straight((ROBOT_X,ROBOT_Y,ROBOT_TH,v),direction,d)            

        elif action in ("F3"):
            (ROBOT_X,ROBOT_Y,ROBOT_TH,v2) = util.go_Straight((ROBOT_X,ROBOT_Y,ROBOT_TH,v),direction,d)

        elif action in ("SL_f","LS_f"):
            if ROBOT_STEER == 0:
                (ROBOT_X,ROBOT_Y,ROBOT_TH,v2) =  util.go_Straight((ROBOT_X,ROBOT_Y,ROBOT_TH,v),direction,d)
            elif ROBOT_STEER == -70:
                (ROBOT_X,ROBOT_Y,ROBOT_TH,v2) = util.turn_Left((ROBOT_X,ROBOT_Y,ROBOT_TH,v),direction,d,util.ROBOT_RADIUS_2)

        elif action == ("SR_f","RS_f"):
            if ROBOT_STEER == 0:
                (ROBOT_X,ROBOT_Y,ROBOT_TH,v2) =  util.go_Straight((ROBOT_X,ROBOT_Y,ROBOT_TH,v),direction,d)
            elif ROBOT_STEER == 70:        
                (ROBOT_X,ROBOT_Y,ROBOT_TH,v2) = util.turn_Right((ROBOT_X,ROBOT_Y,ROBOT_TH,v),direction,d,util.ROBOT_RADIUS_2)        

        elif action == "F_diag":
            (ROBOT_X,ROBOT_Y,ROBOT_TH,v2) = util.go_Straight((ROBOT_X,ROBOT_Y,ROBOT_TH,v),direction,d)
        
        elif action == "F_diag3":
            (ROBOT_X,ROBOT_Y,ROBOT_TH,v2) = util.go_Straight((ROBOT_X,ROBOT_Y,ROBOT_TH,v),direction,d)                        

        elif action == "B_diag":
            (ROBOT_X,ROBOT_Y,ROBOT_TH,v2) = util.go_Straight((ROBOT_X,ROBOT_Y,theta,v),direction,d)
        
        else:
            return
        #time.sleep(0.1)
        if(CAR != None):
            graphics.canvas.delete(CAR[0])
            graphics.canvas.delete(CAR[1])
        CAR = graphics.draw_car(ROBOT_X,ROBOT_Y,ROBOT_TH)
        graphics.canvas.update_idletasks()
        looptime = time.time() - loopstart

        if(not thread_control.isAlive()):
            print odomreturning
            return
        
