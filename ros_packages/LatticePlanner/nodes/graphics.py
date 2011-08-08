"""
Contains functions to draw stuff
"""
from Tkinter import *
import math
import util

root = Tk()
canvas = Canvas(root,width = 300, height = 600,bg = 'white')
cm_to_pixels = 600/3657.6
displayLattice = False

def draw_court():
    """
    Draw the tennis court
    """
    global cm_to_pixels
    canvas.create_rectangle(0.9*50,5.95*50,5.1*50,6.05*50,width=1,outline = 'black',fill = '')
    canvas.create_rectangle(1.2*50,2.1*50,4.8*50,9.9*50,width=1,outline = 'black')
    canvas.create_line(1.65*50,2.1*50,1.65*50,9.9*50,width=1,fill='black')
    canvas.create_line(4.35*50,2.1*50,4.35*50,9.9*50,width=1,fill='black')
    canvas.create_line(1.65*50,3.9*50,4.35*50,3.9*50,width=1,fill='black')
    canvas.create_line(1.65*50,8.1*50,4.35*50,8.1*50,width=1,fill='black')
    canvas.create_line(3*50,3.9*50,3*50,8.1*50,width=1,fill='black')

    d = 0
    while(d <= 300/cm_to_pixels):
        canvas.create_text(d*cm_to_pixels,580,text = str(d),fill = 'red')
        d+=300

    d = 0
    while(d <= 600/cm_to_pixels):
        canvas.create_text(20,600 - d*cm_to_pixels,text = str(d),fill = 'red')
        d+=300

def draw_lattice():
    """
    Draw the Lattice as a background
    """
    global cm_to_pixels
    cellsize = util.CELL_SIZE
    # draw vertical lines
    d = 0
    while(d <= 300/cm_to_pixels):
        canvas.create_line(d*cm_to_pixels,0,d*cm_to_pixels,600,width=1,fill = 'gray',)
        d += cellsize
    d = 0
    while(d<= 600/cm_to_pixels):
        canvas.create_line(0,600-d*cm_to_pixels,300,600-d*cm_to_pixels,width=1,fill = 'gray')
        d += cellsize    
        
def draw_car(x,y,th,color = "RED"):    
    """ 
    draw car on the canvas, and return (car_outline,car_center)
    """
    ROBOT_LENGTH = util.ROBOT_LENGTH
    ROBOT_WIDTH = util.ROBOT_WIDTH
    r = math.sqrt(math.pow(ROBOT_LENGTH,2) + math.pow(ROBOT_WIDTH,2))/2        
    
    # top left
    phi = th + math.pi/2+ math.atan2(ROBOT_LENGTH,ROBOT_WIDTH)
    topleft  = (x + r*math.cos(phi),y+r*math.sin(phi))
    
    #top right
    phi = th + math.atan2(ROBOT_WIDTH,ROBOT_LENGTH) 
    topright = (x + r*math.cos(phi),y+r*math.sin(phi))

    # bottom left
    phi = th + math.pi + math.atan2(ROBOT_WIDTH,ROBOT_LENGTH)
    bottomleft =  (x + r*math.cos(phi),y+r*math.sin(phi))
    
    # bottom right
    phi = th + 3*math.pi/2 + math.atan2(ROBOT_LENGTH,ROBOT_WIDTH)
    bottomright =  (x + r*math.cos(phi),y+r*math.sin(phi))
    
    car_outline = canvas.create_polygon(topleft[0]*cm_to_pixels,600 - topleft[1]*cm_to_pixels,
                        bottomleft[0]*cm_to_pixels,600 - bottomleft[1]*cm_to_pixels,
                        bottomright[0]*cm_to_pixels,600 - bottomright[1]*cm_to_pixels,
                        topright[0]*cm_to_pixels,600 - topright[1]*cm_to_pixels,
                        width = 1, outline = 'blue',fill = '')
    x1 = x*cm_to_pixels
    y1 = y*cm_to_pixels
    car_center = canvas.create_oval(x1-1,600-(y1-1),x1+1,600-(y1+1),outline = color,fill = color)
    return (car_outline,car_center)

def draw_point(x,y,theta,color = 'blue'):
    """
    Draw a point on the canvas. (x,y) are in cm
    """
    global canvas,cm_to_pixels
    x = x*cm_to_pixels
    y = 600 - y*cm_to_pixels
    canvas.create_oval(x-1,y-1,x+1,y+1,width=1,outline = color,fill = color)

def draw_point_directed(x,y,theta,color = 'red'):
    """
    Draw a point with two lines, reperesenting direction Eg: /\ for theta = 90
    """
    x = x*cm_to_pixels
    y = 600 - y*cm_to_pixels
    #x_left = x - 2*math.cos(theta - math.radians(20))
    #y_left = y - 2*math.sin(theta - math.radians(20)) 
    #x_right = x + 2*math.cos(theta 
    canvas.create_line(x,y,x+2*math.cos(theta),y-2*math.sin(theta),arrow = "last")
    
def draw_rectangle(x1,y1,x2,y2,color = 'blue'):
    """
    Draw a rectangle on the canvas. (x,y) in cm
    """
    global canvas,cm_to_pixels
    x1 = x1*cm_to_pixels
    y1 = 600 - y1*cm_to_pixels
    x2 = x2*cm_to_pixels
    y2 = 600 - y2*cm_to_pixels

    rect = canvas.create_rectangle(x1,y1,x2,y2,width=1,outline = color,fill = color)
    return rect

def draw_turn_Right(x,y,theta,v,direction,distance,radius):
    d = 0
    while(d <= distance):
        (x2,y2,theta2,v2) = util.turn_Right((x,y,theta,v),direction,d,radius)
        draw_point(x2,y2,theta2,'green')
        d+=10
    (x_temp,y_temp,theta_temp,v_temp) = util.turn_Right((x,y,theta,v),direction,distance,radius)
    draw_point(x_temp,y_temp,theta_temp,'green')
    return (x_temp,y_temp,theta_temp,v_temp)

def draw_turn_Left(x,y,theta,v,direction,distance,radius):
    d = 0
    while(d <= distance):
        (x2,y2,theta2,v2) = util.turn_Left((x,y,theta,v),direction,d,radius)
        draw_point(x2,y2,theta2,'green')
        d+=10
    (x_temp,y_temp,theta_temp,v_temp) = util.turn_Left((x,y,theta,v),direction,distance,radius)
    draw_point(x_temp,y_temp,theta_temp,'green')
    return (x_temp,y_temp,theta_temp,v_temp)

def draw_go_Straight(x,y,theta,v,direction,distance):
    d = 0
    while(d <= distance):
        (x2,y2,theta2,v2) = util.go_Straight((x,y,theta,v),direction,d)
        draw_point(x2,y2,theta2,'green')
        d+=10
    (x_temp,y_temp,theta_temp,v_temp) = util.go_Straight((x,y,theta,v),direction,distance)
    draw_point(x_temp,y_temp,theta_temp,'green')
    return (x_temp,y_temp,theta_temp,v_temp)

def draw_segment(state,action):
    """
    Given a state and an action, draw the segment corresponding to taking 'action' from 'state'
    """
    global canvas
    (x,y,theta,v) = state.get_stateparams()
    
    # Set direction of motion
    if action in ("B","L_b","R_b","B_diag","B1_26.6","B1_63.4"):
        direction = "b"
    else:
        direction = "f"
    
    if action in ("R_f","R_b"): # turning right
        draw_turn_Right(x,y,theta,v,direction,util.ROBOT_RADIUS_MIN*math.pi/2,util.ROBOT_RADIUS_MIN)        

    elif action in ("L_f","L_b"): # turning left
        draw_turn_Left(x,y,theta,v,direction,util.ROBOT_RADIUS_MIN*math.pi/2,util.ROBOT_RADIUS_MIN)
    
    elif action in ("F","B"):
        draw_go_Straight(x,y,theta,v,direction,util.CELL_SIZE)

    elif action in ("F3"):
        draw_go_Straight(x,y,theta,v,direction,3*util.CELL_SIZE)        

    elif action == "SL_f":
        (x_temp,y_temp,theta_temp,v_temp) = draw_go_Straight(x,y,theta,v,direction,20.5)
        draw_turn_Left(x_temp,y_temp,theta_temp,v_temp,direction,util.ROBOT_RADIUS_2*math.pi/4,util.ROBOT_RADIUS_2)

    elif action == "SR_f":
        (x_temp,y_temp,theta_temp,v_temp) = draw_go_Straight(x,y,theta,v,direction,20.5)
        draw_turn_Right(x_temp,y_temp,theta_temp,v_temp,direction,util.ROBOT_RADIUS_2*math.pi/4,util.ROBOT_RADIUS_2)        
    
    elif action == "LS_f":
        (x_temp,y_temp,theta_temp,v_temp) = draw_turn_Left(x,y,theta,v,direction,util.ROBOT_RADIUS_2*math.pi/4,util.ROBOT_RADIUS_2)
        draw_go_Straight(x_temp,y_temp,theta_temp,v_temp,direction,20.5)

    elif action == "RS_f":
        (x_temp,y_temp,theta_temp,v_temp) = draw_turn_Right(x,y,theta,v,direction,util.ROBOT_RADIUS_2*math.pi/4,util.ROBOT_RADIUS_2)
        draw_go_Straight(x_temp,y_temp,theta_temp,v_temp,direction,20.5)       

    elif action == "F_diag":
        draw_go_Straight(x,y,theta,v,direction,util.CELL_SIZE*math.sqrt(2))  
    
    elif action == "F_diag3":
        draw_go_Straight(x,y,theta,v,direction,3*util.CELL_SIZE*math.sqrt(2))  

    elif action == "B_diag":
        draw_go_Straight(x,y,theta,v,direction,util.CELL_SIZE*math.sqrt(2))

    elif action == "RS_f_short":
        (x_temp,y_temp,theta_temp,v_temp) = draw_turn_Right(x,y,theta,v,direction,util.ROBOT_RADIUS_3*math.pi/4,util.ROBOT_RADIUS_3)
        draw_go_Straight(x_temp,y_temp,theta_temp,v_temp,direction,14.4957)        
                
    elif action == "LS_f_short":
        (x_temp,y_temp,theta_temp,v_temp) = draw_turn_Left(x,y,theta,v,direction,util.ROBOT_RADIUS_3*math.pi/4,util.ROBOT_RADIUS_3)
        draw_go_Straight(x_temp,y_temp,theta_temp,v_temp,direction,14.4957)

    elif action == "sidestep_R_f":
        (x_temp,y_temp,theta_temp,v_temp) = draw_turn_Right(x,y,theta,v,direction,31.189,util.ROBOT_RADIUS_MIN)
        (x_temp,y_temp,theta_temp,v_temp) = draw_go_Straight(x_temp,y_temp,theta_temp,v_temp,direction,49.5)
        draw_turn_Left(x_temp,y_temp,theta_temp,v_temp,direction,31.189,util.ROBOT_RADIUS_MIN)       

    elif action == "sidestep_L_f":
        (x_temp,y_temp,theta_temp,v_temp) = draw_turn_Left(x,y,theta,v,direction,31.189,util.ROBOT_RADIUS_MIN)
        (x_temp,y_temp,theta_temp,v_temp) = draw_go_Straight(x_temp,y_temp,theta_temp,v_temp,direction,49.5)
        draw_turn_Right(x_temp,y_temp,theta_temp,v_temp,direction,31.189,util.ROBOT_RADIUS_MIN) 

    elif action == "L2_f":
        draw_turn_Left(x,y,theta,v,direction,util.ROBOT_RADIUS_4*math.pi/2,util.ROBOT_RADIUS_4)    
    
    elif action == "R2_f":
        draw_turn_Right(x,y,theta,v,direction,util.ROBOT_RADIUS_4*math.pi/2,util.ROBOT_RADIUS_4)        
            
    elif action == "SR_f_2":
        (x_temp,y_temp,theta_temp,v_temp) = draw_go_Straight(x,y,theta,v,direction,13.369)x        draw_turn_Right(x_temp,y_temp,theta_temp,v_temp,direction,(63.4/360.0)*2*math.pi*util.ROBOT_RADIUS_5,util.ROBOT_RADIUS_5)      

    elif action == "SL_f_2":
        (x_temp,y_temp,theta_temp,v_temp) = draw_go_Straight(x,y,theta,v,direction,13.369)
        draw_turn_Left(x_temp,y_temp,theta_temp,v_temp,direction,(63.4/360.0)*2*math.pi*util.ROBOT_RADIUS_5,util.ROBOT_RADIUS_5)        

    elif action == "RSR_f":
        (x_temp,y_temp,theta_temp,v_temp) = draw_turn_Right(x,y,theta,v,direction,13.3968,util.ROBOT_RADIUS_5)
        (x_temp,y_temp,theta_temp,v_temp) = draw_go_Straight(x_temp,y_temp,theta_temp,v,direction,53.9857)
        draw_turn_Right(x_temp,y_temp,theta_temp,v_temp,direction,22.2,util.ROBOT_RADIUS_5)       
        
    elif action == "LSL_f":
        (x_temp,y_temp,theta_temp,v_temp) = draw_turn_Left(x,y,theta,v,direction,13.3968,util.ROBOT_RADIUS_5)
        (x_temp,y_temp,theta_temp,v_temp) = draw_go_Straight(x_temp,y_temp,theta_temp,v,direction,53.9857)
        draw_turn_Left(x_temp,y_temp,theta_temp,v_temp,direction,22.2,util.ROBOT_RADIUS_5)       
            
    elif action in ("F1_26.6","B1_26.6","F1_63.4","B1_63.4"):
        draw_go_Straight(x,y,theta,v,direction,39.131)

    elif action == "LSL1_f_26.6":
        (x_temp,y_temp,theta_temp,v_temp) = draw_turn_Left(x,y,theta,v,direction,41.95,util.ROBOT_RADIUS_MIN)
        (x_temp,y_temp,theta_temp,v_temp) = draw_go_Straight(x_temp,y_temp,theta_temp,v,direction,28.496)
        draw_turn_Left(x_temp,y_temp,theta_temp,v_temp,direction,35.508,util.ROBOT_RADIUS_MIN)        

    elif action == "LSL2_f_26.6":
        (x_temp,y_temp,theta_temp,v_temp) = draw_turn_Left(x,y,theta,v,direction,12.99,util.ROBOT_RADIUS_MIN)
        (x_temp,y_temp,theta_temp,v_temp) = draw_go_Straight(x_temp,y_temp,theta_temp,v,direction,65.6)
        draw_turn_Left(x_temp,y_temp,theta_temp,v_temp,direction,9.487,util.ROBOT_RADIUS_MIN)       

    elif action == "RSR1_f_26.6":
        (x_temp,y_temp,theta_temp,v_temp) = draw_turn_Right(x,y,theta,v,direction,14.625,util.ROBOT_RADIUS_MIN)
        (x_temp,y_temp,theta_temp,v_temp) = draw_go_Straight(x_temp,y_temp,theta_temp,v,direction,39.952)
        draw_turn_Right(x_temp,y_temp,theta_temp,v_temp,direction,17.873,util.ROBOT_RADIUS_MIN)

    elif action == "RSR2_f_26.6":
        (x_temp,y_temp,theta_temp,v_temp) = draw_turn_Right(x,y,theta,v,direction,84.055,util.ROBOT_RADIUS_MIN)
        (x_temp,y_temp,theta_temp,v_temp) = draw_go_Straight(x_temp,y_temp,theta_temp,v,direction,32.613)
        draw_turn_Right(x_temp,y_temp,theta_temp,v_temp,direction,3.42,util.ROBOT_RADIUS_MIN)
        
    elif action == "LSL1_f_63.4":
        (x_temp,y_temp,theta_temp,v_temp) = draw_turn_Left(x,y,theta,v,direction,14.625,util.ROBOT_RADIUS_MIN)
        (x_temp,y_temp,theta_temp,v_temp) = draw_go_Straight(x_temp,y_temp,theta_temp,v,direction,39.952)
        draw_turn_Left(x_temp,y_temp,theta_temp,v_temp,direction,17.783,util.ROBOT_RADIUS_MIN)        

    elif action == "LSL2_f_63.4":
        (x_temp,y_temp,theta_temp,v_temp) = draw_turn_Left(x,y,theta,v,direction,84.055,util.ROBOT_RADIUS_MIN)
        (x_temp,y_temp,theta_temp,v_temp) = draw_go_Straight(x_temp,y_temp,theta_temp,v,direction,32.613)
        draw_turn_Left(x_temp,y_temp,theta_temp,v_temp,direction,3.42,util.ROBOT_RADIUS_MIN)

    elif action == "RSR1_f_63.4":
        (x_temp,y_temp,theta_temp,v_temp) = draw_turn_Right(x,y,theta,v,direction,41.95,util.ROBOT_RADIUS_MIN)
        (x_temp,y_temp,theta_temp,v_temp) = draw_go_Straight(x_temp,y_temp,theta_temp,v,direction,28.496)
        draw_turn_Right(x_temp,y_temp,theta_temp,v_temp,direction,35.508,util.ROBOT_RADIUS_MIN)

    elif action == "RSR2_f_63.4":
        (x_temp,y_temp,theta_temp,v_temp) = draw_turn_Right(x,y,theta,v,direction,12.99,util.ROBOT_RADIUS_MIN)
        (x_temp,y_temp,theta_temp,v_temp) = draw_go_Straight(x_temp,y_temp,theta_temp,v,direction,65.6)
        draw_turn_Right(x_temp,y_temp,theta_temp,v_temp,direction,9.487,util.ROBOT_RADIUS_MIN)
        
    
