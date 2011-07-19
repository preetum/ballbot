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

def draw_rectangle(x1,y1,x2,y2,color = 'blue'):
    """
    Draw a rectangle on the canvas. (x,y) in cm
    """
    global canvas,cm_to_pixels
    x1 = x1*cm_to_pixels
    y1 = 600 - y1*cm_to_pixels
    x2 = x2*cm_to_pixels
    y2 = 600 - y2*cm_to_pixels

    canvas.create_rectangle(x1,y1,x2,y2,width=1,outline = color,fill = color)

def draw_segment(state,action):
    """
    Given a state and an action, draw the segment corresponding to taking 'action' from 'state'
    """
    global canvas
    (x,y,theta,v) = state.get_stateparams()
    
    # Set direction of motion
    if action in ("R_f","L_f","F","F3","SR_f","SL_f","LS_f","RS_f","F_diag","F_diag3"):
        direction = "f"
    else:
        direction = "b"
    
    if action in ("R_f","R_b"): # turning right
        d = 0
        while(d <= util.ROBOT_RADIUS_MIN*math.pi/2):
            (x2,y2,theta2,v2) = util.turn_Right((x,y,theta,v),direction,d,util.ROBOT_RADIUS_MIN)
            draw_point(x2,y2,theta2,'green')
            d+=10

    elif action in ("L_f","L_b"): # turning left
        d = 0
        while(d <= util.ROBOT_RADIUS_MIN*math.pi/2):
            (x2,y2,theta2,v2) = util.turn_Left((x,y,theta,v),direction,d,util.ROBOT_RADIUS_MIN)
            draw_point(x2,y2,theta2,'green')
            d +=10
    
    elif action in ("F","B"):
        d = 0
        while(d <= util.CELL_SIZE):
            (x2,y2,theta2,v2) = util.go_Straight((x,y,theta,v),direction,d)
            draw_point(x2,y2,theta2,'green')
            d +=10

    elif action in ("F3"):
        d = 0
        while(d <= 3*util.CELL_SIZE):
            (x2,y2,theta2,v2) = util.go_Straight((x,y,theta,v),direction,d)
            draw_point(x2,y2,theta2,'green')
            d +=10

    elif action == "SL_f":
        d = 0
        while(d <= 20.5/70.0*util.ROBOT_RADIUS_MIN):
            (x2,y2,theta2,v2) =  util.go_Straight((x,y,theta,v),direction,d)
            draw_point(x2,y2,theta2,'green')
            d += 10
        d = 0
        (x_temp,y_temp,theta_temp,v_temp) = util.go_Straight((x,y,theta,v),direction,20.5/70.0*util.ROBOT_RADIUS_MIN)
        while(d <= util.ROBOT_RADIUS_2*math.pi/4):
            (x2,y2,theta2,v2) = util.turn_Left((x_temp,y_temp,theta_temp,v_temp),direction,d,util.ROBOT_RADIUS_2)
            draw_point(x2,y2,theta2,'green')
            d += 10

    elif action == "SR_f":
        d = 0
        while(d <= 20.5/70.0*util.ROBOT_RADIUS_MIN):
            (x2,y2,theta2,v2) =  util.go_Straight((x,y,theta,v),direction,d)
            draw_point(x2,y2,theta2,'green')
            d += 10
        d = 0
        (x_temp,y_temp,theta_temp,v_temp) = util.go_Straight((x,y,theta,v),direction,20.5/70.0*util.ROBOT_RADIUS_MIN)
        while(d <= util.ROBOT_RADIUS_2*math.pi/4):
            (x2,y2,theta2,v2) = util.turn_Right((x_temp,y_temp,theta_temp,v_temp),direction,d,util.ROBOT_RADIUS_2)
            draw_point(x2,y2,theta2,'green')
            d += 10
    
    elif action == "LS_f":
        d = 0
        while(d <= util.ROBOT_RADIUS_2*math.pi/4):
            (x2,y2,theta2,v2) =  util.turn_Left((x,y,theta,v),direction,d,util.ROBOT_RADIUS_2)
            draw_point(x2,y2,theta2,'green')
            d += 10
        d = 0
        (x_temp,y_temp,theta_temp,v_temp) = util.turn_Left((x,y,theta,v),direction,util.ROBOT_RADIUS_2*math.pi/4,util.ROBOT_RADIUS_2)
        while(d <= 20.5/70.0*util.ROBOT_RADIUS_MIN):
            (x2,y2,theta2,v2) = util.go_Straight((x_temp,y_temp,theta_temp,v_temp),direction,d)
            draw_point(x2,y2,theta2,'green')
            d += 10

    elif action == "RS_f":
        d = 0
        while(d <= util.ROBOT_RADIUS_2*math.pi/4):
            (x2,y2,theta2,v2) =  util.turn_Right((x,y,theta,v),direction,d,util.ROBOT_RADIUS_2)
            draw_point(x2,y2,theta2,'green')
            d += 10
        d = 0
        (x_temp,y_temp,theta_temp,v_temp) = util.turn_Right((x,y,theta,v),direction,util.ROBOT_RADIUS_2*math.pi/4,util.ROBOT_RADIUS_2)
        while(d <= 20.5/70.0*util.ROBOT_RADIUS_MIN):
            (x2,y2,theta2,v2) = util.go_Straight((x_temp,y_temp,theta_temp,v_temp),direction,d)
            draw_point(x2,y2,theta2,'green')
            d += 10

    elif action == "F_diag":
        d = 0
        while(d <= util.CELL_SIZE*math.sqrt(2)):
            (x2,y2,theta2,v2) = util.go_Straight((x,y,theta,v),direction,d)
            draw_point(x2,y2,theta2,'green')
            d +=10
    
    elif action == "F_diag3":
        d = 0
        while(d <= 3*util.CELL_SIZE*math.sqrt(2)):
            (x2,y2,theta2,v2) = util.go_Straight((x,y,theta,v),direction,d)
            draw_point(x2,y2,theta2,'green')
            d +=10

    elif action == "B_diag":
        d = 0
        while(d <= util.CELL_SIZE*math.sqrt(2)):
            (x2,y2,theta2,v2) = util.go_Straight((x,y,theta,v),direction,d)
            draw_point(x2,y2,theta2,'green')
            d +=10
         

            
                 
