"""
Just to visualize motion swaths :)
"""

import math
import util
import time
from Tkinter import *

ROBOT_LENGTH = util.ROBOT_LENGTH
ROBOT_WIDTH  = util.ROBOT_WIDTH
ROBOT_RADIUS_MIN = util.ROBOT_RADIUS_MIN
ROBOT_RADIUS_2   = util.ROBOT_RADIUS_2
ROBOT_SPEED_MAX = util.ROBOT_SPEED_MAX

CELL_SIZE = util.CELL_SIZE
canv = None
cm_to_pixels = 600.0/280.0

def draw_lattice(canv):
    """
    Draw the Lattice as a background
    """
    global cm_to_pixels
    cellsize = util.CELL_SIZE
    # draw vertical lines
    d = 0
    while(d <= 800/cm_to_pixels):
        canv.create_line(d*cm_to_pixels,0,d*cm_to_pixels,600,width=1,fill = 'black',)
        canv.create_line(0,600-d*cm_to_pixels,600,600-d*cm_to_pixels,width=1,fill = 'black')
        d += cellsize
        
def draw_car(x,y,th,canv):    
    r = math.sqrt(math.pow(ROBOT_LENGTH,2) + math.pow(ROBOT_WIDTH,2))/2    
    x = x + 300.0/cm_to_pixels
    y = y + 300.0/cm_to_pixels
    
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
    
    canv.create_polygon(topleft[0]*cm_to_pixels,600 - topleft[1]*cm_to_pixels,
                        bottomleft[0]*cm_to_pixels,600 - bottomleft[1]*cm_to_pixels,
                        bottomright[0]*cm_to_pixels,600 - bottomright[1]*cm_to_pixels,
                        topright[0]*cm_to_pixels,600 - topright[1]*cm_to_pixels,
                        width = 1, outline = 'blue',fill = '')
    x1 = x*cm_to_pixels
    y1 = y*cm_to_pixels
    canv.create_oval(x1-1,600-(y1-1),x1+1,600-(y1+1),outline = 'green',fill = 'green')


def segment(point,action,canv):
    """
    Given a state and an action, draw the segment corresponding to taking 'action' from 'state'
    """
    (x,y,theta) = point
    v = 0
    
    # Set direction of motion
    if action in ("R_f","L_f","F","F3","SR_f","SL_f","LS_f","RS_f","F_diag","F_diag3"):
        direction = "f"
    else:
        direction = "b"
    
    if action in ("R_f","R_b"): # turning right
        d = 0
        while(d <= util.ROBOT_RADIUS_MIN*math.pi/2):
            (x2,y2,theta2,v2) = util.turn_Right((x,y,theta,v),direction,d,util.ROBOT_RADIUS_MIN)
            car = draw_car(x2,y2,theta2,canv)            
            d+=10

    elif action in ("L_f","L_b"): # turning left
        d = 0
        while(d <= util.ROBOT_RADIUS_MIN*math.pi/2):
            (x2,y2,theta2,v2) = util.turn_Left((x,y,theta,v),direction,d,util.ROBOT_RADIUS_MIN)
            draw_car(x2,y2,theta2,canv)
            d +=10
    
    elif action in ("F","B"):
        d = 0
        while(d <= util.CELL_SIZE):
            (x2,y2,theta2,v2) = util.go_Straight((x,y,theta,v),direction,d)
            draw_car(x2,y2,theta2,canv)
            d +=10

    elif action in ("F3"):
        d = 0
        while(d <= 3*util.CELL_SIZE):
            (x2,y2,theta2,v2) = util.go_Straight((x,y,theta,v),direction,d)
            draw_car(x2,y2,theta2,canv)
            d +=10

    elif action == "SL_f":
        d = 0
        while(d <= 20.5/70.0*util.ROBOT_RADIUS_MIN):
            (x2,y2,theta2,v2) =  util.go_Straight((x,y,theta,v),direction,d)
            draw_car(x2,y2,theta2,canv)
            d += 10
        d = 0
        (x_temp,y_temp,theta_temp,v_temp) = util.go_Straight((x,y,theta,v),direction,20.5/70.0*util.ROBOT_RADIUS_MIN)
        while(d <= util.ROBOT_RADIUS_2*math.pi/4):
            (x2,y2,theta2,v2) = util.turn_Left((x_temp,y_temp,theta_temp,v_temp),direction,d,util.ROBOT_RADIUS_2)
            draw_car(x2,y2,theta2,canv)
            d += 10

    elif action == "SR_f":
        d = 0
        while(d <= 20.5/70.0*util.ROBOT_RADIUS_MIN):
            (x2,y2,theta2,v2) =  util.go_Straight((x,y,theta,v),direction,d)
            draw_car(x2,y2,theta2,canv)
            d += 10
        d = 0
        (x_temp,y_temp,theta_temp,v_temp) = util.go_Straight((x,y,theta,v),direction,20.5/70.0*util.ROBOT_RADIUS_MIN)
        while(d <= util.ROBOT_RADIUS_2*math.pi/4):
            (x2,y2,theta2,v2) = util.turn_Right((x_temp,y_temp,theta_temp,v_temp),direction,d,util.ROBOT_RADIUS_2)
            draw_car(x2,y2,theta2,canv)
            d += 10
    
    elif action == "LS_f":
        d = 0
        while(d <= util.ROBOT_RADIUS_2*math.pi/4):
            (x2,y2,theta2,v2) =  util.turn_Left((x,y,theta,v),direction,d,util.ROBOT_RADIUS_2)
            draw_car(x2,y2,theta2,canv)
            d += 10
        d = 0
        (x_temp,y_temp,theta_temp,v_temp) = util.turn_Left((x,y,theta,v),direction,util.ROBOT_RADIUS_2*math.pi/4,util.ROBOT_RADIUS_2)
        while(d <= 20.5/70.0*util.ROBOT_RADIUS_MIN):
            (x2,y2,theta2,v2) = util.go_Straight((x_temp,y_temp,theta_temp,v_temp),direction,d)
            draw_car(x2,y2,theta2,canv)
            d += 10

    elif action == "RS_f":
        d = 0
        while(d <= util.ROBOT_RADIUS_2*math.pi/4):
            (x2,y2,theta2,v2) =  util.turn_Right((x,y,theta,v),direction,d,util.ROBOT_RADIUS_2)
            draw_car(x2,y2,theta2,canv)
            d += 10
        d = 0
        (x_temp,y_temp,theta_temp,v_temp) = util.turn_Right((x,y,theta,v),direction,util.ROBOT_RADIUS_2*math.pi/4,util.ROBOT_RADIUS_2)
        while(d <= 20.5/70.0*util.ROBOT_RADIUS_MIN):
            (x2,y2,theta2,v2) = util.go_Straight((x_temp,y_temp,theta_temp,v_temp),direction,d)
            draw_car(x2,y2,theta2,canv)
            d += 10

    elif action == "F_diag":
        d = 0
        while(d <= util.CELL_SIZE*math.sqrt(2)):
            (x2,y2,theta2,v2) = util.go_Straight((x,y,theta,v),direction,d)
            draw_car(x2,y2,theta2,canv)
            d +=10
    
    elif action == "F_diag3":
        d = 0
        while(d <= 3*util.CELL_SIZE*math.sqrt(2)):
            (x2,y2,theta2,v2) = util.go_Straight((x,y,theta,v),direction,d)
            draw_car(x2,y2,theta2,canv)
            d +=10

    elif action == "B_diag":
        d = 0
        while(d <= util.CELL_SIZE*math.sqrt(2)):
            (x2,y2,theta2,v2) = util.go_Straight((x,y,theta,v),direction,d)
            draw_car(x2,y2,theta2,canv)
            d +=10
            


def main():
    global canv
    root = Tk()
    root.title("Swath computation")    
    #root2 = Tk()
    #root3 = Tk()
    #root4 = Tk()
    #root5 = Tk()
    canv = Canvas(root,width = 600, height = 600,bg = 'white')
    
    """
    canv2 = Canvas(root,width = 600, height = 600,bg = 'white')
    canv3 = Canvas(root2,width = 600, height = 600,bg = 'white')
    canv4 = Canvas(root2,width = 600, height = 600,bg = 'white')
    canv5 = Canvas(root3,width = 600, height = 600,bg = 'white')
    canv6 = Canvas(root3,width = 600, height = 600,bg = 'white')
    canv7 = Canvas(root4,width = 600, height = 600,bg = 'white')
    canv8 = Canvas(root4,width = 600, height = 600,bg = 'white')
    canv9 = Canvas(root5,width = 600, height = 600,bg = 'white')
    """
    canv.pack(side = LEFT)
    """
    canv2.pack(side = LEFT)
    canv3.pack(side = LEFT)
    canv4.pack(side = LEFT)
    canv5.pack(side = LEFT)
    canv6.pack(side = LEFT)
    canv7.pack(side = LEFT)
    canv8.pack(side = LEFT)
    canv9.pack(side = LEFT)
    """

    """
    draw_lattice(canv2)
    draw_lattice(canv3)
    draw_lattice(canv4)
    draw_lattice(canv5)
    draw_lattice(canv6)
    draw_lattice(canv7)
    draw_lattice(canv8)
    draw_lattice(canv9)
    """
    """
    startNode = (0,0,math.pi/2)   
    segment(startNode,"R_f",canv)        
    segment(startNode,"F",canv2)    
    segment(startNode,"F3",canv3)
    segment(startNode,"B",canv4)
    segment(startNode,"R_b",canv5)
    
    segment(startNode,"L_f",canv6)
    segment(startNode,"L_b",canv7)
    segment(startNode,"SR_f",canv8)
    segment(startNode,"SL_f",canv9)    
    """
    startNode2 = (0,0,math.pi/4)
    segment(startNode2,"RS_f",canv)
    segment(startNode2,"LS_f",canv)
    segment(startNode2,"F_diag",canv)
    segment(startNode2,"F_diag3",canv)
    segment(startNode2,"B_diag",canv)
    root.mainloop()

main()
