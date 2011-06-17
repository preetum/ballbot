"""
 dubins.py : Calculates the minimum length dubins curve
 Usage     : python dubins.py x_start y_start alpha x_end y_end beta

"""
import sys
import math
from util import *

def fmodr(x,y):
    return x - y*math.floor(x/y)

def mod2pi(theta):
    return fmodr(theta,2*math.pi)

def dubins(alpha,beta,d,start_p):
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
    

    # find length of curve 1: Left Straight Left                     [Lq(Sp(Lt(0,0,alpha))) = (d,0,beta)]
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


    # find length of curve 2: Right Straight Right                    [Rq(Sp(Rt(0,0,alpha))) = (d,0,beta)]
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

    # find length of curve 3: Left Straight Right                     [Rq(Sp(Lt(0,0,alpha))) = (d,0,beta)]
    tmp1 = -2 + (d*d) + (2*cos_alphaminusbeta) + (2*d*(sin_alpha + sin_beta))
    if(tmp1 >= 0):
        p[2] = math.sqrt(tmp1)
        tmp2 = math.atan((-cos_alpha-cos_beta)/(d+sin_alpha+sin_beta)) - math.atan(-2.0/p[2])
        t[2] = mod2pi(-alpha + tmp2)
        q[2] = mod2pi( - mod2pi(beta) + tmp2)
        lengths[2] = t[2] + p[2] + q[2]
    #print "LSR ",lengths[2]
    #print "t = ",t[2]," p = ",p[2]," q = ",q[2]

    # find length of curve 4: Right Straight Left                      [Lq(Sp(Rt(0,0,alpha))) = (d,0,beta)]
    tmp1 = (d*d) - 2.0 + (2*cos_alphaminusbeta) - (2*d*(sin_alpha + sin_beta))
    if(tmp1 > 0):
        p[3] = math.sqrt(tmp1)
        tmp2 = math.atan((cos_alpha + cos_beta)/(d - sin_alpha - sin_beta)) - math.atan(2.0/p[3])
        t[3] = mod2pi(alpha - tmp2)
        q[3] = mod2pi(beta - tmp2)
        lengths[3] = t[3] + p[3] + q[3]
    #print "RSL ",lengths[3]
    #print "t = ",t[3]," p = ",p[3]," q = ",q[3]

    # find length of curve 5: Right Left Right                         [Rq(Lp(Rt(0,0,alpha))) = (d,0,beta)]
    tmp_rlr = (6.0 - d*d + 2.0*cos_alphaminusbeta + 2.0 * d * (sin_alpha - sin_beta))/8.0
    if(math.fabs(tmp_rlr) < 1):
        p[4] = math.acos(tmp_rlr)
        t[4] = mod2pi(alpha - math.atan2((cos_alpha - cos_beta),(d-sin_alpha + sin_beta)) + mod2pi(p[4]/2.0))
        q[4] = mod2pi(alpha - beta - t[4] + mod2pi(p[4]))
        lengths[4] = t[4] + p[4] + q[4]
    #print "RLR ",lengths[4]
    #print "t = ",t[4]," p = ",p[4]," q = ",q[4]
    
    # find length of curve 6: Left Right Left                          [Lq(Rp(Lt(0,0,alpha))) = (d,0,beta)]
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
    
    if(isSafeDubins(curve_number,t[curve_number],p[curve_number],q[curve_number],start_p)):
        #print "Curve with min length is",curve_number
        return (curve_number,t[curve_number],p[curve_number],q[curve_number]) 
    else:
        return (-1,-1,-1,-1)

def isSafeDubins(curve_number,t,p,q,start_p):
    """
    Given a start position and a Dubins curve, check the generated path for collisions.
    Return true if there's no collision and false if there is
    """
    v = 0.1

    (x,y,theta) = start_p.getVertex()
    (startx,starty,starttheta) = (x,y,theta)
    #print (x,y,theta)," curve_number ",curve_number," t ",t," p ",p," q ",q

    # first action

    if((curve_number == 0) or (curve_number == 2) or (curve_number == 5)):
    #turn left
        action = turnLeft

    elif((curve_number == 1) or (curve_number == 3) or (curve_number == 4)):
    #turn right
        action = turnRight

    i = 0
    while(i < t):
        (x,y,theta) = action(x,y,theta,v)
        point = (x,y,theta)
        if not (newState(point)):
            #print point,"     False"
            return False
        #print point,"     True"
        i = i + v

    #print "first action OK, ends at ",(x,y,theta)
    (x,y,theta) = action(startx,starty,starttheta,t)
    (startx,starty,starttheta) = (x,y,theta)
    # second action
        
    if (curve_number <= 3):
        # go forward
        action = goForward

    elif(curve_number == 4):
        # turn left
        action = turnLeft

    elif(curve_number == 5):
        # turn right
        action = turnRight

    i = 0
    while(i < p):
        (x,y,theta) = action(x,y,theta,v)
        point = (x,y,theta)
        if not (newState(point)):
            #print point,"     False"
            return False
        #print point,"     True"
        i = i+ v
    
    (x,y,theta) = action(startx,starty,starttheta,p)
    (startx,starty,starttheta) = (x,y,theta)

    # third action
    if(curve_number == 0) or (curve_number == 3) or (curve_number == 5):
        # turn left
        action = turnLeft

    elif(curve_number == 1) or (curve_number == 2) or (curve_number == 4):
        # turn right
        action = turnRight

    i = 0
    while(i < q):
        (x,y,theta) = action(x,y,theta,v)
        point = (x,y,theta)
        if not (newState(point)):
            #print point,"     False"
            return False
        i = i + v

   # print "Third action OK"

    return True

