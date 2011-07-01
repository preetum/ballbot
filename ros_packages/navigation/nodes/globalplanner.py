#!/usr/bin/env python
# Global planner - based on RRTs and Dubins curves. Simulated in globalplanner_sim.py
# Returns a plan that looks like [[action1,distance1],[action2,distance2]....] where distance is in cm, action is one of L,S,R
"""
Map definition: 
total size = 12m x 6m
one obstacle in rectangle (1.2,5.95,4.8,6.05)

Each Tree is a list of vertices. 
Each Node is [(x,y,theta),[(neighbor1,action1),(neighbor2,action2)....]


Scaling calculations:
Turning radius of car = 1.91 feet
Length of court = 120 feet = 600 Tk units

User inputs in feet.
All Computation happens using the scale - Turning radius = 1
i.e. 1 computation unit = 1.91 feet
"""

import math
import sys
import time
import util
import RRTNode
import dubins

Tree_start_to_right  = []
Tree_start_to_left = []
Tree_right_to_goal = []
Tree_left_to_goal = []
Tree_start_to_goal = []

ROBOT_RADIUS  = util.ROBOT_RADIUS  # 2.283ft = 69.6cm

WORLD_LENGTH_ft = 120 # feet
WORLD_WIDTH_ft  = 60  # feet

# -------------------------------------------------------------------------------------------------- #

def plan(X,Y,TH,X2,Y2,TH2,Tree):
  t1 = time.time()
  RRTNode.RRT_goalfound = False
  query_node = RRTNode.build_RRT(X,Y,TH,X2,Y2,TH2,Tree)

  if(query_node == False):
    return False

  t1 = time.time() - t1
  count = 0

  t2 = time.time()
  path = RRTNode.query_RRT(query_node,Tree)
  path.reverse()
  count = len(path)
  #print "finish searching RRT for solution"

  RRT_goal_node = path[-1]
  RRT_goal = RRT_goal_node.getVertex()
  goal     = (X2,Y2,TH2)
  
  # --------- Compute arguments for dubins curve -----------------#
  (alpha,beta,d) = dubins.dubinsparams_calc(RRT_goal,(X2,Y2,TH2))
  (curve_number,t,p,q) = dubins.dubins(alpha,beta,d,RRT_goal_node)
  
  t2 = time.time() - t2

  #print "Dubins ",curve_number," t ",t," p ",p," q ",q
   
  drive_cmds = util.path_to_drivecmds(path) 
  drive_cmds_dubins = dubins.dubins_to_drivecmds(curve_number,t,p,q)
  drive_cmds = drive_cmds + drive_cmds_dubins
  for element in drive_cmds:
    print element[0],element[1]
  print "\n\n\n"
  count = count + t + p + q 
  return (drive_cmds,count)

def startPlanner(X1,Y1,TH1,D_goal,TH_goal):
  global Tree_start_to_right,Tree_start_to_left,Tree_right_to_goal,Tree_right_to_left,Tree_start_to_goal
  D_goal = data.d
  TH_goal = data.th
  X1 = cm_to_feet(X1)
  Y1 = cm_to_feet(Y1)
  
  Tree_start_to_right = []
  Tree_right_to_goal = []
  Tree_start_to_left = []
  Tree_left_to_goal = []
  Tree_start_to_goal = []
  path_start_to_right = []
  path_right_to_goal = []
  path_start_to_left = []
  path_left_to_goal = []
  
  #canvas.delete(ALL)
  #util.drawCourt(canvas)

  # compute start and end points in the global coordinate frame.

  X = float(X1)
  Y = float(Y1)
  TH1 = math.radians(float(TH1))
  
  D_goal = util.cm_to_feet(float(D_goal))
  TH_goal = float(TH_goal)
  TH = TH1 - math.radians(TH_goal)            # angle to goal in global coord
                                              # = angle of car in global coord - angle to goal from car (which is in [-90deg,90deg]
  X2 = X + D_goal*math.cos(TH)
  Y2 = Y + D_goal*math.sin(TH)
  TH2 = math.radians(45.0)

  # rescale X and Y to Computation coordinate frame (Turning radius = 1)
  X = X/ROBOT_RADIUS
  Y = Y/ROBOT_RADIUS
  TH = TH1
  X2 = X2/ROBOT_RADIUS
  Y2 = Y2/ROBOT_RADIUS
  TH2 = math.radians(45.0)
  
  if not((util.newState((X,Y,TH))) and (util.newState((X2,Y2,TH2)))):
    print "Invalid start and/or goal configuration"
    return

  # -----------  Check which case the start-goal configuration falls under  ---------------------
  # Case 1: start below and goal above => angle at net = pi/2
  # Case 2: start above and goal below => angle at net = 3*pi/2
  # Case 3: start and goal at same side of net => dont check net at all!

  t1 = time.time()

  if((Y <= 60/ROBOT_RADIUS) and (Y2 >= 60/ROBOT_RADIUS)):
    case = 1
  elif((Y >= 60/ROBOT_RADIUS) and (Y2 <= 60/ROBOT_RADIUS)):
    case = 2
  else:
    case = 3

  if(case == 1):
    angle_net = math.pi/2
  elif(case == 2):
    angle_net = 3*math.pi/2
 
  if((case == 1) or (case == 2)):
      # --------- find path through right side of net --------- #  
    plan_start_to_right =  plan(canvas,X,Y,TH,50/ROBOT_RADIUS,60/ROBOT_RADIUS,angle_net,Tree_start_to_right)
    plan_right_to_goal = plan(canvas,50/ROBOT_RADIUS,60/ROBOT_RADIUS,angle_net,X2,Y2,TH2,Tree_right_to_goal)

    if(plan_start_to_right != False) and (plan_right_to_goal != False):
      plan_right = (plan_start_to_right[0] + plan_right_to_goal[0],plan_start_to_right[1] + plan_right_to_goal[1])
    else:
      plan_right = False

    # -------- find path through left side of net ----------- #
    plan_start_to_left = plan(X,Y,TH,10/ROBOT_RADIUS,60/ROBOT_RADIUS,angle_net,Tree_start_to_left)
    plan_left_to_goal = plan(10/ROBOT_RADIUS,60/ROBOT_RADIUS,angle_net,X2,Y2,TH2,Tree_left_to_goal)
    
    if(plan_start_to_left != False) and (plan_left_to_goal!= False):
      plan_left = (plan_start_to_left[0] + plan_left_to_goal[0],plan_start_to_left[1] + plan_left_to_goal[1])
    else:
      plan_left = False

    # --------- choose shortest path ------------------------ #
    
    if (plan_left == False):
      finalplan = plan_right
    if (plan_right == False):
      finalplan = plan_left
    if (plan_left != False) and (plan_right != False):
      if(plan_left[1]) < (plan_right[1]):
        finalplan = plan_left
      else:
        finalplan = plan_right      

  elif(case == 3):
      # ball is on same side of net, so try to reach it directly
      finalplan = plan(X,Y,TH,X2,Y2,TH2,Tree_start_to_goal)
      

  t1 = time.time() - t1
 #---------- draw path -------------- #
  
  print("Time to complete Planning ",t1," s case",case)
  i = 0
  
  return finalplan
  
