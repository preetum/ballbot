# Investigate RRTs
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
import random
import heapq
import time
from dubins import *
from util import *
from Tkinter import *


Tree_start_to_right  = []
Tree_start_to_left = []
Tree_right_to_goal = []
Tree_left_to_goal = []
Tree_start_to_goal = []

RRT_goalfound = False
  
class RRTNode:
  """
  Stores a node of the RRT.
  Each vertex contains (x,y,theta)
  Neighbors are a list of [(RRTnode1, action1), (RRTnode2,action2)....]
  """

  def __init__(self,vertex):
    self.vertex = vertex
    self.neighbors = []
    self.parent = None
    self.actions = []
    self.CVF = 0
    self.cost = 0

  def getVertex(self):
    """
    Takes a node and returns the vertex (x,y,theta)
    """
    return self.vertex

  def getNeighbors(self):
    """
    Takes a node and returns a list of neighbors and action to reach the neighbor
    """
    return self.neighbors

  def addNeighbor(self,node,action):
    """
    Adds (node,action) to the list of neighbors
    """
    self.neighbors.append((node,action))
  
  def addParent(self,parent):
    self.parent = parent

  def getParent(self):
    return self.parent

  def display(self):
    if not (self.parent == None):
      print "vertex ",self.vertex,"parent ",self.parent.getVertex()
    else:
      print "vertex ",self.vertex,"parent ",self.parent

  def addAction(self,action):
    if action in self.actions:
      return False
    else:
      self.actions.append(action)
      return True

  def updateCVF(self,CVF):
    self.CVF = self.CVF + CVF

  def getCVF(self):
    return self.CVF

  def updateCost(self,cost):
    self.cost = self.cost + cost
    
  def getCost(self):
    return self.cost
# -------------------------------------------------------------------------------------------------- #

def addVertex(tree,p_new,p_near,u_new):

  # find p_near and append p_new as a neighbor
  new_Node = RRTNode(p_new)

  p_near.addNeighbor(new_Node,u_new)
  new_Node.addParent(p_near)
  
  parentCost = p_near.getCost()
  newCost = parentCost + 0.5
  new_Node.updateCost(newCost)

  #  add p_new as a vertex and return a reference to it
  tree.append(new_Node)
  return new_Node


def distance_Euclidian(p1,p2):
    """
    returns square of the Euclidian distance, for speed
    """
    return (p1[0] - p2[0])**2 + (p1[1] - p2[1])**2
    

def nearestNeighbor(p,Tree):
  """
  returns the node which has shortest euclidian distance to the tree
  """
  min_d = distance_Euclidian(p,Tree[0].getVertex())
  nN = Tree[0]

  for node in Tree:
    d = distance_Euclidian(p,node.getVertex())
    if (d < min_d):
      nN = node
      min_d = d

  return nN

def nearestNeighbor_Dubins(p,Tree):
  """ 
  returns the node which has shortest dubins distance to p
  """
  # --------- Compute arguments for dubins curve -----------------#
  d = math.sqrt(distance_Euclidian(p,Tree[0].getVertex()))

  angle_goal = math.atan2(p[1] - Tree[0].getVertex()[1],p[0] - Tree[0].getVertex()[0])
  if(angle_goal < 0):
    angle_goal = 2*math.pi + angle_goal

  alpha = Tree[0].getVertex()[2] - angle_goal
  if(alpha < 0):
    alpha = 2*math.pi + alpha

  beta = math.atan2(p[1] - Tree[0].getVertex()[1], p[0] - Tree[0].getVertex()[0])
  if (beta < 0):
    beta = 2* math.pi + beta
  beta = p[2] - beta
  if(beta < 0):
    beta = 2*math.pi + beta
  

  dubinscurve = dubins(alpha,beta,d,Tree[0])
  min_d = dubinscurve[1] + dubinscurve[2] + dubinscurve[3]
  nN = Tree[0]
  
  count = 0
  for node in Tree:
    d = math.sqrt(distance_Euclidian(p,node.getVertex()))
    
    angle_goal = math.atan2(p[1] - node.getVertex()[1], p[0] - node.getVertex()[0])

    alpha = node.getVertex()[2] - angle_goal
    if(alpha < 0):
      alpha = 2*math.pi + alpha

    beta = math.atan2(p[1] - node.getVertex()[1], p[0] - node.getVertex()[0])
    if (beta < 0):
      beta = 2* math.pi + beta

    beta = p[2] - beta
    if(beta < 0):
      beta = 2*math.pi + beta

    dubinscurve = dubins(alpha,beta,d,node)
    d = dubinscurve[1] + dubinscurve[2] + dubinscurve[3]

    if (d >= 0) and ((d < min_d) or (min_d < 0)):
      nN = node
      min_d = d
    
    count = count + 1
  print "nearest neighbor found ",nN.getVertex()
  return nN


def nearestNeighbor_Dubins_Cost(p,Tree):
  """
  returns the node which has shortest dubins distance to p
  """
  # --------- Compute arguments for dubins curve -----------------#
  d = math.sqrt(distance_Euclidian(p,Tree[0].getVertex()))

  angle_goal = math.atan2(p[1] - Tree[0].getVertex()[1],p[0] - Tree[0].getVertex()[0])
  if(angle_goal < 0):
    angle_goal = 2*math.pi + angle_goal

  alpha = Tree[0].getVertex()[2] - angle_goal
  if(alpha < 0):
    alpha = 2*math.pi + alpha

  beta = math.atan2(p[1] - Tree[0].getVertex()[1], p[0] - Tree[0].getVertex()[0])
  if (beta < 0):
    beta = 2* math.pi + beta
  beta = p[2] - beta
  if(beta < 0):
    beta = 2*math.pi + beta


  dubinscurve = dubins(alpha,beta,d,Tree[0])
  min_d = dubinscurve[1] + dubinscurve[2] + dubinscurve[3]
  if(min_d >= 0):
    min_d = min_d + Tree[0].getCost()
  # --------------------------------------------------------------------- #

  nN = Tree[0]
  count = 0
  for node in Tree:
    d = math.sqrt(distance_Euclidian(p,node.getVertex()))

    angle_goal = math.atan2(p[1] - node.getVertex()[1], p[0] - node.getVertex()[0])

    alpha = node.getVertex()[2] - angle_goal
    if(alpha < 0):
      alpha = 2*math.pi + alpha

    beta = math.atan2(p[1] - node.getVertex()[1], p[0] - node.getVertex()[0])
    if (beta < 0):
      beta = 2* math.pi + beta

    beta = p[2] - beta
    if(beta < 0):
      beta = 2*math.pi + beta

    dubinscurve = dubins(alpha,beta,d,node)
    d = dubinscurve[1] + dubinscurve[2] + dubinscurve[3]
    if(d >= 0):
      d = d + node.getCost()

    if (d >= 0) and ((d < min_d) or (min_d < 0)):
      nN = node
      min_d = d

    count = count + 1
  print "nearest neighbor found ",nN.getVertex()
  return nN




def greedyMove(p_rand,p_near):
    """ Try x,y,straight and see which gets p_near closest
        to p_rand
    """
    v = 0.5
    phi = p_near.getVertex()[2]
    x = p_near.getVertex()[0]
    y = p_near.getVertex()[1]
    p = [-1,-1,-1]
    d = [-1,-1,-1]

    # Try left turn
    if ((phi + v) >= 2*math.pi) :
      angle = phi+v - 2*math.pi
    else:
      angle = phi+v
    p[0] = ((x+math.sin(phi + v) - math.sin(phi)),y-math.cos(phi+v) + math.cos(phi),angle)
    d[0] = distance_Euclidian(p_rand, p[0])
    
    # Try right turn
    if ((phi - v ) <= 0):
        angle = 2*math.pi - (phi-v)
    else:
        angle = phi-v

    p[1] = ((x-math.sin(phi-v) + math.sin(phi), y + math.cos(phi-v) - math.cos(phi),angle))
    d[1]= distance_Euclidian(p_rand,p[1])
    # Try straight
    p[2] = (x+v*math.cos(phi), y + v*math.sin(phi), phi)
    d[2] = distance_Euclidian(p_rand,p[2])

    min_d = d[0]
    p_new = p[0]
    index = 0
    # find min
    for i in range(3):
        distance = distance_Euclidian(p_rand,p[i])
        if min_d > distance:
            p_new = p[i]
            min_d = distance
            index = i
    
    if index == 0:
        u_new = 'L'
    elif index == 1:
        u_new = 'R'
    else:
        u_new = 'S'

    if not(p_near.addAction(u_new)):
      return False
    else:
      return (p_new,u_new)

def extend_RRT(p,Tree):
  global count_frontier
  global count_nonfrontier
  p_near = nearestNeighbor(p,Tree)

  # discard p_near based with probability = CVF
  discard = random.random()
  #print "discard ",discard," CVF ",p_near.getCVF()
  
  if(discard < p_near.getCVF()):
    #print "discard"
    return False
  
  # ------------------------------------------------- #
  
  Move = greedyMove(p,p_near)
  if (Move == False):
    return False
  else:
    (p_new,u_new) = Move
  if(newState(p_new)):    
    newNode = addVertex(Tree,p_new,p_near,u_new)   # p_near is a Node, p_new is a vertex
    return newNode                 # succcess
  
  #print "not newstate"
  # update CVF of this node, and all it's predecessors
  CVF = 1.0/3.0
  node = p_near

  while not(node == None):
    node.updateCVF(CVF)
    CVF = CVF*CVF
    node = node.getParent()      
    
  return False                     # 2 : trapped    

def dubinsparams_calc(p_init,p_goal):
  d = math.sqrt(distance_Euclidian(p_init,p_goal))

  angle_goal = math.atan2(p_goal[1] - p_init[1],p_goal[0] - p_init[0])
  if(angle_goal < 0):
    angle_goal = 2*math.pi + angle_goal
    
  alpha = p_init[2] - angle_goal
  if(alpha < 0):
    alpha = 2*math.pi + alpha

  beta = math.atan2(p_goal[1] - p_init[1], p_goal[0] - p_init[0])
  if (beta < 0):
    beta = 2* math.pi + beta
  beta = p_goal[2] - beta
  if (beta < 0):
    beta = 2*math.pi + beta

  return (alpha,beta,d)


def build_RRT(x1,y1,th1,x2,y2,th2,Tree):
  global RRT_goalfound
  p_init = (x1,y1,th1)
  p_goal = (x2,y2,th2)
  init_Node = RRTNode(p_init)
  Tree.append(init_Node)
  for i in range(1000):
    print i
    
    # 50% of the time, sample the goal
    # 50% of the time, sample other half randomly    

    # extend right Tree to goal
    if(RRT_goalfound == False):
      success = False
      while (success == False) :
        choice = random.random()
        if(choice >= 0.5):
          p_rand = (x2,y2,th2)
        elif(choice>=0.3):
          p_rand = (random.random()*60/1.91,(60+random.random()*60)/1.91,random.random()*2*math.pi)
        else:
           p_rand = (random.random()*60/1.91,(random.random()*120)/1.91,random.random()*2*math.pi)
        #print p_rand
        success = extend_RRT(p_rand,Tree)
    
      success_vertex = success.getVertex()
      (alpha,beta,d) = dubinsparams_calc(success_vertex,p_goal)
      
      dubinscurve = dubins(alpha,beta,d,success)
      min_d = dubinscurve[1] + dubinscurve[2] + dubinscurve[3]

      if(min_d >= 0):
        print "goal node ",success_vertex," start node ",p_goal, "curve number ",dubinscurve[0]
        RRT_goalfound = True
        RightTree_goal = success

        return RightTree_goal

  return False

def heuristic(current_node,goal_node):
    return distance_Euclidian(current_node.getVertex(),goal_node.getVertex())

def goalTest(vertex,goal):
    return (vertex.getVertex() == goal.getVertex())

def getSuccessors(vertex):
    return vertex[1]

def query_RRT(p_goal,Tree):
  
  p_start = Tree[0]
  p_end = p_goal
  print "p_start"
  p_start.display()

  print "p_end"
  p_end.display()

  path = []
  node = p_end
  count = 0
  while(node.getVertex()!= p_start.getVertex()):
    path.append(node)
    node = node.getParent()
    count = count+1
  path.append(node)
  print "Found path of length ",count
  return path

def displayTree(canvas,Tree):
  count = 0
  path = []
  for v in Tree:
    x = v.getVertex()[0]*5*1.91
    y = 600 - v.getVertex()[1]*5*1.91
    canvas.create_oval(x-1,y-1,x+1,y+1,width = 1, outline = 'red', fill = 'red')
    count = count + 1
  print "finished building rrt of size ",count

def plan(canvas,X,Y,TH,X2,Y2,TH2,Tree):
  global RRT_goalfound
  t1 = time.time()
  RRT_goalfound = False
  query_node = build_RRT(X,Y,TH,X2,Y2,TH2,Tree)

  if(query_node == False):
    return False

  t1 = time.time() - t1
  
  canvas.create_oval((X2)*5*1.91 - 2,600-(Y2*5*1.91)-2,(X2*5*1.91) +2,600-(Y2*5*1.91) + 2,width=1,outline = 'green',fill = 'green')

  count = 0
  displayTree(canvas,Tree)

  t2 = time.time()
  path = query_RRT(query_node,Tree)
  path.reverse()
  count = len(path)
  print "finish searching RRT for solution"

  RRT_goal_node = path[-1]
  #RRT_goal_node.display()
  RRT_goal = RRT_goal_node.getVertex()
  goal     = (X2,Y2,TH2)
  
  # --------- Compute arguments for dubins curve -----------------#
  (alpha,beta,d) = dubinsparams_calc(RRT_goal,(X2,Y2,TH2))
  #print "Dubins: d ",d," alpha ",alpha
  (curve_number,t,p,q) = dubins(alpha,beta,d,RRT_goal_node)
  
  t2 = time.time() - t2

  #print "Dubins ",curve_number," t ",t," p ",p," q ",q
  appendDubins(path,curve_number,t,p,q)

  #print "EXECUTION TIME\n RRT built in ",t1,"seconds\n Path found in ",t2,"seconds"

  return path


def startPlanner(canvas,X,Y,TH,X2,Y2,TH2):

  global Tree_start_to_right
  global Tree_start_to_left
  global Tree_right_to_goal
  global Tree_right_to_left
  global Tree_start_to_goal
  global RRT_goalfound
  Tree_start_to_right = []
  Tree_right_to_goal = []
  Tree_start_to_left = []
  Tree_left_to_goal = []
  Tree_start_to_goal = []
  path_start_to_right = []
  path_right_to_goal = []
  path_start_to_left = []
  path_left_to_goal = []
  
  canvas.delete(ALL)
  drawCourt(canvas)


  X = float(X)/1.91
  Y = float(Y)/1.91
  TH = math.radians(float(TH))
  X2 = float(X2)/1.91
  Y2 = float(Y2)/1.91
  TH2 = math.radians(float(TH2))
  if not((newState((X,Y,TH))) and (newState((X2,Y2,TH2)))):
    print "Invalid start and/or goal configuration"
    return

  # -----------  Check which case the start-goal configuration falls under  ---------------------
  # Case 1: start below and goal above => angle at net = pi/2
  # Case 2: start above and goal below => angle at net = 3*pi/2
  # Case 3: start and goal at same side of net => dont check net at all!

  t1 = time.time()

  if((Y <= 60/1.91) and (Y2 >= 60/1.91)):
    case = 1
  elif((Y >= 60/1.91) and (Y2 <= 60/1.91)):
    case = 2
  else:
    case = 3


  if(case == 1):
    angle_net = math.pi/2
  elif(case == 2):
    angle_net = 3*math.pi/2
  # --------- find path through right side of net --------- #
  if((case == 1) or (case == 2)):
    path_start_to_right =  plan(canvas,X,Y,TH,50/1.91,60/1.91,angle_net,Tree_start_to_right)
    path_right_to_goal = plan(canvas,50/1.91,60/1.91,angle_net,X2,Y2,TH2,Tree_right_to_goal)
  
    if((path_start_to_right!=False) and (path_right_to_goal!=False)):
      path_right = list(path_start_to_right + path_right_to_goal)
    else:
      path_right = False

  # -------- find path through left side of net ----------- #
    path_start_to_left = plan(canvas,X,Y,TH,10/1.91,60/1.91,angle_net,Tree_start_to_left)
    path_left_to_goal = plan(canvas,10/1.91,60/1.91,angle_net,X2,Y2,TH2,Tree_left_to_goal)

    if((path_start_to_left!=False) and (path_left_to_goal!=False)):
      path_left = list(path_start_to_left + path_left_to_goal)
    else:
      path_left = False


    if(len(path_right) < len(path_left)):
      path = path_right
    else:
      path = path_left

  elif(case == 3):
    path = plan(canvas,X,Y,TH,X2,Y2,TH2,Tree_start_to_goal)

  t1 = time.time() - t1
 #---------- draw path -------------- #
  
  print("Time to complete Planning ",t1," s")
  i = 0
  for v in path:
    x = v.getVertex()[0]*5*1.91
    y = 600 - v.getVertex()[1]*5*1.91
    color = 'blue'
    canvas.create_oval(x - 1, y-1,x+1,y+1,width=1,outline = color, fill = color)
    i = i+1
  
def appendDubins(path,curve_number,t,p,q):
  """
  Append steps of the Dubins curve onto the given path
  """
  (x,y,theta) = path[-1].getVertex()
  (x_init,y_init,theta_init) = (x,y,theta)
  v = 0.5
  
  # first action

  print "curve number ",curve_number
  if((curve_number == 0) or (curve_number == 2) or (curve_number == 5)):
    #turn left
    action = turnLeft

  elif((curve_number == 1) or (curve_number == 3) or (curve_number == 4)):
    #turn right
    action = turnRight

  i = 0
  while(i < t):
    (x,y,theta) = action(x,y,theta,v)
    path.append(RRTNode((x,y,theta)))
    i = i + v

  (x_init,y_init,theta_init) = action(x_init,y_init,theta_init,t)
  (x,y,theta) = (x_init,y_init,theta_init)
  path.append(RRTNode((x,y,theta)))

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
    path.append(RRTNode((x,y,theta)))
    i = i + v

  (x_init,y_init,theta_init) = action(x_init,y_init,theta_init,p)
  (x,y,theta) = (x_init,y_init,theta_init)
  path.append(RRTNode((x,y,theta)))

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
    path.append(RRTNode((x,y,theta)))
    i = i + v
    
  (x_init,y_init,theta_init) = action(x_init,y_init,theta_init,q)
  (x,y,theta) = (x_init,y_init,theta_init)
  path.append(RRTNode((x,y,theta)))

  return path

def main():
    root = Tk()
    root.title(' RC RRT')
    canvas = Canvas(root,width = 300, height = 600,bg = 'white')
    canvas.pack()

    # x initial
    entryLabel_x = Label(root)
    entryLabel_x["text"] = "init: X"
    entryLabel_x.pack(side = LEFT)
    entryWidget_x = Entry(root)
    entryWidget_x["width"] = 5
    entryWidget_x.pack(side = LEFT)

    # y initial
    entryLabel_y = Label(root)
    entryLabel_y["text"] = "Y"
    entryLabel_y.pack(side = LEFT)
    entryWidget_y = Entry(root)
    entryWidget_y["width"] = 5
    entryWidget_y.pack(side = LEFT)

    # theta initial
    entryLabel_th = Label(root)
    entryLabel_th["text"] = "TH"
    entryLabel_th.pack(side = LEFT)
    entryWidget_th = Entry(root)
    entryWidget_th["width"] = 5
    entryWidget_th.pack(side = LEFT)

    # x goal
    entryLabel_x2 = Label(root)
    entryLabel_x2["text"] = "    goal: X"
    entryLabel_x2.pack(side = LEFT)
    entryWidget_x2 = Entry(root)
    entryWidget_x2["width"] = 5
    entryWidget_x2.pack(side = LEFT)

    # y goal
    entryLabel_y2 = Label(root)
    entryLabel_y2["text"] = "Y"
    entryLabel_y2.pack(side = LEFT)
    entryWidget_y2 = Entry(root)
    entryWidget_y2["width"] = 5
    entryWidget_y2.pack(side = LEFT)

    # theta goal
    entryLabel_th2 = Label(root)
    entryLabel_th2["text"] = "TH"
    entryLabel_th2.pack(side = LEFT)
    entryWidget_th2 = Entry(root)
    entryWidget_th2["width"] = 5
    entryWidget_th2.pack(side = LEFT)

    
    b = Button(root,text = "Go",command = lambda: startPlanner(canvas,entryWidget_x.get(),entryWidget_y.get(),entryWidget_th.get(),entryWidget_x2.get(),entryWidget_y2.get(),entryWidget_th2.get()))
    b.pack(side = RIGHT)

    drawCourt(canvas)
      
    root.mainloop()



main()
