# Investigate RRTs
"""
Map definition: 
total size = 12m x 6m
one obstacle in rectangle (1.2,5.95,4.8,6.05)

Each Tree is a list of vertices. 
Each Node is [(x,y,theta),[(neighbor1,action1),(neighbor2,action2)....]
"""

import math
import sys
import random
import heapq
import time
from dubins import *
from util import *
#from Tkinter import *


Tree = []

class PriorityQueue:
  """
    Implements a priority queue data structure. Each inserted item
    has a priority associated with it and the client is usually interested
    in quick retrieval of the lowest-priority item in the queue. This
    data structure allows O(1) access to the lowest-priority item.
    
    Note that this PriorityQueue does not allow you to change the priority
    of an item.  However, you may insert the same item multiple times with
    different priorities.
  """  
  def  __init__(self):  
    self.heap = []
    
  def push(self, item, priority):
      pair = (priority,item)
      heapq.heappush(self.heap,pair)

  def pop(self):
      (priority,item) = heapq.heappop(self.heap)
      return item
  
  def isEmpty(self):
    return len(self.heap) == 0

# -------------------------------------------------------------------------------------------------- #

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
# -------------------------------------------------------------------------------------------------- #

def addVertex(tree,p_new,p_near,u_new):

  # find p_near and append p_new as a neighbor
  new_Node = RRTNode(p_new)

  p_near.addNeighbor(new_Node,u_new)
  new_Node.addParent(p_near)

  #  add p_new as a vertex
  tree.append(new_Node)

"""
def newState(p):
     
    #Check if p is an allowed state, i.e. does it lie in C_free?
    
    x = p[0]
    y = p[1]
    if (x < 0 or x > 10):
        return False
    elif(y < 0 or y > 10):
        return False
    elif( (x >= 2) and (y >= 4.5) and (x <= 8) and (y <= 5.5)):
        return False
    else:
        return True
"""

def distance_Euclidian(p1,p2):
    """
    returns square of the Euclidian distance, for speed
    """
    return (p1[0] - p2[0])**2 + (p1[1] - p2[1])**2
    

def nearestNeighbor(p):
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

def nearestNeighbor_Dubins(p):
  """ 
  returns the node which has shortest dubins distance from a point in the tree
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


def greedyMove(p_rand,p_near):
    """ Try x,y,straight and see which gets p_near closest
        to p_rand
    """
    v = 0.1
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

def extend_RRT(p):
  p_near = nearestNeighbor(p)

  # discard p_near based with probability = CVF
  discard = random.random()
  #print "discard ",discard," CVF ",p_near.getCVF()
  
  if(discard < p_near.getCVF()):
    #print "discard"
    return False
  
  Move = greedyMove(p,p_near)
  if (Move == False):
    return False
  else:
    (p_new,u_new) = Move
  if(newState(p_new)):
    addVertex(Tree,p_new,p_near,u_new)   # p_near is a Node, p_new is a vertex
    return True                 # succcess
  
  #print "not newstate"
  # update CVF of this node, and all it's predecessors
  CVF = 1.0/3.0
  node = p_near

  while not(node == None):
    node.updateCVF(CVF)
    CVF = CVF*CVF
    node = node.getParent()      
    
  return False                     # 2 : trapped


def build_RRT(x1,y1,th1):
  p_init = (x1,y1,th1)
  init_Node = RRTNode(p_init)
  Tree.append(init_Node)
  for i in range(1000):
    p_rand = (random.random()*6,random.random()*12,random.random()*2*math.pi)
    success = extend_RRT(p_rand)
    while (success == False) :
      p_rand = (random.random()*6,random.random()*12,random.random()*2*math.pi)
      success = extend_RRT(p_rand)

        #print Tree
    

def heuristic(current_node,goal_node):
    return distance_Euclidian(current_node.getVertex(),goal_node.getVertex())

def goalTest(vertex,goal):
    return (vertex.getVertex() == goal.getVertex())

def getSuccessors(vertex):
    return vertex[1]

def query_RRT(p_init,p_goal):
  """
  Find nearest neighbors in RRT to current position and goal 
  """
  
  p_start = Tree[0]
  p_end = nearestNeighbor_Dubins(p_goal)
  #p_end = nearestNeighbor(p_goal)
  """ 
  Run A* search through the RRT
  """
  # Choose random start and end points for now
  
  # p_start_index = int(random.random()*count)
  # p_end_index = int(random.random()*count)
  print "p_start"
  p_start.display()

  print "p_end"
  p_end.display()

  search_tree = PriorityQueue()
  already_seen = []
  path = []
  cost = 0.1

  # Begin A* search
  search_tree.push((p_start,"none",0),heuristic(p_start,p_end))
    
  while not(search_tree.isEmpty()):
    current_term = search_tree.pop()
    current_Node = current_term[0]
    current_Cost = current_term[2]
    if (goalTest(current_Node,p_end)):
      print "goal found!"
      break
    else:
      if not (current_Node in already_seen):
        already_seen.append(current_Node)
        for node_action in current_Node.getNeighbors():
          #print "adding neighbor of ",current_Node.getVertex()
          node = node_action[0]
          node.addParent(current_Node)
          search_tree.push((node,current_Node,current_Cost + 0.1),(current_Cost + 0.1 + heuristic(current_Node,p_end)))
                  
  # find path
  node = current_Node
  count = 0
  print "Goal node"
  node.display()
  while(node.getVertex()!= p_start.getVertex()):
    path.append(node)
    node = node.getParent()
    count = count+1

  print "Found path of length ",count
  return path


def startPlanner(X,Y,TH,X2,Y2,TH2):
  global Tree
  Tree = []
  #canvas.delete(ALL)
  #drawCourt(canvas)

  X = float(X)
  Y = float(Y)
  TH = math.radians(float(TH))
  X2 = float(X2)
  Y2 = float(Y2)
  TH2 = math.radians(float(TH2))
  if not((newState((X,Y,TH))) and (newState((X2,Y2,TH2)))):
    print "Invalid start and/or goal configuration"
    return

  t1 = time.time()
  build_RRT(X,Y,TH)
  t1 = time.time() - t1
  
  count = 0
  """
  for v in Tree:
    x = v.getVertex()[0]*50
    y = 600 - v.getVertex()[1]*50
    canvas.create_oval(x-1,y-1,x+1,y+1,width = 1,outline = 'red',fill = 'red')
    count = count + 1
    #v.display()
    """
  print "finished building RRT in time ",t1

  t2 = time.time()
  path = query_RRT((X,Y,TH),(X2,Y2,TH2))
  path.reverse()
  count = len(path)
  print "finish searching RRT for solution"

  RRT_goal_node = path[-1]
  #RRT_goal_node.display()
  RRT_goal = RRT_goal_node.getVertex()
  goal     = (X2,Y2,TH2)
  
  # --------- Compute arguments for dubins curve -----------------#
  d = math.sqrt(distance_Euclidian(RRT_goal,goal))
  
  angle_goal = math.atan2(goal[1]-RRT_goal[1],goal[0]-RRT_goal[0])
  if(angle_goal < 0):
    angle_goal = 2*math.pi + angle_goal
    
  alpha = RRT_goal[2] - angle_goal
  if(alpha < 0):
    alpha = 2*math.pi + alpha

  beta = math.atan2(goal[1] - RRT_goal[1], goal[0] - RRT_goal[0])
  if(beta < 0):
    beta = 2*math.pi + beta

  beta = goal[2] - beta
  if(beta < 0):
    beta = 2*math.pi + beta

  print "Dubins: d ",d," alpha ",alpha
  (curve_number,t,p,q) = dubins(alpha,beta,d,RRT_goal_node)
  
  t2 = time.time() - t2

  print "Dubins ",curve_number," t ",t," p ",p," q ",q
  appendDubins(path,curve_number,t,p,q)

  # --------------------------------------------------------------#
  """
  i = 0
  print "Total path length = ", len(path)
  for v in path:
    x = v.getVertex()[0]*50
    y = 600 - v.getVertex()[1]*50
    if(i < count):
      color = 'blue'
    else:
      color = 'black'
    canvas.create_oval(x - 1, y-1,x+1,y+1,width=1,outline = color, fill = color)
    i = i+1
  """

  #canvas.create_oval((X2)*50 - 2,600-(Y2)*50 -2,(X2)*50 +2,600-(Y2)*50 + 2,width=1,outline = 'green',fill = 'green')

  print "EXECUTION TIME\n RRT built in ",t1,"seconds\n Path found in ",t2,"seconds"

def appendDubins(path,curve_number,t,p,q):
  """
  Append steps of the Dubins curve onto the given path
  """
  (x,y,theta) = path[-1].getVertex()
  (x_init,y_init,theta_init) = (x,y,theta)
  
  # first action

  if((curve_number == 0) or (curve_number == 2) or (curve_number == 5)):
    #turn left
    action = turnLeft

  elif((curve_number == 1) or (curve_number == 3) or (curve_number == 4)):
    #turn right
    action = turnRight

  i = 0
  while(i < t):
    (x,y,theta) = action(x,y,theta)
    path.append(RRTNode((x,y,theta)))
    i = i + 0.1

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
    (x,y,theta) = action(x,y,theta)
    path.append(RRTNode((x,y,theta)))
    i = i + 0.1

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
    (x,y,theta) = action(x,y,theta)
    path.append(RRTNode((x,y,theta)))
    i = i + 0.1
    
  (x_init,y_init,theta_init) = action(x_init,y_init,theta_init,q)
  (x,y,theta) = (x_init,y_init,theta_init)
  path.append(RRTNode((x,y,theta)))

  return path

def main():
  """
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
  """
  startPlanner(1,1,0,4,8,0)
main()
