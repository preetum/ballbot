"""
RRTNode.py
Contains definition for the RRTNode class and functions relating to manipulating the RRT
"""
import random
from util import *

from dubins import *

RRT_goalfound = False
#ROBOT_RADIUS = 2.283  # 2.283 feet = 69.6 cm

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
    self.actions = [] # list of actions explored from this node.
    self.action_toreachNode = None # action taken to reach this node
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
    """
    Add 'action' to the actions already explored from this node.
    if 'action' has already been tried, return False
    otherwise, add 'action' to the list of actions already explored and return true
    """
    if action in self.actions:
      return False
    else:
      self.actions.append(action)
      return True
  
  def addAction_toreachNode(self,action):
    self.action_toreachNode = action

  def getAction_toreachNode(self):
    """
    return the action taken to reach this node
    """
    return self.action_toreachNode

  def updateCVF(self,CVF):
    self.CVF = self.CVF + CVF

  def getCVF(self):
    return self.CVF

  def updateCost(self,cost):
    self.cost = self.cost + cost
    
  def getCost(self):
    return self.cost


def addVertex(tree,p_new,p_near,u_new):
  # find p_near and append p_new as a neighbor
  new_Node = RRTNode(p_new)

  p_near.addNeighbor(new_Node,u_new)
  new_Node.addParent(p_near)
  
  parentCost = p_near.getCost()
  newCost = parentCost + util.stepsize/util.ROBOT_RADIUS
  new_Node.updateCost(newCost)
  new_Node.addAction_toreachNode(u_new)
  #  add p_new as a vertex and return a reference to it
  tree.append(new_Node)
  return new_Node

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

def extend_RRT(p,Tree):
  """
  given a point 'p' and a tree 'Tree', try to extend the RRT to include that point.
  Adds the new node along with its parent and the action to reach it from its parent.
  Returns a reference to the newNode on success, returns False on failure
  """
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
    return newNode	           # succcess
  
  #print "not newstate"
  # update CVF of this node, and all it's predecessors
  CVF = 1.0/3.0
  node = p_near

  while not(node == None):
    node.updateCVF(CVF)
    CVF = CVF*CVF
    node = node.getParent()      
    
  return False                     # 2 : trapped    

def build_RRT(x1,y1,th1,x2,y2,th2,Tree):
  global RRT_goalfound
  p_init = (x1,y1,th1)
  p_goal = (x2,y2,th2)
  #print ("init ",p_init," goal ", p_goal)
  init_Node = RRTNode(p_init)
  Tree.append(init_Node)
  for i in range(1000):
    #print i
    
    # 50% of the time, sample the goal
    # 50% of the time, sample other half randomly    

    # extend right Tree to goal
    if(RRT_goalfound == False):
      success = False
      while (success == False) :
        choice = random.random()
        if(choice >= 0.3):
          p_rand = (x2,y2,th2)
        else:
          p_rand = (random.random()*60/ROBOT_RADIUS,(random.random()*120)/ROBOT_RADIUS,random.random()*2*math.pi)
        #elif(choice>=0.1):
        #  p_rand = (random.random()*60/ROBOT_RADIUS,(60+random.random()*60)/ROBOT_RADIUS,random.random()*2*math.pi)
        #else:
        #   p_rand = (random.random()*60/ROBOT_RADIUS,(random.random()*120)/ROBOT_RADIUS,random.random()*2*math.pi)
        #print p_rand
        success = extend_RRT(p_rand,Tree)
    
      success_vertex = success.getVertex()

      # try to connect this new vertex to the goal using a dubins curve.
      #if this succeeds, stop building tree and return the node that connects to the goal via dubins

      (alpha,beta,d) = dubinsparams_calc(success_vertex,p_goal)
      
      dubinscurve = dubins(alpha,beta,d,success)
      min_d = dubinscurve[1] + dubinscurve[2] + dubinscurve[3]

      if(min_d >= 0):
        #print "goal node ",success_vertex," start node ",p_goal, "curve number ",dubinscurve[0]
        RRT_goalfound = True
        Tree_end = success

        return Tree_end
  
  #print "build_RRT returning false"
  return False

def displayTree(canvas,Tree):
  count = 0
  path = []
  for v in Tree:
    x = v.getVertex()[0]*5*ROBOT_RADIUS
    y = 600 - v.getVertex()[1]*5*ROBOT_RADIUS
    canvas.create_oval(x-1,y-1,x+1,y+1,width = 1, outline = 'red', fill = 'red')
    count = count + 1
  print "finished building rrt of size ",count


def query_RRT(p_goal,Tree):
  """
  given Node p_goal, returns a list containing parent nodes all the way to Tree[0]
  """  
  p_start = Tree[0]
  p_end = p_goal
  #print "p_start"
  p_start.display()

  #print "p_end"
  p_end.display()

  path = []
  node = p_end
  count = 0
  while(node.getVertex()!= p_start.getVertex()):
    path.append(node)
    node = node.getParent()
    count = count+1
  path.append(node)
  #print "Found path of length ",count
  return path
