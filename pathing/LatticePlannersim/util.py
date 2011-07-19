"""
utils for Lattice planner v1.0 - Reeds-Shepp car
Contains definitions for PriorityQueue, LatticeNode, ControlSet
Contains functions to implement A* search
"""
import math
import heapq
import graphics
import controlset

# ROBOT PROPERTIES
ROBOT_LENGTH = 49        # 49 cm
ROBOT_WIDTH  = 28        # 28 cm
ROBOT_RADIUS_MIN = 70.0  # 70 cm
ROBOT_RADIUS_2   = 119.5 # 119.5 cm
ROBOT_SPEED_MAX = 100.0  #100 cm/s 

#ENVIRONMENT PROPERTIES
COURT_LENGTH = 3657.6    #cm
COURT_WIDTH = 1828.8     #cm

#LATTICE PROPERTIES
CELL_SIZE = ROBOT_RADIUS_MIN/4 # 17.5 cm

controlset = None
costmap = None

def feet_to_cm(feet):
    return feet*30.48

def cm_to_feet(cm):
    return cm/30.48

def distance_Euclidean(x1,y1,x2,y2):
    return math.sqrt(math.pow(x1-x2,2) + math.pow(y1-y2,2))
#############################################################################################

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

####################################################################################################

#####################################################################################################################
class LatticeNode:
  """
  Stores states for lattice based search.
  Each state has pose (x,y,theta), and velocity v
  v takes two possible values
  """
  def __init__(self,stateparams = None,parent = None,action = None,cost = 0):
    global ROBOT_SPEED_MAX
    if(stateparams == None):
      self.x = 0
      self.y = 0
      self.theta = 0
      self.v = ROBOT_SPEED_MAX
    else:
      self.x,self.y,self.theta,self.v = stateparams
      self.theta = self.theta%(2*math.pi)
    self.children = []
    self.parent = parent
    self.action = action    # action to get to this node
    
  def addParent(self,parent):
    self.parent = parent
    
  def getParent(self):
    return self.parent

  def getAction(self):
    """
    Return action taken to reach this node
    """
    return self.action

  def get_stateparams(self):
    return (self.x,self.y,self.theta,self.v)

  def expand(self):
    """
    Expands the node and updates its children
    Returns a list of the children
    """
    global controlset,ROBOT_SPEED_MAX
    actions = controlset.getActions(self)
    (x,y,theta,v) = self.get_stateparams()

    for action in actions:
        # get precomputed state on taking this action from origin   
        stateparams_from_origin = controlset.action_to_stateparams(self,action)         
        # translate to get the new state
        stateparams_from_current = (stateparams_from_origin[0] + x, stateparams_from_origin[1] + y,stateparams_from_origin[2],v) 
        newNode = LatticeNode(stateparams_from_current,self,action) # create a new Node with the new state params, and this node as a parent
        self.children.append(newNode)
      
    return self.children
    
def point_to_lattice(x,y,th=0,v=ROBOT_SPEED_MAX):
    """
    Takes a point (x,y,th,v) in cm and returns (x',y',th',v') where (x',y',th',v') is the closest lattice point to (x,y,th,v)
    """
    global CELL_SIZE
    
    # resolve x 
    closest_x = x/CELL_SIZE
    if(closest_x - int(closest_x)) >= 0.5:
        closest_x = (int(closest_x) + 1)*CELL_SIZE
    else:
        closest_x = int(closest_x)*CELL_SIZE

    # resolve y
    closest_y = y/CELL_SIZE
    if(closest_y - int(closest_y)) >= 0.5:
        closest_y = (int(closest_y) + 1)*CELL_SIZE
    else:
        closest_y = int(closest_y)* CELL_SIZE
    
    # resolve th
    allowed_headings = [0,math.pi/2,math.pi,3*math.pi/2,math.pi/4,3*math.pi/4,5*math.pi/4,7*math.pi/4]
    differences = [abs(allowed_heading - th) for allowed_heading in allowed_headings]
    closest_th = allowed_headings[differences.index(min(differences))]
    
    if v >= 0:
        v = ROBOT_SPEED_MAX
    else:
        v = - ROBOT_SPEED_MAX

    return (closest_x,closest_y,closest_th,v)

class CostMap:
    """
    Defines the cost map. Note that our environment has very few obstacles, all of which are rectangular.
    To use as little memory as possible, we use an implicit description of obstacles, checking for the coordinates
    of a given cell returning the cost of occupying that cell
    The resolution of costmap as well as the lattice is the same
    Cost description:
    1        - no obstacle
    inf       - obstacle
    """
    
    def __init__(self):
        """
        create rectangles representing obstacles
        rectangles represented by topleft and bottom right vertices
        obstacles =
        """
        self.obstacles = []
        # Net
        net = [(feet_to_cm(9.0),feet_to_cm(60.5)),(feet_to_cm(51.0),feet_to_cm(59.5))]
        vertex = net[0]
        (x_topleft,y_topleft,ignore1,ignore2) = point_to_lattice(vertex[0],vertex[1]) 
        x_topleft -= CELL_SIZE # padding
        y_topleft += CELL_SIZE # padding
        
        vertex = net[1]
        (x_bottomright,y_bottomright,ignore3,ignore4) = point_to_lattice(vertex[0],vertex[1])
        x_bottomright += CELL_SIZE
        y_bottomright -= CELL_SIZE    
        self.obstacles.append(((x_topleft,y_topleft),(x_bottomright,y_bottomright)))                    
    
    def cost_cell(self,x,y):
        """
        Given points x,y, return the cost of the cell at that point
        """        
        x_feet = cm_to_feet(x)
        y_feet = cm_to_feet(y)
        
        # check bounds of court
        if(x_feet < 0) or (x_feet>60) or (y_feet < 0) or (y_feet > 120):
            return float('inf')

        # resolve (x,y) onto nearest lattice point and check if there is an obstacle at this point
        (x,y,th,v) = point_to_lattice(x,y)

        for obstacle in self.obstacles:
            if( x>= obstacle[0][0] and x <= obstacle[1][0]) and (y <= obstacle[0][1] and y >= obstacle[1][1]):
                return float('inf')
        
        return 1

    def draw_costmap(self):
        """
        Draw in the costmap
        """
        for obstacle in self.obstacles:
            graphics.draw_rectangle(obstacle[0][0],obstacle[0][1],obstacle[1][0],obstacle[1][1],'red')
            

##################################################################################################
# --------------------------------- Functions for A* search --------------------------------------#
def goalTest(node,goalNode):
  """
  Check if current node is a goal node, 
  i.e. if (x,y) == (goal_x,goal_y). 
  """
  (x,y,theta,v) = node.get_stateparams()
  (x_goal,y_goal,theta_goal,v_goal) = goalNode.get_stateparams()

  if not goal_in_radius(x,y,goalNode):
      return False
  else:
      x_top = x + ROBOT_LENGTH/2 * math.cos(theta)
      y_top = y + ROBOT_LENGTH/2 * math.sin(theta) 
      (x_top,y_top,theta,v) = point_to_lattice(x_top,y_top,theta,v)
      if((x_goal==x_top) and (y_goal == y_top)):# and (node.getAction() not in ("B","B_diag","L_b","R_b"))) :
          return True
      else:
          return False

def heuristic_Euclidean(node,goalNode):
  """
  return Euclidean distance from node to goal
  """
  (x,y,theta,v) = node.get_stateparams()
  (x_goal,y_goal,theta_goal,v_goal) = goalNode.get_stateparams()
  
  return math.sqrt(math.pow(x-x_goal,2) + math.pow(y-y_goal,2))

def heuristic_aroundNet(node,goalNode):
    """
    if the node and goalNode are on opposite sides of the net, then heuristic is 
    the shortest of two (st line distance to end of net + st line distance from end to goal)

    else
    heuristic is euclidean distance
    """
    (x,y,th,v) = node.get_stateparams()
    (x_goal,y_goal,theta_goal,v_goal) = goalNode.get_stateparams()
    
    if(x_goal >= 262.5 and x_goal <=1575.0) and (x >= 262.5 and x <=1575.0):
        if ((y_goal >= 1855) and (y <= 1802.5)) or ((y_goal <= 1802.5) and (y >= 1855)):
            d_right = distance_Euclidean(x,y,1575.0,1825.0) + distance_Euclidean(1575.0,1825.0,x_goal,y_goal)
            d_left  = distance_Euclidean(x,y,262.5,1825.0) + distance_Euclidean(262.5,1825.0,x_goal,y_goal)            
            return min(d_right,d_left)
        return distance_Euclidean(x,y,x_goal,y_goal)
    else:
        return distance_Euclidean(x,y,x_goal,y_goal)

def Astarsearch(startNode,goalNode):
  """
  A* search from startNode to goalNode
  Returns the goalNode reached. Returns None when goalNode cannot be reached
  """
  current_Node = None
  searchTree = PriorityQueue()
  alreadySeen = set()
  searchTree.push((startNode,0),heuristic_Euclidean(startNode,goalNode))
  
  while(not(searchTree.isEmpty())):
      current_item = searchTree.pop()    
      current_Node = current_item[0]
      cost_soFar = current_item[1]
      if goalTest(current_Node,goalNode):
          print "goal cost",cost_soFar
          break
      else:      
          (x,y,theta,v) = current_Node.get_stateparams()
          if (x,y,theta) not in alreadySeen:
              if(x >= feet_to_cm(9.0)) and (x <= feet_to_cm(51.0)) and (y >= feet_to_cm(59.5)) and (y <= feet_to_cm(60.5)):
                  print "x",x,"y",y,"th",theta,"action",current_Node.getAction(),"parent",current_Node.getParent().get_stateparams()
              graphics.draw_point(x,y,theta,'blue')
              alreadySeen.add((x,y,theta))
              children = current_Node.expand()
              for child in children:
                  stepcost = cost(current_Node,child.getAction(),child,goalNode)
                  if stepcost != float('inf'):
                      g = cost_soFar + stepcost 
                      searchTree.push((child,g),g + heuristic_aroundNet(child,goalNode))
          
  return current_Node

def len_action(action):
    """
    returns length of an action
    """
    if action in ('F','B'):          
        len_action =  CELL_SIZE
    elif action in ('F_diag','B_diag'):
        len_action =  CELL_SIZE*math.sqrt(2)
    elif action in ("LS_f","RS_f","SL_f","SR_f"):
        len_action =  (20.5/70.0*ROBOT_RADIUS_MIN + math.pi*ROBOT_RADIUS_2/4)
    elif action in ("L_f","R_f","L_b","R_b"):
        len_action =  (math.pi*ROBOT_RADIUS_MIN/2.0)
    elif action == "F3":
        len_action =  3*CELL_SIZE
    elif action == "F_diag3":
        len_action =  3*CELL_SIZE*math.sqrt(2)
    return len_action

def cost(state,action,newstate,goalNode):
  """
  Given a state and action from that state that reaches newstate, return the cost of executing this action
  Cost of the action is the length of the action * average value of costmap cells under swath of action
  """

  (state_x,state_y,state_th,state_v) = state.get_stateparams()
  (newstate_x,newstate_y,newstate_th,newstate_v) = newstate.get_stateparams()
  (goal_x,goal_y,goal_th,goal_v) = goalNode.get_stateparams()

  length_action = len_action(action)

  if goal_in_radius(state_x,state_y,goalNode):
      nearGoal = True      
  else:
      nearGoal = False
  
  if (nearGoal and goalTest(newstate,goalNode)):
      actionToGoal = True
  else:
      actionToGoal = False


  if not(obstacle_in_radius(state_x,state_y) or goal_in_radius(state_x,state_y,goalNode)):   # state is neither near an obstacle nor near goal
      cost = length_action  
  else:
      swath_indices_origin = controlset.getSwath(state,action)  
      average_cellcost = 0.0 # compute average of all cells in costmap that lie under the car's swath
      num_cells = 0.0
      for strip in swath_indices_origin:   # iterate through vertical strips. each strip is a tuple (x,y1,yn)
          x_coord = state_x + strip[0]*CELL_SIZE     
          for y_coord in range(strip[1],strip[2]+1):        # iterate through points in strip
              y_coord = state_y + y_coord*CELL_SIZE                                                              
              cost_cell = costmap.cost_cell(x_coord,y_coord)          
              if cost_cell == float('inf'):   # if swath has an obstacle, return cost= infinity
                  return cost_cell
              # if swath passes through goal, but this action does not terminate the path, return cost = infinity
              elif ((goal_x == x_coord) and (goal_y == y_coord) and not actionToGoal):
                  return float('inf')
              average_cellcost += cost_cell
              num_cells += (strip[2]+1 - strip[1])
  
      average_cellcost = average_cellcost/num_cells
      cost = length_action*average_cellcost
            
  # Cost of backing up
  if action in ("B","B_diag","L_b","R_b"): 
      reverse_mult = 20
  else:
      reverse_mult = 1
          
  # Cost of turning
  if action not in ("F","F3","F_diag","F_diag3","B","B_diag"):
      turn_mult = 5
  else:
      turn_mult = 1
      
  cost = cost*reverse_mult*turn_mult
  return cost

def obstacle_in_radius(x,y):
    """
    Given a state's x,y coordinates, return True if an obstacle may be within 175 cm of the state
    else return False
    """
    # return True if 'state' is within 175 cm of an obstacle (look at rectangle 
    for obstacle in costmap.obstacles:
        if(y<= obstacle[0][1] + 175) and (y >= obstacle[1][1] - 175) and (x >= obstacle[0][0] - 175) and (x <= obstacle[0][1] + 175):
            return True
    return False

def goal_in_radius(x,y,goalNode):
    """
    Given a state's x,y coordinates, return True if the goal is within 175 cm of the state
    else return false
    """
    (x_g,y_g,th_g,v_g) = goalNode.get_stateparams()
    if (x >= x_g -175) and (x <= x_g+175) and (y >= y_g - 175) and (y <= y_g + 175):
        return True
    else:
        return False
  
##################################################################################################
def turn_Left(state,direction,d,radius):
  """
  given a pose, return pose reached by turning left along circle of radius 'radius'
  direction = 'f' for forward, 'b' for backward
  d = arc length of turn (in cm). turn through a quadrant by default
  """
  d = d/radius
  x1 = state[0]/radius
  y1 = state[1]/radius
  th1 = state[2]
  v = state[3]
  if(direction == 'b'):
    d = d*(-1) 
    
  x2 = (x1 + math.sin(th1+d)- math.sin(th1)) * radius
  y2 = (y1 - math.cos(th1+d)+ math.cos(th1)) * radius
  th2 = th1 + d

  return(x2,y2,th2,v)

def turn_Right(state,direction,d,radius):
  """ 
  given a pose (x,y,th), return pose reached by turning right along circle of radius ROBOT_RADIUS_MIN
  direction = 'f' for forward, 'b' for backward
  d = arc length of turn (in cm). turn through a quadrant by default
  """
  d = d/radius
  x1 = state[0]/radius
  y1 = state[1]/radius
  th1 = state[2]
  v = state[3]
  if(direction == 'b'): #turning backward
    d = d*(-1)
    
  x2 = (x1 - math.sin(th1-d)+ math.sin(th1)) * radius
  y2 = (y1 + math.cos(th1-d)- math.cos(th1)) * radius
  th2 = th1 - d

  return(x2,y2,th2,v)

def go_Straight(state,direction,d = CELL_SIZE):
  """ 
  given a pose (x,y,th), return pose reached by moving straight
  direction = 'f' for forward, 'b' for backward
  d = distance to travel (in cm). Travel through CELL_SIZE by default
  """
  x1 = state[0]
  y1 = state[1]
  th1 = state[2]
  v = state[3]
  if(direction == 'b'): 
    d = d*(-1)
  x2 = x1 + d*math.cos(th1)
  y2 = y1 + d*math.sin(th1)
  th2 = th1
  return(x2,y2,th2,v)

####################################################################################################
