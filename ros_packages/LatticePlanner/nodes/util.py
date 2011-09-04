"""
utils for Lattice planner 
Contains definitions for PriorityQueue, LatticeNode, ControlSet
Contains functions to implement A* search, LPA* search
"""
import math
import heapq
import controlset

startNode = None # startNode
goalNode = None  # goalNode
agentNode = None # node occupied by agent

SEARCHALGORITHM = "MT-AdaptiveA*"
count_expandednodes = 0
S = [] # list of all nodes seen so far by LPA* search, represents the graph S
U = []
plan_LPAstar = []
# ROBOT PROPERTIES
ROBOT_LENGTH = 49         # 49 cm
ROBOT_WIDTH  = 28         # 28 cm
ROBOT_RADIUS_MIN = 70.0   # 70 cm
ROBOT_RADIUS_2   = 119.5  # 119.5 cm
ROBOT_RADIUS_3   = 84.497 # 84.497 cm
ROBOT_RADIUS_4   = 74.246 # 74.246 cm
ROBOT_RADIUS_5   = 76.6845 # 76.6845 cm
ROBOT_SPEED_MAX = 100.0   #100 cm/s 

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

    # LPA* stuff 
    if (SEARCHALGORITHM == "LPA*"):
        self.g = float('inf')
        self.rhs = float('inf')
        self.predecessors = set()   
    
    # MT-AdaptiveA* stuff
    if (SEARCHALGORITHM == "MT-AdaptiveA*"):
        self.search  = 0 # stores the last search that expanded this node
        self.h = 0 # stores the heuristic of this node
    self.children = []
    self.g = float('inf')
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

  def setAction(self,action):
      self.action = action

  def get_stateparams(self):
    return (self.x,self.y,self.theta,self.v)

  def getSuccessors_Astar(self):
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

  # --------- LPA* stuff --------#
  def get_g(self):
      return self.g
  
  def set_g(self,new_g):
      self.g = new_g

  def get_rhs(self):
      return self.rhs

  def set_rhs(self,new_rhs):
      self.rhs = new_rhs

  def add_predecessor(self,predecessor,action,stepcost):
      """ 
      predecessors look like [pred,action,stepcost]
      """
      self.predecessors.add((predecessor,action,stepcost))

  def get_predecessors(self):
      return self.predecessors
  
  def update_cost(self,predecessor,action,cost):
      """
      update the cost of taking action from predecessor to reach node
      """
      for pred in self.predecessors:
          if(pred[0] == predecessor):
              self.predecessors.remove(pred)
              self.predecessors.add((predecessor,action,cost))
              return

  def getSuccessors_LPAstar(self,goalNode):
      """
      Expands the node, creating new nodes only if they haven't been placed in S yet
      """
      global S
      successors = []
      actions = controlset.getActions(self)
      (x,y,theta,v) = self.get_stateparams()
      
      for action in actions:
          # get precomputed state on taking this action from origin   
          stateparams_from_origin = controlset.action_to_stateparams(self,action)         
          # translate to get the new state
          stateparams_from_current = (stateparams_from_origin[0] + x, stateparams_from_origin[1] + y,stateparams_from_origin[2],v)  
          
          node = search_inS(stateparams_from_current) # search for this node in S          
          if(node!= None): # search has already seen this node
              stepcost = cost(self,action,node,goalNode)
              node.add_predecessor(self,action,stepcost)
              successors.append(node)
          else:           # node being expanded for the first time, so create a new node and add it to S
              newsuccessor = LatticeNode(stateparams_from_current,self,action)             
              stepcost = cost(self,action,newsuccessor,goalNode)
              newsuccessor.add_predecessor(self,action,stepcost)
              S.append(newsuccessor)
              successors.append(newsuccessor)

      return successors
  
  # ------  MT-AdaptiveA* stuff ------

  def get_search(self):
      return self.search

  def set_search(self,val):
      self.search = val

  def get_h(self):
      return self.h
  
  def set_h(self,val):
      self.h = val

  def getSuccessors_MTAdaptiveAstar(self,alreadySeen):
      """
      Expands the node, creating new nodes only if they haven't been placed in S yet
      """
      global Generated
      successors = []
      actions = controlset.getActions(self)
      (x,y,theta,v) = self.get_stateparams()
      
      for action in actions:
          # get precomputed state on taking this action from origin   
          stateparams_from_origin = controlset.action_to_stateparams(self,action)         
          # translate to get the new state
          stateparams_from_current = (stateparams_from_origin[0] + x, stateparams_from_origin[1] + y,stateparams_from_origin[2],v)  
          if stateparams_from_current[0:3] not in alreadySeen: 
              
              #if(stateparams_from_current[0:3] == (1470.0,1645.0,0.0)):
              #    print (1470.0,1645.0,0.0),"generated, parent:",self.get_stateparams(),"action",action

              if(Generated.has_key(stateparams_from_current[0:3])):
                  node = Generated[stateparams_from_current[0:3]]                                                      
                  successors.append((node,action))
                  
              else:           # node being expanded for the first time, so create a new node and add it to S
                  newsuccessor = LatticeNode(stateparams_from_current)
                  Generated[stateparams_from_current[0:3]] = newsuccessor
                  successors.append((newsuccessor,action))              
      return successors
  
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
    allowed_headings = [0,math.pi/2,math.pi,3*math.pi/2,math.pi/4,3*math.pi/4,5*math.pi/4,7*math.pi/4,5.177,3.606,2.035,0.464,1.107,5.819,2.677,4.248]
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
        obstacles = list of (id,(x_topleft,y_topleft),(x_bottomright),(y_bottomright))
        """
        self.obstacles = [] 
        self.highcostregions = []
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
               
        """
        # court
        court = [(feet_to_cm(12),feet_to_cm(99)),(feet_to_cm(48),feet_to_cm(21))]
        (x_topleft,y_topleft,ignore1,ignore2) = point_to_lattice(court[0][0],court[0][1])
        x_topleft -= CELL_SIZE # padding
        y_topleft += CELL_SIZE # padding
    
        (x_bottomright,y_bottomright,ignore3,ignore4) = point_to_lattice(court[1][0],court[1][1])
        x_bottomright += CELL_SIZE # padding
        y_bottomright -= CELL_SIZE # padding
        self.highcostregions.append(((x_topleft,y_topleft),(x_bottomright,y_bottomright)))
        """
        
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
        """
        for highcostregion in self.highcostregions:
            #print (x,y),highcostregion[0],highcostregion[1]
            if( x >= highcostregion[0][0] and x <= highcostregion[1][0]) and (y <= highcostregion[0][1] and y >= highcostregion[1][1]):
                #print "returning high cost"
                return 2
        """    
        return 1        

    def draw_costmap(self):
        """
        Draw in the costmap
        """
        for highcostregion in self.highcostregions:
            graphics.draw_rectangle(highcostregion[0][0],highcostregion[0][1],highcostregion[1][0],highcostregion[1][1],'yellow')
                    
        for obstacle in self.obstacles:
            graphics.draw_rectangle(obstacle[0][0],obstacle[0][1],obstacle[1][0],obstacle[1][1],'red')
                   
    def new_obstacle(self,event):      
        """
        Given an event (left mouse button click), create a new square obstacle of size 50cm x 50 cm centered at (x,y)
        """        
        global startNode,goalNode
        x = float(event.x)/graphics.cm_to_pixels
        y = (600 - float(event.y))/graphics.cm_to_pixels     
        (x_topleft,y_topleft,ignore,ignore) = point_to_lattice(x-25,y+25,0,0)
        (x_bottomright,y_bottomright,ignore,ignore) = point_to_lattice(x+25,y-25,0,0)
        obst = graphics.draw_rectangle(x_topleft,y_topleft,x_bottomright,y_bottomright,'red')
        graphics.canvas.update_idletasks()
        self.obstacles.append(((x_topleft,y_topleft),(x_bottomright,y_bottomright)))
        
        if(SEARCHALGORITHM == "LPA*") and (startNode!=None) and (goalNode!=None):
            # find affected edges and replan
            print "finding affected edges"
            edges_affected = compute_edges_affected((x_topleft,y_topleft),(x_bottomright,y_bottomright))
            print "found affected edges"
            LPAstarsearch_replan(edges_affected)

        elif(SEARCHALGORITHM == "MT-AdaptiveA*") and (startNode!=None) and (goalNode!= None):
            return
            
                                                    
##################################################################################################
#                                   COST COMPUTATION
# -------------------------------------------------------------------------------------------------

def cost(state,action,newstate,goalNode):
  """
  Given a state and action from that state that reaches newstate, return the cost of executing this action
  Cost of the action is the length of the action * average value of costmap cells under swath of action
  """

  (state_x,state_y,state_th,state_v) = state.get_stateparams()
  (newstate_x,newstate_y,newstate_th,newstate_v) = newstate.get_stateparams()
  (goal_x,goal_y,goal_th,goal_v) = goalNode.get_stateparams()

  length_action = controlset.len_action(action)
  if action in ("F","F_diag","F_26.6","F_63.4"):
      cost_mult = 1
  else:
      cost_mult = 1.05 

  if goal_in_radius(state_x,state_y,goalNode):
      nearGoal = True      
  else:
      nearGoal = False
  
  if (nearGoal and goalTest(newstate)):
      actionToGoal = True
  else:
      actionToGoal = False

  if not(obstacle_in_radius(state_x,state_y) or nearGoal):   # state is neither near an obstacle nor near goal
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
              elif cost_cell == 2:
                  return length_action*cost_cell*cost_mult
              # if swath passes through goal, but this action does not terminate the path, return cost = infinity
              elif (nearGoal and (goal_x == x_coord) and (goal_y == y_coord) and not actionToGoal):
                  return float('inf')
              average_cellcost += cost_cell
          num_cells += (strip[2]+1 - strip[1])
      average_cellcost = average_cellcost/num_cells      
      cost = length_action*average_cellcost  
      #print cost

  
  cost = cost*cost_mult
  return cost

def obstacle_in_radius(x,y):
    """
    Given a state's x,y coordinates, return True if an obstacle may be within 175 cm of the state
    else return False
    """
    if (abs(y- COURT_LENGTH) <= 175) or (abs(y) <= 175) or (abs(x - COURT_WIDTH) <= 175) or (abs(x- COURT_LENGTH) <= 175) or (abs(x) <= 175):
        return True

    # return True if 'state' is within 175 cm of an obstacle (look at rectangle)
    for obstacle in costmap.obstacles:
        if(y<= obstacle[0][1] + 175) and (y >= obstacle[1][1] - 175) and (x >= obstacle[0][0] - 175) and (x <= obstacle[1][0] + 175):
            return True

    """
    # also return True if 'state' is within 175 cm of a highcost region
    for highcostrgn in costmap.highcostregions:
        if(y<= highcostrgn[0][1] + 175) and (y >= highcostrgn[1][1] - 175) and (x >= highcostrgn[0][0] - 175) and (x <= highcostrgn[1][0] + 175):           
            return True
    """
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
 




###################################################################################################
# ------------------------------------ Heuristics and goal tests -----------------------------------#
def goalTest(node):
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
      if((x_goal==x_top) and (y_goal == y_top)) and (node.getAction() not in ("B","B_diag","L_b","R_b")) :
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
                                                    
#####################################################################################################
# --------------------------------- Search algorithms --------------------------------------#

# ---------------------- A* Search - Start to Goal -----------------------------------------------------------#
def Astarsearch(start,goal):
  """
  A* search from startNode to goalNode
  Returns the goalNode reached. Returns None when goalNode cannot be reached
  """
  global count_expandednodes,startNode,goalNode
  startNode = start
  goalNode = goal
  count_expandednodes = 0
  current_Node = None
  searchTree = PriorityQueue()
  alreadySeen = set()
  searchTree.push((startNode,0),heuristic_aroundNet(startNode,goalNode))
  goalFound = False
  while(not(searchTree.isEmpty())):
      current_item = searchTree.pop()
      current_Node = current_item[0]
      cost_soFar = current_item[1]
      if goalTest(current_Node):
          print "goal cost",cost_soFar, "goalNode",current_Node.get_stateparams()
          goalFound = True
          break
      else:      
          (x,y,theta,v) = current_Node.get_stateparams()
          if (x,y,theta) not in alreadySeen:
              #graphics.draw_point(x,y,theta,'blue')
              count_expandednodes += 1
              alreadySeen.add((x,y,theta))      
              children = current_Node.getSuccessors_Astar()
              for child in children:
                  stepcost = cost(current_Node,child.getAction(),child,goalNode)                  
                  if stepcost != float('inf'):                      
                      g = cost_soFar + stepcost                       
                      h = heuristic_aroundNet(child,goalNode)
                      searchTree.push((child,g),g + h)

  print "expanded nodes",count_expandednodes
  if goalFound == True:
      return current_Node
  else:
      return None
                                                    
# ----------------------------------   LPA* search -----------------------------------------#
def getEntry(queue,Node):
    """ 
    returns the heapqueue entry that contains Node as the item.
    If no entry exists, return None
    """
    for item in queue:
        if (item[2] == Node):
            return item
    return None    
                                                    
def search_inS((x,y,th,v)):
   """
   Finds the the node in S with stateparams = (x,y,th,v).
   If the search fails, return None
   """
   for Node in S:
       (x1,y1,th1,v1) = Node.get_stateparams()
       if(x1,y1,th1) == (x,y,th):
           return Node
   return None

def CalcKey(Node):
    global startNode,goalNode
    g = Node.get_g()
    h = heuristic_aroundNet(Node,goalNode)
    rhs = Node.get_rhs() 
    min_g_rhs = min(g,rhs)
    return (min_g_rhs + h, min_g_rhs)

def compute_edges_affected(obstacle_topleft,obstacle_bottomright):
    """
    Given a new rectangular obstacle, return all edges which have a changed cost
    an edge is a tuple: (predecessor,node,action)
    """
    global S
    """
    for node in S:
        for (predecessor,action,stepcost) in node.get_predecessors():    
            print predecessor.get_stateparams(),action,node.get_stateparams()
        raw_input()
        print "\n\n\n"
    """
    #print "obstacle",obstacle_topleft,obstacle_bottomright    
    edges_affected = []
    obstacle_x = (obstacle_topleft[0] + obstacle_bottomright[0])/2
    obstacle_y = (obstacle_topleft[1] + obstacle_bottomright[1])/2
    # iterate through nodes in graph
    for node in S:
        #print node.get_stateparams()            
        # for each node, iterate through the edges that point to it
        for (predecessor,action,stepcost) in node.get_predecessors():                
            if predecessor not in S:
                continue
            (state_x,state_y,state_th,state_v) = predecessor.get_stateparams()           
            edgechange_seen = False            
            # if node is sufficiently close to the new obstacle, check to see if swath intersects boundary
            if(distance_Euclidean(state_x,state_y,obstacle_x,obstacle_y) < 200):                
                swath_indices_origin = controlset.getSwath(predecessor,action)
                for strip in swath_indices_origin:   # iterate through vertical strips. each strip is a tuple (x,y1,yn)
                    x_coord = state_x + strip[0]*CELL_SIZE     
                    if(obstacle_topleft[0] <= x_coord <= obstacle_bottomright[0]):                        
                        #print state_x,state_y
                        for y_index in range(strip[1],strip[2]+1):        # iterate through points in strip
                            y_coord = state_y + y_index*CELL_SIZE                 
                            if(obstacle_bottomright[1] <= y_coord <= obstacle_topleft[1]):
                                #print "y_coord in range"
                                # this edge runs throught the new obstacle
                                edges_affected.append((predecessor,node,action))
                                # update the edge cost
                                node.update_cost(predecessor,action,float('inf'))
                                edgechange_seen = True
                            if edgechange_seen:
                                break

                if edgechange_seen:
                    break
                                
    print len(edges_affected),"edges affected; len(S) = ",len(S)    
    """
    for edge in edges_affected:
        print edge[0].get_stateparams(),edge[2],edge[1].get_stateparams()
        raw_input()
    """
    return edges_affected

def Initialize():
    global U,S,startNode,goalNode,count_expandednodes
    U = []
    S = []
    startNode.set_rhs(0)
    Key = CalcKey(startNode)
    heapq.heappush(U,(Key[0],Key[1],startNode))
    S.append(startNode)
    S.append(goalNode)
    count_expandednodes = 0

def UpdateVertex(Node):    
    global U,startNode,goalNode
    if(Node.get_stateparams()[0:2] != startNode.get_stateparams()[0:2]): # if node != startnode, rhs = min_all_pred(g(pred) + cost(pred,node))        
        Node_preds = Node.get_predecessors()    
        values = [ (pred[0].get_g() + pred[2]) for pred in Node_preds]
        Node.set_rhs(min(values))
    
    entry = getEntry(U,Node)
    if (entry != None):
        U.remove(entry)
        heapq.heapify(U)

    if(Node.get_rhs()!= Node.get_g()):
        Key = CalcKey(Node)
        heapq.heappush(U,(Key[0],Key[1],Node))

def ComputeShortestPath():    
    global U,plan_LPAstar,startNode,goalNode,count_expandednodes   
    while ((U[0][0],U[0][1]) < CalcKey(goalNode)) or (goalNode.get_rhs() != goalNode.get_g()):            
        (Nodekey1,Nodekey2,Node) = heapq.heappop(U)       
        (x,y,theta,v) = Node.get_stateparams()
        #graphics.draw_point(x,y,theta,'blue')
        count_expandednodes += 1
        #print Node.get_stateparams(),"key",CalcKey(Node)
        #print "Nodekey",(Nodekey1,Nodekey2),"goalKey",CalcKey(goalNode),"goalrhs",goalNode.get_rhs(),"goalg",goalNode.get_g()
        #raw_input("hit enter")
        rhs = Node.get_rhs()
        #print "rhs",rhs
        if (Node.get_g() > rhs): # Underconsistent
            #print "under"
            Node.set_g(rhs)
            for successor in Node.getSuccessors_LPAstar(goalNode):
                UpdateVertex(successor)
        else:                    # Overconsistent
            Node.set_g(float('inf'))
            #print "over"
            toUpdate = Node.getSuccessors_LPAstar(goalNode)
            toUpdate.append(Node)
            for successor in toUpdate:
                UpdateVertex(successor)

    """ populate plan in global list """
    print "expanded",count_expandednodes,"nodes"
    count_expandednodes = 0
    print "computed path...populating!"
    node = goalNode
    plan_LPAstar = []    
    while(node!= startNode):
        predecessors = node.get_predecessors()
        backwardcost_min = float('inf')
        pred_mincost = None
        for (pred,action,stepcost) in predecessors:
            if (stepcost + pred.get_g()) < backwardcost_min:
                pred_mincost = (pred,action)
                backwardcost_min = stepcost + pred.get_g()
        node = pred_mincost[0]
        #print node.get_stateparams(),"cost",pred_mincost[1],"g",node.get_g()
        #raw_input("hit enter")        
        plan_LPAstar.append(pred_mincost)
    plan_LPAstar.reverse()

def LPAstarsearch(start,goal):
    global startNode,goalNode
    startNode = start
    goalNode = goal
    Initialize()   
    print "finished initialize"
    ComputeShortestPath()     
    print "finished computing shortest path"

def LPAstarsearch_replan(edges_affected):
    global startNode,goalNode
    print "replanning!"
    for edge_affected in edges_affected:        
        edge_endNode = edge_affected[1]
        UpdateVertex(edge_endNode)
    print "finished updating, computing path"
    ComputeShortestPath()
    print "finished computing shortest path"
    return            

# ------------------------------------------ Lazy MT-Adaptive A* search ---------------------#
# globals
counter = 0 # stores the current search iteration
pathcost = [] # stores pathcosts of previous searches
deltah = [] # stores the correction values ???
Generated = None # stores all generated nodes
plan_MTAdaptiveAstar = []

def InitializeState_MTAdaptiveAstar(node):
    node_search = node.get_search()
    node_h = node.get_h()
    if (node_search != counter) and (node_search != 0): # node has been generated during a previous search
        if(node.get_g() + node.get_h() < pathcost[node_search]): # node has been expanded during the same search
            node_h = pathcost[node_search] - node.get_g()
            node.set_h(node_h)
        #print "counter",counter
        node_h = node_h - (deltah[counter] - deltah[node_search])
        node_h = max(node_h,heuristic_aroundNet(node,goalNode))
        node.set_h(node_h)
        node.set_g(float('inf'))
    
    elif node_search == 0:
        node_h = heuristic_aroundNet(node,goalNode)
        node.set_h(node_h)
    node.set_search(counter)
            
            
def ComputePath_MTAdaptiveAstar():
  """
  A* search from startNode to goalNode
  Returns the goalNode reached. Returns None when goalNode cannot be reached
  """
  global count_expandednodes
  count_expandednodes = 0
  current_Node = None
  searchTree = PriorityQueue()
  alreadySeen = set()
  searchTree.push((startNode,0,None,None),startNode.get_h())
  goalFound = False
  while(not(searchTree.isEmpty())):
      current_item = searchTree.pop()
      current_Node = current_item[0]

      cost_soFar = current_item[1]      
      #if(current_Node.get_stateparams()[0:3] == (1470.0,1645.0,0.0)):
      #    print (1470.0,1645.0,0.0),"expanded, parent:",current_item[2].get_stateparams(),"action",current_item[3]
      
      if goalTest(current_Node):
          current_Node.set_g(cost_soFar)
          current_Node.addParent(current_item[2])
          current_Node.setAction(current_item[3])
          print "goal cost",cost_soFar, "goalNode",current_Node.get_stateparams()
          goalFound = True
          break
      else:      
          (x,y,theta,v) = current_Node.get_stateparams()
          if (x,y,theta) not in alreadySeen:     
              current_Node.set_g(cost_soFar)
              current_Node.addParent(current_item[2])
              current_Node.setAction(current_item[3])         
              count_expandednodes += 1
              #print count_expandednodes
              alreadySeen.add((x,y,theta))      
              children = current_Node.getSuccessors_MTAdaptiveAstar(alreadySeen)
              for child,action in children:                  
                  InitializeState_MTAdaptiveAstar(child)
                  stepcost = cost(current_Node,action,child,goalNode)                  
                  if stepcost != float('inf'):                      
                      g = cost_soFar + stepcost                                           
                      h = child.get_h()
                      searchTree.push((child,g,current_Node,action),g + h)

  print "expanded nodes",count_expandednodes
  if goalFound == True:
      return current_Node
  else:
      return None
       
def MTAdaptiveAstarsearch_update(start,goal):
    """
    update plan using Lazy MT-AdaptiveA*
    """
    global startNode,goalNode,counter,deltah,Generated,pathcost
    startNode = start    
    
    # check if startNode has already been generated, so we don't generate a duplicate
    if(Generated.has_key(startNode.get_stateparams()[0:3])):
        startNode = Generated[startNode.get_stateparams()[0:3]]
    else:    
        Generated[startNode.get_stateparams()[0:3]] = startNode

    # check if goal has already been generated, so we don't generate a duplicate
    goalNode = goal
    if(Generated.has_key(goalNode.get_stateparams()[0:3])):
        goalNode = Generated[goalNode.get_stateparams()[0:3]]
    else:    
        Generated[goalNode.get_stateparams()[0:3]] = goalNode

    if(not goalTest(startNode)):
        InitializeState_MTAdaptiveAstar(goalNode)
        if (goal.get_g() + goal.get_h() < pathcost[counter]):
            goal.set_h(pathcost[counter] - goal.get_g())
        deltah.append(deltah[counter] + goal.get_h())
        goalNode = goal

        # wrap around (line 28 - 37 in paper)        
        counter += 1
        InitializeState_MTAdaptiveAstar(startNode)
        InitializeState_MTAdaptiveAstar(goalNode)
        startNode.set_g(0)        
        finalnode = ComputePath_MTAdaptiveAstar()               
        pathcost.append(finalnode.get_g())
        if finalnode == None: # goal unreachable
            return None 
        
        node = finalnode
        path = []        
        while(node.getParent()!= None):
            parent = node.getParent()
            print parent.get_stateparams(),node.getAction(),node.get_stateparams()
            path.append((parent,node.getAction()))                       
            node = parent         
        #print "path of length",len(path)
        path.reverse()
        return path

def MTAdaptiveAstarsearch_start(start,goal):
    """
    start new MTAdaptiveAstar search
    """
    global startNode,goalNode,counter,deltah,Generated,pathcost
    startNode = start
    goalNode = goal
    Generated = {}
    Generated[startNode.get_stateparams()[0:3]] = startNode
    Generated[goalNode.get_stateparams()[0:3]] = goalNode
    deltah = []
    deltah.append("head of deltah")
    deltah.append(0)
    pathcost = []
    pathcost.append("head of pathcost")
    counter = 0
    #if(startNode.get_stateparams() != goalNode.get_stateparams()):
    if(not goalTest(startNode)):
        counter += 1
        InitializeState_MTAdaptiveAstar(startNode)
        InitializeState_MTAdaptiveAstar(goalNode)
        startNode.set_g(0)        
        finalnode = ComputePath_MTAdaptiveAstar()                  
        pathcost.append(finalnode.get_g())
        if finalnode == None: # goal unreachable
            return None 
        
        node = finalnode
        path = []        
        while(node.getParent()!= None):
            parent = node.getParent()
            #print parent.get_stateparams(),node.getAction(),node.get_stateparams()
            path.append((parent,node.getAction()))                       
            node = parent         
        #print "path of length",len(path)
        path.reverse()
        return path
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
# -------- utilities for converting plan to path ------------------------------------#
def points_turnLeft(x,y,theta,v,direction,distance,radius):    
    points = []
    d = 5
    while(d <= distance):
        (x2,y2,theta2,v2) = turn_Left((x,y,theta,v),direction,d,radius)
        points.append((x2/100.0,y2/100.0,theta2,"t",direction))
        d+=5
    (x_temp,y_temp,theta_temp,v_temp) = turn_Left((x,y,theta,v),direction,distance,radius)
    
    points.append((x_temp/100.0,y_temp/100.0,theta_temp,"t",direction))
    return points

def points_turnRight(x,y,theta,v,direction,distance,radius):
    points = []
    d = 5
    while(d <= distance):
        (x2,y2,theta2,v2) = turn_Right((x,y,theta,v),direction,d,radius)
        points.append((x2/100.0,y2/100.0,theta2,"t",direction))
        d+=5
    (x_temp,y_temp,theta_temp,v_temp) = turn_Right((x,y,theta,v),direction,distance,radius)
    points.append((x_temp/100.0,y_temp/100.0,theta_temp,"t",direction))
    return points

def points_goStraight(x,y,theta,v,direction,distance):
    points = []
    d = 5
    while(d <= distance):
        (x2,y2,theta2,v2) = go_Straight((x,y,theta,v),direction,d)
        points.append((x2/100.0,y2/100.0,theta2,"s",direction))
        d+=5
    (x_temp,y_temp,theta_temp,v_temp) = go_Straight((x,y,theta,v),direction,distance)
    points.append((x_temp/100.0,y_temp/100.0,theta_temp,"s",direction))
    return points

def plan_to_path(plan):
    path = []
    for (Node,action) in plan:
        (x,y,theta,v) = Node.get_stateparams()

        if action in ("B","L_b","R_b","B_diag","B1_26.6","B1_63.4"):
            direction = "b"    
        else:
            direction = "f"            
    
        if action in ("R_f","R_b"): # turning right            
            path = path + points_turnRight(x,y,theta,v,direction,ROBOT_RADIUS_MIN*math.pi/2,ROBOT_RADIUS_MIN)            

        elif action in ("L_f","L_b"): # turning left
            path = path + points_turnLeft(x,y,theta,v,direction,ROBOT_RADIUS_MIN*math.pi/2,ROBOT_RADIUS_MIN)

        elif action in ("F","B"):                          
            path = path + points_goStraight(x,y,theta,v,direction,CELL_SIZE)            

        elif action in ("F3"):
            path = path + points_goStraight(x,y,theta,v,direction,3*CELL_SIZE)

        elif action == "SL_f":
            path = path + points_goStraight(x,y,theta,v,direction,20.5)
            path = path + points_turnLeft(path[-1][0]*100.0,path[-1][1]*100.0,path[-1][2],v,direction,ROBOT_RADIUS_2*math.pi/4,ROBOT_RADIUS_2)

        elif action == "SR_f":
            path = path + points_goStraight(x,y,theta,v,direction,20.5)            
            path = path + points_turnRight(path[-1][0]*100.0,path[-1][1]*100.0,path[-1][2],v,direction,ROBOT_RADIUS_2*math.pi/4,ROBOT_RADIUS_2)   
    
        elif action == "LS_f":
            path = path + points_turnLeft(x,y,theta,v,direction,ROBOT_RADIUS_2*math.pi/4,ROBOT_RADIUS_2)
            (x_temp,y_temp,theta_temp) = path[-1][0:3]
            path = path + points_goStraight(x_temp*100.0,y_temp*100.0,theta_temp,v,direction,20.5)

        elif action == "RS_f":
            path = path + points_turnRight(x,y,theta,v,direction,ROBOT_RADIUS_2*math.pi/4,ROBOT_RADIUS_2)
            (x_temp,y_temp,theta_temp) = path[-1][0:3]
            path = path + points_goStraight(x_temp*100.0,y_temp*100.0,theta_temp,v,direction,20.5)       

        elif action == "F_diag":            
            path = path + points_goStraight(x,y,theta,v,direction,CELL_SIZE*math.sqrt(2))  
    
        elif action == "F_diag3":
            path = path + points_goStraight(x,y,theta,v,direction,3*CELL_SIZE*math.sqrt(2))  

        elif action == "B_diag":
            path = path + points_goStraight(x,y,theta,v,direction,CELL_SIZE*math.sqrt(2))

        elif action == "RS_f_short":
            path = path + points_turnRight(x,y,theta,v,direction,ROBOT_RADIUS_3*math.pi/4,ROBOT_RADIUS_3)
            (x_temp,y_temp,theta_temp) = path[-1][0:3]
            path = path + points_goStraight(x_temp*100.0,y_temp*100.0,theta_temp,v,direction,14.4957)        
                
        elif action == "LS_f_short":            
            path = path + points_turnLeft(x,y,theta,v,direction,ROBOT_RADIUS_3*math.pi/4,ROBOT_RADIUS_3)
            (x_temp,y_temp,theta_temp) = path[-1][0:3]
            path = path + points_goStraight(x_temp*100.0,y_temp*100.0,theta_temp,v,direction,14.4957)            

        elif action == "sidestep_R_f":
            path = path + points_turnRight(x,y,theta,v,direction,31.189,ROBOT_RADIUS_MIN)
            (x_temp,y_temp,theta_temp) = path[-1][0:3]
            path = path + points_goStraight(x_temp*100.0,y_temp*100.0,theta_temp,v,direction,49.5)
            (x_temp,y_temp,theta_temp) = path[-1][0:3]                    
            path = path + points_turnLeft(x_temp*100.0,y_temp*100.0,theta_temp,v,direction,31.189,ROBOT_RADIUS_MIN)       

        elif action == "sidestep_L_f":
            path = path + points_turnLeft(x,y,theta,v,direction,31.189,ROBOT_RADIUS_MIN)
            (x_temp,y_temp,theta_temp) = path[-1][0:3]
            path = path + points_goStraight(x_temp*100.0,y_temp*100.0,theta_temp,v,direction,49.5)
            (x_temp,y_temp,theta_temp) = path[-1][0:3]                   
            path = path + points_turnRight(x_temp*100.0,y_temp*100.0,theta_temp,v,direction,31.189,ROBOT_RADIUS_MIN)

        elif action == "L2_f":
            path = path + points_turnLeft(x,y,theta,v,direction,ROBOT_RADIUS_4*math.pi/2,ROBOT_RADIUS_4)    
    
        elif action == "R2_f":
            path = path + points_turnRight(x,y,theta,v,direction,ROBOT_RADIUS_4*math.pi/2,ROBOT_RADIUS_4)        
            
        elif action == "SR_f_2":
            path = path + points_goStraight(x,y,theta,v,direction,13.369)
            (x_temp,y_temp,theta_temp) = path[-1][0:3]
            path = path + points_turnRight(x_temp*100.0,y_temp*100.0,theta_temp,v,direction,(63.4/360.0)*2*math.pi*ROBOT_RADIUS_5,ROBOT_RADIUS_5)      

        elif action == "SL_f_2":
            path = path + points_goStraight(x,y,theta,v,direction,13.369)
            (x_temp,y_temp,theta_temp) = path[-1][0:3]
            path = path + points_turnLeft(x_temp*100.0,y_temp*100.0,theta_temp,v,direction,(63.4/360.0)*2*math.pi*ROBOT_RADIUS_5,ROBOT_RADIUS_5)

        elif action == "RSR_f":
            path = path + points_turnRight(x,y,theta,v,direction,13.3968,ROBOT_RADIUS_5)
            (x_temp,y_temp,theta_temp) = path[-1][0:3]
            path = path + points_goStraight(x_temp*100.0,y_temp*100.0,theta_temp,v,direction,53.9857)
            (x_temp,y_temp,theta_temp) = path[-1][0:3]
            path = path + points_turnRight(x_temp*100.0,y_temp*100.0,theta_temp,v,direction,22.2,ROBOT_RADIUS_5)       
        
        elif action == "LSL_f":
            path = path + points_turnLeft(x,y,theta,v,direction,13.3968,ROBOT_RADIUS_5)
            (x_temp,y_temp,theta_temp,v_temp) = path[-1][0:3]
            path = path + points_goStraight(x_temp*100.0,y_temp*100.0,theta_temp,v,direction,53.9857)
            (x_temp,y_temp,theta_temp,v_temp) = path[-1][0:3]
            path = path + points_turnLeft(x_temp*100.0,y_temp*100.0,theta_temp,v,direction,22.2,ROBOT_RADIUS_5)       
            
        elif action in ("F1_26.6","B1_26.6","F1_63.4","B1_63.4"):
            path = path + points_goStraight(x,y,theta,v,direction,39.131)

        elif action == "LSL1_f_26.6":
            path = path + points_turnLeft(x,y,theta,v,direction,41.95,ROBOT_RADIUS_MIN)
            (x_temp,y_temp,theta_temp) = path[-1][0:3]
            path = path +  points_goStraight(x_temp*100.0,y_temp*100.0,theta_temp,v,direction,28.496)
            (x_temp,y_temp,theta_temp) = path[-1][0:3]
            path = path + points_turnLeft(x_temp*100.0,y_temp*100.0,theta_temp,v,direction,35.508,ROBOT_RADIUS_MIN)        

        elif action == "LSL2_f_26.6":
            path = path + points_turnLeft(x,y,theta,v,direction,12.99,ROBOT_RADIUS_MIN)
            (x_temp,y_temp,theta_temp) = path[-1][0:3]
            path = path + points_goStraight(x_temp*100.0,y_temp*100.0,theta_temp,v,direction,65.6)
            (x_temp,y_temp,theta_temp,v_temp) = path[-1][0:3]
            path = path + points_turnLeft(x_temp*100.0,y_temp*100.0,theta_temp,v,direction,9.487,ROBOT_RADIUS_MIN)       

        elif action == "RSR1_f_26.6":
            path = path + points_turnRight(x,y,theta,v,direction,14.625,ROBOT_RADIUS_MIN)
            (x_temp,y_temp,theta_temp,v_temp) = path[-1][0:3]
            path = path + points_goStraight(x_temp*100.0,y_temp*100.0,theta_temp,v,direction,39.952)
            (x_temp,y_temp,theta_temp,v_temp) = path[-1][0:3]
            path = path + points_turnRight(x_temp*100.0,y_temp*100.0,theta_temp,v,direction,17.873,ROBOT_RADIUS_MIN)

        elif action == "RSR2_f_26.6":
            path = path + points_turnRight(x,y,theta,v,direction,84.055,ROBOT_RADIUS_MIN)
            (x_temp,y_temp,theta_temp,v_temp) = path[-1][0:3]
            path = path + points_goStraight(x_temp*100.0,y_temp*100.0,theta_temp,v,direction,32.613)
            (x_temp,y_temp,theta_temp,v_temp) = path[-1][0:3]
            path = path + points_turnRight(x_temp*100.0,y_temp*100.0,theta_temp,v,direction,3.42,ROBOT_RADIUS_MIN)
        
        elif action == "LSL1_f_63.4":
            path = path + points_turnLeft(x,y,theta,v,direction,14.625,ROBOT_RADIUS_MIN)
            (x_temp,y_temp,theta_temp,v_temp) = path[-1][0:3]
            path = path + points_goStraight(x_temp*100.0,y_temp*100.0,theta_temp,v,direction,39.952)
            (x_temp,y_temp,theta_temp,v_temp) = path[-1][0:3]
            path = path + points_turnLeft(x_temp*100.0,y_temp*100.0,theta_temp,v,direction,17.783,ROBOT_RADIUS_MIN)        

        elif action == "LSL2_f_63.4":
            path = path + points_turnLeft(x,y,theta,v,direction,84.055,ROBOT_RADIUS_MIN)            
            (x_temp,y_temp,theta_temp,v_temp) = path[-1][0:3]
            path = path + points_goStraight(x_temp*100.0,y_temp*100.0,theta_temp,v,direction,32.613)
            (x_temp,y_temp,theta_temp,v_temp) = path[-1][0:3]
            path = path + points_turnLeft(x_temp*100.0,y_temp*100.0,theta_temp,v,direction,3.42,ROBOT_RADIUS_MIN)            

        elif action == "RSR1_f_63.4":
            path = path + points_turnRight(x,y,theta,v,direction,41.95,ROBOT_RADIUS_MIN)
            (x_temp,y_temp,theta_temp,v_temp) = path[-1][0:3]
            path = path + points_goStraight(x_temp*100.0,y_temp*100.0,theta_temp,v,direction,28.496)
            (x_temp,y_temp,theta_temp,v_temp) = path[-1][0:3]
            path = path + points_turnRight(x_temp*100.0,y_temp*100.0,theta_temp,v,direction,35.508,ROBOT_RADIUS_MIN)

        elif action == "RSR2_f_63.4":
            path = path + points_turnRight(x,y,theta,v,direction,12.99,ROBOT_RADIUS_MIN)
            (x_temp,y_temp,theta_temp,v_temp) = path[-1][0:3]
            path = path + points_goStraight(x_temp*100.0,y_temp*100.0,theta_temp,v,direction,65.6)
            (x_temp,y_temp,theta_temp,v_temp) = path[-1][0:3]
            path = path + points_turnRight(x_temp*100.0,y_temp*100.0,theta_temp,v_temp,direction,9.487,ROBOT_RADIUS_MIN)

    return path
