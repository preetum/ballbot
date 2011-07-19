import util
import math

# ------------- Precomputed control set nextStates, costs etc ------------------------------------#
class ControlSet:
  """
  implements the control set for Ballbot
  """
  
  def __init__(self):  
    ROBOT_SPEED_MAX  = util.ROBOT_SPEED_MAX
    
    v = ROBOT_SPEED_MAX          

    self.actions_90s = ["R_f","R_b","L_f","L_b","F","F3","B","SR_f","SL_f"]
    self.actions_45s = ["LS_f","RS_f","F_diag","F_diag3","B_diag"]
    self.init_actions(v)
    self.init_swaths()
    
  def init_actions(self,v):
    ROBOT_RADIUS_MIN = util.ROBOT_RADIUS_MIN
    ROBOT_RADIUS_2   = util.ROBOT_RADIUS_2
    CELL_SIZE        = util.CELL_SIZE

    # map actions from origin to lattice points, for heading 0
    state_atOrigin = (0,0,0,v)
    self.theta_0      = {"R_f":(ROBOT_RADIUS_MIN,-ROBOT_RADIUS_MIN,3*math.pi/2),
                         "R_b":(-ROBOT_RADIUS_MIN,-ROBOT_RADIUS_MIN,math.pi/2),
                         "L_f":(ROBOT_RADIUS_MIN,ROBOT_RADIUS_MIN,math.pi/2),
                         "L_b":(-ROBOT_RADIUS_MIN,ROBOT_RADIUS_MIN,3*math.pi/2),                        
                         "F"  :(CELL_SIZE,0,0),
                         "F3" :(3*CELL_SIZE,0,0),
                         "B"  :(-CELL_SIZE,0,0),
                         "SR_f":(3*ROBOT_RADIUS_MIN/2,-ROBOT_RADIUS_MIN/2,7*math.pi/4,v),
                         "SL_f":(3*ROBOT_RADIUS_MIN/2,ROBOT_RADIUS_MIN/2,math.pi/4,v)}

    # map actions from origin to lattice points, for heading 90
    state_atOrigin = (0,0,math.pi/2,v)
    self.theta_90      =  {"R_f":(ROBOT_RADIUS_MIN,ROBOT_RADIUS_MIN,0,v),
                           "R_b":(ROBOT_RADIUS_MIN,-ROBOT_RADIUS_MIN,math.pi,v),
                           "L_f":(-ROBOT_RADIUS_MIN,ROBOT_RADIUS_MIN,math.pi,v),
                           "L_b":(-ROBOT_RADIUS_MIN,-ROBOT_RADIUS_MIN,0,v),
                           "F"  :(0,CELL_SIZE,math.pi/2),
                           "F3" :(0,3*CELL_SIZE,math.pi/2),
                           "B"  :(0,-CELL_SIZE,math.pi/2),
                           "SR_f": (ROBOT_RADIUS_MIN/2,3*ROBOT_RADIUS_MIN/2,math.pi/4,v),
                           "SL_f":(-ROBOT_RADIUS_MIN/2,3*ROBOT_RADIUS_MIN/2,3*math.pi/4,v) }
    
    # map actions from origin to lattice points, for heading 180
    state_atOrigin = (0,0,math.pi,v)
    self.theta_180     =  {"R_f":(-ROBOT_RADIUS_MIN,ROBOT_RADIUS_MIN,math.pi/2,v),
                           "R_b":(ROBOT_RADIUS_MIN,ROBOT_RADIUS_MIN,3*math.pi/2,v),
                           "L_f":(-ROBOT_RADIUS_MIN,-ROBOT_RADIUS_MIN,3*math.pi/2,v),
                           "L_b":(ROBOT_RADIUS_MIN,-ROBOT_RADIUS_MIN,math.pi/2,v),
                           "F"  :(-CELL_SIZE,0,math.pi),
                           "F3": (-3*CELL_SIZE,0,math.pi),
                           "B"  :(CELL_SIZE,0,math.pi),
                           "SR_f":(-3*ROBOT_RADIUS_MIN/2,ROBOT_RADIUS_MIN/2,math.pi/4,v),
                           "SL_f":(-3*ROBOT_RADIUS_MIN/2,-ROBOT_RADIUS_MIN/2,5*math.pi/4,v)}
    
    # map actions from origin to lattice points, for heading 270
    state_atOrigin = (0,0,3*math.pi/2,v)
    self.theta_270     =  {"R_f":(-ROBOT_RADIUS_MIN,-ROBOT_RADIUS_MIN,math.pi,v),
                           "R_b":(-ROBOT_RADIUS_MIN,ROBOT_RADIUS_MIN,0,v),
                           "L_f":(ROBOT_RADIUS_MIN,-ROBOT_RADIUS_MIN,0,v),
                           "L_b":(ROBOT_RADIUS_MIN,ROBOT_RADIUS_MIN,math.pi,v),
                           "F"  :(0,-CELL_SIZE,3*math.pi/2),
                           "F3" : (0,-3*CELL_SIZE,3*math.pi/2),
                           "B"  :(0,CELL_SIZE,3*math.pi/2),
                           "SR_f": (-ROBOT_RADIUS_MIN/2,-3*ROBOT_RADIUS_MIN/2,5*math.pi/4,v),
                           "SL_f": (ROBOT_RADIUS_MIN/2,-3*ROBOT_RADIUS_MIN/2,7*math.pi/4,v)}

    # map actions from origin to lattice points, for heading 45
    state_atOrigin = (0,0,math.pi/4,v)
    self.theta_45  = {"F_diag": (CELL_SIZE,CELL_SIZE,math.pi/4,v),
                      "F_diag3":(3*CELL_SIZE,3*CELL_SIZE,math.pi/4,v),
                      "B_diag": (-CELL_SIZE,-CELL_SIZE,math.pi/4,v),
                      "LS_f"  : (ROBOT_RADIUS_MIN/2,3*ROBOT_RADIUS_MIN/2,math.pi/2,v),
                      "RS_f"  : (3*ROBOT_RADIUS_MIN/2,ROBOT_RADIUS_MIN/2,0,v)}                          

    # map actions from origin to lattice points, for heading 135
    state_atOrigin = (0,0,3*math.pi/4,v)
    self.theta_135  =  {"F_diag": (-CELL_SIZE,CELL_SIZE,3*math.pi/4,v),
                        "F_diag3": (-3*CELL_SIZE,3*CELL_SIZE,3*math.pi/4,v),
                        "B_diag": (CELL_SIZE,-CELL_SIZE,3*math.pi/4,v),
                        "LS_f"  : (-3*ROBOT_RADIUS_MIN/2,ROBOT_RADIUS_MIN/2,math.pi,v),
                        "RS_f"  : (-ROBOT_RADIUS_MIN/2,3*ROBOT_RADIUS_MIN/2,math.pi/2,v)}                          

    # map actions from origin to lattice points, for heading 225
    state_atOrigin = (0,0,5*math.pi/4,v)
    self.theta_225  = {"F_diag": (-CELL_SIZE,-CELL_SIZE,5*math.pi/4,v),
                       "F_diag3":(-3*CELL_SIZE,-3*CELL_SIZE,5*math.pi/4,v),
                       "B_diag": (CELL_SIZE,CELL_SIZE,5*math.pi/4,v),
                       "LS_f"  : (-ROBOT_RADIUS_MIN/2,-3*ROBOT_RADIUS_MIN/2,3*math.pi/2,v),
                       "RS_f"  : (-3*ROBOT_RADIUS_MIN/2,-1*ROBOT_RADIUS_MIN/2,math.pi,v)}  

    state_atOrigin = (0,0,7*math.pi/4,v)
    self.theta_315  = {"F_diag":(CELL_SIZE,-CELL_SIZE,7*math.pi/4,v),
                       "F_diag3":(3*CELL_SIZE,-3*CELL_SIZE,7*math.pi/4,v),
                       "B_diag": (-CELL_SIZE,CELL_SIZE,7*math.pi/4,v),
                       "LS_f"  : (3*ROBOT_RADIUS_MIN/2,-ROBOT_RADIUS_MIN/2,0,v),
                       "RS_f"  : (-3*ROBOT_RADIUS_MIN/2,ROBOT_RADIUS_MIN/2,3*math.pi/2,v)}

  def init_swaths(self):
    self.swath_0 = {}
    for action in self.actions_90s:
      self.swath_0[action] = getindices_origin_0(action)
      
    self.swath_90 = {}
    for action in self.actions_90s:
      self.swath_90[action] = getindices_origin_90(action)

    self.swath_180 = {}
    for action in self.actions_90s:
      self.swath_180[action] = getindices_origin_180(action)

    self.swath_270 = {}
    for action in self.actions_90s:
      self.swath_270[action] = getindices_origin_270(action)

    self.swath_45 = {}
    for action in self.actions_45s:
      self.swath_45[action] = getindices_origin_45(action)

    self.swath_135 = {}
    for action in self.actions_45s:
      self.swath_135[action] = getindices_origin_135(action)

    self.swath_225 = {}
    for action in self.actions_45s:
      self.swath_225[action] = getindices_origin_225(action)
      
    self.swath_315 = {}
    for action in self.actions_45s:
      self.swath_315[action] = getindices_origin_315(action)

  def action_to_stateparams(self,state,action):
    """
    Given current state and action, this function returns the state reached on taking 'action' from origin.
    This has already been precomputed in the __init__ function of this class
    """    
    (x,y,theta,v) = state.get_stateparams()
    if(theta == 0): #heading 0
      control_set = self.theta_0
    elif(theta == math.pi/2): #heading 90
      control_set = self.theta_90 
    elif(theta == math.pi): # heading 180
      control_set = self.theta_180
    elif(theta == 3*math.pi/2): # heading 270
      control_set = self.theta_270  

    elif(theta == math.pi/4): # heading 45
      control_set = self.theta_45
    elif(theta == 3*math.pi/4): # heading 135
      control_set = self.theta_135
    elif(theta == 5*math.pi/4): # heading 225
      control_set = self.theta_225
    elif(theta == 7*math.pi/4): # heading 315
      control_set = self.theta_315

    # get precomputed new state on taking action from origin
    newstate_origin = control_set[action]
    return newstate_origin

  def getActions(self,state):
      """
      return a list of all actions available to that state
      """
      (x,y,theta,v) = state.get_stateparams()
      if theta in (0,math.pi/2,math.pi,3*math.pi/2):
        return self.actions_90s
      elif theta in (math.pi/4,3*math.pi/4,5*math.pi/4,7*math.pi/4):
        return self.actions_45s

  def getSwath(self,state,action):
    """
    Given a state and an action, return the swath of car on taking 'action' from the state
    A swath is a list that looks like: [ (x1,y11,y1n), (x2,y21,y2n)....(xn,yn1,ynn)]
                                where: x1....xn are the x-indices of n-lattice cell strips occupied by car through this motion
                                       (y11,y1n)...(yn1,ynn) are the corresponding bottom and top y-indices for each strip
    cells are identified by their bottom left coordinate, 
    the origin is the bottom left vertex of the cell that has index (0,0)
    
    *** IMPROVE ABOVE DESCRIPTION!! ***
    """
    
    th = state.get_stateparams()[2]
    if(th == 0):
      indices = self.swath_0[action]
    elif(th == math.pi/2):
      indices = self.swath_90[action]
    elif(th == math.pi):
      indices = self.swath_180[action]
    elif(th == 3*math.pi/2):
      indices = self.swath_270[action]

    elif(th == math.pi/4):
      indices = self.swath_45[action]
    elif(th == 3*math.pi/4):
      indices = self.swath_135[action]
    elif(th == 5*math.pi/4):
      indices = self.swath_225[action]
    elif(th == 7*math.pi/4):
      indices = self.swath_315[action]

    return indices

def getindices_origin_0(action):
  """
  return swath indices when 'action' is taken from heading 0 at origin
  indices are written as [(x-index, min_y-index, max_y-index)...]
  """  
  if action == "R_f":
    indices = [(-2,-1,0),(-1,-1,0),(0,-1,0), (1,-2,0), (2,-3,0),(3,-5,-1),(4,-5,-2)]

  elif action == "F":
    indices = [(-2,-1,0),(-1,-1,0),(0,-1,0),(1,-1,0)]

  elif action == "F3":
    indices = [(-2,-1,0),(-1,-1,0),(0,-1,0),(1,-1,0),(2,-1,0),(3,-1,0),(4,-1,0)]

  elif action == "B":
    indices = [(-2,-1,0),(-1,-1,0),(0,-1,0),(1,-1,0)]

  elif action == "R_b":
    indices = [(-5,-5,-2),(-4,-5,-1),(-3,-3,0),(-2,-2,0),(-1,-1,0),(0,-1,0),(1,-1,0)]
  
  elif action == "L_f":
    indices = [(-2,-1,0),(-1,-1,0),(0,-1,0),(1,-1,1),(2,-1,2),(3,0,4),(4,1,4)]

  elif action == "L_b":
    indices = [(-5,1,4),(-4,0,4),(-3,-1,2),(-2,-1,1),(-1,-1,0),(0,-1,0),(1,-1,0)]

  elif action == "SR_f":
    indices = [(-2,-1,0),(-1,-1,0),(0,-1,0),(1,-1,0),(2,-1,0),(3,-2,0),(4,-3,0),(5,-3,-1),(6,-4,-1),(7,-3,-2)]

  elif action == "SL_f":
    indices = [(-2,-1,0),(-1,-1,0),(0,-1,0),(1,-1,0),(2,-1,0),(3,-1,1),(4,-1,2),(5,0,3),(6,0,3),(7,1,2)]               
    
  return indices



def getindices_origin_90(action):
  """
  return swath indices when 'action' is taken from heading 90 at origin
  """  
  if action == "R_f":
    indices = [(-1,-2,2),( 0,-2,3),( 1,1,4),( 2,2,4),(3,3,4),( 4,3,4)]

  elif action == "F":
    indices = [(-1,-2,1), (0,-2,1)]

  elif action == "F3":
    indices = [(-1,-2,4),( 0,-2,4)]

  elif action == "B":
    indices = [(-1,-2,1),( 0,-2,1)]

  elif action == "R_b":
    indices = [(-1,-3,1),( 0,-4,-1),( 1,-5,-2), ( 2,-5,-3),( 3,-5,-4),( 4,-5,-4)]
  
  elif action == "L_f":
    indices = [(-5,3,4),(-4,3,4),(-3,2,4),(-2,1,4),(-1,-2,3),( 0,-2,2)]

  elif action == "L_b":
    indices = [(-5,-5,-4),(-4,-5,-4),(-3,-5,-3),(-2,-5,-2),(-1,-4,1),(1,-3,1)]

  elif action == "SR_f":
    indices = [(-1,-2,4),( 0,-2,6),( 1,3,7),( 2,4,7),( 3,6,6)]

  elif action == "SL_f":
    indices = [(-4,6,6),(-3,4,7),(-2,3,7),(-1,-2,6),( 0,-2,4)]           
    
  return indices

def getindices_origin_180(action):
  """
  return swath indices when 'action' is taken from heading 180 at origin
  """  
  if action == "R_f":
    indices = [(-5,1,4),(-4,0,4),(-3,-1,2),(-2,-1,1),(-1,-1,0),(0,-1,0),(1,-1,0)]

  elif action == "F":
    indices = [(-2,-1,0), (-1,-1,0),(0,-1,0),(1,-1,0)]

  elif action == "F3":
    indices = [(-5,-1,0),(-4,-1,0),(-3,-1,0),(-2,-1,0),(-1,-1,0),(0,-1,0),(1,-1,0)]

  elif action == "B":
    indices = [(-2,-1,0), (-1,-1,0),(0,-1,0),(1,-1,0)]

  elif action == "R_b":
    indices = [(-2,-1,0),(-1,-1,0),(0,-1,0),(1,-1,1),(2,-1,2),(3,0,4),(4,1,4)]
  
  elif action == "L_f":
    indices = [(-5,-5,-2),(-4,-5,-1),(-3,-3,0),(-2,-2,0),(-1,-1,0),(0,-1,0),(1,-1,0)]

  elif action == "L_b":
    indices = [(-2,-1,0),(-1,-1,0),(0,-1,0),(1,-2,0),(2,-3,0),(3,-5,-1),(4,-5,-2)]

  elif action == "SR_f":
    indices = [(-8,1,2),(-7,0,3),(-6,0,2),(-5,-1,2),(-4,-1,1),(-3,-1,0),(-2,-1,0),(-1,-1,0),(0,-1,0),(1,-1,0)]

  elif action == "SL_f":
    indices = [(-8,-3,-2),(-7,-4,-1),(-6,-3,-1),(-5,-3,0),(-4,-2,0),(-3,-1,0),(-2,-1,0),(-1,-1,0),(0,-1,0),(1,-1,0)]           
    
  return indices

def getindices_origin_270(action):
  """
  return swath indices when 'action' is taken from heading 270 at origin
  """  
  if action == "R_f":
    indices = [(-5,-5,-4),(-4,-5,-4),(-3,-5,-3),(-2,-5,-2),(-1,-4,1),(0,-3,1)]

  elif action == "F":
    indices = [(-1,-2,1), (0,-2,1)]

  elif action == "F3":
    indices = [(-1,-5,1),(0,-5,1)]

  elif action == "B":
    indices = [(-1,-2,1), (0,-2,1)]

  elif action == "R_b":
    indices = [(-5,3,4),(-4,3,4),(-3,2,4),(-2,1,4),(-1,-2,3),(0,-2,2)]
  
  elif action == "L_f":
    indices = [(-1,-3,1),(0,-4,1),(1,-5,-2),(2,-5,-3),(3,-5,4),(4,-5,4)]

  elif action == "L_b":
    indices = [(-1,-2,2),( 0,-2,3),( 1,1,4),( 2,2,4),(3,3,4),( 4,3,4)]

  elif action == "SR_f":
    indices = [(-4,-7,-7),(-3,-5,-8),(-2,-4,-8),(-1,-7,1),(0,-5,1)]

  elif action == "SL_f":
    indices = [(-1,-5,1),(0,-7,1),(1,-8,-4),(2,-8,-5),(3,-7,-7)]           
    
  return indices


def getindices_origin_45(action):
  """
  return swath indices when 'action' is taken from heading 45 at origin
  """  
  if action == "RS_f":
    indices = [(-2,-1,0),(-1,-2,1),(0,-1,1),(1,0,2),(2,1,2),(3,1,2),(4,1,2),(5,1,2),(6,1,2),(7,1,2)]
  elif action == "LS_f":
    indices = [(-2,-1,0),(-1,-2,1),(0,-1,3),(0,0,7),(2,1,7)]
  elif action == "F_diag":
    indices = [(-2,-1,0),(-1,-2,1),(0,-1,2),(1,0,2),(2,1,1)]
  elif action == "F_diag3":
    indices = [(-2,-1,0),(-1,-2,1),(0,-1,2),(1,0,3),(2,1,5),(3,2,5),(4,3,4)]
  elif action == "B_diag":
    indices = [(-3,-2,-2),(-2,-3,-1),(-1,-2,1),(0,-2,1),(1,0,0)]

  return indices

def getindices_origin_135(action):
  """
  return swath indices when 'action' is taken from heading 135 at origin
  """  
  if action == "RS_f":
    indices = [(-3,1,7),(-2,-1,7),(-1,-2,1),(0,-2,0),(1,-1,-1)]
  elif action == "LS_f":
    indices = [(-8,1,2),(-7,1,2),(-6,1,2),(-5,1,2),(-4,0,2),(-3,0,2),(-2,-1,2),(-1,-2,1),(0,-2,0),(1,-1,-1)]
  elif action == "F_diag":
    indices = [(-3,0,1),(-2,-1,2),(-1,-2,1),(0,-2,0),(1,-1,-1)]
  elif action == "F_diag3":
    indices = [(-6,2,3),(-5,2,4),(-4,1,4),(-3,0,3),(-2,-1,2),(-1,-2,1),(0,-2,0),(1,-1,-1)]
  elif action == "B_diag":
    indices = [(-2,-1,0),(-1,-1,1),(0,-2,0),(1,-3,0),(2,-2,-2)]

  return indices

def getindices_origin_225(action):
  """
  return swath indices when 'action' is taken from heading 225 at origin
  """  
  if action == "RS_f":
    indices = [(-8,-3,-2),(-7,-3,-2),(-6,-3,-2),(-5,-3,-2),(-4,-3,-2),(-3,-3,-2),(-2,-3,-1),(-1,-2,0),(0,-2,1),(1,-1,0)]
  elif action == "LS_f":
    indices = [(-3,-8,-2),(-2,-8,-1),(-1,-4,0),(0,-2,1),(1,-1,0)]
  elif action == "F_diag":
    indices = [(-3,-2,-2),(-2,-3,-1),(-1,-3,0),(0,-2,1),(1,-1,0)]
  elif action == "F_diag3":
    indices = [(-5,-5,-4),(-4,-6,-3),(-3,-6,-2),(-2,-4,-1),(-1,-3,0),(0,-2,1),(1,-1,0)]
  elif action == "B_diag":
    indices = [(-2,-1,-1),(-1,-2,1),(0,-2,1),(1,0,2),(2,1,1)]

  return indices

def getindices_origin_315(action):
  """
  return swath indices when 'action' is taken from heading 315 at origin
  """  
  if action == "RS_f":
    indices = [(-2,0,0),(-1,-1,1),(0,-2,1),(1,-8,0),(2,-8,-2)]
  elif action == "LS_f":
    indices = [(-2,0,0),(-1,-1,1),(0,-2,1),(1,-3,0),(2,-3,-1),(3,-3,-1),(4,-3,-2),(5,-3,-2),(6,-3,-2),(7,-3,-2)]
  elif action == "F_diag":
    indices = [(-2,0,0),(-1,-1,1),(0,-2,1),(1,-3,0),(2,-2,-1)]
  elif action == "F_diag3":
    indices = [(-2,0,0),(-1,-1,1),(0,-2,1),(1,-3,0),(2,-4,-1),(3,-5,-2),(4,-5,-3),(5,-4,-3)]
  elif action == "B_diag":
    indices = [(-3,1,1),(-2,-1,2),(-1,-1,1),(0,-2,0),(1,-1,0)]

  return indices
