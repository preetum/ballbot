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

    self.allowed_headings = [0,math.pi/2,math.pi,3*math.pi/2,
                             math.pi/4,3*math.pi/4,5*math.pi/4,7*math.pi/4]
                             #5.177,3.606,2.035,0.464,1.107,5.819,2.677,4.248]

    self.actions_90s = ["R_f","R_b","L_f","L_b","F",
                        #"F3",
                        "B",
                        #"SR_f","SL_f",
                        "RS_f_short","LS_f_short",
                        "sidestep_R_f","sidestep_L_f"
                        #"SR_f_2","SL_f_2",
                        #"RSR_f","LSL_f"
                        ]
    self.actions_45s = ["LS_f","RS_f","F_diag",
                        #"F_diag3",
                        "B_diag",
                        "L2_f","R2_f",
                        ]
    self.actions_26_6s = ["F1_26.6","B1_26.6","LSL1_f_26.6","LSL2_f_26.6","RSR1_f_26.6","RSR2_f_26.6"]
    self.actions_63_4s = ["F1_63.4","B1_63.4","LSL1_f_63.4","LSL2_f_63.4","RSR1_f_63.4","RSR2_f_63.4"]
    self.init_actions(v)
    self.init_swaths()
    
  def init_actions(self,v):
    ROBOT_RADIUS_MIN = util.ROBOT_RADIUS_MIN
    ROBOT_RADIUS_2   = util.ROBOT_RADIUS_2
    ROBOT_RADIUS_3  = util.ROBOT_RADIUS_3
    ROBOT_RADIUS_4  = util.ROBOT_RADIUS_4
    ROBOT_RADIUS_5  = util.ROBOT_RADIUS_5
    CELL_SIZE        = util.CELL_SIZE

    # map actions from origin to lattice points, for heading 0
    state_atOrigin = (0,0,0,v)
    self.theta_0      = {"R_f":(ROBOT_RADIUS_MIN,-ROBOT_RADIUS_MIN,3*math.pi/2,v),
                         "R_b":(-ROBOT_RADIUS_MIN,-ROBOT_RADIUS_MIN,math.pi/2,v),
                         "L_f":(ROBOT_RADIUS_MIN,ROBOT_RADIUS_MIN,math.pi/2,v),
                         "L_b":(-ROBOT_RADIUS_MIN,ROBOT_RADIUS_MIN,3*math.pi/2,v),                        
                         "F"  :(CELL_SIZE,0,0,v),
                         "F3" :(3*CELL_SIZE,0,0,v),
                         "B"  :(-CELL_SIZE,0,0,v),
                         "SR_f":(3*ROBOT_RADIUS_MIN/2,-ROBOT_RADIUS_MIN/2,7*math.pi/4,v),
                         "SL_f":(3*ROBOT_RADIUS_MIN/2,ROBOT_RADIUS_MIN/2,math.pi/4,v),
                         "RS_f_short":(ROBOT_RADIUS_MIN,-ROBOT_RADIUS_MIN/2,7*math.pi/4,v),
                         "LS_f_short":(ROBOT_RADIUS_MIN,ROBOT_RADIUS_MIN/2,math.pi/4,v),
                         "sidestep_R_f":(3*ROBOT_RADIUS_MIN/2,-ROBOT_RADIUS_MIN/2,0,v),
                         "sidestep_L_f":(3*ROBOT_RADIUS_MIN/2,ROBOT_RADIUS_MIN/2,0,v),
                         "SR_f_2":(ROBOT_RADIUS_MIN,-ROBOT_RADIUS_MIN/2,5.177,v),
                         "SL_f_2":(ROBOT_RADIUS_MIN,ROBOT_RADIUS_MIN/2,1.107,v),
                         "RSR_f":(5*CELL_SIZE,-CELL_SIZE,5.819,v),
                         "LSL_f":(5*CELL_SIZE,CELL_SIZE,0.464,v)}

    # map actions from origin to lattice points, for heading 90
    state_atOrigin = (0,0,math.pi/2,v)
    self.theta_90      =  {"R_f":(ROBOT_RADIUS_MIN,ROBOT_RADIUS_MIN,0,v),
                           "R_b":(ROBOT_RADIUS_MIN,-ROBOT_RADIUS_MIN,math.pi,v),
                           "L_f":(-ROBOT_RADIUS_MIN,ROBOT_RADIUS_MIN,math.pi,v),
                           "L_b":(-ROBOT_RADIUS_MIN,-ROBOT_RADIUS_MIN,0,v),
                           "F"  :(0,CELL_SIZE,math.pi/2,v),
                           "F3" :(0,3*CELL_SIZE,math.pi/2,v),
                           "B"  :(0,-CELL_SIZE,math.pi/2,v),
                           "SR_f": (ROBOT_RADIUS_MIN/2,3*ROBOT_RADIUS_MIN/2,math.pi/4,v),                           
                           "SL_f":(-ROBOT_RADIUS_MIN/2,3*ROBOT_RADIUS_MIN/2,3*math.pi/4,v),
                           "RS_f_short":(ROBOT_RADIUS_MIN/2,ROBOT_RADIUS_MIN,math.pi/4,v),
                           "LS_f_short":(-ROBOT_RADIUS_MIN/2,ROBOT_RADIUS_MIN,3*math.pi/4,v),
                           "sidestep_R_f":(ROBOT_RADIUS_MIN/2,3*ROBOT_RADIUS_MIN/2,math.pi/2,v),
                           "sidestep_L_f":(-ROBOT_RADIUS_MIN/2,3*ROBOT_RADIUS_MIN/2,math.pi/2,v),
                           "SR_f_2": (ROBOT_RADIUS_MIN/2,ROBOT_RADIUS_MIN,0.464,v),
                           "SL_f_2": (-ROBOT_RADIUS_MIN/2,ROBOT_RADIUS_MIN,2.677,v),
                           "RSR_f": (CELL_SIZE,5*CELL_SIZE,1.107,v),
                           "LSL_f": (-CELL_SIZE,5*CELL_SIZE,2.035,v)}
    
    # map actions from origin to lattice points, for heading 180
    state_atOrigin = (0,0,math.pi,v)
    self.theta_180     =  {"R_f":(-ROBOT_RADIUS_MIN,ROBOT_RADIUS_MIN,math.pi/2,v),
                           "R_b":(ROBOT_RADIUS_MIN,ROBOT_RADIUS_MIN,3*math.pi/2,v),
                           "L_f":(-ROBOT_RADIUS_MIN,-ROBOT_RADIUS_MIN,3*math.pi/2,v),
                           "L_b":(ROBOT_RADIUS_MIN,-ROBOT_RADIUS_MIN,math.pi/2,v),
                           "F"  :(-CELL_SIZE,0,math.pi),
                           "F3": (-3*CELL_SIZE,0,math.pi),
                           "B"  :(CELL_SIZE,0,math.pi),
                           "SR_f":(-3*ROBOT_RADIUS_MIN/2,ROBOT_RADIUS_MIN/2,3*math.pi/4,v),
                           "SL_f":(-3*ROBOT_RADIUS_MIN/2,-ROBOT_RADIUS_MIN/2,5*math.pi/4,v),
                           "RS_f_short":(-ROBOT_RADIUS_MIN,ROBOT_RADIUS_MIN/2,3*math.pi/4,v),
                           "LS_f_short":(-ROBOT_RADIUS_MIN,-ROBOT_RADIUS_MIN/2,5*math.pi/4,v),
                           "sidestep_R_f":(-3*ROBOT_RADIUS_MIN/2,ROBOT_RADIUS_MIN/2,math.pi,v),
                           "sidestep_L_f":(-3*ROBOT_RADIUS_MIN/2,-ROBOT_RADIUS_MIN/2,math.pi,v),
                           "SR_f_2":(-ROBOT_RADIUS_MIN,ROBOT_RADIUS_MIN/2,2.035,v),
                           "SL_f_2":(-ROBOT_RADIUS_MIN,-ROBOT_RADIUS_MIN/2,4.248,v),
                           "RSR_f":(-5*CELL_SIZE,CELL_SIZE,2.677,v),
                           "LSL_f":(-5*CELL_SIZE,-CELL_SIZE,3.606,v)}
    
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
                           "SL_f": (ROBOT_RADIUS_MIN/2,-3*ROBOT_RADIUS_MIN/2,7*math.pi/4,v),
                           "RS_f_short":(-ROBOT_RADIUS_MIN/2,-ROBOT_RADIUS_MIN,5*math.pi/4,v),
                           "LS_f_short":(ROBOT_RADIUS_MIN/2,-ROBOT_RADIUS_MIN,7*math.pi/4,v),
                           "sidestep_R_f":(-ROBOT_RADIUS_MIN/2,-3*ROBOT_RADIUS_MIN/2,3*math.pi/2,v),
                           "sidestep_L_f":(ROBOT_RADIUS_MIN/2,-3*ROBOT_RADIUS_MIN/2,3*math.pi/2,v),
                           "SR_f_2":(-ROBOT_RADIUS_MIN/2,-ROBOT_RADIUS_MIN,3.606,v),
                           "SL_f_2":(ROBOT_RADIUS_MIN/2,-ROBOT_RADIUS_MIN,5.819,v),
                           "RSR_f" :(-CELL_SIZE,-5*CELL_SIZE,4.248,v),
                           "LSL_f": (CELL_SIZE,-5*CELL_SIZE,5.177,v)}

    # map actions from origin to lattice points, for heading 45
    state_atOrigin = (0,0,math.pi/4,v)
    self.theta_45  = {"F_diag": (CELL_SIZE,CELL_SIZE,math.pi/4,v),
                      "F_diag3":(3*CELL_SIZE,3*CELL_SIZE,math.pi/4,v),
                      "B_diag": (-CELL_SIZE,-CELL_SIZE,math.pi/4,v),
                      "LS_f"  : (ROBOT_RADIUS_MIN/2,3*ROBOT_RADIUS_MIN/2,math.pi/2,v),
                      "RS_f"  : (3*ROBOT_RADIUS_MIN/2,ROBOT_RADIUS_MIN/2,0,v),
                      "L2_f"  : (0,3*ROBOT_RADIUS_MIN/2,3*math.pi/4,v),
                      "R2_f"  : (3*ROBOT_RADIUS_MIN/2,0,7*math.pi/4,v)}

    # map actions from origin to lattice points, for heading 135
    state_atOrigin = (0,0,3*math.pi/4,v)
    self.theta_135  =  {"F_diag": (-CELL_SIZE,CELL_SIZE,3*math.pi/4,v),
                        "F_diag3": (-3*CELL_SIZE,3*CELL_SIZE,3*math.pi/4,v),
                        "B_diag": (CELL_SIZE,-CELL_SIZE,3*math.pi/4,v),
                        "LS_f"  : (-3*ROBOT_RADIUS_MIN/2,ROBOT_RADIUS_MIN/2,math.pi,v),
                        "RS_f"  : (-ROBOT_RADIUS_MIN/2,3*ROBOT_RADIUS_MIN/2,math.pi/2,v),
                        "L2_f"  : (-3*ROBOT_RADIUS_MIN/2,0,5*math.pi/4,v),
                        "R2_f"  : (0,3*ROBOT_RADIUS_MIN/2,math.pi/4,v) }

    # map actions from origin to lattice points, for heading 225
    state_atOrigin = (0,0,5*math.pi/4,v)
    self.theta_225  = {"F_diag": (-CELL_SIZE,-CELL_SIZE,5*math.pi/4,v),
                       "F_diag3":(-3*CELL_SIZE,-3*CELL_SIZE,5*math.pi/4,v),
                       "B_diag": (CELL_SIZE,CELL_SIZE,5*math.pi/4,v),
                       "LS_f"  : (-ROBOT_RADIUS_MIN/2,-3*ROBOT_RADIUS_MIN/2,3*math.pi/2,v),
                       "RS_f"  : (-3*ROBOT_RADIUS_MIN/2,-1*ROBOT_RADIUS_MIN/2,math.pi,v),
                       "L2_f"  : (0,-3*ROBOT_RADIUS_MIN/2,7*math.pi/4,v),
                       "R2_f"  : (3*ROBOT_RADIUS_MIN/2,0,3*math.pi/4,v) }  

    # map actions from origin to lattice points, for heading 315
    state_atOrigin = (0,0,7*math.pi/4,v)
    self.theta_315  = {"F_diag":(CELL_SIZE,-CELL_SIZE,7*math.pi/4,v),
                       "F_diag3":(3*CELL_SIZE,-3*CELL_SIZE,7*math.pi/4,v),
                       "B_diag": (-CELL_SIZE,CELL_SIZE,7*math.pi/4,v),
                       "LS_f"  : (3*ROBOT_RADIUS_MIN/2,-ROBOT_RADIUS_MIN/2,0,v),
                       "RS_f"  : (ROBOT_RADIUS_MIN/2,-3*ROBOT_RADIUS_MIN/2,3*math.pi/2,v),
                       "L2_f"  : (3*ROBOT_RADIUS_MIN/2,0,math.pi/4,v),
                       "R2_f"  : (0,-3*ROBOT_RADIUS_MIN/2,5*math.pi/4) }

    # map actions from origin to lattice points, for heading 26.6 (0.464 radians)
    state_atOrigin = (0,0,0.464,v)
    self.theta_26_6 = {"F1_26.6":(2*CELL_SIZE,CELL_SIZE,0.464,v),
                       "B1_26.6":(-2*CELL_SIZE,-CELL_SIZE,0.464,v),                       
                       "LSL1_f_26.6":(3*CELL_SIZE,5*CELL_SIZE,math.pi/2,v),
                       "LSL2_f_26.6":(4*CELL_SIZE,3*CELL_SIZE,math.pi/4,v),
                       "RSR1_f_26.6":(4*CELL_SIZE,CELL_SIZE,0,v),
                       "RSR2_f_26.6":(6*CELL_SIZE,-2*CELL_SIZE,7*math.pi/4,v)}
  
    # map actions from origin to lattice points, for heading 116.6 (2.035 radians)
    self.theta_116_6 = {"F1_26.6":(-CELL_SIZE,2*CELL_SIZE, 2.035,v),
                        "B1_26.6":(CELL_SIZE,-2*CELL_SIZE, 2.035,v),
                        "LSL1_f_26.6":(-5*CELL_SIZE,3*CELL_SIZE,math.pi,v),
                        "LSL2_f_26.6":(-3*CELL_SIZE,4*CELL_SIZE,3*math.pi/4,v),
                        "RSR1_f_26.6":(-CELL_SIZE,4*CELL_SIZE,math.pi/2,v),
                        "RSR2_f_26.6":(2*CELL_SIZE,6*CELL_SIZE,math.pi/4,v)}

    # map actions from origin to lattice points, for heading 206.6 (3.606 radians)
    self.theta_206_6 = {"F1_26.6":(-2*CELL_SIZE,-CELL_SIZE, 3.606,v),
                        "B1_26.6":(2*CELL_SIZE, CELL_SIZE, 3.606,v),
                        "LSL1_f_26.6":(-3*CELL_SIZE,5*CELL_SIZE,3*math.pi/2,v),
                        "LSL2_f_26.6":(-4*CELL_SIZE,-3*CELL_SIZE,5*math.pi/4,v),
                        "RSR1_f_26.6":(-4*CELL_SIZE,-CELL_SIZE,math.pi,v),
                        "RSR2_f_26.6":(-6*CELL_SIZE,2*CELL_SIZE,3*math.pi/4,v)}

    # map actions from origin to lattice points, for heading 296.6 (5.177 radians)
    self.theta_296_6 ={"F1_26.6":(CELL_SIZE,-2*CELL_SIZE, 5.177,v),
                       "B1_26.6":(-CELL_SIZE,2*CELL_SIZE, 5.177,v),
                       "LSL1_f_26.6":(5*CELL_SIZE,-3*CELL_SIZE,0,v),
                       "LSL2_f_26.6":(3*CELL_SIZE,-4*CELL_SIZE,7*math.pi/4,v),
                       "RSR1_f_26.6":(CELL_SIZE,-4*CELL_SIZE,3*math.pi/2,v),
                       "RSR2_f_26.6":(-2*CELL_SIZE,-6*CELL_SIZE,5*math.pi/2,v)}

    # map actions from origin to lattice points, for heading 63.4 (1.107 radians)
    self.theta_63_4 = {"F1_63.4":(CELL_SIZE,2*CELL_SIZE,1.107,v),
                       "B1_63.4":(-CELL_SIZE,-2*CELL_SIZE,1.107,v),
                       "LSL1_f_63.4":(CELL_SIZE,4*CELL_SIZE,math.pi/2,v),
                       "LSL2_f_63.4":(-2*CELL_SIZE,6*CELL_SIZE,3*math.pi/4,v),
                       "RSR1_f_63.4":(5*CELL_SIZE,3*CELL_SIZE,0,v),
                       "RSR2_f_63.4":(3*CELL_SIZE,4*CELL_SIZE,math.pi/4,v)}

    # map actions from origin to lattice points, for heading 153.4 (2.677 radians)
    self.theta_153_4 = {"F1_63.4":(-2*CELL_SIZE,CELL_SIZE,2.677,v),
                        "B1_63.4":(2*CELL_SIZE,-CELL_SIZE,2.677,v),
                        "LSL1_f_63.4":(-4*CELL_SIZE,CELL_SIZE,math.pi,v),
                        "LSL2_f_63.4":(-6*CELL_SIZE,-2*CELL_SIZE,5*math.pi/4,v),
                        "RSR1_f_63.4":(-3*CELL_SIZE,5*CELL_SIZE,math.pi/2,v),
                        "RSR2_f_63.4":(-4*CELL_SIZE,3*CELL_SIZE,3*math.pi/4,v)}

    # map actions from origin to lattice points, for heading 243.4 (4.248 radians)
    self.theta_243_4 = {"F1_63.4":(-CELL_SIZE,-2*CELL_SIZE,4.248,v),
                        "B1_63.4":(CELL_SIZE,2*CELL_SIZE,4.248,v),
                        "LSL1_f_63.4":(-CELL_SIZE,-4*CELL_SIZE,3*math.pi/2,v),
                        "LSL2_f_63.4":(2*CELL_SIZE,-6*CELL_SIZE,7*math.pi/4,v),
                        "RSR1_f_63.4":(-5*CELL_SIZE,-3*CELL_SIZE,math.pi,v),
                        "RSR2_f_63.4":(-3*CELL_SIZE,-4*CELL_SIZE,5*math.pi/4,v)}

    # map actions from origin to lattice points, for heading 333.4 (5.819 radians)
    self.theta_333_4 = {"F1_63.4":(2*CELL_SIZE,-CELL_SIZE,5.819,v),
                        "B1_63.4":(-2*CELL_SIZE,CELL_SIZE,5.819,v),
                        "LSL1_f_63.4":(5*CELL_SIZE,-CELL_SIZE,0,v),
                        "LSL2_f_63.4":(6*CELL_SIZE,2*CELL_SIZE,math.pi/4,v),
                        "RSR1_f_63.4":(3*CELL_SIZE,-5*CELL_SIZE,3*math.pi/2,v),
                        "RSR2_f_63.4":(4*CELL_SIZE,-3*CELL_SIZE,7*math.pi/4,v)}


    # initialize lengths of actions        
    self.len_actions = {'F':CELL_SIZE,
                        'B':CELL_SIZE,                                                
                        "SL_f":(20.5/70.0*ROBOT_RADIUS_MIN + math.pi*ROBOT_RADIUS_2/4),
                        "SR_f":(20.5/70.0*ROBOT_RADIUS_MIN + math.pi*ROBOT_RADIUS_2/4),
                        "L_f":(math.pi*ROBOT_RADIUS_MIN/2.0),
                        "R_f": (math.pi*ROBOT_RADIUS_MIN/2.0),
                        "L_b": (math.pi*ROBOT_RADIUS_MIN/2.0),
                        "R_b": (math.pi*ROBOT_RADIUS_MIN/2.0),
                        "F3":  3*CELL_SIZE,                        
                        "LS_f_short":math.pi*ROBOT_RADIUS_3/4 + 14.497,
                        "RS_f_short":math.pi*ROBOT_RADIUS_3/4 + 14.497,
                        "sidestep_R_f": 111.878,
                        "sidestep_L_f":111.878,
                        "SR_f_2":13.369 + 2*math.pi*ROBOT_RADIUS_5*(63.4/360.0),
                        "SL_f_2":13.369 + 2*math.pi*ROBOT_RADIUS_5*(63.4/360.0),
                        "LSL_f":13.3968 + 53.9857 + 22.2,
                        "RSR_f":13.3968 + 53.9857 + 22.2,

                        "F_diag":CELL_SIZE*math.sqrt(2),
                        "F_diag3":3*CELL_SIZE*math.sqrt(2),
                        "B_diag": CELL_SIZE*math.sqrt(2),                        
                        "L2_f":math.pi*ROBOT_RADIUS_4/2.0,
                        "R2_f":math.pi*ROBOT_RADIUS_4/2.0,
                        "LS_f":(20.5/70.0*ROBOT_RADIUS_MIN + math.pi*ROBOT_RADIUS_2/4),
                        "RS_f":(20.5/70.0*ROBOT_RADIUS_MIN + math.pi*ROBOT_RADIUS_2/4),
                        
                        "F1_26.6": math.sqrt(CELL_SIZE*CELL_SIZE + 2*CELL_SIZE*2*CELL_SIZE),
                        "B1_26.6":  math.sqrt(CELL_SIZE*CELL_SIZE + 2*CELL_SIZE*2*CELL_SIZE),                        
                        "LSL1_f_26.6":41.95 + 28.496 + 35.508,
                        "RSR1_f_26.6": 14.625 + 39.952 + 17.873,
                        "LSL2_f_26.6":12.99 + 65.60 + 9.487,
                        "RSR2_f_26.6":84.055 + 32.613 + 3.42,

                        "F1_63.4": math.sqrt(CELL_SIZE*CELL_SIZE + 2*CELL_SIZE*2*CELL_SIZE),
                        "B1_63.4": math.sqrt(CELL_SIZE*CELL_SIZE + 2*CELL_SIZE*2*CELL_SIZE),
                        "LSL1_f_63.4": 14.625 + 39.952 + 17.873,
                        "LSL2_f_63.4":84.055 + 32.613 + 3.42,
                        "RSR1_f_63.4":41.95 + 28.496 + 35.508,
                        "RSR2_f_63.4":12.99 + 65.6 + 9.487}

     
 
  def init_swaths(self):
    # 90s
    self.swath_0   = getindices_origin_0()         
    self.swath_90  = getindices_origin_90()
    self.swath_180 = getindices_origin_180()
    self.swath_270 = getindices_origin_270()

    self.swath_45  = getindices_origin_45()
    self.swath_135 = getindices_origin_135()
    self.swath_225 = getindices_origin_225()      
    self.swath_315 = getindices_origin_315()

    self.swath_26_6 = getindices_origin_26_6()
    self.swath_116_6 = getindices_origin_116_6()
    self.swath_206_6 = getindices_origin_206_6()
    self.swath_296_6 = getindices_origin_296_6()

    self.swath_63_4 = getindices_origin_63_4()
    self.swath_153_4 = getindices_origin_153_4()
    self.swath_243_4 = getindices_origin_243_4()
    self.swath_333_4 = getindices_origin_333_4()

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

    elif(theta == 0.464):
      control_set = self.theta_26_6
    elif(theta == 2.035):
      control_set = self.theta_116_6
    elif(theta == 3.606):
      control_set = self.theta_206_6
    elif(theta == 5.177):
      control_set = self.theta_296_6  

    elif(theta == 1.107):
      control_set = self.theta_63_4
    elif(theta == 2.677):
      control_set = self.theta_153_4
    elif(theta == 4.248):
      control_set = self.theta_243_4
    elif(theta == 5.819):
      control_set = self.theta_333_4
    
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
      elif theta in (0.464,2.035,3.606,5.177):
        return self.actions_26_6s
      elif theta in (1.107,2.677,4.248,5.819):
        return self.actions_63_4s

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

    elif(th == 0.464):
      indices = self.swath_26_6[action]
    elif(th == 2.035):
      indices = self.swath_116_6[action]
    elif(th == 3.606):
      indices = self.swath_206_6[action]
    elif(th == 5.177):
      indices = self.swath_296_6[action]
  
    elif(th == 1.107): # 63.4 degreees
      indices = self.swath_63_4[action]
    elif(th == 2.677): 
      indices = self.swath_153_4[action]
    elif(th == 4.248):
      indices = self.swath_243_4[action]
    elif(th == 5.819):
      indices = self.swath_333_4[action]

    return indices

  def len_action(self,action):
    """
    returns length of an action
    """          
    return self.len_actions[action]

def getindices_origin_0():
  """
  return swath indices when 'action' is taken from heading 0 at origin
  indices are written as [(x-index, min_y-index, max_y-index)...]
  """  
  
  indices_origin_0 = { "R_f": [(-2,-1,0),(-1,-1,0),(0,-1,0), (1,-2,0), (2,-3,0),(3,-5,-1),(4,-5,-2)],
                       "F": [(-2,-1,0),(-1,-1,0),(0,-1,0),(1,-1,0)],
                       "F3":[(-2,-1,0),(-1,-1,0),(0,-1,0),(1,-1,0),(2,-1,0),(3,-1,0),(4,-1,0)],
                       "B":[(-2,-1,0),(-1,-1,0),(0,-1,0),(1,-1,0)],
                       "R_b":[(-5,-5,-2),(-4,-5,-1),(-3,-3,0),(-2,-2,0),(-1,-1,0),(0,-1,0),(1,-1,0)],  
                       "L_f": [(-2,-1,0),(-1,-1,0),(0,-1,0),(1,-1,1),(2,-1,2),(3,0,4),(4,1,4)],
                       "L_b": [(-5,1,4),(-4,0,4),(-3,-1,2),(-2,-1,1),(-1,-1,0),(0,-1,0),(1,-1,0)],
                       "SR_f":[(-2,-1,0),(-1,-1,0),(0,-1,0),(1,-1,0),(2,-1,0),(3,-2,0),(4,-3,0),(5,-3,-1),(6,-4,-1),(7,-3,-2)],
                       "SL_f":[(-2,-1,0),(-1,-1,0),(0,-1,0),(1,-1,0),(2,-1,0),(3,-1,1),(4,-1,2),(5,0,3),(6,0,3),(7,1,2)],               
                       "sidestep_R_f":[(-2,-1,0),(-1,-1,0),(0,-1,0),(1,-2,0),(2,-2,0),(3,-3,-1),(4,-3,-1),(5,-3,-2),(6,-3,-2),(7,-3,-2)],
                       "sidestep_L_f":[(-2,-1,0),(-1,-1,0),(0,-1,0),(1,-1,1),(2,-1,1),(3,0,2),(4,0,2),(5,1,2),(6,1,2),(7,1,2)],
                       "RS_f_short":[(-2,-1,0),(-1,-1,0),(0,-1,0),(1,-2,0),(2,-2,0),(3,-4,-1),(4,-4,-1),(5,-3,-2)],                       
                       "LS_f_short":[(-2,-1,0),(-1,-1,0),(0,-1,0),(1,-1,1),(2,-1,1),(3,0,3),(4,0,3),(5,1,2)],  
                       "SR_f_2":[(-2,-1,0),(-1,-1,0),(0,-1,0),(1,-1,0),(2,-2,0),(3,-3,0),(4,-4,-1),(5,-4,-2)],
                       "SL_f_2":[(-2,-1,0),(-1,-1,0),(0,-1,0),(1,-1,0),(2,-1,0),(3,-1,1),(4,-1,2),(5,0,3),(6,1,3)],
                       "RSR_f":[(-2,-1,0),(-1,-1,0),(0,-1,0),(1,-2,0),(2,-2,0),(3,-2,0),(4,-2,0),(5,-3,-1),(6,-2,-1)],                       
                       "LSL_f":[(-2,-1,0),(-1,-1,0),(0,-1,0),(1,-1,1),(2,-1,1),(3,-1,1),(4,-1,1),(5,0,2),(6,0,1)]}

  return indices_origin_0

def getindices_origin_90():
  """
  return swath indices when 'action' is taken from heading 90 at origin
  """  
  indices_origin_90 ={"R_f": [(-1,-2,2),( 0,-2,3),( 1,1,4),( 2,2,4),(3,3,4),(4,3,4)],
                      "F": [(-1,-2,1), (0,-2,1)],
                      "F3":[(-1,-2,4),( 0,-2,4)],
                      "B":[(-1,-2,1),( 0,-2,1)],
                      "R_b":[(-1,-3,1),( 0,-4,-1),( 1,-5,-2), ( 2,-5,-3),( 3,-5,-4),( 4,-5,-4)],  
                      "L_f":[(-5,3,4),(-4,3,4),(-3,2,4),(-2,1,4),(-1,-2,3),( 0,-2,2)],
                      "L_b":[(-5,-5,-4),(-4,-5,-4),(-3,-5,-3),(-2,-5,-2),(-1,-4,1),(1,-3,1)],
                      "SR_f":[(-1,-2,4),( 0,-2,6),( 1,3,7),( 2,4,7),( 3,6,6)],
                      "SL_f":[(-4,6,6),(-3,4,7),(-2,3,7),(-1,-2,6),( 0,-2,4)],    
                      "sidestep_R_f":[(-1,-2,2),(0,-1,4),(1,1,7),(2,3,7)],
                      "sidestep_L_f": [(-3,3,7),(-2,1,7),(-1,-2,4),(0,-2,2)],
                      "RS_f_short": [(-1,-2,2),(0,-2,4),(1,1,5),(2,3,5),(3,3,4)],
                      "LS_f_short":[(-4,2,3),(-3,3,5),(-2,1,5),(-1,-2,4),(0,-2,2)],
                      "SR_f_2":[(-1,-2,3),(0,-2,4),(1,2,5),(2,3,5),(3,4,5)],
                      "SL_f_2":[(-4,5,6),(-3,4,6),(-2,3,6),(-1,-2,5),(0,-2,4)],
                      "RSR_f":[(-1,-2,4),(0,-2,6),(1,1,6),(2,5,5)],
                      "LSL_f":[(-3,5,5),(-2,1,6),(-1,-2,6),(0,-2,4)]}

  return indices_origin_90

def getindices_origin_180():
  """
  return swath indices when 'action' is taken from heading 180 at origin
  """  
  indices_origin_180 = {"R_f":[(-5,1,4),(-4,0,4),(-3,-1,2),(-2,-1,1),(-1,-1,0),(0,-1,0),(1,-1,0)],
                        "F":[(-2,-1,0), (-1,-1,0),(0,-1,0),(1,-1,0)],
                        "F3":[(-5,-1,0),(-4,-1,0),(-3,-1,0),(-2,-1,0),(-1,-1,0),(0,-1,0),(1,-1,0)],
                        "B":[(-2,-1,0), (-1,-1,0),(0,-1,0),(1,-1,0)],
                        "R_b":[(-2,-1,0),(-1,-1,0),(0,-1,0),(1,-1,1),(2,-1,2),(3,0,4),(4,1,4)],
                        "L_f":[(-5,-5,-2),(-4,-5,-1),(-3,-3,0),(-2,-2,0),(-1,-1,0),(0,-1,0),(1,-1,0)],
                        "L_b":[(-2,-1,0),(-1,-1,0),(0,-1,0),(1,-2,0),(2,-3,0),(3,-5,-1),(4,-5,-2)],
                        "SR_f":[(-8,1,2),(-7,0,3),(-6,0,2),(-5,-1,2),(-4,-1,1),(-3,-1,0),(-2,-1,0),(-1,-1,0),(0,-1,0),(1,-1,0)],
                        "SL_f":[(-8,-3,-2),(-7,-4,-1),(-6,-3,-1),(-5,-3,0),(-4,-2,0),(-3,-1,0),(-2,-1,0),(-1,-1,0),(0,-1,0),(1,-1,0)],      
                        "sidestep_R_f":[(-8,1,2),(-7,1,2),(-6,1,2),(-5,0,2),(-4,0,2),(-3,-1,1),(-2,-1,1),(-1,-1,0),(0,-1,0),(1,-1,0)],
                        "sidestep_L_f":[(-8,-3,-2),(-7,-3,-2),(-6,-3,-2),(-5,-3,-1),(-4,-3,-1),(-3,-2,0),(-2,-2,0),(-1,-1,0),(0,-1,0),(1,-1,0)],
                        "RS_f_short":[(-6,1,2),(-5,0,3),(-4,0,3),(-3,-1,1),(-2,-1,1),(-1,-1,0),(0,-1,0),(1,-1,0)],
                        "LS_f_short":[(-6,-3,-2),(-5,-4,-1),(-4,-5,-1),(-3,-2,0),(-2,-2,0),(-1,-1,0),(0,-1,0),(1,-1,0)],
                        "SR_f_2":[(-6,1,3),(-5,0,3),(-4,-1,2),(-3,-1,1),(-2,-1,0),(-1,-1,0),(0,-1,0),(1,-1,0)],
                        "SL_f_2":[(-7,-4,-2),(-6,-4,-1),(-5,-3,-0),(-4,-2,0),(-3,-1,0),(-2,-1,0),(-1,-1,0),(0,-1,0),(1,-1,0)],
                        "RSR_f":[(-7,0,1),(-6,-1,1),(-5,-1,1),(-4,-1,1),(-3,-1,1),(-2,-1,1),(-1,-1,0),(0,-1,0),(1,-1,0)],
                        "LSL_f":[(-7,-2,-1),(-6,-3,-1),(-5,-2,0),(-4,-2,0),(-3,-2,0),(-2,-2,0),(-1,-1,0),(0,-1,0),(1,-1,0)]}

  return indices_origin_180

def getindices_origin_270():
  """
  return swath indices when 'action' is taken from heading 270 at origin
  """  
  indices_origin_270 = {"R_f":[(-5,-5,-4),(-4,-5,-4),(-3,-5,-3),(-2,-5,-2),(-1,-4,1),(0,-3,1)],
                         "F":[(-1,-2,1), (0,-2,1)],
                         "F3":[(-1,-5,1),(0,-5,1)],
                         "B":[(-1,-2,1), (0,-2,1)],
                         "R_b":[(-5,3,4),(-4,3,4),(-3,2,4),(-2,1,4),(-1,-2,3),(0,-2,2)],  
                         "L_f":[(-1,-3,1),(0,-4,1),(1,-5,-2),(2,-5,-3),(3,-5,4),(4,-5,4)],
                         "L_b":[(-1,-2,2),( 0,-2,3),( 1,1,4),( 2,2,4),(3,3,4),( 4,3,4)],
                         "SR_f":[(-4,-7,-7),(-3,-5,-8),(-2,-4,-8),(-1,-7,1),(0,-5,1)],
                         "SL_f":[(-1,-5,1),(0,-7,1),(1,-8,-4),(2,-8,-5),(3,-7,-7)],
                         "sidestep_R_f":[(-3,-8,-4),(-2,-8,-2),(-1,-5,1),(0,-3,1)],
                         "sidestep_L_f":[(-1,-3,1),(0,-5,1),(1,-8,-2),(2,-8,-4)],
                         "RS_f_short":[(-4,-5,-4),(-3,-6,-4),(-2,-6,-2),(-1,-5,1),(0,-3,1)],
                         "LS_f_short":[(-1,-3,1),(0,-5,1),(1,-6,-2),(2,-6,-4),(3,-5,-4)],
                         "SR_f_2":[(-4,-6,-5),(-3,-6,-4),(-2,-6,-3),(-1,-5,1),(0,-4,1)],
                         "SL_f_2":[(-1,-5,1),(0,-6,1),(1,-7,-4),(2,-7,-5),(3,-7,-6)],
                         "RSR_f":[(-3,-6,-6),(-2,-7,-2),(-1,-7,0),(0,-5,0)],
                         "LSL_f":[(-1,-5,1),(0,-7,1),(1,-7,-2),(2,-6,-6)]}
    
  return indices_origin_270


def getindices_origin_45():
  """
  return swath indices when 'action' is taken from heading 45 at origin
  """  
  indices_origin_45 = { "RS_f":[(-2,-1,0),(-1,-2,1),(0,-1,1),(1,0,2),(2,1,2),(3,1,2),(4,1,2),(5,1,2),(6,1,2),(7,1,2)],
                        "LS_f":[(-2,-1,0),(-1,-2,1),(0,-1,3),(0,0,7),(2,1,7)],
                        "F_diag": [(-2,-1,0),(-1,-2,1),(0,-1,2),(1,0,2),(2,1,1)],
                        "F_diag3":[(-2,-1,0),(-1,-2,1),(0,-1,2),(1,0,3),(2,1,5),(3,2,5),(4,3,4)],
                        "B_diag":[(-3,-2,-2),(-2,-3,-1),(-1,-2,1),(0,-2,1),(1,0,0)],
                        "L2_f":[(-2,-1,-1),(-2,5,5),(-1,-2,1),(-1,4,6),(0,-2,6),(1,-1,5),(2,1,3)],
                        "R2_f":[(-2,-1,0),(-1,-2,1),(0,-2,1),(1,-1,2),(2,0,2),(3,0,2),(4,0,1),(5,0,1),(6,0,1),(7,0,0)],
                          }

  return indices_origin_45

def getindices_origin_135():
  """
  return swath indices when 'action' is taken from heading 135 at origin
  """  
  indices_origin_135 = {"RS_f":[(-3,1,7),(-2,-1,7),(-1,-2,1),(0,-2,0),(1,-1,-1)],
                        "LS_f":[(-8,1,2),(-7,1,2),(-6,1,2),(-5,1,2),(-4,0,2),(-3,0,2),(-2,-1,2),(-1,-2,1),(0,-2,0),(1,-1,-1)],
                        "F_diag":[(-3,0,1),(-2,-1,2),(-1,-2,1),(0,-2,0),(1,-1,-1)],
                        "F_diag3":[(-6,2,3),(-5,2,4),(-4,1,4),(-3,0,3),(-2,-1,2),(-1,-2,1),(0,-2,0),(1,-1,-1)],
                        "B_diag":[(-2,-1,0),(-1,-1,1),(0,-2,0),(1,-3,0),(2,-2,-2)],
                        "L2_f":[(-7,-1,0),(-6,-2,1),(-5,-1,1),(-4,0,2),(-3,0,2),(-2,-1,2),(-1,-1,1),(0,-2,1),(1,-1,0)],
                        "R2_f": [(-3,1,3),(-2,-1,6),(-1,-2,7),(0,-2,1),(1,-1,0)]}
  
  return indices_origin_135

def getindices_origin_225():
  """
  return swath indices when 'action' is taken from heading 225 at origin
  """  
                         
  indices_origin_225 = {"RS_f":[(-8,-3,-2),(-7,-3,-2),(-6,-3,-2),(-5,-3,-2),(-4,-3,-2),(-3,-3,-2),(-2,-3,-1),(-1,-2,0),(0,-2,1),(1,-1,0)],
                        "LS_f":[(-3,-8,-2),(-2,-8,-1),(-1,-4,0),(0,-2,1),(1,-1,0)],
                        "F_diag":[(-3,-2,-2),(-2,-3,-1),(-1,-3,0),(0,-2,1),(1,-1,0)],
                        "F_diag3":[(-5,-5,-4),(-4,-6,-3),(-3,-6,-2),(-2,-4,-1),(-1,-3,0),(0,-2,1),(1,-1,0)],
                        "B_diag":[(-2,-1,-1),(-1,-2,1),(0,-2,1),(1,0,2),(2,1,1)],
                        "L2_f":[(-3,-4,-2),(-2,-6,0),(-1,-7,1),(0,-2,1),(0,-7,-5),(1,-6,-6),(1,0,0)],
                        "R2_f":[(-8,-1,-1),(-7,-2,-1),(-6,-2,-1),(-5,-2,-1),(-4,-3,-1),(-3,-3,-1),(-2,-3,0),(-1,-2,1),(0,-2,1),(1,-1,0)]}

  return indices_origin_225

def getindices_origin_315():
  """
  return swath indices when 'action' is taken from heading 315 at origin
  """  
  
  indices_origin_315 = {"RS_f":[(-2,0,0),(-1,-1,1),(0,-2,1),(1,-8,0),(2,-8,-2)],
                        "LS_f":[(-2,0,0),(-1,-1,1),(0,-2,1),(1,-3,0),(2,-3,-1),(3,-3,-1),(4,-3,-2),(5,-3,-2),(6,-3,-2),(7,-3,-2)],
                        "F_diag":[(-2,0,0),(-1,-1,1),(0,-2,1),(1,-3,0),(2,-2,-1)],
                        "F_diag3":[(-2,0,0),(-1,-1,1),(0,-2,1),(1,-3,0),(2,-4,-1),(3,-5,-2),(4,-5,-3),(5,-4,-3)],
                        "B_diag":[(-3,1,1),(-2,-1,2),(-1,-1,1),(0,-2,0),(1,-1,0)],
                        "L2_f":[(-2,-1,0),(-1,-2,1),(0,-2,0),(1,-3,0),(2,-3,-1),(3,-3,-1),(4,-2,0),(5,-2,1),(6,-1,0)],
                        "R2_f":[(-2,-1,0),(-1,-2,1),(0,-8,1),(1,-7,0),(2,-4,-2)]}

  return indices_origin_315

def getindices_origin_26_6():
  """
  return swath indices when 'action' is taken from heading 26.6 at origin
  """
  indices_origin_26_6 = {"F1_26.6":[(-2,-1,0),(-1,-2,0),(0,-1,1),(1,-1,1),(2,0,2),(3,0,1)],
                         "B1_26.6":[(-4,-2,-1),(-3,-3,-1),(-2,-2,0),(-1,-2,0),(0,-1,1),(1,-1,0)],
                         "LSL1_f_26.6":[(-2,-1,0),(-1,-2,0),(0,-1,1),(1,-1,3),(2,0,6),(3,2,6)],
                         "LSL2_f_26.6":[(-2,-1,0),(-1,-2,0),(0,-1,1),(1,-1,2),(2,0,3),(3,1,3),(4,2,4),(5,2,3)],
                         "RSR1_f_26.6":[(-2,-1,0),(-1,-2,0),(0,-1,2),(1,-1,1),(2,-1,1),(3,0,1),(4,0,1),(5,0,1)],
                         "RSR2_f_26.6":[(-2,-1,0),(-1,-2,0),(0,-1,1),(1,-1,1),(2,-1,1),(3,-2,1),(4,-3,0),(5,3,-1),(6,-4,-2),(7,-3,-2)]}  
  return indices_origin_26_6

def getindices_origin_116_6():
  """
  return swath indices when 'action' is taken from heading 116.6 at origin
  """
  indices_origin_116_6 = {"F1_26.6":[(-3,2,2),(-2,0,3),(-1,-2,3),(0,-2,1),(1,-1,-1)],
                         "B1_26.6":[(-2,0,0),(-1,-2,1),(0,-4,1),(1,-4,-1),(2,-3,-3)],
                         "LSL1_f_26.6":[(-7,2,3),(-6,2,3),(-5,2,3),(-4,1,3),(-3,1,3),(-2,0,2),(-1,-2,2),(0,-2,1),(1,-1,-1)],
                         "LSL2_f_26.6":[(-5,4,4),(-4,2,5),(-3,1,5),(-2,0,3),(-1,-2,2),(0,-2,1),(1,-1,-1)],
                         "RSR1_f_26.6":[(-2,0,5),(-1,-2,5),(0,-2,2),(1,-1,-1)],
                         "RSR2_f_26.6":[(-2,0,3),(-1,-2,4),(0,-2,5),(1,-1,-1),(1,3,7),(2,4,7),(3,6,6)]}
  return indices_origin_116_6

def getindices_origin_206_6():
  """
  return swath indices when 'action' is taken from heading 116.6 at origin
  """
  indices_origin_206_6 = {"F1_26.6":[(-4,-2,-1),(-3,-3,-1),(-2,-2,0),(-1,-2,0),(0,-1,1),(1,-1,0)],
                         "B1_26.6":[(-2,-1,0),(-1,-2,0),(0,-1,1),(1,-1,1),(2,0,2),(3,0,1)],
                         "LSL1_f_26.6":[(-4,-7,-3),(-3,-7,-1),(-2,-4,0),(-1,-2,0),(0,-1,1),(1,-1,0)],
                         "LSL2_f_26.6":[(-6,-4,-3),(-5,-5,-3),(-4,-4,-2),(-3,-4,-1),(-2,-3,0),(-1,-2,0),(0,-1,1),(1,-1,0)],
                         "RSR1_f_26.6":[(-6,-2,-1),(-5,-2,-1),(-4,-2,-1),(-3,-2,0),(-2,-2,0),(-1,-2,0),(0,-1,1),(1,-1,0)],
                         "RSR2_f_26.6":[(-8,1,2),(-7,1,3),(-6,0,2),(-5,-1,2),(-4,-2,1),(-3,-2,0),(-2,-2,0),(-1,-2,0),(0,-1,1),(1,-1,0)]}
  return indices_origin_206_6


def getindices_origin_296_6():
  """
  return swath indices when 'action' is taken from heading 116.6 at origin
  """
  indices_origin_296_6 = {"F1_26.6":[(-2,0,0),(-1,-2,1),(0,-4,1),(1,-4,-1),(2,-3,-3)],
                         "B1_26.6":[(-3,2,2),(-2,0,3),(-1,-2,3),(0,-2,1),(1,-1,-1)],
                         "LSL1_f_26.6":[(-2,0,0),(-1,-2,1),(0,-3,1),(1,-3,-1),(2,-4,-2),(3,-4,-2),(4,-4,-3),(5,-4,-3),(6,-4,-3)],
                         "LSL2_f_26.6":[(-2,0,0),(-1,-2,1),(0,-3,1),(1,-4,-1),(2,-6,-2),(3,-6,-3),(4,-5,-5)],
                         "RSR1_f_26.6":[(-2,0,0),(-1,-3,1),(0,-6,1),(1,-6,-1)],
                         "RSR2_f_26.6":[(-4,-7,-7),(-3,-8,-5),(-2,-8,-4),(-2,0,0),(-1,-6,1),(0,-5,1),(1,-4,-1)]}
  return indices_origin_296_6

def getindices_origin_63_4():
  """
  return swath indices when 'action' is taken from heading 116.6 at origin
  """
  indices_origin_63_4 = {"F1_63.4":[(-2,-1,-1),(-1,-2,1),(0,-2,3),(0,0,3),(2,2,2)],
                         "B1_63.4":[(-3,-3,-3),(-2,-4,-1),(-1,-4,1),(0,-2,1),(1,0,0)],
                         "LSL1_f_63.4":[(-2,-1,-1),(-1,-2,2),(0,-2,5),(1,0,5)],
                         "LSL2_f_63.4":[(-4,6,6),(-3,5,7),(-2,3,7),(-2,-1,-1),(-1,-2,5),(0,-2,4),(1,0,3)],
                         "RSR1_f_63.4":[(-2,-1,-1),(-1,-2,1),(0,-2,2),(1,0,2),(2,1,3),(3,1,3),(4,2,3),(5,2,3),(6,2,3)],
                         "RSR2_f_63.4":[(-2,-1,-1),(-1,-2,1),(0,-2,2),(1,0,3),(2,1,5),(3,3,5),(4,4,4)]}
  return indices_origin_63_4 
  
def getindices_origin_153_4():  
  """
  return swath indices when 'action' is taken from heading 153.4 at origin
  """
  indices_origin_153_4 = {"F1_63.4":[(-4,0,1),(-3,0,2),(-2,-1,1),(-1,-1,1),(0,-2,0),(1,-1,0)],
                         "B1_63.4":[(-2,-1,0),(-1,-1,1),(0,-2,0),(1,-2,0),(2,-3,-1),(3,-2,-1)],
                         "LSL1_f_63.4":[(-6,0,1),(-5,0,1),(-4,0,1),(-3,-1,1),(-2,-1,1),(-1,-1,1),(0,-2,0),(1,-1,0)],
                         "LSL2_f_63.4":[(-8,-3,-2),(-7,-4,-2),(-6,-3,-1),(-5,-2,0),(-4,-2,0),(-3,-1,2),(-2,-1,2),(-1,-1,2),(0,-2,0),(1,-1,0)],
                         "RSR1_f_63.4":[(-4,2,6),(-3,0,6),(-2,-1,3),(-1,-1,1),(0,-2,0),(1,-1,0)],
                         "RSR2_f_63.4":[(-6,2,3),(-5,2,4),(-4,1,3),(-3,0,2),(-2,-1,2),(-1,-1,1),(0,-2,0),(1,-1,0)]}
  return indices_origin_153_4

def getindices_origin_243_4():
  """
  return swath indices when 'action' is taken from heading 243.4 at origin
  """
  indices_origin_243_4 = {"F1_63.4":[(-3,-3,-3),(-2,-4,-1),(-1,-4,1),(0,-2,1),(1,0,0)],
                          "B1_63.4":[(-2,-1,-1),(-1,-2,1),(0,-2,3),(1,0,3),(2,2,2)],
                          "LSL1_f_63.4":[(-2,-6,-1),(-1,-6,1),(0,-3,1),(1,0,0)],
                          "LSL2_f_63.4":[(-2,-4,-1),(-1,-5,1),(0,-6,1),(1,0,0),(1,-8,-4),(2,-8,-6),(3,-7,-7)],
                          "RSR1_f_63.4":[(-7,-4,-3),(-6,-4,-3),(-5,-4,-3),(-4,-4,-2),(-3,-4,-2),(-2,-3,-1),(-1,-3,1),(0,-2,1),(1,0,0)],
                          "RSR2_f_63.4":[(-5,-5,-5),(-4,-6,-4),(-3,-6,-2),(-2,-4,-1),(-1,-3,1),(0,-2,1),(1,0,0)]}
  return indices_origin_243_4

def getindices_origin_333_4():
   """
   return swath indices when 'action' is taken from heading 333.4 at origin
   """
   indices_origin_333_4 = {"F1_63.4":[(-2,-1,0),(-1,-1,1),(0,-2,0),(1,-2,0),(2,-3,-1),(3,-2,-1)],
                           "B1_63.4":[(-4,0,1),(-3,0,2),(-2,-1,1),(-1,-1,1),(0,-2,0),(1,-1,0)],
                           "LSL1_f_63.4":[(-2,-1,0),(-1,-1,1),(0,-2,0),(1,-2,0),(2,-2,0),(3,-2,-1),(4,-2,-1),(5,-2,-1)],
                           "LSL2_f_63.4":[(-2,-1,0),(-1,-1,1),(0,-2,0),(1,-2,0),(2,-2,0),(3,-2,1),(4,-1,1),(5,0,2),(6,1,3),(7,1,2)],
                           "RSR1_f_63.4":[(-2,-1,0),(-2,-1,1),(0,-2,0),(1,-4,0),(2,-7,-1),(3,-7,-3)],
                           "RSR2_f_63.4":[(-2,-1,0),(-1,-1,1),(0,-2,0),(1,-3,0),(2,-3,0),(3,-4,-2),(4,-5,-3),(5,-4,-3)]}
   return indices_origin_333_4
