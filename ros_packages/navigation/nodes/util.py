import math
ROBOT_RADIUS = 2     # 2ft = 24 inches 
stepsize = 0.5       # Step size (in ft) by which planner forward simulates each action of car

def feet_to_cm(feet):
    return feet*30.48

def cm_to_feet(cm):
    return cm/30.48

def drawCourt(canvas):
 # draw court
    canvas.create_rectangle(1.2*50,5.95*50,4.8*50,6.05*50,width=1,outline = 'blue',fill = 'blue')
    canvas.create_rectangle(1.2*50,2.1*50,4.8*50,9.9*50,width=1,outline = 'black')
    canvas.create_line(1.65*50,2.1*50,1.65*50,9.9*50,width=1,fill='black')
    canvas.create_line(4.35*50,2.1*50,4.35*50,9.9*50,width=1,fill='black')
    canvas.create_line(1.65*50,3.9*50,4.35*50,3.9*50,width=1,fill='black')
    canvas.create_line(1.65*50,8.1*50,4.35*50,8.1*50,width=1,fill='black')
    canvas.create_line(3*50,3.9*50,3*50,8.1*50,width=1,fill='black')

    # add obstacles
    canvas.create_rectangle(1.2*50,5.95*50,4.8*50,6.05*50,width=1,outline = 'blue',fill = 'blue') # net
    canvas.create_rectangle(0.5*50,4.7*50,0.7*50,5.8*50,width=1,outline = 'blue',fill = 'blue') # bench 1
    canvas.create_rectangle(0.5*50,6.2*50,0.7*50,7.3*50,width=1,outline = 'blue',fill = 'blue') # bench 2
    canvas.create_rectangle(5.3*50,4.7*50,5.5*50,5.8*50,width=1,outline = 'blue',fill = 'blue') # bench 3
    canvas.create_rectangle(5.3*50,6.2*50,5.5*50,7.3*50,width=1,outline = 'blue',fill = 'blue') # bench 4
    canvas.create_rectangle(2.0*50,600-8.8*50,2.5*50,600-8.2*50,width=1,outline = 'blue',fill = 'yellow') # player 1
    canvas.create_rectangle(4.0*50,600-3.2*50,4.5*50,600-3.8*50,width=1,outline = 'blue',fill = 'yellow') # player 2


    for i in range(6):
      canvas.create_text(i*50,11.8*50,text = str(i*10))
    for i in range(12):
      canvas.create_text(0.2*50,i*50,text = str((12-i)*10))


def newState(p):
  """
  Check if p is an allowed state, i.e. does it lie in C_free
  """
  # return True; # removing all obstacles!

  x = p[0]*ROBOT_RADIUS
  y = p[1]*ROBOT_RADIUS
  if(x<0 or x>60):
      return False
  elif(y<0 or y>120):
      return False
  elif ((x>=12) and (y>=59.5) and (x <= 48) and (y <= 60.5)):
      return False
  elif((x>=05) and (y>=47) and (x<=07) and (y<=58)):
      return False
  elif((x>=05) and (y>=62) and (x<=7) and (y<=73)):
      return False
  elif((x>=53) and (y>=47) and (x<=55) and (y<=58)):
      return False
  elif((x>=53) and (y>=62) and (x<=55) and (y<=73)):
      return False
  elif((x>=20) and (y>=82) and (x<=25) and (y<=88)):
      return False
  elif((x>=40) and (y>=32) and (x<=45) and (y<=38)):
      return False
  else:
      return True

def distance_Euclidian(p1,p2):
  """
  returns square of the Euclidian distance, for speed
  """
  return (p1[0] - p2[0])**2 + (p1[1] - p2[1])**2

def turnLeft_sim(x,y,theta,v=0.5/ROBOT_RADIUS):
  angle = theta + v
  if(angle >= 2*math.pi):
    angle = angle - 2*math.pi

  return(x + math.sin(theta+v) - math.sin(theta),
         y - math.cos(theta+v) + math.cos(theta),
         angle)

def turnRight_sim(x,y,theta,v=0.5/ROBOT_RADIUS):
  angle = theta - v
  if(angle < 0):
    angle =2*math.pi + angle

  return (x - math.sin(theta - v) + math.sin(theta),
          y + math.cos(theta-v) - math.cos(theta),
          angle)

def goForward_sim(x,y,theta,v=0.5/ROBOT_RADIUS):
  return (x + v*math.cos(theta),
          y + v*math.sin(theta),
          theta)

def greedyMove(p_rand,p_near):
    """ Try x,y,straight and see which gets p_near closest
        to p_rand
        Return u_new
    """
    global stepsize
    v = stepsize/ROBOT_RADIUS
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

def path_to_drivecmds(path):
    """
    takes a list of RRT Nodes contained in path, and strings them together into sublists of [action,distance] where
    action : the action we want the car to take, i.e. L,S or R 
    distance: the distance for which we want the car to execute that action
    """
    global stepsize
    d_action = 0
    drive_cmds = []
    i = 0
    distancepermove = feet_to_cm(stepsize)
    current_Action = None

    for node in path:
        if node.getParent() == None: # first node
            print "first node"
            continue
        else:
            action_toNode = node.getAction_toreachNode()
            #print "action to reach node", action_toNode
            if len(drive_cmds) == 0 :
                drive_cmds.append([action_toNode,distancepermove])
                current_Action = action_toNode
    
            elif (action_toNode == current_Action): #if node can be reached by the current action, just increase length of current segment
                    drive_cmds[-1][1] += distancepermove
            
            else:  # start of a new action
                    drive_cmds.append([action_toNode,distancepermove])
                    current_Action = action_toNode

    return drive_cmds        
