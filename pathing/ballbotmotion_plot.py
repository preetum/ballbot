import math

ROBOTRADIUS = 58.2168

def turnLeft(x,y,theta,v=0.5):
  angle = theta + v
  if(angle >= 2*math.pi):
    angle = angle - 2*math.pi

  return(x + math.sin(theta+v) - math.sin(theta),
         y - math.cos(theta+v) + math.cos(theta),
         angle)

def turnRight(x,y,theta,v=0.5):
  angle = theta - v
  if(angle < 0):
    angle =2*math.pi + angle

  return (x - math.sin(theta - v) + math.sin(theta),
          y + math.cos(theta-v) - math.cos(theta),
          angle)

def goForward(x,y,theta,v=0.5):
  return (x + v*math.cos(theta),
          y + v*math.sin(theta),
          theta)

def return_path_straightline(d):
    """
    Append points along the calculated straight line.
    Points are appended as (x,y,theta) where (x,y) are in cm and theta is in radians
    """
    path = []
    path.append((200,200,math.pi/2))
    (x,y,theta) = ((200/ROBOTRADIUS,200/ROBOTRADIUS,math.pi/2))
    v = 0.1
    i = 0+v
    action = goForward
    while(i < d):
        (x,y,theta) = action(x,y,theta,v)
        path.append((x*ROBOTRADIUS,y*ROBOTRADIUS,theta))
        i = i+v
    return path

def return_path_circle(d):
    """ 
    Append points along the calculated circle
    Points are appended as (x,y,theta) where (x,y) are in cm and theta is in radians
    """
    path = []
    path.append((200,200,math.pi/2))
    (x,y,theta) = (200/ROBOTRADIUS,200/ROBOTRADIUS,math.pi/2)
    v = 0.1
    i = 0+v
    action = turnRight
    while(i < d):
        (x,y,theta) = action(x,y,theta,v)
        path.append((x*ROBOTRADIUS,y*ROBOTRADIUS,theta))
        i = i+v
    return path

def return_path_dubins(curve_number,t,p,q):
  """
  Append points along the calculated Dubins curve. 
  Points are appended as (x,y,theta) where (x,y) are in cm and theta is in radians
  """
  path = []
  path.append((200,200,math.pi/2))
  (x,y,theta) = (200/ROBOTRADIUS,200/ROBOTRADIUS,math.pi/2)
  (x_init,y_init,theta_init) = (x,y,theta)
  v = 0.1

  t = t/ROBOTRADIUS
  p = p/ROBOTRADIUS
  q = q/ROBOTRADIUS
  # first action
  print "curve number ",curve_number
  if((curve_number == 0) or (curve_number == 2) or (curve_number == 5)):
    #turn left
    action = turnLeft

  elif((curve_number == 1) or (curve_number == 3) or (curve_number == 4)):
    #turn right
    action = turnRight

  i = 0+v
  while(i < t):
    (x,y,theta) = action(x,y,theta,v)
    path.append((x*ROBOTRADIUS,y*ROBOTRADIUS,theta))
    i = i + v

  (x_init,y_init,theta_init) = action(x_init,y_init,theta_init,t)
  (x,y,theta) = (x_init,y_init,theta_init)
  path.append((x*ROBOTRADIUS,y*ROBOTRADIUS,theta))

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
    path.append((x*ROBOTRADIUS,y*ROBOTRADIUS,theta))
    i = i + v

  (x_init,y_init,theta_init) = action(x_init,y_init,theta_init,p)
  (x,y,theta) = (x_init,y_init,theta_init)
  path.append((x*ROBOTRADIUS,y*ROBOTRADIUS,theta))

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
    path.append((x*ROBOTRADIUS,y*ROBOTRADIUS,theta))
    i = i + v
    
  (x_init,y_init,theta_init) = action(x_init,y_init,theta_init,q)
  (x,y,theta) = (x_init,y_init,theta_init)
  path.append((x*ROBOTRADIUS,y*ROBOTRADIUS,theta))

  return path
