import math

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

  x = p[0]*1.91
  y = p[1]*1.91
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

