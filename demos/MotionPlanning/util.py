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
      canvas.create_text(i*50,11.8*50,text = str(i))
    for i in range(12):
      canvas.create_text(0.2*50,i*50,text = str(12-i))




def newState(p):
  """
  Check if p is an allowed state, i.e. does it lie in C_free
  """
  x = p[0]
  y = p[1]
  r = 1
  if(x<0 or x>6):
      return False
  elif(y<0 or y>12):
      return False
  elif ((x>=1.2) and (y>=5.95) and (x <= 4.8) and (y <= 6.05)):
      return False
  elif((x>=0.5) and (y>=4.7) and (x<=0.7) and (y<=5.8)):
      return False
  elif((x>=0.5) and (y>=6.2) and (x<=0.7) and (y<=7.3)):
      return False
  elif((x>=5.3) and (y>=4.7) and (x<=5.5) and (y<=5.8)):
      return False
  elif((x>=5.3) and (y>=6.2) and (x<=5.5) and (y<=7.3)):
      return False
  elif((x>=2.0) and (y>=8.2) and (x<=2.5) and (y<=8.8)):
      return False
  elif((x>=4.0) and (y>=3.2) and (x<=4.5) and (y<=3.8)):
      return False
  else:
      return True

def distance_Euclidian(p1,p2):
  """
  returns square of the Euclidian distance, for speed
  """
  return (p1[0] - p2[0])**2 + (p1[1] - p2[1])**2

def turnLeft(x,y,theta,v=0.1):
  angle = theta + v
  if(angle >= 2*math.pi):
    angle = angle - 2*math.pi

  return(x + math.sin(theta+v) - math.sin(theta),
         y - math.cos(theta+v) + math.cos(theta),
         angle)

def turnRight(x,y,theta,v=0.1):
  angle = theta - v
  if(angle < 0):
    angle =2*math.pi + angle

  return (x - math.sin(theta - v) + math.sin(theta),
          y + math.cos(theta-v) - math.cos(theta),
          angle)

def goForward(x,y,theta,v=0.1):
  return (x + v*math.cos(theta),
          y + v*math.sin(theta),
          theta)

