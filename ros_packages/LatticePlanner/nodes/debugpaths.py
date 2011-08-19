# contains hardcoded debug paths
import math
import util
import LatticePlannersim

def straightLine((x1,y1,th1,v1),(x2,y2,th2,v2)):
    path = []
    dist = util.distance_Euclidean(x1,y1,x2,y2)
    state = (x1,y1,th1,v1)
    d = 0
    while d <= dist:
        newstate = util.go_Straight(state,'f',5)
        path.append((newstate[0]/100,newstate[1]/100,newstate[2]))
        d += 5
        state = newstate
            #print "appended",d,"of",dist

    path_to_send = LatticePlannersim.Path()
    for point in path:
        pose = LatticePlannersim.Pose()
        # plan uses center of car, so transform such that path has points to be traversed by rear axle center,                                                      # which is 17.41 cm away from the center                                                                                                                                                                                                                                                                     
        pose.x = point[0] - 17.41*math.cos(point[2])/100.0
        pose.y = point[1] - 17.41*math.sin(point[2])/100.0
        pose.theta = point[2]
        path_to_send.poses.append(pose)
            
    LatticePlannersim.pub_path.publish(path_to_send)
    print "path published"

def figureofeight((x1,y1,th1,v1)):

    path = []
    state = (x1,y1,th1,v1)
    newstate = None
    d = 0
    while d <=  2*math.pi*util.ROBOT_RADIUS_2:
        newstate = util.turn_Left(state,'f',5,util.ROBOT_RADIUS_2)
        path.append((newstate[0]/100,newstate[1]/100,newstate[2]))
        state = newstate
        d += 5

    (x1,y1,th1,v1) = state
    newstate = None
    d = 0
    while d <= 2*math.pi*util.ROBOT_RADIUS_2:
        newstate = util.turn_Right(state,'f',5,util.ROBOT_RADIUS_2)
        path.append((newstate[0]/100,newstate[1]/100,newstate[2]))
        state = newstate
        d += 5
        
    path_to_send = LatticePlannersim.Path()
    for point in path:
        pose = LatticePlannersim.Pose()
        # plan uses center of car, so transform such that path has points to be traversed by rear axle center,                                                      # which is 17.41 cm away from the center                                                                                                                                                                                                                                                                     
        pose.x = point[0] - 17.41*math.cos(point[2])/100.0
        pose.y = point[1] - 17.41*math.sin(point[2])/100.0
        pose.theta = point[2]
        path_to_send.poses.append(pose)
            
    LatticePlannersim.pub_path.publish(path_to_send)
    print "path published"

    
        


    


