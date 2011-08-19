#!/usr/bin/env python
import math
import util
import time
import controlset

import roslib; roslib.load_manifest('LatticePlanner')
import rospy
from std_msgs.msg import String
from bb_msgs.msg import Pose,Path,Goal

startNode = None
goalNode = None
plan = [] # stores the computed plan, as [(node1,action1),(node2,action2)....]
path = [] # stores the computed path, as a sequence of (x,y,theta) values

pub_path = rospy.Publisher('path',Path)
Ballbot_x = 0
Ballbot_y = 0
Ballbot_theta = 0

def received_odometry(data):
    global Ballbot_x,Ballbot_y,Ballbot_theta
    Ballbot_x = data.x*100.0
    Ballbot_y = data.y*100.0
    Ballbot_theta = data.theta

def startPlanner(data):
    global startNode,goalNode,plan,path
    
    (x1,y1,th1,v) = util.point_to_lattice(Ballbot_x,Ballbot_y,Ballbot_theta,util.ROBOT_SPEED_MAX)    
 
    v2 = util.ROBOT_SPEED_MAX
    (x2,y2,th2,v2) = util.point_to_lattice(data.pose.x,data.pose.y,data.pose.theta,util.ROBOT_SPEED_MAX)
    
    if(x1 < 0) or (x1 > util.COURT_WIDTH) or (y1 < 0) or (y1 > util.COURT_LENGTH):
        print "Invalid start!"
        return
    if(x2 < 0) or (x2 > util.COURT_WIDTH) or (y2 < 0) or (y2 > util.COURT_LENGTH):
        print "Invalid goal!"
        return    

    startNode = util.LatticeNode(stateparams = (x1,y1,th1,v))
    goalNode  = util.LatticeNode(stateparams = (x2,y2,th2,v2))    
    
    print " start ",startNode.get_stateparams()," goal ",goalNode.get_stateparams()
    (x1,y1,th1,v1) = startNode.get_stateparams()
    (x2,y2,th2,v2) = goalNode.get_stateparams()

    ########################################## DEBUGGING 
    if (util.SEARCHALGORITHM == "straightline"):
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

        path_to_send = Path()
        for point in path:
            pose = Pose()
        # plan uses center of car, so transform such that path has points to be traversed by rear axle center,                                                     # which is 17.41 cm away from the center                                                                                                                                                                                                                                                                       
            pose.x = point[0] - 17.41*math.cos(point[2])/100.0
            pose.y = point[1] - 17.41*math.sin(point[2])/100.0
            pose.theta = point[2]
            path_to_send.poses.append(pose)

        pub_path.publish(path_to_send)
        print "path published"
        return

    elif (util.SEARCHALGORITHM == "A*"):
        Astarsearch(startNode,goalNode)
    elif (util.SEARCHALGORITHM == "LPA*"):
        print "running LPA*"
        LPAstarsearch(startNode,goalNode)
    elif (util.SEARCHALGORITHM == "MT-AdaptiveA*"):
        print "running MT-AdaptiveA*"
        MTAdaptiveAstarsearch(startNode,goalNode,data.goaltype.data)    
        
    path_to_send = Path()
    for point in path:
        pose = Pose()
        # plan uses center of car, so transform such that path has points to be traversed by rear axle center,                                               
        # which is 17.41 cm away from the center                                                                                                             
        pose.x = point[0] - 17.41*math.cos(point[2])/100.0
        pose.y = point[1] - 17.41*math.sin(point[2])/100.0
        pose.theta = point[2]
        path_to_send.poses.append(pose)

    pub_path.publish(path_to_send)
    print "path published"


    path = []
    path.append((x1/100.0,y1/100.0,th1))
    path = path + util.plan_to_path(plan)        

    print "plan of length",len(plan)
    for (Node,action) in plan:
        print Node.get_stateparams(),action

    path_to_send = Path()    
    for point in path:
        pose = Pose()
        # plan uses center of car, so transform such that path has points to be traversed by rear axle center,
        # which is 17.41 cm away from the center        
        pose.x = point[0] - 17.41*math.cos(point[2])/100.0
        pose.y = point[1] - 17.41*math.sin(point[2])/100.0
        pose.theta = point[2]
        path_to_send.poses.append(pose)

    pub_path.publish(path_to_send)
    print "path published"

def Astarsearch(startNode,goalNode):
    global plan
    plan = []
    (x1,y1,th1,v1) = startNode.get_stateparams()
    time_exec = time.time()
    goalNode = util.Astarsearch(startNode,goalNode)    
    if(goalNode != None):
        print "found goal! in",time.time() - time_exec,"s"            
        node = goalNode
        while(node.getParent()!= None):
            parent = node.getParent()
            #print parent.get_stateparams(),node.getAction(),node.get_stateparams()
            #print node.getAction(),util.controlset.len_action(node.getAction())
            plan.append((parent,node.getAction()))                   
            node = parent  
        print ""            
        plan.reverse()
        (x2,y2,th2,v2) = goalNode.get_stateparams()                        
    else:
        print "No path to goal!"

def LPAstarsearch(startNode,goalNode):
    global plan   

    # draw plan        
    for (node,action) in util.plan_LPAstar:
        graphics.draw_segment(node,action)   

def MTAdaptiveAstarsearch(startNode,goalNode,goaltype):
    global plan    
    if(goaltype == "newball"):
        plan = util.MTAdaptiveAstarsearch_start(startNode,goalNode)    
    elif(goaltype == "updategoal"):
        plan = util.MTAdaptiveAstarsearch_update(startNode,goalNode)
    else:
        print "unknown goal type",goaltype


def obstacle_added(event):        
    util.costmap.new_obstacle(event)
    if(startNode != None) and (goalNode != None):
        graphics.canvas.delete(ALL)
        util.costmap.draw_costmap()
        graphics.draw_court() 

        # draw plan
        for (node,action) in util.plan_LPAstar:            
            graphics.draw_segment(node,action)

        (start_x,start_y,start_th,start_v) = startNode.get_stateparams()
        (goal_x,goal_y,goal_th,goal_v) = goalNode.get_stateparams()
        graphics.draw_point_directed(start_x,start_y,start_th,'red')
        graphics.draw_point_directed(goal_x,goal_y,goal_th,'red')


def init_planner():
    rospy.init_node('LatticePlanner', anonymous=True)
    rospy.Subscriber("goal", Goal, startPlanner)
    rospy.Subscriber("odometry",Pose,received_odometry)
    rospy.spin()


if __name__ == '__main__':
    try:
        util.controlset = controlset.ControlSet()
        util.costmap = util.CostMap()
        init_planner()
    except rospy.ROSInterruptException: pass

