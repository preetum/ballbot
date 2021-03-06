#!/usr/bin/env python
import math
import parameters
import util
import time
import controlset
import debugpaths

import roslib; roslib.load_manifest('lattice_planner')
import rospy
from std_msgs.msg import String
from bb_msgs.msg import Pose,Path,PathElement,Goal

startNode = None
goalNode = None
plan = [] # stores the computed plan, as [(node1,action1),(node2,action2)....]
path = [] # stores the computed path, as a sequence of (x,y,theta) values

pub_path = rospy.Publisher('path',Path)

Ballbot_x = 1000
Ballbot_y = 1000
Ballbot_theta = math.pi/2

def received_odometry(data):
    global Ballbot_x,Ballbot_y,Ballbot_theta
    # Coordinate frame conversion from localization frame to planner frame
    Ballbot_x = (data.y + 3.658)*100.0
    Ballbot_y = (30.17 - data.x)*100.0
    Ballbot_theta = (data.theta - math.pi/2)%(2*math.pi)
    
def startPlanner(data):    
    global startNode,goalNode,plan,path
    
    (x1,y1,th1,v) = util.point_to_lattice(Ballbot_x,Ballbot_y,Ballbot_theta,parameters.ROBOT_SPEED_MAX)    
 
    v2 = util.ROBOT_SPEED_MAX
    (x2,y2,th2,v2) = util.point_to_lattice(data.pose.x,data.pose.y,data.pose.theta,parameters.ROBOT_SPEED_MAX)
    
    if(x1 < 0) or (x1 > parameters.COURT_WIDTH) or (y1 < 0) or (y1 > parameters.COURT_LENGTH):
        print "Invalid start!",(x1,y1,th1)
        return
    if(x2 < 0) or (x2 > parameters.COURT_WIDTH) or (y2 < 0) or (y2 > parameters.COURT_LENGTH):
        print "Invalid goal!",(x2,y2,th2)
        return   
 
    util.goaltype = data.goaltype.data

    startNode = util.LatticeNode(stateparams = (x1,y1,th1,v))
    if(util.goaltype == "gotopose"):
        goalNode  = util.LatticeNode(stateparams = (x2,y2,th2,v2))    
    else:
        goalNode  = util.LatticeNode(stateparams = (data.pose.x,data.pose.y,th2,v2))    
    
    print " start ",startNode.get_stateparams()," goal ",goalNode.get_stateparams()
    (x1,y1,th1,v1) = startNode.get_stateparams()
    (x2,y2,th2,v2) = goalNode.get_stateparams()

    ########################################## DEBUGGING 
    if (parameters.SEARCHALGORITHM == "straightline"):
        debugpaths.straightLine((x1,y1,th1,v1),(x2,y2,th2,v2))
        return

    elif (parameters.SEARCHALGORITHM == "figureofeight"):
        debugpaths.figureofeight((x1,y1,th1,v1))
        return
    ###########################################################################################################
        
    elif (parameters.SEARCHALGORITHM == "A*"):
        Astarsearch(startNode,goalNode)
    elif (parameters.SEARCHALGORITHM == "LPA*"):
        print "running LPA*"
        LPAstarsearch(startNode,goalNode)
    elif (parameters.SEARCHALGORITHM == "MT-AdaptiveA*"):
        rospy.loginfo("running MT-AdaptiveA* %s",(startNode.get_stateparams(), goalNode.get_stateparams()))
        util.goaltype = data.goaltype.data
        MTAdaptiveAstarsearch(startNode,goalNode)

    if plan == None:
        print "Plan of length 0"
        return
    path = []
    path.append((x1/100.0,y1/100.0,th1,'s','f'))
    path = path + util.plan_to_path(plan)        
    path[0] = (x1/100.0,y1/100.0,th1,path[1][3],path[1][4])
    print "plan of length",len(plan)
    actionlist=[]
    for (Node,action) in plan:
        actionlist.append(action)
        #print Node.get_stateparams(),action,Node.get_g()
    rospy.loginfo(actionlist)    
    raw_input("Hit any key to confirm path")
    path_to_send = Path()    
    for point in path:        
        pose = Pose()
        # plan uses center of car, so transform such that path has points to be traversed by rear axle center,
        # which is 17.41 cm away from the center        
        pose.x = point[0] - 17.41*math.cos(point[2])/100.0
        pose.y = point[1] - 17.41*math.sin(point[2])/100.0
        pose.theta = point[2]
        
        path_element = PathElement()
        path_element.pose = pose
        path_element.type = point[3]
        path_element.direction = point[4]
        path_to_send.poses.append(path_element)
    
    #path[0] = (x1/100.0,y1/100.0,th1,path[1][3],path[1][4])
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

def MTAdaptiveAstarsearch(startNode,goalNode):
    global plan   
    t = time.time()
    if(util.goaltype == "newball" or util.goaltype == "gotopose"):
        plan = util.MTAdaptiveAstarsearch_start(startNode,goalNode)    
    elif(util.goaltype == "updategoal"):
        plan = util.MTAdaptiveAstarsearch_update(startNode,goalNode)
    else:
        print "unknown goal type",util.goaltype
    t = time.time() - t
    rospy.loginfo("time to plan %f",t)

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
    rospy.init_node('lattice_planner', anonymous=True)
    rospy.Subscriber("goal", Goal, startPlanner)
    rospy.Subscriber("pose",Pose,received_odometry)    
    rospy.spin()

if __name__ == '__main__':
    try:
        util.controlset = controlset.ControlSet()        
        util.costmap = util.CostMap()             
        init_planner()    
    except rospy.ROSInterruptException: pass
