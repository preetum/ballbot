#!/usr/bin/env python
import math
import util
import time
import graphics
import controlset
from Tkinter import *

import roslib; roslib.load_manifest('LatticePlanner')
import rospy
from std_msgs.msg import String

startNode = None
goalNode = None
plan = [] # stores the computed plan, as [(node1,action1),(node2,action2)....]
path = [] # stores the computed path, as a sequence of (x,y,theta) values
pub = rospy.Publisher('ValidPath',String)

def startPlanner(x1,y1,th1,d_goal,th_goal):
    """
    Start A* search to connect start to goal :)
    """
    """
    Draw court
    """
    global startNode,goalNode,plan,path
    graphics.canvas.delete(ALL)
    util.costmap.draw_costmap()
    graphics.draw_court()    
    th1 = math.radians(th1)

    (x1,y1,th1,v) = util.point_to_lattice(x1,y1,th1,util.ROBOT_SPEED_MAX)
    
    th_goal = th1 - math.radians(th_goal)     # angle to goal in global coord
                                              # = angle of car in global coord - angle to goal from car (which is in [-90deg,90deg]
    x2 = x1 + d_goal*math.cos(th_goal)
    y2 = y1 + d_goal*math.sin(th_goal)
    th2 = th1                                   # doesn't matter for goal test
    v2 = util.ROBOT_SPEED_MAX
    (x2,y2,th2,v2) = util.point_to_lattice(x2,y2,th2,util.ROBOT_SPEED_MAX)
    
    if(x1 < 0) or (x1 > util.COURT_WIDTH) or (y1 < 0) or (y1 > util.COURT_LENGTH):
        print "Invalid start!"
        return
    if(x2 < 0) or (x2 > util.COURT_WIDTH) or (y2 < 0) or (y2 > util.COURT_LENGTH):
        print "Invalid goal!"
        return    

    startNode = util.LatticeNode(stateparams = (x1,y1,th1,v))
    goalNode  = util.LatticeNode(stateparams = (x2,y2,th2,v2))    
    
    print "start ",startNode.get_stateparams()," goal ",goalNode.get_stateparams()
    (x1,y1,th1,v1) = startNode.get_stateparams()
    (x2,y2,th2,v2) = goalNode.get_stateparams()

    #return
    graphics.draw_point_directed(x1,y1,th1,'red')
    graphics.draw_point(x2,y2,th2,'red')
    graphics.canvas.update_idletasks()

    if (util.SEARCHALGORITHM == "A*"):
        Astarsearch(startNode,goalNode)
    elif (util.SEARCHALGORITHM == "LPA*"):
        print "running LPA*"
        LPAstarsearch(startNode,goalNode)
    elif (util.SEARCHALGORITHM == "MT-AdaptiveA*"):
        print "running MT-AdaptiveA*"
        MTAdaptiveAstarsearch(startNode,goalNode)
        
    print "Hit enter to drive along path"
    raw_input()    
    path.append((x1/100.0,y1/100.0,th1))
    path = path + util.plan_to_path(plan)
    print path
    raw_input()
    pub.publish(String("ValidPath"))

def Astarsearch(startNode,goalNode):
    global plan
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
            graphics.draw_segment(parent,node.getAction())        
            node = parent  
        print ""            
        plan.reverse()
        (x2,y2,th2,v2) = goalNode.get_stateparams()        
        graphics.draw_point_directed(x1,y1,th1,'red')
        graphics.draw_point_directed(x2,y2,th2,'red')
    #simulator.init_simulator(startNode,path)
    else:
        print "No path to goal!"

def LPAstarsearch(startNode,goalNode):
    global plan
    util.LPAstarsearch(startNode,goalNode)

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

def MTAdaptiveAstarsearch(startNode,goalNode):
    global plan
    (start_x,start_y,start_th,start_v) = startNode.get_stateparams()
    (goal_x,goal_y,goal_th,goal_v) = goalNode.get_stateparams()
    graphics.draw_point_directed(start_x,start_y,start_th,'red')
    graphics.draw_point_directed(goal_x,goal_y,goal_th,'red')
    plan = util.MTAdaptiveAstarsearch(startNode,goalNode)    
    
    graphics.draw_point_directed(start_x,start_y,start_th,'red')
    graphics.draw_point_directed(goal_x,goal_y,goal_th,'red')
    

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

def main():
    util.controlset = controlset.ControlSet()
    util.costmap = util.CostMap()
    root = graphics.root
    root.title("Lattice Planner")
    root.configure(background = 'grey')
    graphics.canvas.pack()

    # x initial
    entryLabel_x1 = Label(root)
    entryLabel_x1["text"] = "init: X"
    entryLabel_x1.pack(side = LEFT)
    entryWidget_x1 = Entry(root)
    entryWidget_x1["width"] = 5
    entryWidget_x1.pack(side = LEFT)

    # y initial
    entryLabel_y1 = Label(root)
    entryLabel_y1["text"] = "Y"
    entryLabel_y1.pack(side = LEFT)
    entryWidget_y1 = Entry(root)
    entryWidget_y1["width"] = 5
    entryWidget_y1.pack(side = LEFT)

    # theta initial
    entryLabel_th1 = Label(root)
    entryLabel_th1["text"] = "TH"
    entryLabel_th1.pack(side = LEFT)
    entryWidget_th1 = Entry(root)
    entryWidget_th1["width"] = 5
    entryWidget_th1.pack(side = LEFT)

    # d goal (cm)
    entryLabel_d = Label(root)
    entryLabel_d["text"] = "    goal: d"
    entryLabel_d.pack(side = LEFT)
    entryWidget_d = Entry(root)
    entryWidget_d["width"] = 5
    entryWidget_d.pack(side = LEFT)

    # theta goal
    entryLabel_th = Label(root)
    entryLabel_th["text"] = "TH"
    entryLabel_th.pack(side = LEFT)
    entryWidget_th = Entry(root)
    entryWidget_th["width"] = 5
    entryWidget_th.pack(side = LEFT)
    
    b = Button(root,text = "Go",command = lambda: startPlanner(float(entryWidget_x1.get()),float(entryWidget_y1.get()),float(entryWidget_th1.get()),float(entryWidget_d.get()),float(entryWidget_th.get())))
    g = Button(root,text = "Show lattice",command = lambda:graphics.draw_lattice())
    g.pack(side = RIGHT)
    b.pack(side = RIGHT)
    util.costmap.draw_costmap()
    graphics.draw_court()

    graphics.canvas.configure(cursor = "crosshair")
    graphics.canvas.bind("<Button-1>",obstacle_added)
   
    root.mainloop()

main()
