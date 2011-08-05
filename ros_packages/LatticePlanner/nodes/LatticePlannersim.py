import math
import util
import time
import graphics
import controlset
from Tkinter import *

startNode = None
goalNode = None

def startPlanner(x1,y1,th1,d_goal,th_goal):
    """
    Start A* search to connect start to goal :)
    """
    """
    Draw court
    """

    global startNode,goalNode

    graphics.canvas.delete(ALL)
    util.costmap.draw_costmap()
    graphics.draw_court()    
    th1 = math.radians(th1)

    """
    state = util.LatticeNode(stateparams = (787.5,1715.0,5*math.pi/4,util.ROBOT_SPEED_MAX))
    nextstate  = util.LatticeNode(stateparams = (682.5,1820.0,math.pi,util.ROBOT_SPEED_MAX))
    print "cost of action",util.cost(state,"RS_f",nextstate)
    print "cost of cell at 682.5,1820.0",util.costmap.cost_cell(682.5,1820.0)
    return
    """

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

    """
    Print allowed car motions from startNode 
    
    #graphics.draw_segment(startNode,"F")
    #graphics.draw_segment(startNode,"F3")
    graphics.draw_segment(startNode,"B")
    graphics.draw_segment(startNode,"R_f")
    graphics.draw_segment(startNode,"R_b")
    graphics.draw_segment(startNode,"L_f")
    graphics.draw_segment(startNode,"L_b")
    #graphics.draw_segment(startNode,"SR_f")
    #graphics.draw_segment(startNode,"SL_f")
    graphics.draw_segment(startNode,"RS_f_short")
    graphics.draw_segment(startNode,"LS_f_short")
    graphics.draw_segment(startNode,"sidestep_R_f")
    graphics.draw_segment(startNode,"sidestep_L_f")   
    graphics.draw_segment(startNode,"SL_f_2")
    graphics.draw_segment(startNode,"SR_f_2")
    
    graphics.draw_segment(startNode,"LSL_f")
    graphics.draw_segment(startNode,"RSR_f")
    return        
    graphics.draw_segment(startNode,"RS_f")
    graphics.draw_segment(startNode,"LS_f")
    graphics.draw_segment(startNode,"F_diag")
    graphics.draw_segment(startNode,"F_diag3")
    graphics.draw_segment(startNode,"B_diag")
    graphics.draw_segment(startNode,"L2_f")
    graphics.draw_segment(startNode,"R2_f")
    return    
    graphics.draw_segment(startNode,"F1_26.6")
    graphics.draw_segment(startNode,"B1_26.6")
    return
    
    graphics.draw_segment(startNode,"LSL1_f_63.4")
    return
    """
    if (util.SEARCHALGORITHM == "A*"):
        Astarsearch(startNode,goalNode)
    elif (util.SEARCHALGORITHM == "LPA*"):
        print "running LPA*"
        LPAstarsearch(startNode,goalNode)
    elif (util.SEARCHALGORITHM == "MT-AdaptiveA*"):
        print "running MT-AdaptiveA*"
        MTAdaptiveAstarsearch(startNode,goalNode)

def Astarsearch(startNode,goalNode):
    (x1,y1,th1,v1) = startNode.get_stateparams()
    time_exec = time.time()
    goalNode = util.Astarsearch(startNode,goalNode)    
    if(goalNode != None):
        print "found goal! in",time.time() - time_exec,"s"    
        path = []
        node = goalNode
        while(node.getParent()!= None):
            parent = node.getParent()
            #print parent.get_stateparams(),node.getAction(),node.get_stateparams()
            #print node.getAction(),util.controlset.len_action(node.getAction())
            path.append(node.getAction())
            graphics.draw_segment(parent,node.getAction())        
            node = parent  
        print ""            
        path.reverse()
        (x2,y2,th2,v2) = goalNode.get_stateparams()        
        graphics.draw_point_directed(x1,y1,th1,'red')
        graphics.draw_point_directed(x2,y2,th2,'red')
    #simulator.init_simulator(startNode,path)
    else:
        print "No path to goal!"

def LPAstarsearch(startNode,goalNode):
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

    (start_x,start_y,start_th,start_v) = startNode.get_stateparams()
    (goal_x,goal_y,goal_th,goal_v) = goalNode.get_stateparams()
    graphics.draw_point_directed(start_x,start_y,start_th,'red')
    graphics.draw_point_directed(goal_x,goal_y,goal_th,'red')
    path = util.MTAdaptiveAstarsearch(startNode,goalNode)    
    for segment in path:
        (node,action) = segment
        util.agentNode = node
        (car_x,car_y,car_th,car_v) = util.agentNode.get_stateparams()
        print "drew car"
        car_drawing = graphics.draw_car(car_x,car_y,car_th)
        graphics.canvas.update_idletasks()
        time.sleep(0.5)
        graphics.canvas.delete(car_drawing[0])
        graphics.canvas.delete(car_drawing[1])
    
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
