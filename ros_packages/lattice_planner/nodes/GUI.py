#!/usr/bin/env python

"""
GUI display for ballbot
You can send go-to-ball (end angle not specifed) as well as go-to-pose (end angle specified) commands.
Publishes to topics:  goal,obstacles
Subscribes to topics: path,odom
"""
import roslib; roslib.load_manifest('lattice_planner')
import rospy
import graphics
import math
from Tkinter import *

from std_msgs.msg import String
from bb_msgs.msg import Pose,Path,Goal

pub_goal = rospy.Publisher('goal',Goal)
Ballbot_x = 10.0
Ballbot_y = 10.0
Ballbot_theta = math.pi/2
Ballbot_Tkobjects = None

Goal_x = 0.0
Goal_y = 0.0

root = None

def startPlanner(d_goal=0.0,th_goal=0.0,x_goal=0.0,y_goal=0.0,th_goal_abs=0.0,goaltype = "None"):
    """
    Send a goal message to topic 'goal'
    """
    """
    Draw court
    """
    global Goal_x,Goal_y

    graphics.canvas.delete(ALL)    
    graphics.draw_court()        

    if(goaltype == "newball"):        
        th_goal = Ballbot_theta - math.radians(th_goal)     # angle to goal in global coord
                                                     # = angle of car in global coord - angle to goal from car (which is in [-90deg,90deg]
        x2 = Ballbot_x*100.0 + d_goal*math.cos(th_goal)
        y2 = Ballbot_y*100.0 + d_goal*math.sin(th_goal)
        th2 = 0                                   # doesn't matter for goal test

        # publish goal
        goal_msg = Goal()
        goal_msg.goaltype = String("newball")
        goal_msg.pose = Pose(x2,y2,th2)
        graphics.draw_point(x2,y2,th2,color='red')
        graphics.canvas.update_idletasks()
        pub_goal.publish(goal_msg)
        print "sent goal"
        Goal_x = x2
        Goal_y = y2
    
    elif(goaltype == "gotopose"):        
        x2 = x_goal
        y2 = y_goal
        th2 = math.radians(th_goal_abs)

        # publish goal
        goal_msg = Goal()
        goal_msg.goaltype = String("gotopose")
        goal_msg.pose = Pose(x2,y2,th2)
        graphics.draw_point(x2,y2,th2,color='red')
        graphics.canvas.update_idletasks()
        pub_goal.publish(goal_msg)
        print "sent goal"
        Goal_x = x2
        Goal_y = y2

        """
    #return # remove for MTA*    
        while not rospy.is_shutdown():
            raw_input()
        # publish updated goal
            goal_msg = Goal()
            goal_msg.goaltype = String("updategoal")
            y2+=20
            goal_msg.pose = Pose(x2,y2,th2)
            graphics.draw_point(x2,y2,th2,color='red')
            graphics.canvas.update_idletasks()
            pub_goal.publish(goal_msg)
            print "sent goal"
            
            Goal_x = x2
            Goal_y = y2
        """
            

def received_odometry(data):
    global Ballbot_x,Ballbot_y,Ballbot_theta,Ballbot_Tkobjects
    # Coordinate frame conversion from localization frame to planner frame
    Ballbot_x = (data.y + 3.658)
    Ballbot_y = (30.17 - data.x)
    Ballbot_theta = (data.theta - math.pi/2)%(2*math.pi)
    #print (Ballbot_x,Ballbot_y,Ballbot_theta)
    """
    delete old car
    """
    if(Ballbot_Tkobjects!=None):
        graphics.canvas.delete(Ballbot_Tkobjects[0])
        #graphics.canvas.delete(Ballbot_Tkobjects[1])
    """
    redraw car at new position
    """            
    #rospy.loginfo("drawing car at %f,%f,%f",Ballbot_x,Ballbot_y,Ballbot_theta)
    Ballbot_Tkobjects = graphics.draw_car(Ballbot_x*100.0 + 17.41*math.cos(Ballbot_theta) ,Ballbot_y*100.0+ 17.41*math.sin(Ballbot_theta),Ballbot_theta)

def received_path(data):
    """
    draw the new path
    """
    graphics.canvas.delete(ALL)
    graphics.draw_court()    
    graphics.draw_point(Goal_x,Goal_y,0,color='red')
        
    for element in data.poses:
        pose = element.pose
        graphics.draw_point(pose.x*100.0,pose.y*100.0,pose.theta,color='blue')    
        #print "drew",pose.x*100.0,pose.y*100.0,pose.theta
    graphics.draw_point_directed(data.poses[0].pose.x*100.0,data.poses[0].pose.y*100.0,data.poses[0].pose.theta)
    graphics.draw_point_directed(data.poses[-1].pose.x*100.0,data.poses[-1].pose.y*100.0,data.poses[-1].pose.theta)
        
    graphics.canvas.update_idletasks()
    print "drew path"

def windowclosed():
    rospy.signal_shutdown("Window closed by user!") 
    root.quit()

def onclick(event):
    """ 
    print cost of clicked point
    """
    x = event.x/graphics.cm_to_pixels
    y = event.y/graphics.cm_to_pixels    
    

def initialize_gui():    
    global root                 
    rospy.init_node('GUI')
    while not rospy.is_shutdown():
        root = graphics.root
        root.protocol("WM_DELETE_WINDOW", windowclosed)
        root.title("Lattice Planner")
        root.configure(background = 'grey')
        graphics.canvas.pack()        

        # d goal (cm)
        entryLabel_d = Label(root)
        entryLabel_d["text"] = "goal: d"
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
    
        # Buttons
        
        b = Button(root,text = "Newball",command = lambda: startPlanner(d_goal = float(entryWidget_d.get()),th_goal = float(entryWidget_th.get()),
                                                                        goaltype = "newball"))
        
        b.pack(side = LEFT)
        g = Button(root,text = "Show lattice",command = lambda:graphics.draw_lattice())
        g.pack(side = RIGHT)

        # goal_x (cm)
        entryLabel_x = Label(root)
        entryLabel_x["text"] = "goal: x"
        entryLabel_x.pack(side = LEFT)
        entryWidget_x = Entry(root)
        entryWidget_x["width"] = 5
        entryWidget_x.pack(side = LEFT)

        # goal_y (cm)
        entryLabel_y = Label(root)
        entryLabel_y["text"] = "goal: y"
        entryLabel_y.pack(side = LEFT)
        entryWidget_y = Entry(root)
        entryWidget_y["width"] = 5
        entryWidget_y.pack(side = LEFT)

        # goal_theta (cm)
        entryLabel_th2 = Label(root)
        entryLabel_th2["text"] = "goal: theta"
        entryLabel_th2.pack(side = LEFT)
        entryWidget_th2 = Entry(root)
        entryWidget_th2["width"] = 5
        entryWidget_th2.pack(side = LEFT)

        b2 = Button(root,text = "GoToPose",command = lambda: startPlanner(x_goal = float(entryWidget_x.get()),y_goal = float(entryWidget_y.get()),
                                                                         th_goal_abs = float(entryWidget_th2.get()),
                                                                        goaltype = "gotopose"))
        b2.pack(side = RIGHT)
    
        graphics.draw_court()
        graphics.draw_landmarks()
        graphics.canvas.configure(cursor = "crosshair")        
        graphics.canvas.bind("<Button-1>", onclick)

        rospy.Subscriber("pose",Pose, received_odometry)
        rospy.Subscriber("path",Path,received_path)
        root.mainloop()

if __name__ == '__main__':
    try:
       initialize_gui() 
    except rospy.ROSInterruptException: pass
