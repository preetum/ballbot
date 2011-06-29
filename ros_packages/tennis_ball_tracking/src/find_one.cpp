#include "ros/ros.h"
#include "ros_ard_to_fro/drive_cmd.h"
#include "navigation/goal_msg.h"

using namespace ros;

Publisher pub;

void goal_callback(const navigation::goal_msg::ConstPtr& goal) {
  if (goal->d < 45) {
    ROS_INFO("Stopping");
    ros_ard_to_fro::drive_cmd cmd;
    cmd.drive_speed = 0;
    cmd.steer_angle = 0;
    pub.publish(cmd);
    
  } else {
    int steer = (int)(1.2 * goal->th + 0.5);
    ros_ard_to_fro::drive_cmd cmd;
    cmd.drive_speed = 60;
    cmd.steer_angle = steer;
    pub.publish(cmd);
  }
}

int main (int argc, char **argv) {
  init(argc, argv, "find_one");
  NodeHandle n;

  Subscriber sub = n.subscribe("goal", 100, goal_callback);
  pub = n.advertise<ros_ard_to_fro::drive_cmd>("vel_cmd", 100);
}
