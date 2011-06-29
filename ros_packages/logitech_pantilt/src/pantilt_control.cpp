/*
  Remarks:
  Pan/tilt only work for small changes for some reason.
  For pan, try not to move more than 20 degrees at once.
  For tilt, set and forget should be good enough for now.
 */
#include <webcam.h>
#include <unistd.h>

#include "ros/ros.h"
#include "logitech_pantilt/Orientation2.h"
#include "logitech_pantilt/SetAngle.h"
#include "logitech_pantilt/Reset.h"

using namespace ros;
using namespace logitech_pantilt;

bool is_moving = false;
int pan = 0,
  tilt = 0,
  pan_target = 0,
  tilt_target = 0;
Time move_start,
  move_stop;

double pan_angular_velocity = 72;  // in degrees/s
double tilt_angular_velocity = 48;

double pan_min = -72,
  pan_max = 72,
  tilt_min = -20,
  tilt_max = 40;

CHandle webcam_device;

inline void sleep_ms(ulong ms) {
  sleep(ms / 1000);
  usleep((ms % 1000) * 1000);
}

/* Resets both pan and tilt */
void webcam_reset(void) {
  CControlValue control_value = {CC_TYPE_BYTE, {1}};
  is_moving = true;
  c_set_control(webcam_device, CC_PAN_RESET, &control_value);
  sleep(3);
  c_set_control(webcam_device, CC_TILT_RESET, &control_value);
  sleep_ms(1700);
  is_moving = false;
}

/* Set pan angle in degrees */
Duration webcam_set_pan(int new_pan) {
  if (new_pan > pan_max) new_pan = pan_max;
  else if (new_pan < pan_min) new_pan = pan_min;

  ROS_INFO("setting pan to %d", new_pan);

  if (new_pan != pan) {

    int relative_angle = -64 * (new_pan - pan);
    ROS_INFO("-> set pan relative by %d", relative_angle);
    CControlValue control_value = {CC_TYPE_WORD, {relative_angle}};
    c_set_control(webcam_device, CC_PAN_RELATIVE, &control_value);

    pan_target = new_pan;
    is_moving = true;
    move_start = Time::now();
  }

  Duration estimate = Duration(abs(new_pan - pan) / pan_angular_velocity);
  move_stop = move_start + estimate;
  return estimate;
}

/* Set tilt angle in degrees */
Duration webcam_set_tilt(int new_tilt) {
  if (new_tilt > tilt_max) new_tilt = tilt_max;
  else if (new_tilt < tilt_min) new_tilt = tilt_min;

  ROS_INFO("setting tilt to %d", new_tilt);

  if (new_tilt != tilt) {
    int relative_angle = -64 * (new_tilt - tilt);
    ROS_INFO("-> set tilt relative by %d", relative_angle);
    CControlValue control_value = {CC_TYPE_WORD, {relative_angle}};
    c_set_control(webcam_device, CC_TILT_RELATIVE, &control_value);
    
    tilt_target = new_tilt;
    is_moving = true;
    move_start = Time::now();
  }

  // Calculate and return time estimate
  Duration estimate = Duration(abs(new_tilt - tilt) / tilt_angular_velocity);
  move_stop = move_start + estimate;
  return estimate;
}

bool set_pan_callback(SetAngle::Request &req, SetAngle::Response &res) {
  res.move_time = webcam_set_pan(req.angle);
  return true;
}

bool set_tilt_callback(SetAngle::Request &req, SetAngle::Response &res) {
  res.move_time = webcam_set_tilt(req.angle);
  return true;
}

bool reset_callback(Reset::Request &req, Reset::Response &res) {
  webcam_reset();
  return true;
}

int main(int argc, char **argv)
{
  const char *webcam_name = "/dev/video0";

  // Initialize ROS
  init(argc, argv, "talker");
  NodeHandle n;

  // Process command line args
  if (argc > 1) {
    webcam_name = argv[1];
  }

  // Initialize webcam
  c_init();
  ROS_INFO("opening %s using libwebcam\n", webcam_name);
  webcam_device = c_open_device(webcam_name);
  webcam_reset();

  CControlValue control_value = {CC_TYPE_BYTE, {1}};
  c_set_control(webcam_device, CC_AUTO_EXPOSURE_MODE, &control_value);
  control_value.value = 160;
  c_set_control(webcam_device, CC_EXPOSURE_TIME_ABSOLUTE, &control_value);
  control_value.value = 96;
  c_set_control(webcam_device, CC_BRIGHTNESS, &control_value);
  control_value.value = 16;
  c_set_control(webcam_device, CC_CONTRAST, &control_value);
  control_value.value = 200;
  c_set_control(webcam_device, CC_GAIN, &control_value);

  // Advertise services
  ServiceServer pan_service = n.advertiseService("set_pan", set_pan_callback);
  ServiceServer tilt_service = n.advertiseService("set_tilt", set_tilt_callback);
  ServiceServer reset_service = n.advertiseService("reset", reset_callback);

  Publisher orientation_pub = n.advertise<Orientation2>("orientation", 100);
  Rate loop_rate(20);

  while (ros::ok()) {
    int current_pan = pan,
      current_tilt = tilt;

    // If moving, calculate pan/tilt from known velocities
    //  and check whether moving finished
    // The global pan and tilt values contain the start value; they are not
    //  updated every loop to prevent error from propagating
    if (is_moving) {
      Time now = Time::now();
      Duration elapsed = now - move_start;

      // Pan was moving
      if (pan_target != pan) {
	// If we've reached our goal
	if (now > move_stop) {
	  ROS_INFO("finished panning");
	  is_moving = false;
	  current_pan = pan = pan_target;
	} 
	// Otherwise try estimating the pan angle based on measured velocity
	else {
	  int pan_dir = (pan_target > pan) ? 1 : -1;
	  current_pan = 
	    pan + pan_dir * (int)(elapsed.toSec()*pan_angular_velocity + 0.5);
	}
      }

      // Tilt was moving
      else if (tilt_target != tilt) {
	// If we've reached our goal
	if (now > move_stop) {
	  ROS_INFO("finished tilting");
	  is_moving = false;
	  current_tilt = tilt = tilt_target;
	} 
	// Otherwise try estimating the pan angle based on measured velocity
	else {
	  int tilt_dir = (tilt_target > tilt) ? 1 : -1;
	  current_tilt = 
	    tilt + tilt_dir * (int)(elapsed.toSec()*tilt_angular_velocity + 0.5);
	}
      }
    }

    // Publish pan/tilt orientation
    Orientation2 orient;
    orient.pan = current_pan;
    orient.tilt = current_tilt;
    orientation_pub.publish(orient);

    // Process callbacks ONLY IF not moving
    if (!is_moving) spinOnce();
    loop_rate.sleep();
  }


  return 0;
}
