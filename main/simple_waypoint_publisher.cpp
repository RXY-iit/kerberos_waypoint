#include <ros/ros.h>
#include "kerberos_waypoint_publisher/simple_waypoint_manager.h"

int main(int argc, char** argv)
{ /*{{{*/
  ros::init(argc, argv, "waypoint_publisher");
  WaypointManager wpm;
  wpm.spin();

  return 0;
} /*}}}*/
