#ifndef __SIMPLE_WAYPOINT_MANAGER_H__
#define __SIMPLE_WAYPOINT_MANAGER_H__

#include <actionlib/client/simple_action_client.h>
#include <dynamic_reconfigure/Config.h>
#include <dynamic_reconfigure/DoubleParameter.h>
#include <dynamic_reconfigure/Reconfigure.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <move_base_msgs/MoveBaseActionResult.h>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <Eigen/Core>
#include <iostream>

class WaypointManager
{
public:
  struct Waypoint
  { /*{{{*/
    // std::string label;
    double x;
    double y;
    double yaw;
    double vellocity;
    std::string mode;
  }; /*}}}*/

  struct Parameter
  { /*{{{*/
    std::string name;
    std::string data_type;
    std::string value;
  }; /*}}}*/

  struct Mode
  { /*{{{*/
    std::string mode;
    std::vector<Parameter> parameters;
  }; /*}}}*/

  WaypointManager();
  void spin();

private:
  void readWaypointFile(std::string filename);
  void readModeFile(std::string filename);
  void robotPoseCallback(const ros::TimerEvent&);
  void updateWP();
  void publishWP(int index);
  void updateMode();

  ros::NodeHandle nh, private_nh;
  ros::Timer robot_frame_callback_timer_;
  ros::Publisher goal_pub_;
  tf::TransformListener tf_listener_;
  tf::StampedTransform robot_pose_tf_;

  std::vector<Waypoint> wp_list_;
  // std::unorderd_map<std::string, Waypoint> wp_list_;
  std::vector<Mode> mode_list_;
  Mode default_mode_;
  int latest_wp_index_;
  Eigen::Vector2d boundary_vec_;

  // not in use
  actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> mb_client_;

  std::string map_frame_, robot_frame_;
  std::string wp_filename_, mode_filename_;
  std::string default_mode_str_;
  double update_period_;
  double publish_distance_;

};  // class WaypointManager

#endif