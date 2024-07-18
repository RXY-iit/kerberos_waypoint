#include "kerberos_waypoint_publisher/simple_waypoint_manager.h"
#include <fstream>
#include <sstream>

WaypointManager::WaypointManager() : private_nh("~"), mb_client_("move_base", true)
{ /*{{{*/
  private_nh.param("waypoint_file", wp_filename_, std::string("none"));
  private_nh.param("default_mode", default_mode_str_, std::string("default"));
  private_nh.param("map_frame", map_frame_, std::string("map"));
  private_nh.param("robot_frame", robot_frame_, std::string("base_link"));
  private_nh.param("update_period", update_period_, 0.1);
  private_nh.param("publish_distance", publish_distance_, 5.0);

  goal_pub_ = nh.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 10);

  // current_wp_index_ = 0;
  latest_wp_index_ = 0;

  readWaypointFile(wp_filename_);

  // wait for the action server to come up
  while (!mb_client_.waitForServer(ros::Duration(5.0)))
  {
    ROS_INFO("[WaypointManager] Waiting for the move_base action server to come up");
  }

  robot_frame_callback_timer_ =
      nh.createTimer(ros::Duration(update_period_), &WaypointManager::robotPoseCallback, this, false);

  ROS_INFO("[WaypointManager] Ready to publish waypoint");

  // publieh first way point
  publishWP(latest_wp_index_);
} /*}}}*/

void WaypointManager::readWaypointFile(std::string filename)
{ /*{{{*/
  std::ifstream file(filename);
  if (!file)
  {
    ROS_INFO("[WaypointManager] cannot read waypoint file (%s)", filename.c_str());
    return;
  }

  wp_list_.clear();

  std::string reading_line;
  while (1)
  {
    std::getline(file, reading_line);
    if (reading_line.empty())
    {
      ROS_INFO("[WaypointManager] %d waypoints are loaded", (int)wp_list_.size());
      break;
    }
    std::stringstream line_ss(reading_line);

    Waypoint wp;
    std::string hash;
    line_ss >> wp.x >> wp.y >> wp.yaw >> wp.vellocity >> wp.mode;
    wp_list_.push_back(wp);
    // wp_list_[hash] = wp;
  }
} /*}}}*/

void WaypointManager::robotPoseCallback(const ros::TimerEvent&)
{ /*{{{*/
  try
  {
    tf_listener_.lookupTransform(map_frame_, robot_frame_, ros::Time(0), robot_pose_tf_);
  }
  catch (tf::TransformException ex)
  {
    ROS_ERROR("%s", ex.what());
    return;
  }

  updateWP();
  // updateMode();
} /*}}}*/

void WaypointManager::updateWP()
{ /*{{{*/
  if (wp_list_.size() <= latest_wp_index_)
    return;

  double sq_dist = pow(robot_pose_tf_.getOrigin().getX() - wp_list_[latest_wp_index_].x, 2) +
                   pow(robot_pose_tf_.getOrigin().getY() - wp_list_[latest_wp_index_].y, 2);

  if (sq_dist < publish_distance_ * publish_distance_)
  {
    ++latest_wp_index_;
    publishWP(latest_wp_index_);
    ROS_INFO("[WaypointManager] Send goal (No. %d, %s)", latest_wp_index_, wp_list_[latest_wp_index_].mode.c_str());
  }
} /*}}}*/

void WaypointManager::publishWP(int index)
{ /*{{{*/
  geometry_msgs::PoseStamped goal;
  goal.header.frame_id = map_frame_;
  goal.header.stamp = ros::Time::now();
  goal.pose.position.x = wp_list_[index].x;
  goal.pose.position.y = wp_list_[index].y;
  goal.pose.position.z = 0.0;
  geometry_msgs::Quaternion quaternion;
  quaternion = tf::createQuaternionMsgFromRollPitchYaw(0.0, 0.0, wp_list_[index].yaw);
  goal.pose.orientation = quaternion;
  goal_pub_.publish(goal);
} /*}}}*/

void WaypointManager::spin()
{ /*{{{*/
  ros::spin();
} /*}}}*/
