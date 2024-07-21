#include <actionlib/client/simple_action_client.h>
#include <dynamic_reconfigure/Reconfigure.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <std_msgs/Bool.h>
#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <wp_generation/wp_array.h>
#include <wp_generation/wp_data.h>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <list>
#include <sstream>
#include <string>
#include <thread>
#include "../include/kerberos_waypoint_publisher_cpmode.hpp"

void WayPointManager::ForwardWaypointByKeyboardInterrupt()
{
  /*
  while(start_flag ==false){
      std::cout << "press return key" << std::endl;
      getchar();
      ros::Duration(1.5).sleep();
      start_flag = true;
      std::cout << "starting!!" << std::endl;
      forward_wp_flag = true;
  }
  */
  // std::cout << "Key" << std::endl;
  while (start_flag == true)
  {
    getchar();
    forward_wp_flag = true;
    std::cout << "KeyboardInterrupt" << std::endl;
  }
}
void WayPointManager::WpContinueCallback(const std_msgs::Bool::ConstPtr &msg)
{
  static ros::Time prev_restart_time = ros::Time::now();
  if (msg->data && (ros::Time::now() - prev_restart_time) > ros::Duration(10.0))
  {
    forward_wp_flag = true;
    std_msgs::String message;
    message.data = "Aボタンが押されたので、自動走行を再開します";
    std::cout << message.data << std::endl;
    talk_publisher.publish(message);
    prev_restart_time = ros::Time::now();
  }
}

WayPoint WayPointManager::GetRobotPose()
{
  WayPoint robot_pose;
  tf::StampedTransform transform;
  bool got_transform = false;
  while (got_transform == false)
  {
    got_transform = true;
    try
    {
      tf_listener.waitForTransform("map", "base_link", ros::Time(0), ros::Duration(1.0));
      tf_listener.lookupTransform("map", "base_link", ros::Time(0), transform);
    }
    catch (tf::TransformException ex)
    {
      ROS_ERROR("%s", ex.what());
      got_transform = false;
      ros::Duration(0.1).sleep();
    }
    robot_pose.x = transform.getOrigin().x();
    robot_pose.y = transform.getOrigin().y();
    robot_pose.yaw = tf::getYaw(transform.getRotation());
  }
  return robot_pose;
}
std::string WayPointManager::GetState() { return state; }

void WayPointManager::SignalGoCallback(const std_msgs::String::ConstPtr &go_flag)
{
  if (go_flag->data == "start" && signal_flag == true)
  {
    // forward_wp_flag =true;
    signal_ok = true;
    signal_count++;
    // std_msgs::String ktm;
    // ktm.data = "Stop";
    // signal_start_flag_publisher.publish(ktm);
  }
}

void WayPointManager::LetterRecogCallback(const std_msgs::String::ConstPtr &letter_result)
{
  // letter recog communation
  if (letter_flag == true){
    if (letter_result->data == "a")
    {
      letter_ok = true;
      box_letter = "a";
    }
    else if(letter_result->data == "b")
    {
      letter_ok = true;
      box_letter = "b";
    }
    else if(letter_result->data == "c")
    {
      letter_ok = true;
      box_letter = "c";
    }else{
      // box_letter = "none"; //no need
    }
  }
  // blue box communation
  if (box_flag == true){
    if(letter_result->data == "box found")
    {
      blue_box_ok = true;
    }else if(letter_result->data == "box not found") // time up calculation in AGS code
    {
      blue_box_ok = false;
    }else{
      // box_letter = "none"; //no need
    }
  }
}

void WayPointManager::LuggageCallback(const std_msgs::String::ConstPtr &luggage_result)
{
 // GET luggage on green box
  if (letter_flag == true){
    if (luggage_result->data == "get ok") //need discuss
    {
      luggage_get_ok = true;
    }
  }
// Throw luggage on green box
  if (box_flag == true){
    if (luggage_result->data == "throw ok") //need discuss
    {
      luggage_throw_ok = true;
    }
  }
}

void WayPointManager::CallBackCmdvel(const geometry_msgs::Twist::ConstPtr &msg) { cmd_vel_info = *msg; }

void WayPointManager::LineCallBack(const std_msgs::Bool::ConstPtr &msg) { line_flag = *msg; }

void WayPointManager::CarCallBack(const std_msgs::Bool::ConstPtr &msg) { car_flag = *msg; }

void WayPointManager::RoadClosureCallBack(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
  action_client->cancelAllGoals();
  forward_wp_flag = true;
  closure_flag = true;
}

void WayPointManager::BlockCallback(const std_msgs::Bool::ConstPtr &msg)
{
  if (msg->data == true)
  {
    /*
    std_msgs::String message;
    std::stringstream ss;
    ss << "経路封鎖を認識しました";
    message.data = ss.str();
    talk_publisher.publish(message);
    */
    action_client->cancelAllGoals();
    ros::Duration(5.0).sleep();
    block_flag = true;
    forward_wp_flag = true;
    // start_flag = false;
  }
}

void WayPointManager::WpArrayCallback(const wp_generation::wp_array::ConstPtr &msg) //msg type is defined in "wp_generation/msg/wp_generation"
{
  /*
  std_msgs::String message;
  std::stringstream ss;
  ss << "ウェイポイント情報を受けとりました";
  message.data = ss.str();
  talk_publisher.publish(message);
  */
  // std::cout<<"ウェイポイント情報を受けとりました"<<std::endl;
  if (block_flag)
  {
    std_msgs::String message;
    std::stringstream ss;
    /*     wp_mode.data = "START";
        wp_mode_publisher.publish(wp_mode); */
    ss << "ウェイポイントを更新します";
    message.data = ss.str();
    talk_publisher.publish(message);
    std::cout << "ウェイポイントを更新します" << std::endl;

    // tjm_wp_array = *msg;
    // ReadWayPointFile(tjm_wp_array);
    ReadWayPointFile(*msg);
    ConvertToWayPointMsg();
    block_flag = false;
    closure_flag = false;
    // start_flag = true;
  }
}
void WayPointManager::Hokuyo3dStatusCallback(const std_msgs::Bool::ConstPtr msg)
{
  dynamic_reconfigure::Config lpconf;
  dynamic_reconfigure::DoubleParameter double_param;
  dynamic_reconfigure::Reconfigure srv;
  if (hokuyo3d_status.data != msg->data && (!block_flag && current_wp_iter->wp_type != "crosswalk"))
  {
    lpconf.doubles.clear();
    if (msg->data)
    { // hokuyo3d 生きてる
      double_param.name = "max_vel_x";
      double_param.value = NORMAL_SPEED;
      lpconf.doubles.push_back(double_param);
      double_param.name = "sim_time";
      double_param.value = 2.0;
    }
    else
    { // hokuyo3d 死んでる
      double_param.name = "max_vel_x";
      double_param.value = SLOW_SPEED;
      lpconf.doubles.push_back(double_param);
      double_param.name = "sim_time";
      double_param.value = 5.0;
    }

    lpconf.doubles.push_back(double_param);
    srv.request.config = lpconf;
    localplanner_client.call(srv);
  }
  hokuyo3d_status = *msg;
}

void WayPointManager::CallBackTimer(const ros::TimerEvent &)
{
  static ros::Time t = ros::Time::now();
  if (ros::Time::now() - t > ros::Duration(10.0))
  {
    t = ros::Time::now();
    ConvertToWayPointMsg();
  }

  StateTransition();
  if (current_wp_iter == wp_array.end())
  {
    geometry_msgs::Twist cmd_vel;
    cmd_vel.linear.x = 0.0;
    cmd_vel.linear.y = 0.0;
    cmd_vel.linear.z = 0.0;
    cmd_vel.angular.x = 0.0;
    cmd_vel.angular.y = 0.0;
    cmd_vel.angular.z = 0.0;
    cmd_publisher.publish(cmd_vel);
    wp_mode.data = "STOP";
    wp_mode_publisher.publish(wp_mode);
    ros::spinOnce();
  }
}

WayPointManager::WayPointManager(ros::NodeHandle nh, ros::NodeHandle pnh)
{
  pnh.param("goal_tolerance_m", goal_tolerance_m, 8.0);
  pnh.param("skip_time_sec", skip_time_sec, 30.0);

  ros::param::get("/move_base/TrajectoryPlannerROS/max_vel_x", robot_max_velocity);
  nh.param("/move_base/TrajectoryPlannerROS/yaw_goal_tolerance", goal_tolerance_rad, M_PI / 9);
  goal_tolerance_rad = goal_tolerance_rad * 1.5;

  cmd_publisher = nh.advertise<geometry_msgs::Twist>("cmd_vel", 100);
  nav_goal_publisher = nh.advertise<geometry_msgs::PoseStamped>("move_base_simple/goal", 1);
  wp_publisher = nh.advertise<geometry_msgs::PoseArray>("waypoints", 1);
  talk_publisher = nh.advertise<std_msgs::String>("speech_text", 5);
  // reset_flag_publisher = nh.advertise<std_msgs::Empty>("/move_base/YmgGPHybROS/reset_flag", 1);
  // force_ymg_publisher = nh.advertise<std_msgs::Int32>("/move_base/YmgGPHybROS/use_ymggp_force", 1);
  reset_flag_publisher = nh.advertise<std_msgs::Empty>("/move_base/YmgGPBGP/reset_flag", 1);
  force_ymg_publisher = nh.advertise<std_msgs::Int32>("/move_base/YmgGPBGP/use_ymggp_force", 1);
  signal_start_flag_publisher = nh.advertise<std_msgs::String>("/flag/detection_switch", 1);
  recog_start_flag_publisher = nh.advertise<std_msgs::String>("/flag/image_recog_sign", 1);//box_finder and box_deliver add
  // 探索対象認識も行う場合
  //  target_start_flag_publisher = nh.advertise<std_msgs::Bool>("/edr_image_proc/detection_switch", 1);
  // 経路封鎖認識のみ行なう場合
  target_start_flag_publisher = nh.advertise<std_msgs::Bool>("/edr_image_proc/road_closure_switch", 1);
  grass_road_publisher = nh.advertise<std_msgs::Bool>("wp_grass_road", 1);
  wp_mode_publisher = nh.advertise<std_msgs::String>("wp_mode", 1);

  start_flag = true;

  hnd_flag_publisher = nh.advertise<std_msgs::String>("/localization_switch_name", 1);

  block_subscriber = nh.subscribe("/is_road_closure", 1, &WayPointManager::BlockCallback, this);
  hokuyo3d_status_subscriber = nh.subscribe("hokuyo3d_status", 1, &WayPointManager::Hokuyo3dStatusCallback, this);
  wp_array_subscriber = nh.subscribe("/all_wp_array", 1, &WayPointManager::WpArrayCallback, this);

  target_pose_subscriber = nh.subscribe("target_pose", 1, &WayPointManager::TargetPoseCallback, this);
  // signal_go_flag_subscriber = nh.subscribe("signal_go_flag",10, &WayPointManager::SignalGoCallback, this);
  signal_go_flag_subscriber = nh.subscribe("/trigger", 10, &WayPointManager::SignalGoCallback, this);
  recog_result_subscriber = nh.subscribe("/image_recog", 10, &WayPointManager::LetterRecogCallback, this); // box_finder mode add
  luggage_result_subscriber = nh.subscribe("/luggage", 10, &WayPointManager::LuggageCallback, this); // box_finder mode add
  use_bgp_subscriber = nh.subscribe("/move_base/YmgGPBGP/use_bgp_flag", 1, &WayPointManager::UseBGPCallback, this);
  initial_pose_subscriber = nh.subscribe<geometry_msgs::PoseWithCovarianceStamped>(
      "initialpose", 1, &WayPointManager::CallBackInitialPose, this);
  cmd_subscriber = nh.subscribe("/cmd_vel", 1, &WayPointManager::CallBackCmdvel, this);
  line_flag_subscriber = nh.subscribe("/odogp/line_flag", 1, &WayPointManager::LineCallBack, this);
  car_flag_subscriber = nh.subscribe("/crosswalk_vehicle_detection/car_flag", 1, &WayPointManager::CarCallBack, this);
  road_closure_subscriber = nh.subscribe("/road_closure_pose", 1, &WayPointManager::RoadClosureCallBack, this);
  wp_continue_subscriber = nh.subscribe("/is_runnable", 1, &WayPointManager::WpContinueCallback, this);
  timer = nh.createTimer(ros::Duration(0.1), &WayPointManager::CallBackTimer, this);
  globalplanner_client = nh.serviceClient<dynamic_reconfigure::Reconfigure>("/move_base/YmgGPBGP/set_parameters");
  localplanner_client = nh.serviceClient<dynamic_reconfigure::Reconfigure>("/move_base/YmgLPKerROS/set_parameters");
  laser_fixer_client1 = nh.serviceClient<dynamic_reconfigure::Reconfigure>("/laser_fixer_front_left/set_parameters");
  laser_fixer_client2 = nh.serviceClient<dynamic_reconfigure::Reconfigure>("/laser_fixer_front_right/set_parameters");

  amcl_update_client = nh.serviceClient<dynamic_reconfigure::Reconfigure>("/amcl_adjuster/set_parameters");

  action_client = new Client("move_base", true);
  action_client->cancelAllGoals();

  amcl_client = nh.serviceClient<std_srvs::Empty>("request_nomotion_update");
  use_initial_pose_server =
      nh.advertiseService("use_initial_pose_server", &WayPointManager::UseInitialPoseServer, this);

  signal_acceleration = 0.8;
  normal_acceleration = 0.8;

  use_initial_pose_flag = true;
  wp_array.clear();
  current_wp_iter = wp_array.begin();
}

bool WayPointManager::UseInitialPoseServer(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res)
{
  use_initial_pose_flag = req.data;

  res.success = true;
  if (req.data)
  {
    res.message = "succeeded to change flag to true";
  }
  else
  {
    res.message = "succeeded to change flag to false";
  }
  return true;
}

void WayPointManager::ReadWayPointFile(wp_generation::wp_array msg)
{
  std::vector<WayPoint> targets;
  std::list<WayPoint>::iterator temp_iter = current_wp_iter;
  id = current_wp_iter->id;
  if (current_wp_iter->id == 0)
    id = 1;
  // temp_iter--;

  for (; temp_iter != wp_array.end(); temp_iter++)
  {
    if (temp_iter->wp_type == "target")
    {
      std::cout << "探索対象いた" << std::endl;
      WayPoint tmp_wp;
      tmp_wp.x = temp_iter->x;
      tmp_wp.y = temp_iter->y;
      tmp_wp.yaw = temp_iter->yaw;
      tmp_wp.velocity = temp_iter->velocity;
      tmp_wp.wp_type = temp_iter->wp_type;
      targets.push_back(tmp_wp);
    }
  }

  wp_array.clear();
  for (int i = 0; i < msg.data.size(); ++i)
  {
    WayPoint tmp_wp;
    tmp_wp.x = msg.data[i].x;
    tmp_wp.y = msg.data[i].y;
    tmp_wp.yaw = msg.data[i].yaw;
    tmp_wp.velocity = msg.data[i].vel;
    tmp_wp.wp_type = msg.data[i].label;
    tmp_wp.id = id;
    id += 1;
    // tmp_wp.wp_type = "normal";
    wp_array.push_back(tmp_wp);
  }

  for (int i = 0; i < targets.size(); ++i)
  {
    std::cout << "探索対象いたよね" << std::endl;
    double closest_distance = INFINITY;
    std::list<WayPoint>::iterator closest_iter;
    temp_iter = wp_array.begin();
    for (; temp_iter != wp_array.end(); temp_iter++)
    {
      const double diff_x_m = targets[i].x - temp_iter->x;
      const double diff_y_m = targets[i].y - temp_iter->y;
      const double diff_dis_m = std::hypot(diff_x_m, diff_y_m);
      if (closest_distance > diff_dis_m)
      {
        closest_distance = diff_dis_m;
        closest_iter = temp_iter;
      }
    }
    std::cout << "insert:" << closest_iter->id << std::endl;
    targets[i].id = closest_iter->id * 1.5;
    wp_array.insert(closest_iter, targets[i]);
  }

  current_wp_iter = wp_array.begin();
}

void WayPointManager::ConvertToWayPointMsg()
{
  wp_array_msg.header.stamp = ros::Time::now();
  ;
  wp_array_msg.header.frame_id = "map";
  wp_array_msg.poses.clear();

  for (auto &wp : wp_array)
  {
    geometry_msgs::Pose wp_msg;
    wp_msg.position.x = wp.x;
    wp_msg.position.y = wp.y;
    wp_msg.position.z = 0.0;
    wp_msg.orientation = tf::createQuaternionMsgFromYaw(wp.yaw);
    wp_array_msg.poses.push_back(wp_msg);
  }
  wp_publisher.publish(wp_array_msg);
}

std::list<WayPoint>::iterator WayPointManager::SearchNearWaypointIterator(const WayPoint &target_wp)
{
  const auto most_near_iter = std::min_element(std::begin(wp_array), std::end(wp_array), [&target_wp](WayPoint lhs, WayPoint rhs)
                                               { return std::hypot(target_wp.x - lhs.x, target_wp.y - lhs.y) < std::hypot(target_wp.x - rhs.x, target_wp.y - rhs.y); });
  return most_near_iter;
}

// rvizの2D Pose Estimateを設定された場合、次のwaypointをもっとも近い位置に変更する
// 最も近いwaypointの探索は、initial_poseのx,yのみに依存する
void WayPointManager::CallBackInitialPose(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &initial_pose)
{
  if (use_initial_pose_flag == false)
    return;
  std::cout << "Initial pose" << std::endl;
  forward_wp_flag = true;

  WayPoint tmp_wp;
  tmp_wp.x = initial_pose->pose.pose.position.x;
  tmp_wp.y = initial_pose->pose.pose.position.y;
  tmp_wp.yaw = tf::getYaw(initial_pose->pose.pose.orientation);

  current_wp_iter = SearchNearWaypointIterator(tmp_wp);
  std::cout << std::distance(std::begin(wp_array), current_wp_iter) << std::endl;

  // for ndt_localizer
  // geometry_msgs::PoseWithCovarianceStamped initial_pose3D_msg;
  // initial_pose3D_msg.header = initial_pose->header;
  // initial_pose3D_msg.pose.pose.position.x = initial_pose->pose.pose.position.x;
  // initial_pose3D_msg.pose.pose.position.y = initial_pose->pose.pose.position.y;
  // initial_pose3D_msg.pose.pose.position.z = g_wp_array_iter->z;
  // tf::Quaternion tmp_q;
  // tmp_q.setRPY(g_wp_array_iter->roll, g_wp_array_iter->pitch, tmp_wp.yaw);
  // tf::quaternionTFToMsg(tmp_q, initial_pose3D_msg.pose.pose.orientation);
  // initial_pose3D_publisher.publish(initial_pose3D_msg);
}

void WayPointManager::TargetPoseCallback(const geometry_msgs::PoseStamped::Ptr &msg)
{
  bool target_add_flag = true;
  if (!first_target_flag)
  {
    get_target.clear();
    first_target_flag = true;
  }
  std::cout << "target_size : " << get_target.size() << std::endl;
  for (int i = 0; i < get_target.size(); ++i)
  {
    const double dxm = msg->pose.position.x - get_target[i].pose.position.x;
    const double dym = msg->pose.position.y - get_target[i].pose.position.y;
    const double ddm = std::hypot(dxm, dym);
    if (ddm < 1.0)
    {
      target_add_flag = false;
    }
  }

  if (target_add_flag)
  {
    // find closest waypoint from the robot;
    int n_lookback = 10;
    double closest_dist = INFINITY;
    WayPoint current_pose = GetRobotPose();
    std::list<WayPoint>::iterator closest_iter;
    std::list<WayPoint>::iterator loop_iter = current_wp_iter;
    for (int i = 0; i < n_lookback * 2 && loop_iter != wp_array.begin(); i++)
    {
      loop_iter--;
    }
    for (int i = 0; i < n_lookback * 3 && loop_iter != wp_array.end(); i++, loop_iter++)
    {
      const double diff_x_m = current_pose.x - loop_iter->x;
      const double diff_y_m = current_pose.y - loop_iter->y;
      const double diff_dis_m = std::hypot(diff_x_m, diff_y_m);
      if (closest_dist > diff_dis_m)
      {
        closest_dist = diff_dis_m;
        closest_iter = loop_iter;
      }
    }

    current_wp_iter = closest_iter;

    get_target.push_back(*msg);
    // std::list<WayPoint>::iterator loop_iter = current_wp_iter;
    loop_iter = current_wp_iter;
    std::list<WayPoint>::iterator closest_waypoint_iter;
    double closest_distance = INFINITY;

    ROS_INFO_STREAM("received target_pose" << std::endl);
    action_client->cancelAllGoals();
    std_msgs::Empty emp_msg;
    reset_flag_publisher.publish(emp_msg);

    target_already_found = true;

    // start loop from 10 waypoints back;
    // calculate closest waypoint from target pose
    ROS_INFO_STREAM("start looking nearest pose");
    // int n_lookback  = 10;
    for (int i = 0; i < n_lookback && loop_iter != wp_array.begin(); i++)
    {
      loop_iter--;
    }
    auto next_iter = loop_iter;
    next_iter++;
    for (; next_iter != wp_array.end(); loop_iter++)
    {
      const double diff_x_m = msg->pose.position.x - loop_iter->x;
      const double diff_y_m = msg->pose.position.y - loop_iter->y;
      const double diff_dis_m = std::hypot(diff_x_m, diff_y_m);
      if (closest_distance > diff_dis_m && loop_iter->wp_type != "target" && next_iter->wp_type != "target")
      {
        closest_distance = diff_dis_m;
        closest_waypoint_iter = loop_iter;
      }
      next_iter++;
    }
    // insert waypoint for target
    WayPoint target_wp;
    std::list<WayPoint>::iterator temp_iter = closest_waypoint_iter;
    temp_iter++;
    if (temp_iter == wp_array.end())
      target_wp.id = closest_waypoint_iter->id + 1;
    else
      target_wp.id = (closest_waypoint_iter->id + temp_iter->id) / 2;
    const double initial_radius_m = 1.2;

    int n = 1;
    if (closest_distance > 1.2)
    {
      target_wp.yaw =
          atan2(msg->pose.position.y - closest_waypoint_iter->y, msg->pose.position.x - closest_waypoint_iter->x);
      target_wp.x = msg->pose.position.x - initial_radius_m * n * cos(target_wp.yaw);
      target_wp.y = msg->pose.position.y - initial_radius_m * n * sin(target_wp.yaw);
      target_wp.wp_type = "target";
      wp_array.insert(temp_iter, target_wp);
    }
    else
    {
      target_wp = *closest_waypoint_iter;
      target_wp.wp_type = "target";
      wp_array.insert(temp_iter, target_wp);
    }

    ROS_INFO_STREAM("finished looking nearest pose");

    std_msgs::String message;
    std::stringstream ss;
    ss << "探索対象を見つけました";
    message.data = ss.str();
    talk_publisher.publish(message);

    // ConvertToWayPointMsg();

    ROS_INFO_STREAM("start looking nearest point from robot");

    // find closest waypoint from the robot;
    int tmp1 = std::distance(wp_array.begin(), current_wp_iter);
    int tmp2 = std::distance(wp_array.begin(), closest_waypoint_iter);

    // found target at back
    if (tmp1 > tmp2)
    {
      double robot_closest_dist = INFINITY;
      WayPoint robot_pose = GetRobotPose();
      std::list<WayPoint>::iterator robot_closest_iter;
      loop_iter = current_wp_iter;
      for (int i = 0; i < n_lookback && loop_iter != wp_array.begin(); i++)
      {
        loop_iter--;
      }
      for (int i = 0; i < n_lookback * 2 && loop_iter != wp_array.end(); i++, loop_iter++)
      {
        const double diff_x_m = robot_pose.x - loop_iter->x;
        const double diff_y_m = robot_pose.y - loop_iter->y;
        const double diff_dis_m = std::hypot(diff_x_m, diff_y_m);
        if (robot_closest_dist > diff_dis_m)
        {
          robot_closest_dist = diff_dis_m;
          robot_closest_iter = loop_iter;
        }
      }
      int tmp1 = std::distance(wp_array.begin(), current_wp_iter);
      int tmp2 = std::distance(wp_array.begin(), closest_waypoint_iter);

      current_wp_iter = robot_closest_iter;
    }
    forward_wp_flag = true;

    ROS_INFO_STREAM("finished looking nearest point from robot");
  }
}
