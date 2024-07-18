#ifndef __KERBEROS_WAYPOINT_PUBLISHER_CPMODE_HPP__
#define __KERBEROS_WAYPOINT_PUBLISHER_CPMODE_HPP__

#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int32.h>
#include <dynamic_reconfigure/Reconfigure.h>

#include <cmath>
#include <iostream>
#include <fstream>
#include <sstream>
#include <cstdlib>
#include <thread>
#include <string>
#include <list>
#include <actionlib/client/simple_action_client.h>
#include <mutex>
#include <std_srvs/Empty.h>
#include <std_srvs/SetBool.h>
#include <wp_generation/wp_array.h>
#include <wp_generation/wp_data.h>
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> Client;

typedef struct
{
    double x;
    double y;
    double yaw;
    double id;
    double velocity;
    std::string wp_type;
} WayPoint;

typedef struct
{
    geometry_msgs::PoseStamped target;
    std::list<WayPoint>::iterator closest_waypoint_iter;
} TargetPose;

class WayPointManager
{
public:
    WayPointManager(ros::NodeHandle nh, ros::NodeHandle pnh);
    void StateTransition();
    void ReadWayPointFile(wp_generation::wp_array msg);
    void ConvertToWayPointMsg();
    void TargetPoseCallback(const geometry_msgs::PoseStamped::Ptr &msg);
    void ForwardWaypointByKeyboardInterrupt();
    std::string GetState();
    WayPoint GetRobotPose();
    void Hokuyo3dStatusCallback(const std_msgs::Bool::ConstPtr msg);

    void CallBackInitialPose(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &initial_pose);
    void CallBackTimer(const ros::TimerEvent &);
    void SignalGoCallback(const std_msgs::String::ConstPtr &go_flag);
    void BlockCallback(const std_msgs::Bool::ConstPtr &msg);
    void WpArrayCallback(const wp_generation::wp_array::ConstPtr &msg);
    void UseBGPCallback(const std_msgs::Bool::ConstPtr &msg);
    void CallBackCmdvel(const geometry_msgs::Twist::ConstPtr &msg);
    void LineCallBack(const std_msgs::Bool::ConstPtr &msg);
    void CarCallBack(const std_msgs::Bool::ConstPtr &msg);
    void RoadClosureCallBack(const geometry_msgs::PoseStamped::ConstPtr &msg);
    void WpContinueCallback(const std_msgs::Bool::ConstPtr &msg);
    std::list<WayPoint>::iterator SearchNearWaypointIterator(const WayPoint &target_wp);
    ros::Timer timer;
    ros::Publisher cmd_publisher;
    bool start_flag;
    // ros::Service
    bool forward_wp_flag;
    bool UseInitialPoseServer(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res);

private:
    double robot_max_velocity;
    std::list<WayPoint> wp_array;
    std::list<WayPoint>::iterator current_wp_iter;
    geometry_msgs::PoseArray wp_array_msg;
    std_msgs::Bool is_grass_road;
    std_msgs::String wp_mode;
    double normal_acceleration;
    double signal_acceleration;

    bool target_already_found;
    std::string state;
    tf::TransformListener tf_listener;
    void PublishWayPointMode();
    bool MoveBackward();
    void StateNormal();
    void StateBack();
    void StateFindTarget();
    void StateWait();
    void StateTurning();
    void StateTurningT2();
    void StateLine();
    void StateLineBefore();
    void StateCrosswalk();
    void StateTurningPoint();
    void StateSignal();
    void StateEnd();
    void AmclUpdate(int update_time);
    ros::Publisher grass_road_publisher;
    ros::Publisher wp_mode_publisher;
    ros::Publisher nav_goal_publisher;
    ros::Publisher wp_publisher;
    ros::Publisher talk_publisher;
    ros::Publisher reset_flag_publisher;
    ros::Publisher force_ymg_publisher;
    ros::Publisher signal_start_flag_publisher;
    ros::Publisher letter_start_flag_publisher;//boxfinder letter  recog add
    ros::Publisher target_start_flag_publisher;
    ros::Publisher hnd_flag_publisher;
    ros::Subscriber target_pose_subscriber;
    ros::Subscriber initial_pose_subscriber;
    ros::Subscriber signal_go_flag_subscriber;
    ros::Subscriber letter_result_subscriber;//boxfinder letter  recog add
    ros::Subscriber luggage_result_subscriber;//boxfinder letter  recog add
    ros::Subscriber use_bgp_subscriber;
    ros::Subscriber hokuyo3d_status_subscriber;
    ros::Subscriber cmd_subscriber;
    ros::Subscriber line_flag_subscriber;
    ros::Subscriber car_flag_subscriber;
    ros::Subscriber block_subscriber;
    ros::Subscriber wp_array_subscriber;
    ros::Subscriber road_closure_subscriber;
    ros::Subscriber wp_continue_subscriber;
    ros::ServiceClient amcl_client;
    ros::ServiceServer use_initial_pose_server;
    bool use_initial_pose_flag;
    ros::ServiceClient localplanner_client;
    ros::ServiceClient globalplanner_client;
    ros::ServiceClient laser_fixer_client1;
    ros::ServiceClient laser_fixer_client2;
    ros::ServiceClient amcl_update_client;

    bool signal_flag;
    bool letter_flag;//box_finder add
    int signal_count = 0;

    Client *action_client;
    // parameters
    double goal_tolerance_m;
    double goal_tolerance_rad;
    double skip_time_sec;

    geometry_msgs::Twist cmd_vel_info;
    std::vector<geometry_msgs::PoseStamped> get_target;
    bool first_target_flag = false;
    std_msgs::Bool line_flag;
    bool stop_return = false;
    std_msgs::Bool car_flag;
    bool signal_ok = false;
    bool letter_ok = false;//box_finder add
    bool luggage_get_ok = false;//box_finder add
    std::string box_letter;//box_finder add
    bool block_flag = true;
    bool closure_flag = false;
    wp_generation::wp_array tjm_wp_array;
    int id = 1;
    bool targettry_cancel = false;
    std_msgs::Bool hokuyo3d_status;

    float SLOW_SPEED = 0.3;
    float NORMAL_SPEED = 1.1;
};

#endif
