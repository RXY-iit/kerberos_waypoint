#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <std_msgs/String.h>
#include <cmath>
#include <iostream>
#include <fstream>
#include <sstream>
#include <cstdlib>
#include <thread>
#include <string>
#include <list>
#include <actionlib/client/simple_action_client.h>
#include "../include/kerberos_waypoint_publisher_cpmode.hpp"
#include <wp_generation/wp_array.h>
#include <wp_generation/wp_data.h>

int main(int argc, char **argv) {

    // 固定小数点記法に設定
    std::cout << std::fixed;
    std::cerr << std::fixed;

    ros::init(argc, argv, "kerberos_waypoint_publisher");
    ros::NodeHandle public_node_handle;
    ros::NodeHandle private_node_handle("~");
 //   ros::Subscriber target_pose_subscriber = public_node_handle.subscribe("deep_result", 10, set_target_waypoint);
    WayPointManager manager(public_node_handle, private_node_handle);

    std::thread keyboad_thread(&WayPointManager::ForwardWaypointByKeyboardInterrupt, &manager);
    keyboad_thread.detach();

    //manager.ReadWayPointFile(argv[1]);
    //manager.ConvertToWayPointMsg();
    //std::string a;
    
    ros::Time start_time;
    ros::Duration(0.2).sleep();
    ros::AsyncSpinner spinner(4);
    spinner.start();
    ros::waitForShutdown();
    // ros::spin(); 
    // while (ros::ok() && manager.GetState() != "done") {
    //     if(ros::Time::now() - start_time > ros::Duration(1.0)) manager.ConvertToWayPointMsg();
    //     ros::Duration(0.1).sleep();
    //     ros::spinOnce();
    //     manager.StateTransition();
    //     WayPoint robot_pose = manager.GetRobotPose();
    //     //std::cout << robot_pose.x << " " << robot_pose.y << " " << robot_pose.yaw << std::endl;
    // }

    // // ゴールした後で停止するためにコマンド発行
    // geometry_msgs::Twist cmd_vel;
    // cmd_vel.linear.x = 0.0;
    // cmd_vel.linear.y = 0.0;
    // cmd_vel.linear.z = 0.0;
    // cmd_vel.angular.x = 0.0;
    // cmd_vel.angular.y = 0.0;
    // cmd_vel.angular.z = 0.0;
    // manager.cmd_publisher.publish(cmd_vel);
    // ros::spinOnce();

    return 0;
}
