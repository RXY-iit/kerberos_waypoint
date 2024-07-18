#include <ros/ros.h>
#include <tf/tf.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <cmath>
#include <iostream>
#include <string> 
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>

class TargetPublisher{
public:
    TargetPublisher(){
        ros::NodeHandle private_node_handle("~");

        sub = nh.subscribe("clicked_point", 1, &TargetPublisher::Callback, this);
        pub = nh.advertise<geometry_msgs::PoseStamped>("target_pose", 10);
    }

    void Callback(const geometry_msgs::PointStamped::ConstPtr& point){
        geometry_msgs::PoseStamped msg;
        msg.header.stamp = ros::Time::now();
        msg.header.frame_id = "map";
        msg.pose.position.x = point->point.x;
        msg.pose.position.y = point->point.y;
        msg.pose.position.z = 0;
        msg.pose.orientation = tf::createQuaternionMsgFromYaw(0);
        pub.publish(msg);
    }

private:
    ros::NodeHandle nh;
    ros::Subscriber sub;
    ros::Publisher pub;
};

int main (int argc, char *argv[]){
    ros::init(argc, argv, "target_publisher");
    
    TargetPublisher target_pub;

    ros::Rate r(100);
    while(ros::ok()){
        ros::spinOnce();
        r.sleep();
    }
    
    return 0;
}