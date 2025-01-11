#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TransformStamped.h>


void odomCallbackTimeStamped(const nav_msgs::Odometry::ConstPtr& msg){
    current_time = ros::Time::now();
    static tf::TransformBroadcaster odom_broadcaster;
    geometry_msgs::TransformStamped odom_trans;

    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = "world";
    odom_trans.child_frame_id = "odom";

    odom_trans.transform.translation.x = msg->pose.pose.position.x;
    odom_trans.transform.translation.y = msg->pose.pose.position.y;
    odom_trans.transform.translation.z = 0.0;

    odom_trans.transform.rotation.x = msg->pose.pose.orientation.x;
    odom_trans.transform.rotation.y = msg->pose.pose.orientation.y;
    odom_trans.transform.rotation.z = msg->pose.pose.orientation.z;
    odom_trans.transform.rotation.w = msg->pose.pose.orientation.w;

    odom_broadcaster.sendTransform(odom_trans);

}

int main(int argc, char** argv){
    ros::init(argc, argv, "odom_to_transform");
    ros::Time current_time, last_time;

    ros:NodeHandle n;
    tf::TransformBroadcaster odom_broadcaster;

    ros::Subscriber odom_sub = n.subscribe("/odoms", 1, odomCallbackTimeStamped);

    ros::spin();
}