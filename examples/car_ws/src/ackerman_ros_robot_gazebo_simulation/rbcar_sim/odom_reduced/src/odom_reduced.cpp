#include "ros/ros.h"
#include "nav_msgs/Odometry.h"


ros::Publisher pub = n.advertise<nav_msgs::Odometry>("odom_reduced", 1);


void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    nav_msgs::Odometry odom_slower;

    odom_slower.header = msg->header;
    odom_slower.pose.pose = msg->pose.pose;
    odom_slower.twist.twist = msg->twist.twist;

    odom_slower.header.stamp = ros::Time::now();
    pub.publish(odom_slower);
}


// Subscribe and publish odom with the lower rate
int main(int argc, char **argv)
{
  ros::init(argc, argv, "odom_reduced");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("/odoms", 1, odomCallback);
  ros::Rate loop_rate(10);

  while (ros::ok())
  {
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}