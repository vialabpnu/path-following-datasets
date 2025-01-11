#include <ros/ros.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <std_msgs/Float64.h>


#define RBCAR_D_WHEELS_M      2.48    // distance from front to back axis, car-like kinematics
#define PI 3.1415926535

ros::Publisher publisher = nh.advertise<std_msgs::Float64>("/rbcar/right_steering_joint_controller/command", 1);
ros::Publisher publisher2 = nh.advertise<std_msgs::Float64>("/rbcar/left_steering_joint_controller/command", 1);

void messageCallback(const ackermann_msgs::AckermannDriveStamped::ConstPtr& msg)
{      
    // Get the latest ackermann message
    double alfa_ref_ = msg->drive.steering_angle;

    // Single steering 
    double d1 =0.0;
    double d = RBCAR_D_WHEELS_M; // divide by 2 for dual Ackermann steering
    double alfa_ref_left = 0.0;
    double alfa_ref_right = 0.0;
    if (alfa_ref_!=0.0) {  // div/0
        d1 =  d / tan (alfa_ref_);
        alfa_ref_left = atan2( d, d1 - 0.105);
        alfa_ref_right = atan2( d, d1 + 0.105);
        if (alfa_ref_<0.0) {
            alfa_ref_left = alfa_ref_left - PI;
            alfa_ref_right = alfa_ref_right - PI;
            }     
        }
    else {
        alfa_ref_left = 0.0;
        alfa_ref_right = 0.0;
        }

    // Publish the new steering angle
    std_msgs::Float64 frw_msg;
    std_msgs::Float64 flw_msg;
    frw_msg.data = alfa_ref_right;
    flw_msg.data = alfa_ref_left;
    publisher.publish(frw_msg);
    publisher2.publish(flw_msg);
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "delay_republish");
    ros::NodeHandle nh;

    ros::Subscriber subscriber = nh.subscribe("/rbcar_robot_control/command", 1, &messageCallback);

    // Set the loop rate
    ros::Rate rate(50); // 50 hz

    while (ros::ok())
    {
        ros::spinOnce();
        rate.sleep();
    }

}   