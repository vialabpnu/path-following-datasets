#include <ros/ros.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <std_msgs/Float64.h>


// Parameters for the car-like kinematics
#define RBCAR_D_WHEELS_M            2.48    // distance from front to back axis, car-like kinematics
#define PI                          3.14159265358979323846


class DelayRepublish
{
public:
    DelayRepublish()
    {
        // Initialize the subscriber and publisher
        publisher_ = nh_.advertise<std_msgs::Float64>("/rbcar/right_steering_joint_controller/command", 1);
        publisher2_ = nh_.advertise<std_msgs::Float64>("/rbcar/left_steering_joint_controller/command", 1);
        subscriber_ = nh_.subscribe("/rbcar_robot_control/command", 1, &DelayRepublish::messageCallbackandPublish, this);
    }

    ~DelayRepublish()
    {
    }

    void messageCallback(const ackermann_msgs::AckermannDriveStamped::ConstPtr& msg)
    {      
        // ROS_INFO("TIME DELAY STEER: %f", TIME_DELAY);
        // std::cout << TIME_DELAY << std::endl;
        // Get the latest ackermann message
        _alfa_ref = msg->drive.steering_angle;
    }

    void messageCallbackandPublish(const ackermann_msgs::AckermannDriveStamped::ConstPtr& msg)
    {
        // Get the latest ackermann message
        _alfa_ref = msg->drive.steering_angle;
        ros::param::get("/rbcar_robot_control/time_delay_steering", _TIME_DELAY);
        // Single steering
        double d1 =0.0;
        double d = RBCAR_D_WHEELS_M; // divide by 2 for dual Ackermann steering
        if (_alfa_ref!=0.0) {  // div/0
            d1 =  d / tan (_alfa_ref);
            _alfa_ref_left = atan2( d, d1 - 0.105);
            _alfa_ref_right = atan2( d, d1 + 0.105);
            if (_alfa_ref<0.0) {
                _alfa_ref_left = _alfa_ref_left - PI;
                _alfa_ref_right = _alfa_ref_right - PI;
                }     
            }
        else {
            _alfa_ref_left = 0.0;
            _alfa_ref_right = 0.0;
            }
        // Publish the new steering angle
        std_msgs::Float64 frw_msg;
        std_msgs::Float64 flw_msg;
        frw_msg.data = _alfa_ref_right;
        flw_msg.data = _alfa_ref_left;
        // std::cout<<  "before delay" << std::endl;
        if (_TIME_DELAY > 0.0) {
            ros::Duration(_TIME_DELAY).sleep();
        }
        publisher_.publish(frw_msg);
        publisher2_.publish(flw_msg);
    }

    void publish()
    {
        ros::param::get("/rbcar_robot_control/time_delay_steering", _TIME_DELAY);
        // Single steering 
        double d1 =0.0;
        double d = RBCAR_D_WHEELS_M; // divide by 2 for dual Ackermann steering
        if (_alfa_ref!=0.0) {  // div/0
            d1 =  d / tan (_alfa_ref);
            _alfa_ref_left = atan2( d, d1 - 0.105);
            _alfa_ref_right = atan2( d, d1 + 0.105);
            if (_alfa_ref<0.0) {
                _alfa_ref_left = _alfa_ref_left - PI;
                _alfa_ref_right = _alfa_ref_right - PI;
                }     
            }
        else {
            _alfa_ref_left = 0.0;
            _alfa_ref_right = 0.0;
            }

        // Publish the new steering angle
        std_msgs::Float64 frw_msg;
        std_msgs::Float64 flw_msg;
        frw_msg.data = _alfa_ref_right;
        flw_msg.data = _alfa_ref_left;
        // std::cout<<  "before delay" << std::endl;
        if (_TIME_DELAY > 0.0) {
            ros::Duration(_TIME_DELAY).sleep();
        }
        publisher_.publish(frw_msg);
        publisher2_.publish(flw_msg);
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber subscriber_;
    ros::Publisher publisher_;
    ros::Publisher publisher2_;
    double _alfa_ref_left = 0.0;
    double _alfa_ref_right = 0.0;
    double _alfa_ref = 0.0;
    double _TIME_DELAY = 0.0;
};


int main(int argc, char** argv)
{
    ros::init(argc, argv, "delay_republish");
    DelayRepublish DelayObj;
    ros::Rate rate(50);

    while (ros::ok())
    {
        ros::spinOnce();
        // DelayObj.publish();
        rate.sleep();
    }

}   