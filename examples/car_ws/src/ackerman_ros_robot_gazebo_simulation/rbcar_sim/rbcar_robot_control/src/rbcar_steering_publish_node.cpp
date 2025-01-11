#include <ros/ros.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <std_msgs/Float64.h>
#include <tf/transform_datatypes.h>
#include <nav_msgs/Odometry.h>
#include <random>
#include <cmath>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <ctime>
#include <ros/package.h>
#include <sys/stat.h>
#include <errno.h>

// Parameters for the car-like kinematics
#define RBCAR_D_WHEELS_M            2.48    // distance from front to back axis, car-like kinematics
#define PI                          3.14159265358979323846


// Utility function to create directory
bool createDirectoryIfNotExists(const std::string& path) {
    struct stat st = {0};
    
    if (stat(path.c_str(), &st) == -1) {
        // Directory does not exist, try to create it
        int mkdir_result = mkdir(path.c_str(), 0755);
        
        if (mkdir_result == -1) {
            // Error handling
            ROS_ERROR("Failed to create directory: %s. Error: %s", 
                      path.c_str(), strerror(errno));
            return false;
        }
        ROS_INFO("Created directory: %s", path.c_str());
    }
    return true;
}

class DelayRepublish
{
    public:
        // Single steering front wheels parameters
        double d1 = 0.0;
        double d = RBCAR_D_WHEELS_M; // divide by 2 for dual Ackermann steering
        
        // Linear line parameters
        double m = 6.916378;
        double b = 0.002233;
        double SD_fit_line = 0.046973;
        double steer_diff = 0.0;

        DelayRepublish()
        {   
            std::string package_path = ros::package::getPath("rbcar_robot_control");
            
            // Create logs directory path
            std::string logs_dir = package_path + "/logs";
            
            // Create logs directory if it doesn't exist
            if (!createDirectoryIfNotExists(logs_dir)) {
                ROS_WARN("Could not create logs directory. Logging may not work.");
            }

            // Get the time delay parameter
            ros::param::param("/rbcar_robot_control/use_delay_estimation_function", _DELAY_WITH_ESTIMATION, true);
            std::cout << "Delay Estimation Activated in Steering? " << _DELAY_WITH_ESTIMATION << std::endl;
            
            // Open log file with timestamp
            std::time_t now = std::time(nullptr);
            std::stringstream filename;
            filename << logs_dir << "/delay_log_steer" 
                     << std::put_time(std::localtime(&now), "%Y%m%d_%H%M%S") 
                     << ".txt";
            
            delay_logger.open(filename.str(), std::ios::out);
            if (!delay_logger.is_open()) {
                ROS_ERROR("Could not open delay log file: %s", filename.str().c_str());
            } else {
                delay_logger << "Timestamp, Ref Steering Angle, Current Steering Angle, Steering Angle Difference, Estimated Delay" << std::endl;
            }

            // Initialize the subscriber and publisher
            publisher_ = nh_.advertise<std_msgs::Float64>("/rbcar/right_steering_joint_controller/command", 1);
            publisher2_ = nh_.advertise<std_msgs::Float64>("/rbcar/left_steering_joint_controller/command", 1);
            subscriber_ = nh_.subscribe("/rbcar_robot_control/command", 1, &DelayRepublish::messageCallbackandPublish, this);
        }

        ~DelayRepublish()
        {
        }

        // Delay calc with linear function
        double delayEstimationLinear(double cur_steer_state, double cur_steer_command)
        {
            // Calculate the delay
            steer_diff = abs(cur_steer_state - cur_steer_command);
            return m * steer_diff + b;
        }

        // Create time delay gaussian noise for the throttle
        double generateGaussianDelay(double mean, double stddev)
        {
            static std::random_device rd;
            static std::mt19937 gen(rd());
            std::normal_distribution<double> dist(mean, stddev);
            return dist(gen);
        }

        void messageCallbackandPublish(const ackermann_msgs::AckermannDriveStamped::ConstPtr& msg)
        {
            // Get the latest ackermann message
            _alfa_ref = msg->drive.steering_angle;

            // Calculate the single steering for the front wheels (right and left)
            std_msgs::Float64 frw_msg;
            std_msgs::Float64 flw_msg;
            
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
            
            // Calculate the delay
            double estimated_delay = 0.0;
            if (_DELAY_WITH_ESTIMATION){
                // This simulator assumes instantaneous steering response 
                // The angle difference between the previous and current steering command is used to estimate the delay
                // Because there is no measurement of the actual steering angle in the simulation
                // Therefore, the delay is estimated based on the difference between the previous and current steering commands (by assuming the steering response is instantaneous)
                double mean_delay = delayEstimationLinear(_prev_alfa_ref, _alfa_ref);
                _TIME_DELAY = std::abs(generateGaussianDelay(mean_delay, SD_fit_line));
                // Clip the delay to a minimum of 0.0 to avoid negative delays
                _TIME_DELAY = std::max(0.0, _TIME_DELAY);
                estimated_delay = _TIME_DELAY;
            }
            else {
                ros::param::get("/rbcar_robot_control/time_delay_steering", _TIME_DELAY);
                estimated_delay = _TIME_DELAY;
            }

            // Log delay information
            if (delay_logger.is_open()) {
                std::time_t now = std::time(nullptr);
                delay_logger << std::put_time(std::localtime(&now), "%Y-%m-%d %H:%M:%S") 
                             << "," << _alfa_ref 
                             << "," << _prev_alfa_ref 
                             << "," << steer_diff
                             << "," << estimated_delay << std::endl;
            }

            // Publish the new steering angle
            frw_msg.data = _alfa_ref_right;
            flw_msg.data = _alfa_ref_left;
            
            // Make sure the delay is not negative
            if (_TIME_DELAY >= 0.0) {
                // std::cout << "Steering difference: " << steer_diff << std::endl;
                // std::cout << "Steering delay: " << _TIME_DELAY << std::endl;
                ros::Duration(_TIME_DELAY).sleep();
            }
            
            publisher_.publish(frw_msg);
            publisher2_.publish(flw_msg);

            // Update the previous steering command (as the curent state of the steering)
            _prev_alfa_ref = _alfa_ref;
        }

        // void publish()
        // {
        //     ros::param::get("/rbcar_robot_control/time_delay_steering", _TIME_DELAY);
        //     // Single steering 
        //     double d1 =0.0;
        //     double d = RBCAR_D_WHEELS_M; // divide by 2 for dual Ackermann steering
        //     if (_alfa_ref!=0.0) {  // div/0
        //         d1 =  d / tan (_alfa_ref);
        //         _alfa_ref_left = atan2( d, d1 - 0.105);
        //         _alfa_ref_right = atan2( d, d1 + 0.105);
        //         if (_alfa_ref<0.0) {
        //             _alfa_ref_left = _alfa_ref_left - PI;
        //             _alfa_ref_right = _alfa_ref_right - PI;
        //             }     
        //         }
        //     else {
        //         _alfa_ref_left = 0.0;
        //         _alfa_ref_right = 0.0;
        //         }

        //     // Publish the new steering angle
        //     std_msgs::Float64 frw_msg;
        //     std_msgs::Float64 flw_msg;
        //     frw_msg.data = _alfa_ref_right;
        //     flw_msg.data = _alfa_ref_left;
        //     // std::cout<<  "before delay" << std::endl;
        //     if (_TIME_DELAY > 0.0) {
        //         ros::Duration(_TIME_DELAY).sleep();
        //     }
        //     publisher_.publish(frw_msg);
        //     publisher2_.publish(flw_msg);
        // }

    private:
        ros::NodeHandle nh_;
        ros::Subscriber subscriber_;
        ros::Publisher publisher_;
        ros::Publisher publisher2_;
        double _alfa_ref_left = 0.0;
        double _alfa_ref_right = 0.0;
        double _alfa_ref = 0.0;
        double _prev_alfa_ref = 0.0;
        double _TIME_DELAY = 0.0;
        // Flag for enabling the delay estimation instead of the fixed delay
        bool _DELAY_WITH_ESTIMATION = true;
        // Create logger for the delay
        std::ofstream delay_logger;
};


int main(int argc, char** argv)
{
    ros::init(argc, argv, "rbcar_steering_publish_node");
    DelayRepublish DelayObj;
    ros::Rate rate(50);

    while (ros::ok())
    {
        ros::spinOnce();
        rate.sleep();
    }

}   