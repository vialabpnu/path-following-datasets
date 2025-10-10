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
# define RBCAR_WHEEL_DIAMETER 0.470 // diameter of the wheels in meters
# define PI 3.14159265358979323846


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

class VelocityPublishNode
{
    public:
        // RBCAR Parameters
        double wheel_diameter = RBCAR_WHEEL_DIAMETER;

        // Linear line parameters
        double m = 1.5407;
        double b = 0.2053;
        double SD_fit_line = 0.134433;
        double vel_diff = 0.0;

        // Constructor and Destructor
        VelocityPublishNode()
        {
            // Get the package path
            std::string package_path = ros::package::getPath("rbcar_robot_control");
            
            // Create logs directory path
            std::string logs_dir = package_path + "/logs";
            
            // Create logs directory if it doesn't exist
            if (!createDirectoryIfNotExists(logs_dir)) {
                ROS_WARN("Could not create logs directory. Logging may not work.");
            }

            // Get the time delay parameter
            ros::param::param("/rbcar_robot_control/use_delay_estimation_function", _DELAY_WITH_ESTIMATION, true);
            std::cout << "Delay Estimation Activated? " << _DELAY_WITH_ESTIMATION << std::endl;
            
            // Open log file with timestamp
            std::time_t now = std::time(nullptr);
            std::stringstream filename;
            filename << logs_dir << "/delay_log_" 
                     << std::put_time(std::localtime(&now), "%Y%m%d_%H%M%S") 
                     << ".txt";
            
            delay_logger.open(filename.str(), std::ios::out);
            if (!delay_logger.is_open()) {
                ROS_ERROR("Could not open delay log file: %s", filename.str().c_str());
            } else {
                delay_logger << "Timestamp,Velocity Reference,Velocity State,Velocity Difference,Estimated Delay\n";
            }
            
            // Init Subscriber and Publisher
            ros_command_sub = nh.subscribe("/rbcar_robot_control/command", 1, &VelocityPublishNode::messageCallback, this);
            odom_sub = nh.subscribe("/INS/odom", 1, &VelocityPublishNode::odomCallback, this);
            ref_vel_frw = nh.advertise<std_msgs::Float64>("rbcar/right_front_axle_controller/command", 1, true);
            ref_vel_flw = nh.advertise<std_msgs::Float64>("rbcar/left_front_axle_controller/command", 1, true);
            ref_vel_blw = nh.advertise<std_msgs::Float64>("rbcar/left_rear_axle_controller/command", 1, true);
            ref_vel_brw = nh.advertise<std_msgs::Float64>("rbcar/right_rear_axle_controller/command", 1, true);
        }
        
        ~VelocityPublishNode()
        {
            // Close log file in destructor
            if (delay_logger.is_open()) {
                delay_logger.close();
            }
        }

        void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
        {
            // Get the latest ackermann message and inverse transform the velocity
            double vx = msg->twist.twist.linear.x;
            double vy = msg->twist.twist.linear.y;
            double yaw_from_quat = tf::getYaw(msg->pose.pose.orientation);
            // Inverse transform the velocity
            _v_state = vx * cos(yaw_from_quat) + vy * sin(yaw_from_quat);
        }

        void messageCallback(const ackermann_msgs::AckermannDriveStamped::ConstPtr& msg)
        {
            // Get the latest ackermann message
            _v_ref = msg->drive.speed;
        }

        // Delay calc with linear function
        double delayEstimationLinear(double cur_vel_state, double cur_vel_command)
        {
            // Calculate the delay
            vel_diff = abs(cur_vel_state - cur_vel_command);
            return m * vel_diff + b;
        }

        // Create time delay gaussian noise for the throttle
        double generateGaussianDelay(double mean, double stddev)
        {
            static std::random_device rd;
            static std::mt19937 gen(rd());
            std::normal_distribution<double> dist(mean, stddev);
            return dist(gen);
        }

        void publish_speed()
        {
            // Create message for the axle speed
            std_msgs::Float64 frw_ref_pos_msg;
            std_msgs::Float64 flw_ref_pos_msg;
            std_msgs::Float64 brw_ref_pos_msg;
            std_msgs::Float64 blw_ref_pos_msg;
            
            // Linear speed ref publish (could be improved by setting correct speed to each wheel according to turning state
            // w = v_mps / (PI * D);   w_rad = w * 2.0 * PI
            double ref_speed_joint = 2.0 * _v_ref / wheel_diameter;
            frw_ref_pos_msg.data = -ref_speed_joint;
            flw_ref_pos_msg.data = -ref_speed_joint;
            brw_ref_pos_msg.data = -ref_speed_joint;
            blw_ref_pos_msg.data = -ref_speed_joint;
            
            // Calculate delay
            double estimated_delay = 0.0;
            if (_DELAY_WITH_ESTIMATION){
                double mean_delay = delayEstimationLinear(_v_state, _v_ref);
                _TIME_DELAY = std::abs(generateGaussianDelay(mean_delay, SD_fit_line));
                // Clip the delay to a minimum of 0.0 to avoid negative delays
                _TIME_DELAY = std::max(0.0, _TIME_DELAY);
                estimated_delay = _TIME_DELAY;
            }
            else {
                ros::param::get("/rbcar_robot_control/time_delay_throttle", _TIME_DELAY);
                estimated_delay = _TIME_DELAY;
            }
            
            // Log delay information
            if (delay_logger.is_open()) {
                std::time_t now = std::time(nullptr);
                delay_logger << std::put_time(std::localtime(&now), "%Y-%m-%d %H:%M:%S") 
                             << "," << _v_ref 
                             << "," << _v_state 
                             << "," << vel_diff 
                             << "," << estimated_delay << "\n";
                delay_logger.flush(); // Ensure immediate writing
            }
            
            // Impose delay (make sure the delay is not negative)
            if (_TIME_DELAY >= 0.0) {
                // std::cout << "DELAY APPLIED" << std::endl;
                ros::Duration(_TIME_DELAY).sleep();
            }
            
            // Publish msgs traction and direction
            ref_vel_frw.publish(frw_ref_pos_msg);
            ref_vel_flw.publish(flw_ref_pos_msg);
            ref_vel_brw.publish(brw_ref_pos_msg);
            ref_vel_blw.publish(blw_ref_pos_msg);         
        }

    private:
        ros::NodeHandle nh;
        ros::Publisher ref_vel_frw;
        ros::Publisher ref_vel_flw;
        ros::Publisher ref_vel_brw;
        ros::Publisher ref_vel_blw;
        ros::Subscriber ros_command_sub;
        ros::Subscriber odom_sub;
        double _v_ref = 0.0;
        double _v_state = 0.0;
        double _TIME_DELAY = 0.0;
        // Flag for enabling the delay estimation instead of the fixed delay
        bool _DELAY_WITH_ESTIMATION = true;
        // Create logger for the delay
        std::ofstream delay_logger;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "rbcar_velocity_publish_node");
    VelocityPublishNode velocityPublishNode;
    ros::Rate rate(50);

    while (ros::ok())
    {
        ros::spinOnce();
        velocityPublishNode.publish_speed();
        rate.sleep();
    }

    return 0;
}