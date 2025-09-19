#!/usr/bin/env python2.7
import rospy
import numpy as np
import argparse
import sys
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler

class NoisyOdomNode:
    def __init__(self, args):
        # Parameters from ROS or args
        self.input_topic = rospy.get_param("~input_topic", args.input_topic)
        self.output_topic = rospy.get_param("~output_topic", args.output_topic)

        # Enable/disable noise
        self.enable_x = args.enable_x
        self.enable_y = args.enable_y
        self.enable_heading = args.enable_heading
        self.enable_speed = args.enable_speed

        # Standard deviations
        self.x_stddev = args.x_stddev
        self.y_stddev = args.y_stddev
        self.heading_stddev = args.heading_stddev
        self.speed_stddev = args.speed_stddev

        self.sub = rospy.Subscriber(self.input_topic, Odometry, self.odom_callback, queue_size=1)
        self.pub = rospy.Publisher(self.output_topic, Odometry, queue_size=1)

    def odom_callback(self, msg):
        noisy_msg = Odometry()
        noisy_msg.header = msg.header
        noisy_msg.child_frame_id = msg.child_frame_id

        # Add noise to pose
        noisy_msg.pose.pose.position.x = msg.pose.pose.position.x
        noisy_msg.pose.pose.position.y = msg.pose.pose.position.y
        noisy_msg.pose.pose.position.z = msg.pose.pose.position.z

        if self.enable_x:
            noisy_msg.pose.pose.position.x += np.random.normal(0, self.x_stddev)
        if self.enable_y:
            noisy_msg.pose.pose.position.y += np.random.normal(0, self.y_stddev)

        # Orientation (heading noise)
        noisy_msg.pose.pose.orientation = msg.pose.pose.orientation
        if self.enable_heading:
            q = msg.pose.pose.orientation
            euler = euler_from_quaternion([q.x, q.y, q.z, q.w])
            noisy_yaw = euler[2] + np.random.normal(0, self.heading_stddev)
            noisy_quat = quaternion_from_euler(euler[0], euler[1], noisy_yaw)
            noisy_msg.pose.pose.orientation.x = noisy_quat[0]
            noisy_msg.pose.pose.orientation.y = noisy_quat[1]
            noisy_msg.pose.pose.orientation.z = noisy_quat[2]
            noisy_msg.pose.pose.orientation.w = noisy_quat[3]

        # Add noise to twist (speed)
        noisy_msg.twist.twist.linear.x = msg.twist.twist.linear.x
        noisy_msg.twist.twist.linear.y = msg.twist.twist.linear.y
        noisy_msg.twist.twist.linear.z = msg.twist.twist.linear.z
        noisy_msg.twist.twist.angular = msg.twist.twist.angular

        if self.enable_speed:
            noisy_msg.twist.twist.linear.x += np.random.normal(0, self.speed_stddev)
            noisy_msg.twist.twist.linear.y += np.random.normal(0, self.speed_stddev)

        # Covariances (optional: you can set these to reflect the noise)
        noisy_msg.pose.covariance = msg.pose.covariance
        noisy_msg.twist.covariance = msg.twist.covariance

        self.pub.publish(noisy_msg)

def parse_args():
    parser = argparse.ArgumentParser(description="Add configurable Gaussian noise to odometry.")
    parser.add_argument('--input_topic', type=str, default='/INS/odom_raw', help='Input odometry topic')
    parser.add_argument('--output_topic', type=str, default='/INS/odom', help='Output odometry topic')
    parser.add_argument('--enable_x', action='store_true', help='Enable Gaussian noise for x position')
    parser.add_argument('--enable_y', action='store_true', help='Enable Gaussian noise for y position')
    parser.add_argument('--enable_heading', action='store_true', help='Enable Gaussian noise for heading (yaw)')
    parser.add_argument('--enable_speed', action='store_true', help='Enable Gaussian noise for linear speed (x, y)')
    parser.add_argument('--x_stddev', type=float, default=0.01, help='Stddev for x position noise (meters)')
    parser.add_argument('--y_stddev', type=float, default=0.01, help='Stddev for y position noise (meters)')
    parser.add_argument('--heading_stddev', type=float, default=0.01, help='Stddev for heading noise (radians)')
    parser.add_argument('--speed_stddev', type=float, default=0.01, help='Stddev for speed noise (m/s)')
    return parser.parse_known_args()[0]

if __name__ == "__main__":
    args = parse_args()
    rospy.init_node("noisy_odom_generator", anonymous=True)
    NoisyOdomNode(args)
    rospy.spin()
