#!/usr/bin/env python2.7
import math
import csv
import rospy    
import numpy as np

from nav_msgs.msg import Path, Odometry
from motion_planner.msg import Local_path
from geometry_msgs.msg import PoseStamped

class VisualizeLocalPath(object):
    def __init__(self):
        self._local_path = None
        self._global_path = None
        self._local_path_viz_pub = rospy.Publisher('/motion_planner_local_path', Path, queue_size=1)
        self._global_path_viz_pub = rospy.Publisher('/global_path', Path, queue_size=1)
        self._local_path_sub = rospy.Subscriber('/path_motion_planner', Local_path, self.visualization_cb, queue_size=1)
        self._global_path_sub = rospy.Subscriber('/move_base/GlobalPlanner/plan', Path, self.global_path_cb, queue_size=1)
        self._odom_sub = rospy.Subscriber('/INS/odom', Odometry, self.odom_cb, queue_size=1)
        self._use_motion_planner = rospy.get_param('/use_motion_planner', False)
        self._load_path_from_file = rospy.get_param('/ref_path_load_from_file', False)
        self._local_path_msg = None
        self._process_path = True
        self.path_idx = 0
        self.ref_x = None
        self.ref_y = None
        self.ref_yaw = None
        
    def odom_cb(self, msg):
        self.cur_pose = [msg.pose.pose.position.x, msg.pose.pose.position.y]
        
    def global_path_cb(self, msg):
        if self._load_path_from_file:
            if self._process_path:
                self.file_path = '/home/vialab/car_ws/src/ackerman_ros_robot_gazebo_simulation/custom_controllers/ackermann-drive-teleop/script_for_mpc_prod/data/splined_path.csv'
                trajectory = np.loadtxt(self.file_path, delimiter=',', skiprows=1)[:,:]
                self.ref_x = trajectory[:, 0].tolist()
                self.ref_y = trajectory[:, 1].tolist()
                self.ref_yaw = trajectory[:, 2].tolist()
                self._process_path = False
            # Publish the path
            self._global_path = Path()
            self._global_path.header.stamp = rospy.Time.now()
            self._global_path.header.frame_id = "map"
            for i in range(len(self.ref_x)):
                pose = PoseStamped()
                pose.header.stamp = rospy.Time.now()
                pose.header.frame_id = "map"
                pose.pose.position.x = self.ref_x[i]
                pose.pose.position.y = self.ref_y[i]
                pose.pose.position.z = 0.0
                # Quaternion from yaw
                quat = self.get_quaternion_from_euler(0, 0, self.ref_yaw[i])
                pose.pose.orientation.x = quat[0]
                pose.pose.orientation.y = quat[1]
                pose.pose.orientation.z = quat[2]
                pose.pose.orientation.w = quat[3]

                self._global_path.poses.append(pose)
            self._global_path_viz_pub.publish(self._global_path)
            pass
        
    def visualization_cb(self, local_path):
        self._local_path = local_path
        path_len = len(local_path.ref_state[0].data)
        # Convert math.array to Path message
        self._local_path_msg = Path()
        self._local_path_msg.header.stamp = rospy.Time.now()
        self._local_path_msg.header.frame_id = "map"
        for i in range(path_len):
            pose = PoseStamped()
            pose.header.stamp = rospy.Time.now()
            pose.header.frame_id = "map"
            pose.pose.position.x = local_path.ref_state[0].data[i]
            pose.pose.position.y = local_path.ref_state[1].data[i]
            pose.pose.position.z = 0.0
            # Quaternion from yaw
            quat = self.get_quaternion_from_euler(0, 0, local_path.ref_state[2].data[i])
            pose.pose.orientation.x = quat[0]
            pose.pose.orientation.y = quat[1]
            pose.pose.orientation.z = quat[2]
            pose.pose.orientation.w = quat[3]
            
            self._local_path_msg.poses.append(pose)
        self._local_path_viz_pub.publish(self._local_path_msg)

    @staticmethod
    def get_quaternion_from_euler(roll, pitch, yaw):
        """
        Convert an Euler angle to a quaternion.
        https://automaticaddison.com/how-to-convert-euler-angles-to-quaternions-using-python/
        
        Input
            :param roll: The roll (rotation around x-axis) angle in radians.
            :param pitch: The pitch (rotation around y-axis) angle in radians.
            :param yaw: The yaw (rotation around z-axis) angle in radians.
        
        Output
            :return qx, qy, qz, qw: The orientation in quaternion [x,y,z,w] format
        """
        qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
        qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
        qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        
        return [qx, qy, qz, qw]

if __name__ == "__main__":
    rospy.init_node('visualize_local_path', anonymous=True)
    visualizer = VisualizeLocalPath()
    rospy.spin()
    