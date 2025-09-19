#!/usr/bin/env python2.7

import rospy
import tf.transformations
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Twist
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState


class GazeboSimControl(object):
    def __init__(self):
        self.pause_sim = rospy.ServiceProxy('/gazebo/pause_physics', persistent=True)
        self.unpause_sim = rospy.ServiceProxy('/gazebo/unpause_physics', persistent=True)
        self.reset_sim = rospy.ServiceProxy('/gazebo/reset_simulation', persistent=True)
        self.set_model_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState, persistent=True)
        
    def pause(self):
        self.pause_sim()
        
    def unpause(self):
        self.unpause_sim()
    
    def reset(self):
        self.reset_sim()
    
    def set_model_state(self, model_name, pose, twist):
        state_msg = ModelState()
        state_msg.model_name = model_name

        if type(pose) is Pose:
            state_msg.pose = pose
        else:
            state_msg.pose.position.x = pose[0]
            state_msg.pose.position.y = pose[1]
            state_msg.pose.position.z = pose[2]
            quat = tf.transformations.quaternion_from_euler(pose[3], pose[4], pose[5])
            state_msg.pose.orientation.x = quat[0]
            state_msg.pose.orientation.y = quat[1]
            state_msg.pose.orientation.z = quat[2]
            state_msg.pose.orientation.w = quat[3]

        if type(twist) is Twist:
            state_msg.twist = twist
        else:
            state_msg.twist.linear.x = twist[0]
            state_msg.twist.linear.y = twist[1]
            state_msg.twist.linear.z = twist[2]
            state_msg.twist.angular.x = twist[3]
            state_msg.twist.angular.y = twist[4]
            state_msg.twist.angular.z = twist[5]

        self.set_model_state(state_msg)
        