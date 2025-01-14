#!/usr/bin/env python2.7

import rospy
import tf.transformations
from std_srvs.srv import Empty
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Twist
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState


class GazeboSimControl:
    def __init__(self):
        self.pause_sim = rospy.ServiceProxy('/gazebo/pause_physics', Empty, persistent=True)
        self.unpause_sim = rospy.ServiceProxy('/gazebo/unpause_physics', Empty, persistent=True)
        self.reset_sim = rospy.ServiceProxy('/gazebo/reset_simulation', Empty, persistent=True)
        self.set_model_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState, persistent=True)
        
    def pause(self):
        rospy.wait_for_service('/gazebo/pause_physics')
        self.pause_sim()
        
    def unpause(self):
        rospy.wait_for_service('/gazebo/unpause_physics')
        self.unpause_sim()
    
    def reset(self):
        rospy.wait_for_service('/gazebo/reset_simulation')
        self.reset_sim()
    
    def set_model_state_func(self, model_name='rbcar', pose=[-20.8261, -11.55, 0.097952, 0.0, 0.0, 0.0], twist=[0, 0, 0, 0, 0, 0]):
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

        rospy.wait_for_service('/gazebo/set_model_state')
        self.set_model_state(state_msg)
        
    @staticmethod
    def wait_for_a_moment(start_time, waiting_duration=rospy.Duration(0.1)):
        # rospy.loginfo("start_time: %s", start_time)
        # rospy.loginfo("current_time: %s", rospy.Time.now())
        time_passed = False
        while not time_passed:
            if rospy.Time.now() - start_time > waiting_duration:
                time_passed = True
            else:
                pass
            # rospy.loginfo("Time difference: %s", rospy.Time.now() - start_time)
            # rospy.loginfo("Waiting for a gazebo simulation step")
