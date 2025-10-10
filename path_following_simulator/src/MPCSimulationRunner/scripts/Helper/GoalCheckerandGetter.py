#!/usr/bin/env python2.7

import re
import rospy

from tf.transformations import quaternion_from_euler
from move_base_msgs.msg import MoveBaseGoal
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped


class GoalCheckerandGetter():
    def __init__(self):
        pass
    
    # Checking the move_base goal status
    @staticmethod
    def get_the_goal_reached_status(goal_status):
        goal = goal_status.status_list
        if not goal:
            pass
        else:        
            goal = " ".join("{} ".format(item) for item in goal)        
            pattern = r"status:\s+(?P<number>\d+)"
            match = re.search(pattern, goal)
            if match:
                status = match.group("number")
                return status          
    
    # Getting the goal points
    @staticmethod
    def get_the_goal(goal_points, pickup=False):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        if pickup:
            goal.target_pose.pose.position.x = goal_points[0]
            goal.target_pose.pose.position.y = goal_points[1]
        else: 
            goal.target_pose.pose.position.x = goal_points[0]
            goal.target_pose.pose.position.y = goal_points[1]
        rpy_goal = quaternion_from_euler(0, 0, goal_points[2])
        goal.target_pose.pose.orientation.x = rpy_goal[0]
        goal.target_pose.pose.orientation.y = rpy_goal[1]
        goal.target_pose.pose.orientation.z = rpy_goal[2]
        goal.target_pose.pose.orientation.w = rpy_goal[3]
        return goal
    
    # Setting amcl pose
    @staticmethod
    def set_amcl_pose(amcl_pose_points):
        amcl_pose = PoseWithCovarianceStamped()
        amcl_pose.header.frame_id = "map"
        amcl_pose.header.stamp = rospy.Time.now()
        amcl_pose.pose.pose.position.x = amcl_pose_points[0]
        amcl_pose.pose.pose.position.y = amcl_pose_points[1]
        rpy_amcl = quaternion_from_euler(0, 0, amcl_pose_points[2])
        amcl_pose.pose.pose.orientation.x = rpy_amcl[0]
        amcl_pose.pose.pose.orientation.y = rpy_amcl[1]
        amcl_pose.pose.pose.orientation.z = rpy_amcl[2]
        amcl_pose.pose.pose.orientation.w = rpy_amcl[3]
        return amcl_pose


if __name__ == '__main__':
    pass