#!/usr/bin/env python2.7
import math
import rospy    

from nav_msgs.msg import Odometry
from geometry_msgs.msg import PolygonStamped, Point32
from tf.transformations import euler_from_quaternion

class VisualizeLocalPath(object):
    def __init__(self):
        self.vehicle_footprint = [ [-1.1,-0.6], [2.68,-0.6], [2.68,0.6], [-1.1,0.6] ]
        self._odom_sub = rospy.Subscriber('/INS/odom', Odometry, self.odom_cb, queue_size=1)
        self._marker_pub = rospy.Publisher('/vehicle_marker', PolygonStamped, queue_size=1)
        
    def odom_cb(self, msg):
        self.cur_pose = [msg.pose.pose.position.x, msg.pose.pose.position.y]
        orientation = msg.pose.pose.orientation
        euler = euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])
        yaw = euler[2]
                
        # Publish current pose vehicle footprint
        vehicle_footprint = PolygonStamped()
        vehicle_footprint.header.stamp = rospy.Time.now()
        vehicle_footprint.header.frame_id = "map"
        # Transform vehicle footprint to current pose
        for i in range(len(self.vehicle_footprint)):
            point = Point32()
            point.x = self.vehicle_footprint[i][0]*math.cos(yaw) - self.vehicle_footprint[i][1]*math.sin(yaw) + self.cur_pose[0]
            point.y = self.vehicle_footprint[i][0]*math.sin(yaw) + self.vehicle_footprint[i][1]*math.cos(yaw) + self.cur_pose[1]
            vehicle_footprint.polygon.points.append(point)
        self._marker_pub.publish(vehicle_footprint)


if __name__ == "__main__":
    rospy.init_node('visualize_local_path', anonymous=True)
    VisualizeLocalPath()
    rospy.spin()