import geometry_msgs
import rospy

import tf.transformations
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState
from ackermann_msgs.msg import AckermannDriveStamped

class GazeboHelper:
    @staticmethod
    def setModelPoseAndSpeed(modelName, poseXyzRpyOrGeometryPoseMsg, speedXyzRpyOrGeometryTwistMsg):
        state_msg = ModelState()
        state_msg.model_name = modelName

        if type(poseXyzRpyOrGeometryPoseMsg) is geometry_msgs.msg.Pose:
            state_msg.pose = poseXyzRpyOrGeometryPoseMsg
        else:
            state_msg.pose.position.x = poseXyzRpyOrGeometryPoseMsg[0]
            state_msg.pose.position.y = poseXyzRpyOrGeometryPoseMsg[1]
            state_msg.pose.position.z = poseXyzRpyOrGeometryPoseMsg[2]
            quat = tf.transformations.quaternion_from_euler(poseXyzRpyOrGeometryPoseMsg[3], poseXyzRpyOrGeometryPoseMsg[4], poseXyzRpyOrGeometryPoseMsg[5])
            state_msg.pose.orientation.x = quat[0]
            state_msg.pose.orientation.y = quat[1]
            state_msg.pose.orientation.z = quat[2]
            state_msg.pose.orientation.w = quat[3]

        if type(speedXyzRpyOrGeometryTwistMsg) is geometry_msgs.msg.Twist:
            state_msg.twist = speedXyzRpyOrGeometryTwistMsg
        else:
            state_msg.twist.linear.x = speedXyzRpyOrGeometryTwistMsg[0]
            state_msg.twist.linear.y = speedXyzRpyOrGeometryTwistMsg[1]
            state_msg.twist.linear.z = speedXyzRpyOrGeometryTwistMsg[2]
            state_msg.twist.angular.x = speedXyzRpyOrGeometryTwistMsg[3]
            state_msg.twist.angular.y = speedXyzRpyOrGeometryTwistMsg[4]
            state_msg.twist.angular.z = speedXyzRpyOrGeometryTwistMsg[5]

        rospy.wait_for_service('/gazebo/set_model_state')
        try:
            set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
            resp = set_state( state_msg )
            return resp
        except rospy.ServiceException, e:
            print "Service call failed: %s" % e
            return None
        
    @staticmethod
    def reset_controller():
        pub_reset = rospy.Publisher('/rbcar_robot_control/command', AckermannDriveStamped, queue_size=1)
        msg = AckermannDriveStamped()
        msg.drive.speed = 0
        msg.drive.steering_angle = 0
        pub_reset.publish(msg)
        