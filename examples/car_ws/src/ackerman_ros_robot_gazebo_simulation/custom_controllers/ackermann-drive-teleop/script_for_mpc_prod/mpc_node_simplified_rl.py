#!/usr/bin/env python2.7
import argparse
import socket
import atexit
import os
import math
import time
import csv
import logging

import pandas as pd
import numpy as np
import rospy
import tf

from ackermann_msgs.msg import AckermannDriveStamped
from motion_planner.msg import Local_path
from helper import GazeboSimHelper
from std_msgs.msg import Bool
from nav_msgs.msg import Odometry, Path


pub = rospy.Publisher("/rbcar_robot_control/command", AckermannDriveStamped, queue_size=1)
pub_log = rospy.Publisher("/mpc_command", AckermannDriveStamped, queue_size=1)
pub_stop = rospy.Publisher("/stop_signal", Bool, queue_size=1)
pub_first_mpc_control_published = rospy.Publisher("/first_mpc_control_published", Bool, queue_size=1, latch=True)

t = 0
t_count = 0
time_cum_odom = 0
time_cum_socket = 0
path_temp = None
oa, odelta = None, None
ref_x, ref_y, ref_yaw = None, None, None
ref_v = None
ref_curv = None
goal_pose = None
goal_verdict = None
read_path = True
path_global = None
path_change = True
path = None
stop_signal = False
is_sim = True
t_stop = 0
stop = False
WB = 2.48
activate = False
use_motion_planner = True
train_rl_model = False
combine_velocity = False
halt_car_active = True
halt_car_count = 0
halt_car_count_threshold = 4
cnt = 0
save_computation_time = True
first_control_obtained = False
f = None

# pub = rospy.Publisher("/ack_cmd", AckermannDriveStamped, queue_size=1)


# Gazebo Service Helper
gazebo_helper = GazeboSimHelper.GazeboSimControl()

HEADERSIZE = 10
BUFFERSIZE = 100000
PORT = 12345

HEADERSIZE = 10
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
# Set the timeout for the socket
sock.settimeout(240)
ipaddress = socket.gethostname()

# Getting the path from the motion planner
def get_path_from_motion_planner(msg):
    global ref_x, ref_y, ref_yaw, ref_v, goal_pose, ref_curv, goal_verdict
    ref_x = msg.ref_state[0].data
    ref_y = msg.ref_state[1].data
    ref_yaw = msg.ref_state[2].data
    ref_v = msg.ref_state[3].data 
    ref_curv = msg.ref_state[4].data
    goal_pose = msg.goal_pose
    goal_verdict = msg.goal_reached
    
    # print("Goal Verdict: ", goal_verdict)
    # rospy.loginfo("Ref x size: %s", ref_x)
    # rospy.loginfo("Ref y size: %s", ref_y)
    # rospy.loginfo("Ref v size: %s", ref_v)
    # rospy.loginfo("Ref Curvature size: %s", ref_curv)
    
    
# Getting Goal Verdict from the motion planner
def get_goal_from_mp_cb(bool):
    global goal_verdict
    goal_verdict = bool.data
    # print("Goal Verdict: ", goal_verdict)
    pass


# Getting the activation flag for the Linear MPC (20231017)
def activate_lmpc_cb(bool):
    global activate
    activate = bool.data
    pass


# Control Callback only for the LMPC
def control_loop_cb(odom):
    global t, t_count, path_temp, oa, odelta, path, ref_y, ref_x, ref_yaw, ref_curv, WB, stop, t_stop, stop_signal, path_change, cnt, activate, use_motion_planner, goal_pose, ref_v, gazebo_helper, combine_velocity, halt_car_active, f, halt_car_count, halt_car_count_threshold
    
    # If the path is not received, halt the car
    if halt_car_active or (ref_x is None or ref_y is None or ref_yaw is None):
        # Check velocity 
        if halt_car_count < halt_car_count_threshold:
            halt_car_once()
            halt_car_count += 1
            # # if the car is moving in the beginning, halt the car
            # if np.abs(v_long) > 0.1:
            #     # Halt the car
            #     halt_car_once()
            # else:
            #     # If the car is not moving, increment the count and wait until the maximum count is reached
            #     halt_car_count += 1
        else:
            rospy.logwarn("Waiting for the path from the motion planner")
            halt_car_active = False
        
    else:        
        t += 1
        # print("Time step", t)
        quaternion = (
            odom.pose.pose.orientation.x,
            odom.pose.pose.orientation.y,
            odom.pose.pose.orientation.z,
            odom.pose.pose.orientation.w
        )

        x = odom.pose.pose.position.x
        y = odom.pose.pose.position.y

        # Obtain reference path
        euler = tf.transformations.euler_from_quaternion(quaternion)
        psi = euler[2]

        ## TODO: Fix transform
        if combine_velocity:
            v = math.sqrt(odom.twist.twist.linear.x ** 2 + odom.twist.twist.linear.y ** 2)
            # Determine the car whether moving forward or backward using the heading and the velocity vector
            if np.dot([math.cos(psi), math.sin(psi)], [odom.twist.twist.linear.x, odom.twist.twist.linear.y]) > 0:
                v *= 1
            else:
                v *= -1
            current_state = [x, y, psi, v]
        else:
            # Transform Twist to Odometry Frame
            psi = -psi
            rotMat = np.array([[math.cos(psi), -math.sin(psi)], [math.sin(psi), math.cos(psi)]])
            velVec = np.array([[odom.twist.twist.linear.x], [odom.twist.twist.linear.y]])
            transformed_twist = np.matmul(rotMat, velVec)
            v_long = transformed_twist[0][0]
            v_lat = transformed_twist[1][0]
            current_state = [x, y, euler[2], v_long]

        # TODO: Receive obstacle information from obstacle detection node
        # Not Implemented yet
        vehicle_control = AckermannDriveStamped()

        ########################################
        # MPC Controller calculation
        ########################################

        # Obtain the reference path and trigger the MPC server to calculate the control inputs
        if not use_motion_planner:
            path = np.array([ref_x, ref_y, ref_yaw], dtype=np.float32)
        else:
            if ref_x is not None and ref_y is not None:
                print("Received path x length in MPC Node ", len(ref_x))
                print("Received path y length in MPC Node ", len(ref_y))
                print("Received path v length in MPC Node ", len(ref_v))
                print("Received path yaw length in MPC Node ", len(ref_yaw))
                # rospy.logwarn("Received first 8 values of ref yaw in MPC Node: %s", ref_yaw[:8])
            path = np.array([ref_x, ref_y, ref_v, ref_yaw], dtype=np.float32)

        # Check if the path is nan
        if np.isnan(path).any() and path_temp is None:
            return

        elif path_temp is not None and path is None:
            t_count += 1
            path = path_temp
            start_time_socket = time.time()
            if not use_motion_planner:
                vehicle_control, stop_signal = socket_sending_mpc(path, current_state, vehicle_control)
            else:
                vehicle_control, stop_signal = socket_sending_mpc(path, current_state, vehicle_control, goal_pose, goal_verdict)
            time_count = time.time() - start_time_socket
            # time_cum_socket += time

        else:
            t_count += 1
            path_temp = path
            start_time_socket = time.time()
            if not use_motion_planner:
                vehicle_control, stop_signal = socket_sending_mpc(path, current_state, vehicle_control)
            else:
                vehicle_control, stop_signal = socket_sending_mpc(path, current_state, vehicle_control, goal_pose, goal_verdict)
            time_count = time.time() - start_time_socket
            # time_cum_socket += time
        
        # print(path)

        # TODO: Publish the control command to the actuator
        if stop_signal == 1:
            stop = True
            t_stop += 1
        
        # isstop = Bool(stop)
        
        if t_stop > 0:
            path_change = True
            t_stop = 0
        
        if cnt == 1 and path_change == False and stop == True:
            stop = False
        elif cnt == 2 and path_change == False and stop == True:
            path_change = False
            stop = False
        
        isstop = Bool(stop)

        pub.publish(vehicle_control)
        # pub_stop.publish(isstop)
        if train_rl_model:
            # Unpause the simulation to get the next state and send the control inputs
            # Wait for the simulation to reach the next state (step)
            cur_time = rospy.Time.now()
            gazebo_helper.wait_for_a_moment(cur_time, rospy.Duration(0.085))
        # # To tell that the first control command is ready (for synchronization with the motion planner)
        # is_first_mpc_control_published = Bool(True)
        # pub_first_mpc_control_published.publish(is_first_mpc_control_published)
        pass


def halt_car_once():
    msg = AckermannDriveStamped()
    msg.header.stamp = rospy.Time.now()
    msg.drive.steering_angle = 0.0
    msg.drive.speed = 0.0
    pub.publish(msg)
    rospy.loginfo("Car halted from MPC!")
    rospy.sleep(0.15)
    
    
def halt_car_during_exit():
    msg = AckermannDriveStamped()
    msg.header.stamp = rospy.Time.now()
    msg.drive.steering_angle = 0.0
    msg.drive.speed = 0.0
    pub.publish(msg)
    rospy.loginfo("Car halted from MPC!")
    

def socket_sending_mpc(path, current_state, vehicle_control, goal_pose=None, goal_verdict=None):
    global train_rl_model, ref_curv, sock, HEADERSIZE, BUFFERSIZE, ipaddress, PORT, f, eval_mode, save_computation_time, first_control_obtained

    print("Received path curve length in MPC Node ", len(ref_curv))
    # print("Current State length in MPC Node", len(current_state))      
    print("Current state in MPC", current_state)
    path_len = path.shape[0] * path.shape[1] + len(ref_curv)
    path_len_arr = np.array([path_len], dtype=np.float32)
    print("Path Length to send: ", path_len_arr)
    path_flat = path.flatten()
    ref_curv_flat = np.array(ref_curv, dtype=np.float32).flatten()
    vehicle_state = np.array(current_state, dtype=np.float32).flatten()
    # print("Current State length to be sent in MPC Node ", vehicle_state.shape)    
    # rospy.loginfo("Goal Verdict %s", goal_verdict)
    if goal_pose is not None and goal_verdict is not None:
        goal_pose = np.array(goal_pose, dtype=np.float32).flatten()
        goal_verdict = 1 if goal_verdict else 0
        goal_verdict = np.array([goal_verdict], dtype=np.float32)
        data = np.hstack((path_len_arr, path_flat, ref_curv_flat, vehicle_state, goal_pose, goal_verdict))
    else:
        data = np.hstack((path_len_arr, path_flat, vehicle_state))
    # print("Data size to be sent from MPC ", data.shape)
    data = data.tobytes()

    msg = bytes('{:<{}}'.format(len(data), HEADERSIZE).encode("utf-8")) + data
    if train_rl_model:
        # Pause the simulation to wait for the control inputs
        gazebo_helper.pause()
        # rospy.loginfo("Gazebo simulation paused")
    count_start = time.time()
    sock.sendto(msg, (ipaddress, PORT))
    # print("Waiting for the control inputs from the MPC server")
    # Receive the calculated control inputs from the MPC server
    data_raw = sock.recvfrom(BUFFERSIZE)
    count_end = time.time()
    time_total_mpc = count_end - count_start
    if save_computation_time:
        f.write(str(time_total_mpc) + '\n')
    print("Total Computation time For Solving MPC: ", time_total_mpc)
    gazebo_helper.unpause()
    # print("Control inputs received")
    if data_raw:
        data = data_raw[0]
        data = data[HEADERSIZE:]
        data = np.frombuffer(data, dtype=np.float32)
        # print("Control inputs received", data)
        oa = data[0]
        odelta = data[1]
        ov = data[2]
        stop_signal = data[3]
        ojerk = data[4]
        rospy.loginfo("Acceleration: %f, Steering Angle: %f, Speed: %f, Stop Signal: %f, Jerk: %f", oa, odelta, ov, stop_signal, ojerk)
    else:
        print('no data from', ipaddress)
        return

    vehicle_control.header.stamp = rospy.Time.now()
    vehicle_control.drive.steering_angle = odelta
    vehicle_control.drive.acceleration = oa
    vehicle_control.drive.speed = ov
    vehicle_control.drive.jerk = ojerk
    
    
    # To tell that the first control command is ready (for synchronization with the motion planner)
    is_first_mpc_control_published = Bool(True)
    # pub_first_mpc_control_published.publish(is_first_mpc_control_published)
    
    if not first_control_obtained:
        # Write a text file to indicate that the first control command is obtained
        base_dir = os.environ.get('BASE_DIR')
        if base_dir is not None:
            if base_dir != '':
                first_control_file = os.path.join(os.environ.get('BASE_DIR'), 'car_ws/src/motion_planner/scripts/control_flag/first_control_obtained.txt')
            else:
                first_control_file = '/car_ws/src/motion_planner/scripts/control_flag/first_control_obtained.txt'
        else:
            first_control_file = '/home/vialab/car_ws/src/motion_planner/scripts/control_flag/first_control_obtained.txt'
        with open(first_control_file, 'w') as fs:
            fs.write('1')
    return vehicle_control, stop_signal


@atexit.register
def close_file_and_wrap_up():
    global f, save_computation_time
    halt_car_during_exit()
    if save_computation_time:
        f.close()
        rospy.loginfo("MPC Node gets Killed")
        # Remove the first control obtained file
        base_dir = os.environ.get('BASE_DIR')
        if base_dir is not None:
            if base_dir != '':
                first_control_file = os.path.join(os.environ.get('BASE_DIR'), 'car_ws/src/motion_planner/scripts/control_flag/first_control_obtained.txt')
            else:
                first_control_file = '/car_ws/src/motion_planner/scripts/control_flag/first_control_obtained.txt'
        else:
            first_control_file = '/home/vialab/car_ws/src/motion_planner/scripts/control_flag/first_control_obtained.txt'
        if os.path.exists(first_control_file):
            # remove the file
            os.remove(first_control_file)
            rospy.loginfo("First Control Obtained File Removed")
    # Halt the car
    pass


def main():
    global use_motion_planner
    
    # For Golf Cart
    sub = rospy.Subscriber("/INS/odom", Odometry, control_loop_cb, queue_size=1)

    if use_motion_planner:
        sub_path = rospy.Subscriber("/path_motion_planner", Local_path, get_path_from_motion_planner, queue_size=1)
        sub_goal = rospy.Subscriber("/path_motion_planner/goal_reached", Bool, get_goal_from_mp_cb, queue_size=1)
    else:
        # get_path_from_file()
        # sub_parking_path = rospy.Subscriber("/pathGenerator/parkingPath", Path, get_path_steer_cb, queue_size=1)
        sub_activate = rospy.Subscriber("/activate_lmpc", Bool, activate_lmpc_cb, queue_size=1)

    rospy.spin()
    pass


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--eval_mode', type=bool, default=False, help='Evaluation Mode')
    parser.add_argument('--file_path_dir', type=str, default='/home/vialab/car_ws/SLM-Lab/slm_lab/env/car/log/20240411_1520', help='File Path Directory')
    parser.add_argument('--file_path_name', type=str, default='first_reset', help='File Path Name')
    parser.add_argument('--horizon_type', type=str, default="DRL", help='Specify the type of Horizon Sampling')
    rospy.init_node("mpc_node", disable_signals=False)
    # Specify the file path for saving the computation time
    args, unknown = parser.parse_known_args()  # Parse the arguments
    # Check the args passed
    rospy.loginfo("Evaluation Mode: %s", args.eval_mode)
    rospy.loginfo("File Path Name: %s", args.file_path_name)
    rospy.loginfo("File Path Directory: %s", args.file_path_dir)
    rospy.loginfo("Sampling Type: %s", args.horizon_type)
    # Set the file path name
    file_path_name = args.file_path_name
    # Removes extension from the file name
    file_path_name = file_path_name.split('.')[0]
    saving_log_folder = os.path.join(args.file_path_dir, file_path_name)
    rospy.loginfo("Saving Log Folder: %s", saving_log_folder)
    save_computation_time = args.eval_mode
    sampling_type = args.horizon_type
    # Check the arguments passed
    if save_computation_time:
        file_name = saving_log_folder + '_' + sampling_type + '.txt'
        f = open(file_name, 'w')
    else:
        f = None
    rate = rospy.Rate(10)
    try:
        main()
        rate.sleep()
    except rospy.ROSInterruptException:
        pass
