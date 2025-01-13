#!/usr/bin/env python2.7
import argparse
import os
import random
import datetime
import math
import csv

import rospy
import numpy as np
import actionlib
import LatticePlanner as lp
import roslaunch
import random
import matplotlib.pyplot as plt
import GeometryHelper

from os import environ
from shapely.geometry import Point, LineString
from move_base_msgs.msg import MoveBaseAction
from nav_msgs.msg import Path, Odometry
from std_msgs.msg import Bool
from ackermann_msgs.msg import AckermannDriveStamped
from motion_planner.msg import Local_path, State
from tf.transformations import euler_from_quaternion
from Helper import PathSmoother, GazeboHelper, GoalCheckerandGetter


def get_car_ws_path():
    """
    Determines the car_ws path by navigating up from the current file's location.
    """
    current_file_path = os.path.abspath(__file__)
    current_dir = os.path.dirname(current_file_path)
    # Navigate up to car_ws/src
    src_dir = os.path.abspath(os.path.join(current_dir, '..', '..'))
    # Navigate up to car_ws
    car_ws_path = os.path.abspath(os.path.join(src_dir, '..'))
    return car_ws_path


class MotionPlannerNode:
    car_ws_path = get_car_ws_path()
    environ['OMP_NUM_THREADS'] = '6'
    def __init__(self, file_path=None, eval_mode=False):
        # Planner Options
        if os.environ.get('BASE_DIR') is not None:
            self.basedir = os.environ.get('BASE_DIR')
        else:
            self.basedir = '/home/vialab'
        self.eval_mode = rospy.get_param('~eval_mode', eval_mode)
        self.lattice_planner = lp.LatticePlanner()
        self.client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
        # Path Related Parameters
        self.ref_path_load_from_file = rospy.get_param('~ref_path_load_from_file', True)
        rospy.loginfo("Ref Path Load from File: " + str(self.ref_path_load_from_file))
        self.global_path_receive_flag = rospy.get_param('~global_path_receive_flag', True)
        self.global_path_receive_once = rospy.get_param('~global_path_receive_once', True)
        self.save_global_path = rospy.get_param('~save_global_path', True)
        self.save_global_path_dir = os.path.join(self.car_ws_path, "src/ackerman_ros_robot_gazebo_simulation/custom_controllers/ackermann-drive-teleop/script_for_mpc_prod/data/")
        self.manual_path_generation = rospy.get_param('~manual_path_generation', True)
        if self.manual_path_generation:
            self.save_global_path_dir = os.path.join(self.car_ws_path, '/car_ws/path/')
            cur_date = datetime.datetime.now().strftime("%Y%m%d_%H%M")
            self.save_global_path_dir = os.path.join(self.save_global_path_dir, "manual_generated_path")
            if not os.path.exists(self.save_global_path_dir):
                os.makedirs(self.save_global_path_dir)
        self.smooth_the_path = rospy.get_param('~smooth_the_path', True)
        self.save_global_path_spline = rospy.get_param('~save_global_path_spline', True)
        self.ros_global_planner_launch_dir = rospy.get_param('~ros_global_planner_launch_dir', os.path.join(self.car_ws_path, '/nonlinear_mpc_sim/src/mpc_local_planner/mpc_local_planner_examples/launch/carlike_global_sim.launch'))
        self.goal_lists_dir = rospy.get_param('~goal_lists_dir',os.path.join(self.car_ws_path, 'src/motion_planner/scripts/data/parking_slot_locations.txt'))
        self.train_rl_model = rospy.get_param('~train_rl_model', False)
        self.mode = rospy.get_param('~mode', 'train')
        self.path_hist_seed_dir = rospy.get_param('~path_hist_seed_dir',os.path.join(self.car_ws_path, 'src/motion_planner/logs/path_hist_seeds_target.txt'))
        self.path_hist_seed_idx_dir = rospy.get_param('~path_hist_seed_idx_dir',os.path.join(self.car_ws_path, 'src/motion_planner/logs/random_seed_index.csv'))
        self.path_hist_seed_file_dir = rospy.get_param('~path_hist_seed_file_dir', os.path.join(self.car_ws_path, 'src/motion_planner/logs/20240705_0811_train_S8/path_hist_seed.csv'))
        self.reverse_path = True if self.mode == 'train' else False
        self.rerun_mpc_node_command = ""
        self.ref_x = None
        self.ref_y = None
        self.ref_yaw = None
        self.goal_pose = None
        self.goal_dist_in_frenet = None
        self.ref_path = None
        self.path_available = False
        #quit()
        if self.smooth_the_path:
            self.path_smoother = PathSmoother.PathSmoother()
        # TODO: Check Backward Path
        # ROS Parameters
        self.sub = rospy.Subscriber('/INS/odom', Odometry, self.odom_callback, queue_size=1)
        # self.sub_first_control_signal = rospy.Subscriber('/first_mpc_control_published', Bool, self.first_control_signal_callback, queue_size=10)
        self.first_control_received = False
        self.first_control_not_received_count = 0
        self.first_control_not_received_count_threshold = 5
        self.initial_reference_path = None
        if not self.ref_path_load_from_file:
            self.global_path_sub = rospy.Subscriber('/move_base/GlobalPlanner/plan', Path, self.global_path_callback)
            # self.global_path_sub = rospy.Subscriber('/move_base/HybridAStarPlanner/plan', Path, self.global_path_callback)
        self.pub = rospy.Publisher('/path_motion_planner', Local_path, queue_size=1)
        self.pub_global_path = rospy.Publisher('/global_path', Path, queue_size=1)
        self.goal_reached_mp = rospy.Publisher('/path_motion_planner/goal_reached', Bool, queue_size=1)
        self.halting_pub = rospy.Publisher('/rbcar_robot_control/command', AckermannDriveStamped, queue_size=1)
        self.path_train_pre = self.basedir + '/car_ws/path/'
        self.path_train_pre_dir = rospy.get_param('~path_train_pre_dir', '20240406_1410')
        self.path_train_pre_dir = os.path.join(self.path_train_pre, self.path_train_pre_dir)
        if not self.eval_mode:
            self.file_path = rospy.get_param('~path_file_dir', self.basedir + '/car_ws/path/20240404_0400/train/path_147.csv')
            file_path = None
        elif self.eval_mode:
            path_file_folder = self.path_train_pre_dir + '/test' # val or test depending on the datasets naming
            self.file_path = os.path.join(path_file_folder, file_path)
        # ROS Waiting for the Odom Message (Count the number of messages received in the first determined time before using the message)
        # To avoid the first message to be used, because it sometimes contains wrong velocity values from the previous simulation
        self.odom_wait_count = 0
        self.odom_wait_count_threshold = 10 #10Hz * 10 = 1 second
            
        # Current Pose and Speed
        self.cur_pose = None
        self.cur_pos_in_frenet = None
        self.cur_twist = None
        self.speed_threshold_to_move = 0.08
        self.cur_speed = [None, None]
        self.goal_reached = False
        self.init_speed = [0, 0]
        self.init_accel = [0, 0]
        self.accel = [0, 0]
        self.prefer_feedback = False
        self.first_time = True
        self.prev_s_point = 0.0
        self.prev_l_point = 0.0
        # Constants
        self.DT = self.lattice_planner.DT
        self.GOAL_THRESHOLD = rospy.get_param('~goal_distance_threshold', 0.60)
        rospy.loginfo("Goal Distance Threshold: " + str(self.GOAL_THRESHOLD))
        # Simulation and Debug Flag
        self.is_sim = True
        self.DEBUG = False
        rospy.loginfo("Is Debug Mode: " + str(self.DEBUG))
        self.goal_point_list = np.loadtxt(self.goal_lists_dir, delimiter=',')
        self.goal_utils = GoalCheckerandGetter.GoalCheckerandGetter()
        self.send_goal_to_move_base = False if self.ref_path_load_from_file else True
        self.randomize_goal_and_initial_pose = True if self.train_rl_model and self.send_goal_to_move_base else False
        self.manually_set_goal = rospy.get_param('~manually_set_goal', True)
        # Global Planner
        self.use_global_planner_from_MATLAB = rospy.get_param('~use_global_planner_from_MATLAB', False)
        # When loading path from file
        if self.ref_path_load_from_file:
            if self.train_rl_model:
                if self.mode == 'train':
                    self.file_path_list = os.path.join(self.path_train_pre_dir, 'train')
                elif self.mode == 'val':
                    self.file_path_list = os.path.join(self.path_train_pre_dir, 'val')
                elif self.mode == 'free':
                    self.file_path_list = os.path.join(self.path_train_pre_dir, 'free')
                rospy.loginfo("Mode: %s", self.mode)
                path_lists_selected = os.listdir(self.file_path_list)
                path_csv_files = []
                for path in path_lists_selected:
                    if path.endswith('.csv'):
                        path_csv_files.append(path)
                if self.mode == 'train':
                    use_past_seed = rospy.get_param('use_past_seed', False)
                    # Read current seed number from RL env
                    with open(self.path_hist_seed_idx_dir, 'r') as f:
                        seed_idx = f.readline()
                    seed_idx = int(seed_idx)
                    if use_past_seed:
                        # Read the file to get to where the seed number is saved (from env)
                        try:
                            with open(self.path_hist_seed_file_dir, 'r') as f:
                                reader = csv.reader(f)
                                for i, row in enumerate(reader):
                                    if i == seed_idx:
                                        num_seed, path_file_selected = row
                                        break
                            random.seed(num_seed)
                            rospy.loginfo("Current Seed: %s", num_seed)
                        except Exception as e:
                            rospy.logwarn("Seed Number Out of Range, getting a new seed number!")
                            num_seed = random.randint(0, 100000)
                            # Set the seed number
                            random.seed(num_seed)
                            path_file_selected = random.choice(path_csv_files)
                    else:
                        num_seed = random.randint(0, 100000)
                        rospy.loginfo("Current Seed: %s", num_seed)
                        # Set the seed number
                        random.seed(num_seed)
                        path_file_selected = random.choice(path_csv_files)
                    # Read the file to get to where the seed number is saved (from env)
                    with open(self.path_hist_seed_dir, 'r') as f:
                        path_to_save_seed = f.readline()
                    # Save the seed number and the path file selected
                    with open(path_to_save_seed, 'a') as f:
                        f.write(str(num_seed) + ',' + path_file_selected + '\n')
                if eval_mode:
                    rospy.loginfo("Eval mode!")
                    path_file_selected = self.file_path
                self.file_path = os.path.join(self.file_path_list, path_file_selected)
            rospy.loginfo("Path File Directory: " + self.file_path)
            trajectory = np.loadtxt(self.file_path, delimiter=',', skiprows=1)[:,:]
            # make sure that the car is stopped
            # self.halt_car()
            self.ref_x = trajectory[:, 0].tolist()
            self.ref_y = trajectory[:, 1].tolist()
            self.ref_yaw = trajectory[:, 2].tolist()
            if self.reverse_path:
                # Set the probability of reversing the path 
                probs = [0.5, 0.5]
                reverse_path_final = np.random.choice([True, False], p=probs)
                if reverse_path_final:
                    rospy.loginfo("Reversing the Path!")
                    self.ref_x = self.ref_x[::-1]
                    self.ref_y = self.ref_y[::-1]
                    # The yaw angle should be reversed by pi
                    self.ref_yaw = [yaw + math.pi for yaw in self.ref_yaw[::-1]]
            # Set Goal
            self.goal_pose = [self.ref_x[-1], self.ref_y[-1]]
            self.lattice_planner.set_goal(self.goal_pose[0], self.goal_pose[1])
            # Spline parameters
            self.use_linear_spline = rospy.get_param('~use_linear_spline', True)   
            self.get_cubic_splined_path = False
            if not self.use_linear_spline:
                self.lattice_planner.use_linear_spline = False
                skip_points = rospy.get_param('~skip_points_num', 1) # 1 means no skipping
                if skip_points > 1:
                    ref_x_temp = self.ref_x
                    ref_y_temp = self.ref_y
                    ref_yaw_temp = self.ref_yaw
                    self.ref_x = self.ref_x[::skip_points]
                    self.ref_y = self.ref_y[::skip_points]
                    self.ref_yaw = self.ref_yaw[::skip_points]
                    # Check if the last point is included
                    if self.ref_x[-1] != ref_x_temp[-1]:
                        self.ref_x.append(ref_x_temp[-1])
                        self.ref_y.append(ref_y_temp[-1])
                        self.ref_yaw.append(ref_yaw_temp[-1])
                    self.ref_x, self.ref_y, self.ref_yaw, _, self.ref_path = self.lattice_planner.get_reference_line(self.ref_x, self.ref_y)
                else:
                    self.ref_x, self.ref_y, self.ref_yaw, _, self.ref_path = self.lattice_planner.get_reference_line(self.ref_x, self.ref_y)
                self.lattice_planner.ref_x_from_path = self.ref_x
                self.lattice_planner.ref_y_from_path = self.ref_y
                self.lattice_planner.ref_yaw_from_path = self.ref_yaw
                # Set the maximum path length
                self.lattice_planner.set_the_maximum_distance_cubic_spline(self.ref_path)
            else:
                self.ref_path_wo_spline = [self.ref_x, self.ref_y, self.ref_yaw]
                self.lattice_planner.create_line_string_from_waypoints(self.ref_path_wo_spline)
                self.ref_path = np.zeros((len(self.ref_x), 3))
            # Set the maximum path length for goal checking
            self.goal_distance_max = self.lattice_planner.ref_path_max_dist
            # Set the Mode CHange Threshold
            # self.SCALE_FACTOR = 0.8 #1.0 #0.5
            # self.DIST_FACTOR = self.SCALE_FACTOR * (self.lattice_planner.MAX_T + self.lattice_planner.MIN_T) / 2
            # self.STOPPING_DIST_THRESHOLD = self.DIST_FACTOR * (self.lattice_planner.SPEED_THRES_STOP**2) / (2 * self.lattice_planner.MAX_ACCEL) #4.48m
            # Set the distance as proportion between the cruising and stopping distance
            proportion_of_stop_dist = 0.25
            self.STOPPING_DIST_THRESHOLD = proportion_of_stop_dist * self.goal_distance_max
            rospy.loginfo("Stopping Distance Threshold: " + str(self.STOPPING_DIST_THRESHOLD))
            if self.get_cubic_splined_path:
                ref_x_cubic, ref_y_cubic, ref_yaw_cubic, _, _ = self.lattice_planner.get_reference_line(self.ref_x, self.ref_y)
                # Save the cubic splined path
                column_names = ['ref_x', 'ref_y', 'ref_yaw']
                data = np.array([ref_x_cubic, ref_y_cubic, ref_yaw_cubic]).T
                path_folder_splined = self.basedir + "/car_ws/path/20240531_0130_filtered_MATLAB_M_E/splined_path/sparse_{}/".format(skip_points)
                if not os.path.exists(path_folder_splined):
                    os.makedirs(path_folder_splined)
                file_path_path_name = self.file_path.split('/')[-1]
                # Without the extension
                file_path_path_name = file_path_path_name.split('.')[0]
                path_file_name = path_folder_splined + file_path_path_name + '_splined.csv'
                np.savetxt(path_file_name, data, delimiter=',', header=','.join(column_names))
                # Plot the cubic splined path
                plt.plot(ref_x_cubic, ref_y_cubic, 'r')
                plt.plot(self.ref_x, self.ref_y, 'b')
                # Legend
                plt.legend(['Cubic Splined Path', 'Original Path', 'Starting Point', 'Goal Point'])
                plt.xlabel('X (m)')
                plt.ylabel('Y (m)')
                # Plot starting point and goal point (o and square shape)
                plt.plot(self.ref_x[0], self.ref_y[0], 'go')
                plt.plot(self.ref_x[-1], self.ref_y[-1], 'bs')
                plt.title('Cubic Splined Path vs Original Path')
                plt.savefig(path_folder_splined + file_path_path_name + '_splined.png')
                plt.clf()
                # Plot the yaw angle
                plt.plot(ref_yaw_cubic, 'r')
                plt.plot(self.ref_yaw, 'b')
                # Legend
                plt.legend(['Cubic Splined Yaw', 'Original Yaw'])
                plt.xlabel('Index')
                plt.ylabel('Yaw Angle')
                plt.title('Cubic Splined Yaw vs Original Yaw')
                plt.savefig(path_folder_splined + file_path_path_name + '_yaw_splined.png')
                plt.clf()
            self.goal_dist_in_frenet, _ = self.XY_2_SL(self.goal_pose[0], self.goal_pose[1])
            self.lattice_planner.prev_head = self.ref_yaw[0]
            # Save Splined Path
            if self.save_global_path_spline:
                rospy.loginfo("Saving Global Path!")
                column_names = ['ref_x', 'ref_y', 'ref_yaw']
                data = np.array([self.ref_x, self.ref_y, self.ref_yaw]).T
                path_file_name = os.path.join(self.car_ws_path, 'src/ackerman_ros_robot_gazebo_simulation/custom_controllers/ackermann-drive-teleop/script_for_mpc_prod/data/splined_path.csv')
                np.savetxt(path_file_name, data, delimiter=',', header=','.join(column_names))            
            rospy.loginfo("Loaded Path from File!")
        if self.is_sim:
            self.gh = GazeboHelper.GazeboHelper()
            self.goal_setter = GoalCheckerandGetter.GoalCheckerandGetter()
            if not self.ref_path_load_from_file:
                init_y_offset = np.random.uniform(-2.5, 2.5)
                self.starting_pose = [29.997, -44.092+init_y_offset, 0.0620, 0.0, 0.0, 1.59]
                init_x = np.random.uniform(0, 15)
                self.starting_vel = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
            else:
                # Set the starting pose to the first point of the reference path (prev z-axis default 0.097952)
                self.starting_pose = [self.ref_x[0], self.ref_y[0], 0.0620, 0.0, 0.0, self.ref_yaw[0]]
                self.starting_vel = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
            # rospy.loginfo("Resetting the Simulation World")
            # Before resetting the simulation world, make sure that the car is stopped if not stopped then set zero the velocity
            while self.cur_speed[0] is None or np.abs(self.cur_speed[0]) > 0.1:
                if self.cur_speed[0] is None:
                    rospy.logwarn("Odom Message is not received yet!")
                self.halt_car()
                rospy.logwarn("Waiting for the car to stop!")
                rospy.sleep(0.75)
            self.gh.setModelPoseAndSpeed("rbcar", self.starting_pose, self.starting_vel)
            self.gh.reset_controller()
            # self.halt_car()
            rospy.loginfo("Reset simulation world!")
        pass
    
    def first_control_signal_callback(self, msg):
        self.first_control_received = msg.data
        if not self.first_control_received:
            self.first_control_not_received_count += 1
            # rospy.loginfo("First Control Received: " + str(self.first_control_received))
        pass
    
    def global_path_callback(self, msg):
        if self.global_path_receive_flag and not self.ref_path_load_from_file:
            path_length = len(msg.poses)
            ref_yaw = []
            ref_x = [msg.poses[i].pose.position.x for i in range(path_length)]
            ref_y = [msg.poses[i].pose.position.y for i in range(path_length)]

            for i in range(path_length):
                quaternion = (
                    msg.poses[i].pose.orientation.x,
                    msg.poses[i].pose.orientation.y,
                    msg.poses[i].pose.orientation.z,
                    msg.poses[i].pose.orientation.w
                )
                euler = euler_from_quaternion(quaternion)
                ref_yaw.append(euler[2])
                
            # Stop receiving the global path after receiving it once 
            if self.global_path_receive_once:
                self.pub_global_path.publish(msg)
                self.global_path_receive_flag = False
            rospy.loginfo("Received Global Path!")
            # Set Goal
            # self.ref_x = ref_x
            # self.ref_y = ref_y
            # self.ref_yaw = ref_yaw
            if self.smooth_the_path:
                path_len = len(ref_x)
                path_lists = [[ref_x[i], ref_y[i], ref_yaw[i]] for i in range(path_len)]
                smoothed_path = self.path_smoother.path_smoother(self.path_smoother.path_smoother_forward(path_lists), self.path_smoother.path_smoother_backward(path_lists))
                ref_x = [smoothed_path[i][0] for i in range(path_len)]
                ref_y = [smoothed_path[i][1] for i in range(path_len)]
                ref_yaw = [smoothed_path[i][2] for i in range(path_len)]
                rospy.loginfo("Path Smoothing is Done!")
            self.goal_pose = [ref_x[-1], ref_y[-1]]
            if self.use_global_planner_from_MATLAB:
                # If the global planner is from MATLAB, then the remove the reverse path by taking the first segment of the path
                pathList, cuspPointIdx = self.segmentPathByCuspPoint(ref_x, ref_y)
                if len(cuspPointIdx) > 0:
                    self.logger.info('Path contains cusp points!, Getting the first path segment')
                    skip_points_num = 10
                    ref_x, ref_y = pathList[0][:-skip_points_num, 0], pathList[0][:-skip_points_num, 1]
                    ref_yaw = np.arctan2(np.diff(ref_y), np.diff(ref_x))
                    # Pad the last element of the path_yaw
                    ref_yaw = np.append(ref_yaw, ref_yaw[-1])
            if self.save_global_path:
                rospy.loginfo("Saving Global Path!")
                column_names = ['ref_x', 'ref_y', 'ref_yaw']
                data = np.array([ref_x, ref_y, ref_yaw]).T
                if not self.manual_path_generation:
                    path_file_name = self.save_global_path_dir + datetime.datetime.now().strftime("%Y%m%d_%H%M") + '_path.csv'
                else:
                    manual_path_folder = self.basedir + '/car_ws/path/manual_generated_path'
                    path_file_name = manual_path_folder + '/' + datetime.datetime.now().strftime("%Y%m%d_%H%M%s") + '_path.csv'
                rospy.loginfo("Path File Name: " + path_file_name)
                np.savetxt(path_file_name, data, delimiter=',', header=','.join(column_names))
            self.lattice_planner.set_goal(self.goal_pose[0], self.goal_pose[1])
            self.ref_x, self.ref_y, self.ref_yaw, _, self.ref_path = self.lattice_planner.get_reference_line(ref_x, ref_y)
            self.goal_dist_in_frenet, _ = self.XY_2_SL(self.goal_pose[0], self.goal_pose[1])
            pass                
    
    def odom_callback(self, msg):
        if not self.goal_reached and self.odom_wait_count >= self.odom_wait_count_threshold:
            rospy.loginfo("Current Pose: " + str([msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z]))
            self.cur_pose = [msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z]
            if self.is_sim:
                self.cur_twist = [msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.linear.z]
                quaternion = (
                    msg.pose.pose.orientation.x,
                    msg.pose.pose.orientation.y,
                    msg.pose.pose.orientation.z,
                    msg.pose.pose.orientation.w
                )
                # Obtain reference path
                euler = euler_from_quaternion(quaternion)
                psi = -euler[2]
                # Transform Twist to Odometry Frame
                rotMat = np.array([[math.cos(psi), -math.sin(psi)], [math.sin(psi), math.cos(psi)]])
                velVec = np.array([[msg.twist.twist.linear.x], [msg.twist.twist.linear.y]])
                transformed_twist = np.matmul(rotMat, velVec)
                if self.DEBUG:
                    rospy.loginfo("Rotational Matrix: %s", rotMat)
                    rospy.loginfo("Shape of Rotational Matrix: %s", rotMat.shape)
                    rospy.loginfo("Velocity Vector: %s", velVec)
                    rospy.loginfo("Shape of Velocity Vector: %s", velVec.shape)
                    rospy.loginfo("Transformed Twist: %s", transformed_twist)
                self.cur_speed = [transformed_twist[0], transformed_twist[1]]
        else:
            self.odom_wait_count += 1
                
    def path_pub(self):
        if self.send_goal_to_move_base:
            self.halt_car()
            uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
            roslaunch.configure_logging(uuid)
            launch = roslaunch.parent.ROSLaunchParent(uuid, [self.ros_global_planner_launch_dir])
            launch.start()
            while not self.client.wait_for_server(rospy.Duration.from_sec(5.0)):
                rospy.logwarn("Server not available, retrying again...")
                self.halt_car()
                rospy.sleep(1)
                rospy.logwarn("####################### RESTARTING THE MOVE BASE CLIENT #######################")
                launch.shutdown()
                launch = roslaunch.parent.ROSLaunchParent(uuid, [self.ros_global_planner_launch_dir])
                launch.start()
            rospy.loginfo("Move Base Client is Ready")
            rospy.loginfo("Sending Goal to Move Base!")
            # Sending randomized goal points
            # Top-left corner goal points (staright)
            goal_points = [-8, 5.044, -1.58]
            # Bottom-left corner goal points (straight path)
            goal_points = [-17.744, -12.153, -0.008]
            if self.ref_path_load_from_file:
                goal_points = [self.goal_pose[0], self.goal_pose[1], self.ref_yaw[-1]]
            if self.train_rl_model:
                self.halt_car()
                sample_y_pos = np.random.uniform(0.5, -1.5)
                goal_points = [23.5, sample_y_pos, -0.0162]
            if self.manually_set_goal:
                # Waiting path to be available by setting the goal manually
                while self.ref_x is None and self.ref_y is None and self.ref_yaw is None:
                    self.halt_car()
                    rospy.logwarn("Waiting for the path to be available!")
                    rospy.logwarn("Please set the goal manually!")
                    rospy.sleep(1)
            else:
                goal_points_msg = self.goal_utils.get_the_goal(goal_points)
                rospy.loginfo("Goal Points: " + str(goal_points))
                while not self.client.wait_for_server():
                    rospy.sleep(0.1)
                rospy.loginfo("Server is Ready!")
                self.client.send_goal(goal_points_msg)
            rospy.loginfo("Trajectory Generation Started!")
            self.send_goal_to_move_base = False
        if self.cur_pose is not None and self.cur_speed is not None and self.ref_x is not None and self.ref_y is not None and self.ref_path is not None and (self.first_time or self.first_control_received):
            # rospy.loginfo("Path Generation Started!")
            s0, l0 = self.XY_2_SL(self.cur_pose[0], self.cur_pose[1], calculate_cur_pos=True)
            self.cur_pos_in_frenet = s0
            # Assume no movement in the lateral direction
            l0 = 0.0
            v0y = 0.0
            if self.first_time:
                v0x = float(np.abs(self.cur_speed[0]))
                a0x = float((v0x - np.abs(self.init_speed[0])) / self.DT)
                self.first_time = False
            else:
                v0x = self.init_speed[0]
                a0x = self.init_accel[0]
            # a0x = self.init_accel[0]
            a0y = 0.0
            if self.DEBUG:
                rospy.loginfo("current speed: " + str(v0x) + " current acceleration: " + str(a0x) + " current lateral speed: " + str(v0y) + " current lateral acceleration: " + str(a0y))
            goal_reached, dist_to_goal = self.goal_reached_check()
            rospy.loginfo("Distance to Goal in Current Path in Frenet: " + str(dist_to_goal))
            self.goal_reached = goal_reached
            if not goal_reached and dist_to_goal > self.STOPPING_DIST_THRESHOLD:
                self.path = self.lattice_planner.lattice_planner_for_cruising(l0, v0y, a0y, s0, v0x, a0x, self.ref_path, cur_speed=self.cur_speed[0])
            elif not goal_reached and dist_to_goal <= self.STOPPING_DIST_THRESHOLD:     
                self.path = self.lattice_planner.lattice_planner_for_stopping(l0, v0y, a0y, s0, v0x, a0x, self.ref_path, self.goal_dist_in_frenet, cur_speed=self.cur_speed[0])
            else:
                rospy.loginfo("Goal Reached!")
                try:
                    # Send previous path and goal
                    ref_x = State()
                    ref_x.data = self.path.x
                    ref_y = State()
                    ref_y.data = self.path.y
                    ref_yaw = State()
                    ref_yaw.data = self.path.yaw
                    ref_v = State()
                    ref_v.data = [0.0] * len(self.path.x)
                    curv = State()
                    curv.data = [0.0] * len(self.path.x)
                    ref_path = Local_path()
                    ref_path.header.stamp = rospy.Time.now()
                    ref_path.ref_state.append(ref_x)
                    ref_path.ref_state.append(ref_y)
                    ref_path.ref_state.append(ref_yaw)
                    ref_path.ref_state.append(ref_v)
                    ref_path.ref_state.append(curv)
                    ref_path.goal_pose = self.goal_pose
                    ref_path.goal_reached = goal_reached
                    self.pub.publish(ref_path)
                    self.lattice_planner.prev_head = self.path.yaw[0]
                except Exception as e:
                    rospy.logerr("Failed to send the previous path!")
                    rospy.logerr(e)
                    pass
                # Break the loop
                rospy.sleep(1)
                return
            # Send the Goal Reached Message
            msg = Bool()
            msg.data = goal_reached
            self.goal_reached_mp.publish(msg)                
            try:
                # Calculate the desired length
                desired_length = max(len(self.path.x), len(self.path.y), len(self.path.yaw), len(self.path.s_v), len(self.path.curv))

                # Extend the paths if necessary
                path_attributes = [self.path.x, self.path.y, self.path.yaw, self.path.s_v, self.path.curv]
                for attr in path_attributes:
                    if len(attr) < desired_length:
                        attr.extend([attr[-1]] * (desired_length - len(attr)))
                if self.DEBUG:
                    rospy.loginfo("Length of Path After x: %s", len(self.path.x))
                    rospy.loginfo("Length of Path After y: %s", len(self.path.y))
                # Clip to make sure the vref is within the target speed limits
                self.path.s_v = np.clip(self.path.s_v, -self.lattice_planner.TARGET_SPEED, self.lattice_planner.TARGET_SPEED)
                rospy.loginfo("Generated Path!, Length of Path: %s", len(self.path.x))
                self.path.curv = np.clip(self.path.curv, -self.lattice_planner.MAX_CURVATURE, self.lattice_planner.MAX_CURVATURE)
                # Convert to ROS Path
                ref_x = State()
                ref_x.data = self.path.x
                ref_y = State()
                ref_y.data = self.path.y
                ref_yaw = State()
                ref_yaw.data = self.path.yaw
                ref_v = State()
                ref_v.data = self.path.s_v
                curv = State()
                curv.data = self.path.curv
                if self.DEBUG:
                    rospy.loginfo("Length of ref x: %d", len(self.path.x))
                    rospy.loginfo("Length of ref y: %d", len(self.path.y))
                ref_path = Local_path()
                ref_path.header.stamp = rospy.Time.now()
                ref_path.ref_state.append(ref_x)
                ref_path.ref_state.append(ref_y)
                ref_path.ref_state.append(ref_yaw)
                ref_path.ref_state.append(ref_v)
                ref_path.ref_state.append(curv)
                ref_path.goal_pose = self.goal_pose
                ref_path.goal_reached = goal_reached
                if self.STOPPING_DIST_THRESHOLD < dist_to_goal:    
                    if self.prefer_feedback:
                        self.init_speed = [float(self.cur_speed[0]), float(self.cur_speed[1])]
                    else:
                        self.init_speed = [self.path.s_v[1], self.cur_speed[1]]
                else:
                    if self.prefer_feedback:
                        self.init_speed = [float(self.cur_speed[0]), float(self.cur_speed[1])]
                    else:
                        self.init_speed = [self.path.s_v[1], self.cur_speed[1]]
                        # self.init_speed = [float(self.cur_speed[0]), float(self.cur_speed[1])]
                self.init_accel = [self.path.s_a[1], 0]
                self.pub.publish(ref_path)
                if not self.first_control_received:
                    self.initial_reference_path = ref_path
                # Set previous heading
                self.lattice_planner.prev_head = self.path.yaw[0]
            except Exception as e:
                rospy.logerr(e)
                rospy.logfatal("Failed to generate path!")
                pass
        else:
            if not self.first_control_received:
                rospy.logwarn("Waiting for the first control signal from Controller!")
                # If the first control signal is not received after some time, then halt the car and send 
                if self.first_control_not_received_count > 3:
                    # self.halt_car()
                    self.pub.publish(self.initial_reference_path)
                    rospy.logwarn("Path Republished!")
                self.first_control_not_received_count += 1
                # Check also the first control obtained file, if exists then set the first control received to True
                if os.environ.get('BASE_DIR') is not None:
                    first_control_file = os.environ.get('BASE_DIR') + '/car_ws/src/motion_planner/scripts/control_flag/first_control_obtained.txt'
                else:
                    first_control_file = '/home/vialab/car_ws/src/motion_planner/scripts/control_flag/first_control_obtained.txt'
                if os.path.exists(first_control_file):
                    self.first_control_received = True
                    rospy.loginfo("First Control Received!")
                    # Remove the file
                    os.remove(first_control_file)
                    rospy.loginfo("First Control File Removed!")
            else:
                rospy.logwarn("No reference path or current pose available!")
    
    def goal_reached_check(self):
        if self.cur_pose is not None:
            # Simple Euclidean Distance
            distance_euclidian = math.sqrt((self.cur_pose[0] - self.goal_pose[0])**2 + (self.cur_pose[1] - self.goal_pose[1])**2)
            distance_euclidian = np.round(distance_euclidian, 2)
            # rospy.loginfo("Euclidean Distance to Goal: " + str(distance_euclidian))
            dx = np.abs(self.cur_pose[0] - self.goal_pose[0])
            dy = np.abs(self.cur_pose[1] - self.goal_pose[1])
            dx = np.round(dx, 2)
            dy = np.round(dy, 2)
            # if distance < self.GOAL_THRESHOLD:
            #     return True, distance
            # else:
            #     return False, distance
            # Measurement based on the frenet coordinates
            distance = np.abs(self.goal_distance_max - self.cur_pos_in_frenet)
            distance = np.round(distance, 2)
            # if distance <= self.GOAL_THRESHOLD and (distance_euclidian <= self.GOAL_THRESHOLD or (dx <= self.GOAL_THRESHOLD and dy <= self.GOAL_THRESHOLD)):
            if (distance_euclidian <= self.GOAL_THRESHOLD or (dx <= self.GOAL_THRESHOLD and dy <= self.GOAL_THRESHOLD)):
                return True, distance
            else:
                return False, distance
        else:
            return False, None

    #TODO: Convert Cartesian Coordinates to Frenet Coordinates (XY_2_SL) DONE
    def XY_2_SL(self, x, y, calculate_cur_pos=False):
        # project x, y to the reference line
        cur_pos = Point(x, y)
        ref_x = self.ref_x
        ref_y = self.ref_y
        zip_ref = list(zip(ref_x, ref_y))
        ref_line = LineString(zip_ref)
        # Calculate the Distance from start of the reference line to the current position (S)
        s = ref_line.project(cur_pos)
        # Make sure that previous s is not greater than the current s (which can cause the car reverse to the previous passed trajectory)
        # Only when converting the current position to the frenet coordinates not when calculating the specific point on the reference line
        if calculate_cur_pos:
            if s <= self.prev_s_point:
                s = self.prev_s_point
            else:
                self.prev_s_point = s
        projected_point = ref_line.interpolate(s).coords
        projected_point = list(projected_point)
        # Calculate the Lateral Distance from the reference line (L)
        # Get the tangent to determine the sign of the lateral distance
        projected_point_1 = ref_line.interpolate(s + 0.1).coords
        projected_point_1 = list(projected_point_1)
        # Calc Determinant
        projected_point_to_cur_pos = np.array([x - projected_point[0][0], y - projected_point[0][1]])
        projected_point_to_cur_pos_1 = np.array([x - projected_point_1[0][0], y - projected_point_1[0][1]])
        det = np.linalg.det([projected_point_to_cur_pos, projected_point_to_cur_pos_1])
        # Determine the sign of the lateral distance
        if det > 0:
            l = cur_pos.distance(Point(projected_point))
        else:
            l = -cur_pos.distance(Point(projected_point))
        return s, l
    
    def halt_car(self):
        msg = AckermannDriveStamped()
        msg.header.stamp = rospy.Time.now()
        msg.drive.steering_angle = 0.0
        msg.drive.speed = 0.0
        self.halting_pub.publish(msg)
        rospy.loginfo("Car halted!")
        
    @staticmethod
    def segmentPathByCuspPoint(path_x, path_y, minCuspPointAngleDeg=90):
        path = np.column_stack((path_x, path_y))
        pathLen = path.shape[0]
        if pathLen < 3:
            return [path], []
        lineSegAngle = np.array([GeometryHelper.getAngleBetweenTwoLineSeg(p1, p2, p2, p3) for p1, p2, p3 in zip(path[:-2,0:2], path[1:-1,0:2], path[2:,0:2])])
        cuspPointIdx = np.array(np.where(lineSegAngle >= np.deg2rad(minCuspPointAngleDeg))[0]) + 1
        pathPoint = np.array([0] + list(cuspPointIdx) + [pathLen-1])

        pathList = []
        for i in range(len(pathPoint)-1):
            pathList.append(path[pathPoint[i]:pathPoint[i+1]+1,:])
        
        return pathList, cuspPointIdx
    

def str2bool(v):
    if isinstance(v, bool):
       return v
    if v.lower() in ('yes', 'true', 't', 'y', '1'):
        return True
    elif v.lower() in ('no', 'false', 'f', 'n', '0'):
        return False
    else:
        raise argparse.ArgumentTypeError('Boolean value expected.')
    

if __name__ == '__main__':
    rospy.init_node('motion_planner', anonymous=True)
    via_roslaunch = False
    # if not via_roslaunch:
    parser = argparse.ArgumentParser()
    parser.add_argument('--eval_mode', type=str2bool, default=False, help='Evaluation Mode')
    parser.add_argument('--file_path_dir', type=str, default=None, help='File Path Directory')
    args, unknown = parser.parse_known_args()
    file_path_dir = args.file_path_dir
    eval_mode = args.eval_mode
    if file_path_dir is not None:
        rospy.loginfo("File Path Directory: " + file_path_dir)
    else:
        # Define the reference path directory
        file_path_dir = None
    motion_planner = MotionPlannerNode(file_path=file_path_dir, eval_mode=eval_mode)
    rospy.sleep(3)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        motion_planner.path_pub()
        rate.sleep()
    rospy.spin()
