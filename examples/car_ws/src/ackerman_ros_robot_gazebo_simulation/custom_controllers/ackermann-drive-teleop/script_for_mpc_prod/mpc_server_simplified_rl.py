import socket
import timeit
import argparse
import os
import yaml
import time
import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
import datetime

import LinearMPCwithMP as LinearMPCwithMP

import signal
import sys
import traceback
import atexit

# Global variables
path_available = False
data = np.zeros((100, 1))
new_data = 0


def get_car_ws_path():
    """
    Determines the car_ws path by navigating up from the current file's location.
    """
    current_file_path = os.path.abspath(__file__)
    current_dir = os.path.dirname(current_file_path)
    src_dir = os.path.abspath(os.path.join(current_dir, '..', '..', '..', '..'))
    car_ws_path = os.path.abspath(os.path.join(src_dir, '..'))
    return car_ws_path


class ControlCommand:
    # Get the car_ws path
    car_ws_path = get_car_ws_path()
    def __init__(self, file_path_name = None, horizon_type="nonuni_sparse_var", eval_path_folder=None):
        # Load vehicle_params.yaml from the project root (five levels up)
        script_dir = os.path.dirname(os.path.abspath(__file__))
        yaml_file_path = os.path.abspath(
            os.path.join(script_dir, '../../../../../../..', 'vehicle_params.yaml')
        )
        with open(yaml_file_path, 'r') as f:
            self.vehicle_params = yaml.safe_load(f)
        
        self.HEADER_SIZE = 10
        self.ipaddress = None
        self.CLIENT_ADDRESS = None
        self.PORT = 12345
        self.server_address = (self.ipaddress, self.PORT)
        self.BUFFER_SIZE = 60000
        self.u0 = None
        self.t = 0
        self.horizon_type = horizon_type
        self.eval_path_folder = eval_path_folder
        
        print(f"Horizon type: {self.horizon_type}")
        print(f"Current Evaluation Path Folder: {self.eval_path_folder}")
        self.mpc = LinearMPCwithMP.LinearMPC(N=9, type=self.horizon_type) # N=9 (Current step 1 + 8 future steps)
        print(f"Weighting matrices: {self.mpc.Q_curve}")
        self.vehicle_state = LinearMPCwithMP.VehicleState()
        
        # Set the vehicle parameters in the MPC class
        self.mpc.WB = self.vehicle_params['wheelbase']
        self.mpc.MAX_STEER = self.vehicle_params['steering_angle_limit_rad']
        self.mpc.MAX_DSTEER = self.vehicle_params['steering_angle_rate_limit_rad_s']
        
        self.server_dt = self.mpc.DT
        self.plot_now = False
        self.plot_interval = 10
        self.plot_goal = 0
        self.second_path = False
        self.trajRef = []
        self.steer = []
        self.state_v = []
        self.input_a = []
        self.state_x = []
        self.vel_state = []
        self.state_y = []
        self.state_yaw = []
        self.ref_yaw = []
        self.ref_x = []
        self.ref_y = []
        self.ref_v = []
        self.state_cost = []
        self.control_cost = []
        self.control_diff_cost = []
        self.objective_cost_total = []
        self.lateral_error_raw = []
        self.path_curvature = []
        self.dt_array = []
        self.oa = None
        self.od = None
        self.ALPHA = None
        self.BETA = None
        self.prev_oa = 0
        self.ox = []
        self.oy = []
        self.control_send = np.zeros((5, 1))
        self.ov = []
        self.ojerk = 0
        self.max_time = 0
        self.oyaw = []
        
        # Simulation termination conditions
        self.vel_stuck_count = 0
        self.vel_stuck_count_threshold = 20
        self.heading_error_threshold = np.pi/2
        self.heading_error_counter = 0
        self.heading_error_counter_threshold = 30
        self.vel_threshold_to_terminate = 0.01
        self.accel_threshold_to_terminate = 0.1
        self.distance_error_threshold = 0.9 #1.0 # meters
        self.distance_error_violation = False
        self.current_lat_distance_error = 0.0 # meters
        self.mpc_solving_failure_threshold = 10
        self.mpc_solving_failure_counter = 0
        
        self.prefer_state_feedback = True
        self.nominal_speed = 0.0
        
        self.is_sim = True
        self.send_first_control = True
        self.use_motion_planner = True
        self.goal_is_reached = False
        self.current_goal = None
        self.mpc.use_motion_planner = self.use_motion_planner
        self.goal_is_reached_flag_array = []
        self.receive_curvature = True
        
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.ipaddress = socket.gethostname()
        # Bind the socket to the port
        self.server_address = (self.ipaddress, self.PORT)
        self.server_address = self.server_address
        print('starting up on {} port {}'.format(*self.server_address))
        # Add reuse address option
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind(self.server_address)
        
        if self.is_sim:
            # Read the parameters from the yaml file for the simulation
            delay_yaml_file_dir = os.path.join(self.car_ws_path, "src/ackerman_ros_robot_gazebo_simulation/rbcar_sim/rbcar_robot_control/config/delay_params.yaml")
            motion_planner_yaml_file_dir = os.path.join(self.car_ws_path, "src/motion_planner/cfg/motion_planner.yaml")
            with open(delay_yaml_file_dir, 'r') as stream:
                try:
                    self.delay_params = yaml.safe_load(stream)
                except yaml.YAMLError as exc:
                    print(exc)
            self.throttle_delay = self.delay_params['time_delay_throttle']
            self.steer_delay = self.delay_params['time_delay_steering']
            print(f"Throttle delay: {self.throttle_delay}")
            print(f"Steering delay: {self.steer_delay}")
            
            #TODO: Refactor this to be more clean
            with open(motion_planner_yaml_file_dir, 'r') as stream:
                try:
                    self.motion_planner_params = yaml.safe_load(stream)
                except yaml.YAMLError as exc:
                    print(exc)
            self.mpc.use_motion_planner = self.motion_planner_params['use_motion_planner']

        self.mpc.setup_mpc()

        self.header_measurement = ["odom_x", "odom_y", "odom_vel", "odom_yaw", "goal_is_reached"]
        self.header_mpcOutput = ["mpc_vel", "mpc_accel", "mpc_steer", "dt_array", "path_curv"]
        self.header_mpcCost = ["lateral_error_raw", "state_cost", "control_cost", "control_diff_cost", "objective_cost_total"]
        self.header_trajPlanner = ["path_x", "path_y", "path_vel", "path_yaw", "hor_x", "hor_y", "hor_yaw"]
        for i in range(1, self.mpc.N):
            self.header_trajPlanner.extend(["path_x_" + str(i), "path_y_" + str(i), "path_vel_" + str(i), "path_yaw_" + str(i), "hor_x_" + str(i), "hor_y_" + str(i), "hor_yaw_" + str(i)])
        # self.header = header_measurement + header_mpcOutput + header_trajPlanner

        self.file_path_name = file_path_name
        self.save_path_dir = None
        atexit.register(self.save_data)
        # for saving data when KeyboardInterrupt
        signal.signal(signal.SIGINT, lambda signum, frame: self.save_data() or sys.exit(0))
        signal.signal(signal.SIGTERM, lambda signum, frame: self.save_data() or sys.exit(0))

    def save_data(self):
        minLen = []
        for i in range(len(self.trajRef)):
            minLen.append(len(self.trajRef[i]))
        
        if minLen != []:
            minLen = min(minLen)

            dataFrame = {}

            data_measurement = [self.state_x, self.state_y, self.vel_state, self.state_yaw, self.goal_is_reached_flag_array]
            data_mpcCost = [self.lateral_error_raw, self.state_cost, self.control_cost, self.control_diff_cost, self.objective_cost_total]
            data_mpcOutput = [self.state_v, self.input_a, self.steer, self.dt_array, self.path_curvature]

            for i in range(len(self.header_measurement)):
                dataFrame[self.header_measurement[i]] = data_measurement[i][:minLen]

            for i in range(len(self.header_mpcCost)):
                dataFrame[self.header_mpcCost[i]] = data_mpcCost[i][:minLen]

            for i in range(len(self.header_mpcOutput)):
                dataFrame[self.header_mpcOutput[i]] = data_mpcOutput[i][:minLen]

            for i in range(len(self.header_trajPlanner)):
                dataFrame[self.header_trajPlanner[i]] = self.trajRef[i][:minLen]

             # Saving path directory
            if self.file_path_name is None:
                cur_date = datetime.datetime.now()
                cur_date = cur_date.strftime("%Y%m%d%H%M")
                self.save_path_dir = f"./data/Sim Results/{cur_date}_DT_max_{self.mpc.DT_MAX}_type_{self.mpc.type}_alpha_{self.ALPHA}_beta_{self.BETA}_throttleDelay_{self.throttle_delay}_steeringDelay_{self.steer_delay}.csv"
            else:
                if self.goal_is_reached_flag_array[-1] == 1:
                    verdict = "success"
                else:
                    verdict = "failure"
                if self.horizon_type == "nonuni_sparse_var":
                    horizon_type = "nonunisparsevar"
                else:
                    horizon_type = self.horizon_type
                folder_for_save = "MPCSimulationRunner"
                save_path_dir = os.path.join(self.car_ws_path, f"src/{folder_for_save}/data/eval_test")
                save_path_dir_current_folder = os.path.join(save_path_dir, self.eval_path_folder)
                self.save_path_dir = f"{save_path_dir_current_folder}/{horizon_type}_{verdict}_{self.file_path_name}"
            
            pd.DataFrame(dataFrame).to_csv(self.save_path_dir)
            print(f"Data saved to {self.save_path_dir}")
            
    def sigint_handler(self, signal, frame):
        print ('KeyboardInterrupt is caught')
        self.save_data()
        sys.exit(0)
        
    def sigterm_handler(self, signal, frame):
        print ('SIGTERM is caught')
        self.save_data()
        sys.exit(0)

    def recv_path(self, connection):
        global path_available
        global data
        global new_data
        global time_receive

        msg_len = 0
        new_msg = True
        full_msg = b''

        msg = connection.recvfrom(self.BUFFER_SIZE)
        time_receive = time.time()
        self.CLIENT_ADDRESS = msg[1]
        msg = msg[0]

        if new_msg:
            # print("in new message")
            msg_len = int(msg[:self.HEADER_SIZE])
            full_msg += msg

        if len(full_msg) - self.HEADER_SIZE == msg_len:
            new_data = np.frombuffer(full_msg[self.HEADER_SIZE:], dtype=np.float32)
            # print("Full msg recvd")

            new_msg = True
            full_msg = b''
            path_available = True

    def plotter(self, path_x, path_y, path_yaw, ax, ay, v, d, yaw, ox=None, oy=None, ov=None, oyaw=None):
        t = np.linspace(0, self.t, self.t)
        t_v = np.linspace(0, self.t, self.t)

        plt.close("all")
        plt.subplots()
        plt.plot(path_x, path_y, "-r", label="Trajectory Path")
        plt.plot(ax, ay, "-b", label="MPC Tracking")
        # plt.plot(ox, oy, "xr", label="Prediction Horizon")
        plt.grid(True)
        plt.axis("equal")
        plt.title("MPC Path Tracking Result")
        plt.xlabel("x[m]")
        plt.ylabel("y[m]")
        plt.legend()

        plt.subplots()
        plt.plot(t, path_yaw, "-r", label="ref heading angle")
        plt.plot(t, yaw, "-b", label="heading angle")
        plt.legend()
        plt.grid(True)

        plt.title("Vehicle Heading vs Ref Heading")
        plt.xlabel("Step")
        plt.ylabel("Angle [rad]")

        plt.subplots()
        plt.plot(t_v, self.vel_state, "-r", label="speed from odom")
        plt.plot(t_v, self.state_v, "-b", label="speed from MPC")
        plt.plot(t_v, self.ref_v1, "-y", label="speed reference (k+1)")
        plt.grid(True)
        plt.title("Speed Profile")
        plt.xlabel("Step")
        plt.ylabel("Speed [ms]")
        plt.legend()

        plt.subplots()
        plt.plot(t_v, d, "-r", label="steering angle")
        plt.grid(True)
        plt.title("Steering Angle")
        plt.xlabel("Step")
        plt.ylabel("Angle [rad]")

        plt.show()

    def send_control_inputs(self, connection):
        global path_available
        global data
        global new_data
        global time_receive

        try:
            if path_available:
                path_available = False
                data = new_data

            path_len = int(data[0])
            sign = np.array(0, dtype=np.float32)

            # Path preprocessing
            if not self.use_motion_planner:
                path = np.array_split(data[1:path_len + 1], 3)
                vehicle_state = data[path_len + 1:].flatten()
                vehicle = LinearMPCwithMP.VehicleState(vehicle_state[0], vehicle_state[1], vehicle_state[2], vehicle_state[3])
            else:
                if self.receive_curvature:
                    path = np.array_split(data[1:path_len + 1], 5)
                else:
                    path = np.array_split(data[1:path_len + 1], 4)
                vehicle_state = data[path_len + 1: path_len + 5].flatten()
                # print(f"Vehicle state: {vehicle_state}")
                # print(f"Path: {path}")
                print(data[path_len + 5:].flatten())
                self.current_goal = data[path_len + 5: path_len + 7].flatten() 
                self.goal_is_reached = bool(data[-1])
                self.mpc.goal = self.current_goal
                print(f"Current Goal: {self.mpc.goal}")
                print(f"Goal is reached: {self.goal_is_reached}")
                # Update the vehicle state
                self.update_vehicle_state(vehicle_state)
                vehicle = self.vehicle_state
            
            ox, oy, oyaw, oa, od, ovs, isstop, isgoal, xref = self.mpc.solve_mpc(vehicle, path, self.oa, self.od, goal_reached_from_motion_planner=self.goal_is_reached, receive_curvature=self.receive_curvature)                
            self.state_x.append(vehicle_state[0].item(0))
            self.state_y.append(vehicle_state[1].item(0))
            self.state_yaw.append(vehicle_state[2])
            self.vel_state.append(vehicle_state[3])
            self.path_curvature.append(self.mpc.cur_path_curve)
            self.dt_array.append(self.mpc.DT_array)
            self.ALPHA = self.mpc.ALPHA
            self.BETA = self.mpc.BETA
            self.send_first_control = self.mpc.send_first_control
            # Append the lateral error and cost values
            self.lateral_error_raw.append(self.mpc.lat_error)
            self.state_cost.append(self.mpc.state_cost)
            self.control_cost.append(self.mpc.control_cost)
            self.control_diff_cost.append(self.mpc.control_diff_cost)
            self.objective_cost_total.append(self.mpc.objective_cost)

            xref = np.vstack((xref, ox))
            xref = np.vstack((xref, oy))
            xref = np.vstack((xref, oyaw))
            xref = xref.T.tolist()
            # print(f"Reference Length {xref}")
            tempData = []
            for i in range(len(xref)):
                tempData.extend(xref[i])
            
            if len(self.trajRef) == 0:
                for i in range(len(tempData)):
                    self.trajRef.append([tempData[i]])
            else:
                for i in range(len(tempData)):
                    self.trajRef[i].append(tempData[i])

            # Send control inputs to server
            if oa is not None and od is not None:
                self.oa = oa
                self.od = od
            else:
                self.oa = None
                self.od = None
                oa = np.zeros((1, self.mpc.N - 1)).flatten()
                od = np.zeros((1, self.mpc.N - 1)).flatten()
                # Update the MPC solving failure counter
                self.mpc_solving_failure_counter += 1

            if (isstop and isgoal) or self.goal_is_reached:
                ovs = 0
                oa_send = 0.0
                od_send = od[0]
                sign = 1
            else:
                if not self.send_first_control:
                    oa_send = oa[self.ALPHA]
                    od_send = od[self.BETA]
                else:
                    oa_send = oa[0]
                    od_send = od[0]
            
            if not self.prefer_state_feedback:
                self.nominal_speed += oa_send * self.server_dt
                ovs = self.nominal_speed
                print("Nominal speed: ", ovs)
            
            # Velocity clipping
            if ovs > self.mpc.MAX_SPEED:
                ovs = self.mpc.MAX_SPEED
            elif ovs < self.mpc.MIN_SPEED:
                ovs = self.mpc.MIN_SPEED
            
            self.prev_oa = oa
                        
            # Set the control inputs
            self.control_send[0] = oa_send
            self.control_send[1] = od_send
            self.control_send[2] = ovs
            self.control_send[3] = sign
            self.control_send[4] = 0.0
            self.control_send = self.control_send.astype(dtype=np.float32) * 1.0
            
            data = self.control_send.tobytes()
            msg = bytes(f'{len(data):<{self.HEADER_SIZE}}', "utf-8") + data
            connection.sendto(msg, self.CLIENT_ADDRESS)

            self.state_v.append(ovs)
            self.input_a.append(oa_send)
            self.steer.append(od_send)
            self.goal_is_reached_flag_array.append(sign)

            # if goal is reached terminate the program and save the data
            if sign == 1:
                self.save_data()
                sys.exit(0)
            
            # Check violation of simulation termination conditions
            if np.abs(vehicle_state[3]) < self.vel_threshold_to_terminate:
                if np.abs(oa_send) < self.accel_threshold_to_terminate:
                    self.vel_stuck_count += 1 
                    
            # Check violation of lateral distance error
            if self.mpc.lat_error > self.distance_error_threshold:
                print(f"Current lateral distance error: {self.mpc.lat_error}")
                self.distance_error_violation = True
            
            # if np.abs(vehicle_state[2] - xref[0][3]) > self.heading_error_threshold:
            #     self.heading_error_counter += 1
            
            # if self.vel_stuck_count > self.vel_stuck_count_threshold or self.heading_error_counter > self.heading_error_counter_threshold or self.mpc_solving_failure_counter > self.mpc_solving_failure_threshold or self.distance_error_violation:
            timeout_file_dir = os.path.join(self.car_ws_path, "src/MPCSimulationRunner/data/eval_test/timeout.txt")
            if self.vel_stuck_count > self.vel_stuck_count_threshold or self.mpc_solving_failure_counter > self.mpc_solving_failure_threshold or self.distance_error_violation:
                # Create a timeout file to save the data and exit
                with open(timeout_file_dir, "w") as f:
                    f.write("Timeout")        
            
            # Check if the timeout file exists (kill the program if it does)
            if os.path.exists(timeout_file_dir):
                self.save_data()
                sys.exit(0)
                
        except Exception as e:
            print("Error in sending control inputs")
            print(e)
            print(traceback.format_exc())
            sys.exit(1)
            
    def update_vehicle_state(self, current_state):
        self.vehicle_state.x = current_state[0]
        self.vehicle_state.y = current_state[1]
        self.vehicle_state.yaw = current_state[2]
        self.vehicle_state.v = current_state[3]
        pass
    

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="MPC server for the vehicle")
    parser.add_argument("--horizon_type", type=str, default="nonuni_sparse_var", help="Type of the horizon")
    parser.add_argument("--use_motion_planner", type=bool, default=True, help="Use the motion planner")
    parser.add_argument("--is_sim", type=bool, default=True, help="Is the simulation")
    parser.add_argument("--receive_curvature", type=bool, default=True, help="Receive curvature")
    parser.add_argument("--eval", type=bool, default=False, help="Evaluate the MPC controller")
    parser.add_argument("--file_path_name", type=str, default=None, help="File path name")
    parser.add_argument("--eval_path_folder", type=str, default=None, help="Evaluation path folder")
    parser.add_argument("--noise_on_state", type=bool, default=False, help="Add noise to the state")
    parser.add_argument("--noise_on_position", type=float, default=0.0, help="SD of Noise on position x and y state (m)")
    parser.add_argument("--noise_on_velocity", type=float, default=0.0, help="SD of Noise on velocity state (m/s)")
    parser.add_argument("--noise_on_yaw", type=float, default=0.0, help="SD of Noise on yaw state (rad)")
    
    args = parser.parse_args()
    file_path_name = args.file_path_name
    horizon_type = args.horizon_type
    comm = ControlCommand(file_path_name=file_path_name, horizon_type=horizon_type, eval_path_folder=args.eval_path_folder)
    signal.signal(signal.SIGTERM, comm.sigterm_handler)

    while True:
        # Wait for a connection
        comm.recv_path(comm.sock)
        comm.send_control_inputs(comm.sock)        
