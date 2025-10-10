import argparse
import subprocess
import sys
import os
import time
import logging
import datetime

import yaml

from typing import Dict, List, Any

# Add the path to the update scripts to the system path
script_dir = os.path.dirname(os.path.abspath(__file__))
config_path = os.path.join(script_dir, '..', '..', '..', 'config')
sys.path.append(config_path)

# Try to import new unified script, fall back to old one if not available
try:
    import update_simulation_from_yaml as update_script
    USING_NEW_UPDATER = True
except ImportError:
    import update_xacro_from_yaml as update_script
    USING_NEW_UPDATER = False

# Import TmuxManager for process management
try:
    from tmux_manager import TmuxManager
    USING_TMUX = True
except ImportError:
    USING_TMUX = False
    logging.warning("TmuxManager not available, ROS processes must be launched externally")

def shutdown_sim(process_list, gracetime_s=0.1):
    """Shuts down a list of processes gracefully."""
    for process in process_list:
        if process.poll() is not None:
            logging.debug(
                "Process %d was shut down already with returncode %d.",
                process.pid,
                process.returncode,
            )
            continue
        process.terminate()
        try:
            process.wait(gracetime_s)
        except subprocess.TimeoutExpired:
            logging.warning(
                "Process %d did not shut down gracefully, sending kill.",
                process.pid,
            )
            process.kill()
        assert (
            process.poll() and process.returncode is not None
        ), "Process could not be killed."
    return


class RunandGatherResults:
    def __init__(self, config_file: str, current_workspace_dir: str, current_dataset_dir: str, noisy_odom_params: Dict[str, Any], dataset_class_params: Dict[str, Any], run_params: Dict[str, Any], vehicle_params_list: List[Dict[str, Any]], environment_params_list: List[Dict[str, Any]] = None) -> None:
        self.run_command = config_file['run_command']
        self.run_command_mpc_server = self.run_command['mpc_server']
        self.run_command_mpc_node = self.run_command['mpc_node']
        self.run_command_motion_planner = self.run_command['motion_planner']
        self.run_unpause_physics = self.run_command['unpause_physics']
        self.current_workspace_dir = current_workspace_dir
        self.current_dataset_dir = current_dataset_dir
        self.timeout = float(config_file['timeout'])
        self.path_files = config_file['path_files']
        self.user_prefix = config_file['user_prefix_path']
        self.workspace_list = config_file['workspace_list'][0]
        self.sleep_list = config_file['sleep_list']
        self.path_files_list = self.current_dataset_dir + '/' + self.path_files
        self.path_files_list = os.listdir(self.path_files_list)
        self.path_files_list = [path_file for path_file in self.path_files_list if path_file.endswith('.csv')]
        self.count_path_files = len(self.path_files_list)
        self.eval_results_path = config_file['eval_results_path']
        self.eval_folder_count = 0
        self.current_time = datetime.datetime.now().strftime("%Y%m%d_%H%M")
        self.eval_results_path = self.current_workspace_dir + self.eval_results_path
        self.eval_results_path_target = os.path.join(self.eval_results_path, self.current_time)
        self.save_results_in_eval_test_folder = True
        self.process_list = []
        self.gazebo_reset_command = "rosservice call /gazebo/reset_simulation {}"
        self.vehicle_params_list = vehicle_params_list

        # Environment parameters (optional - for road friction, wind, etc.)
        if environment_params_list:
            self.environment_params_list = environment_params_list
        else:
            # Default: no environment changes (use current settings)
            self.environment_params_list = [None] * len(vehicle_params_list)

        if not noisy_odom_params:
            # Default: all noise disabled, stddevs can be set to 0 or any default
            self.noisy_odom_params = [{
                'enable_x': False,
                'enable_y': False,
                'enable_heading': False,
                'enable_speed': False,
                'x_stddev': 0.0,
                'y_stddev': 0.0,
                'heading_stddev': 0.0,
                'speed_stddev': 0.0
            }]
        else:
            self.noisy_odom_params = noisy_odom_params

        if not dataset_class_params:
            self.dataset_class_params = [{
                'enable_easy': True,
                'enable_moderate': True,
                'enable_hard': True
            }]
        else:
            self.dataset_class_params = dataset_class_params

        if not run_params:
            self.run_params = [{
                'which_mpc': ['uniform', 'fsmpc']
            }]
        else:
            self.run_params = run_params

        if not os.path.exists(self.eval_results_path_target):
            os.mkdir(self.eval_results_path_target)

        console_handler = logging.StreamHandler()
        console_handler.setLevel(logging.INFO)
        formatter = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s')
        console_handler.setFormatter(formatter)
        self.logger = logging.getLogger(__name__)
        self.logger.setLevel(logging.INFO)
        self.logger.addHandler(console_handler)
        self.logger.info("Run and Gather Results")
        self.logger.info(f"Number of path files: {self.count_path_files}")
        self.logger.info(f"Path files: {self.path_files_list}")
        self.logger.info(f"Sleep list: {self.sleep_list}")
        self.logger.info(f"Timeout: {self.timeout}")
        self.logger.info(f"Evaluation results path: {self.eval_results_path}")
        self.logger.info(f"Evaluation results path target: {self.eval_results_path_target}")

        # Initialize TmuxManager if available
        self.tmux = TmuxManager("mpc_experiment") if USING_TMUX else None
        self.ros_stack_launched = False

    def launch_ros_stack(self):
        """Launch ROS stack (roscore, Gazebo, robot control) using TmuxManager"""
        if not USING_TMUX or not self.tmux:
            self.logger.warning("TmuxManager not available, assuming ROS stack is already running")
            self.ros_stack_launched = True
            return True

        self.logger.info("="*60)
        self.logger.info("LAUNCHING ROS STACK WITH TMUXMANAGER")
        self.logger.info("="*60)

        try:
            # Start tmux session
            self.logger.info("Starting tmux session...")
            self.tmux.start_session()

            # Launch roscore
            self.logger.info("Launching roscore...")
            self.tmux.send_command("main", "source /opt/ros/melodic/setup.bash && roscore")
            time.sleep(2)

            # Wait for roscore to be ready
            if not self.tmux.wait_for_pattern("main", "started core service", timeout=10):
                self.logger.error("Roscore failed to start")
                return False

            # Launch Gazebo (ros-bringup)
            self.logger.info("Launching Gazebo simulation...")
            self.tmux.create_pane("gazebo", split_from="main", split_direction='h')
            gazebo_cmd = (
                f"source /opt/ros/melodic/setup.bash && "
                f"source {self.current_workspace_dir}/devel/setup.bash && "
                f"roslaunch rbcar_sim_bringup rbcar_complete_rl.launch"
            )
            self.tmux.send_command("gazebo", gazebo_cmd)
            time.sleep(5)

            # Launch robot control
            self.logger.info("Launching robot control...")
            self.tmux.create_pane("control", split_from="main", split_direction='v')
            control_cmd = (
                f"source /opt/ros/melodic/setup.bash && "
                f"source {self.current_workspace_dir}/devel/setup.bash && "
                f"roslaunch rbcar_control rbcar_control.launch"
            )
            self.tmux.send_command("control", control_cmd)
            time.sleep(3)

            self.logger.info("ROS stack launched successfully")
            self.ros_stack_launched = True
            return True

        except Exception as e:
            self.logger.error(f"Failed to launch ROS stack: {e}")
            return False

    def shutdown_ros_stack(self):
        """Shutdown ROS stack and cleanup tmux session"""
        if not self.ros_stack_launched:
            return

        self.logger.info("="*60)
        self.logger.info("SHUTTING DOWN ROS STACK")
        self.logger.info("="*60)

        if USING_TMUX and self.tmux:
            self.logger.info("Killing tmux session...")
            self.tmux.kill_session()
            time.sleep(2)

        self.ros_stack_launched = False

    def run_simulation(self):
        # Launch ROS stack if using TmuxManager
        if USING_TMUX:
            if not self.launch_ros_stack():
                self.logger.error("Failed to launch ROS stack, aborting")
                return

        try:
            for idx, vehicle_params in enumerate(self.vehicle_params_list):
                # Update environment parameters if specified
                environment_params = self.environment_params_list[idx] if idx < len(self.environment_params_list) else None

                if environment_params and USING_NEW_UPDATER:
                    self.logger.info(f"Updating environment parameters: {environment_params}")
                    environment_params_path = os.path.join(self.current_workspace_dir, 'config', 'environment_params.yaml')

                    # Read existing environment params
                    if os.path.exists(environment_params_path):
                        with open(environment_params_path, 'r') as f:
                            env_config = yaml.safe_load(f)

                        # Update with new values
                        if 'road_condition' in environment_params:
                            env_config['active_road_condition'] = environment_params['road_condition']
                            self.logger.info(f"Setting road condition: {environment_params['road_condition']}")

                        if 'wind_speed' in environment_params:
                            env_config['wind']['mean_speed_mps'] = environment_params['wind_speed']
                            self.logger.info(f"Setting wind speed: {environment_params['wind_speed']} m/s")

                        # Write updated environment params
                        with open(environment_params_path, 'w') as f:
                            yaml.dump(env_config, f)

                # Update vehicle parameters
                self.logger.info(f"Updating vehicle parameters: {vehicle_params}")
                vehicle_params_path = os.path.join(self.current_workspace_dir, 'config', 'vehicle_params.yaml')
                with open(vehicle_params_path, 'w') as f:
                    yaml.dump(vehicle_params, f)

                # Run update script (updates xacro files and/or world file based on which script is available)
                self.logger.info("Updating simulation files from YAML...")
                update_script.main()

                for noise_param, dataset_param, run_param in zip(self.noisy_odom_params, self.dataset_class_params, self.run_params):
                    # --- START: NEW PATH FILTERING LOGIC ---
                    current_path_files_list = []

                    # 1. Filter Easy paths (starting with 'E_')
                    if dataset_param.get('enable_easy', False):
                        current_path_files_list.extend([p for p in self.path_files_list if p.startswith('E_')])

                    # 2. Filter Moderate paths (starting with 'M_')
                    if dataset_param.get('enable_moderate', False):
                        current_path_files_list.extend([p for p in self.path_files_list if p.startswith('M_')])

                    # 3. Filter Hard paths (starting with 'H_')
                    if dataset_param.get('enable_hard', False):
                        current_path_files_list.extend([p for p in self.path_files_list if p.startswith('H_')])

                    if not current_path_files_list:
                        self.logger.warning(f"No path files selected for dataset parameters: {dataset_param}. Skipping run.")
                        continue # Skip this run if no paths are selected

                    # self.logger.info(f"Selected path files for this run: {current_path_files_list}")
                    # --- END: NEW PATH FILTERING LOGIC ---

                    noisy_odom_cmd = self.build_rosrun_cmd(noise_param)
                    self.logger.info(f"Running noisy odometry node with command: {noisy_odom_cmd}")

                    # 1. 딕셔너리(run_param)에서 'which_mpc' 키의 값(리스트)을 가져옵니다.
                    mpc_type_list = run_param.get('which_mpc', [])

                    # Start the noisy odom node as a subprocess
                    noisy_proc = subprocess.Popen(noisy_odom_cmd, shell=True, executable="/bin/bash")
                    # Create a folder based on the evaluation results path target and count of the noisy odom params
                    self.eval_folder_count += 1
                    self.eval_results_path_target_run = os.path.join(self.eval_results_path_target, 'run_' + str(self.eval_folder_count))
                    if not os.path.exists(self.eval_results_path_target_run):
                        os.mkdir(self.eval_results_path_target_run)


                    for i, path_file in enumerate(current_path_files_list):
                        self.logger.info(f"Running the simulation for path file: {path_file}")

                        for sampling_param in mpc_type_list:
                            self.process_list = []
                            run_command_dict = {
                                'mpc_node': self.run_command_mpc_node,
                                'motion_planner': self.run_command_motion_planner,
                                'mpc_server': self.run_command_mpc_server + ' --horizon_type ' + sampling_param + ' --file_path_name ' + path_file + ' --eval_path_folder ' + self.current_time
                            }
                            print(f"MPC Server: {run_command_dict['mpc_server']}")
                            idx = 0

                            time.sleep(3)
                            for key, value in run_command_dict.items():
                                if key == 'mpc_server':
                                    command = [f"cd {self.current_workspace_dir}",
                                            ". ~/anaconda3/etc/profile.d/conda.sh && conda activate mpc-gen",
                                            str(value)]
                                elif key == 'mpc_node':
                                    command = [f"source {self.current_workspace_dir}" + "/devel/setup.bash",
                                            f"sleep {self.sleep_list[idx]}",
                                            str(value) + ' --eval_mode True' + ' --file_path_name ' + path_file + ' --file_path_dir ' + self.eval_results_path_target_run \
                                            + ' --horizon_type ' + sampling_param
                                            ]
                                elif key == 'motion_planner':
                                    command = ["cd ",
                                            f"source {self.current_workspace_dir}" + "/devel/setup.bash",
                                            f"sleep {self.sleep_list[idx]}",
                                            str(value) + ' eval_mode:=true' + ' file_path:=' + path_file
                                            ]
                                else:
                                    command = [f"cd {self.current_workspace_dir}",
                                            f"sleep {self.sleep_list[idx]}",
                                            str(value)
                                            ]
                                print_output = True
                                stdout = sys.stdout if print_output else subprocess.DEVNULL
                                logging.info(f"Running command: {';'.join(command)}")
                                process = subprocess.Popen((';'.join(command)), shell=True, executable="/bin/bash")
                                self.process_list.append(process)
                                idx += 1

                            subprocess.run(self.run_unpause_physics, shell=True)
                            start_time = time.time()
                            what_to_kill_in_ros = "rosnode kill motion_planner_node"
                            what_to_kill_in_ros_mpc_node = "rosnode kill mpc_node"
                            fuse_udp = "fuser -k 12345/udp"
                            while time.time() - start_time < self.timeout:
                                if self.process_list[2].poll() is not None:
                                    self.process_list[2].terminate()
                                    self.logger.info("MPC server is done!")
                                    subprocess.run(fuse_udp, shell=True)
                                    subprocess.run(self.gazebo_reset_command, shell=True)
                                    subprocess.run(what_to_kill_in_ros, shell=True)
                                    subprocess.run(what_to_kill_in_ros_mpc_node, shell=True)
                                    time.sleep(1)
                                    break
                            else:
                                with open(os.path.join(self.eval_results_path, 'timeout.txt'), 'w') as f:
                                    f.write('Timeout has occurred!')
                                time.sleep(1)
                                self.logger.info("Timeout! Killing the processes")
                                self.process_list[2].terminate()
                                subprocess.run(fuse_udp, shell=True)
                                subprocess.run(self.gazebo_reset_command, shell=True)
                                subprocess.run(what_to_kill_in_ros, shell=True)
                                subprocess.run(what_to_kill_in_ros_mpc_node, shell=True)
                                time.sleep(1)

                            time.sleep(4)
                            if os.path.exists(os.path.join(self.eval_results_path, 'timeout.txt')):
                                os.remove(os.path.join(self.eval_results_path, 'timeout.txt'))
                            if self.save_results_in_eval_test_folder:
                                self.logger.info("Moving the results to the evaluation folder")
                                get_list_files = os.listdir(self.eval_results_path_target)
                                for file in get_list_files:
                                    if file.endswith('.csv'):
                                        os.rename(os.path.join(self.eval_results_path_target, file), os.path.join(self.eval_results_path_target_run, file))
                                with open(os.path.join(self.eval_results_path_target, 'path_dir.csv'), 'w') as f:
                                    f.write(self.path_files)

                                self.logger.info("Finished moving the results to the evaluation folder")
                        self.logger.info("Finished running the results gathering!")
                    self.logger.info("Finished running the simulation!")

                    # Terminate the noisy odom node after the simulation
                    self.logger.info("Terminating noisy odometry node.")
                    noisy_proc.terminate()
                    try:
                        noisy_proc.wait(timeout=5)
                    except subprocess.TimeoutExpired:
                        self.logger.warning("Noisy odometry node did not terminate gracefully, killing.")
                        noisy_proc.kill()
                    # Wait for the noisy odom node to terminate totally
                    time.sleep(1)
        finally:
            # Always shutdown ROS stack when done (success or failure)
            self.shutdown_ros_stack()

    def build_rosrun_cmd(self, params: Dict[str, Any]) -> str:
        """Build the rosrun command for the noisy odometry node."""
        rosrun_cmd = (
            f"source {self.current_workspace_dir}/devel/setup.bash && "
            f"rosrun rbcar_localization noisy_odom_generator.py "
            f"--input_topic /INS/odom_raw --output_topic /INS/odom"
        )
        if params['enable_x']:
            rosrun_cmd += f" --enable_x --x_stddev {params['x_stddev']}"
        if params['enable_y']:
            rosrun_cmd += f" --enable_y --y_stddev {params['y_stddev']}"
        if params['enable_heading']:
            rosrun_cmd += f" --enable_heading --heading_stddev {params['heading_stddev']}"
        if params['enable_speed']:
            rosrun_cmd += f" --enable_speed --speed_stddev {params['speed_stddev']}"
        return rosrun_cmd


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    # Get the path to the CAR_WS_PATH environment variable
    car_ws_path = os.environ.get('CAR_WS_PATH')
    datasets_path = os.environ.get('DATASET_PATH')
    # Convert the path to a string
    car_ws_path = str(car_ws_path)
    datasets_path = str(datasets_path)

    default_path = os.path.join(car_ws_path, 'src/MPCSimulationRunner/config/config_run_gather.yaml')
    parser.add_argument('--config_file', type=str, default=default_path, help='Path to the configuration file', required=False)
    config_file_path = parser.parse_args().config_file
    config_file = yaml.safe_load(open(config_file_path, 'r'))

    # Define a list of vehicle parameter sets to test
    # If this list is empty, the script will fall back to reading the vehicle_params.yaml file
    vehicle_params_list = [
        {
            # Golf Cart
            'wheelbase': 2.48,
            'weight': 600,
            'length': 3.74,
            'width': 1.20,
            'height': 0.2, # Using default value
            'steering_angle_limit_rad': 0.4433, # 25.4 deg
            'steering_angle_rate_limit_rad_s': 0.1400 # 8.02 deg/s
        },
        {
            # Sedan (Sonata)
            'wheelbase': 2.84,
            'weight': 1700,
            'length': 4.90,
            'width': 1.86,
            'height': 0.2, # Using default value
            'steering_angle_limit_rad': 0.5934, # 34.0 deg
            'steering_angle_rate_limit_rad_s': 0.3840 # 22.0 deg/s
        }
    ]

    if not vehicle_params_list:
        vehicle_params_path = os.path.join(car_ws_path, 'config', 'vehicle_params.yaml')
        vehicle_params = yaml.safe_load(open(vehicle_params_path, 'r'))
        vehicle_params_list.append(vehicle_params)

    # Environment parameters (optional - for testing different road conditions)
    # Set to None or empty list to skip environment parameter updates
    # Example: Test different friction levels for each vehicle
    environment_params_list = [
        {'road_condition': 'dry', 'wind_speed': 0.0},    # Golf cart on dry road
        {'road_condition': 'wet', 'wind_speed': 0.0},    # Sedan on wet road
    ]
    # To disable environment parameter updates, set to None:
    # environment_params_list = None

    # Set the noisy odometry parameters
    noisy_odom_params = [{'enable_x': False, 'enable_y': False, 'enable_heading': False, 'enable_speed': False, 'x_stddev': 0.0625, 'y_stddev': 0.0625, 'speed_stddev': 0.06, 'heading_stddev': 0.0174},
                         {'enable_x': False, 'enable_y': False, 'enable_heading': False, 'enable_speed': False, 'x_stddev': 0.0625, 'y_stddev': 0.0625, 'speed_stddev': 0.06, 'heading_stddev': 0.0349},
                         {'enable_x': False, 'enable_y': False, 'enable_heading': False, 'enable_speed': False, 'x_stddev': 0.0625, 'y_stddev': 0.0625, 'speed_stddev': 0.06, 'heading_stddev': 0.0698}]
    dataset_class_params = [{'enable_easy': True, 'enable_moderate': False, 'enable_hard': False},
                            {'enable_easy': False, 'enable_moderate': True, 'enable_hard': False},
                            {'enable_easy': False, 'enable_moderate': False, 'enable_hard': True}]
    run_params = [{'which_mpc': ['uniform', 'fsmpc']},
                  {'which_mpc': ['uniform', 'fsmpc']},
                  {'which_mpc': ['uniform', 'fsmpc']}]

    # Create and run the experiment
    run_and_gather_results = RunandGatherResults(
        config_file,
        car_ws_path,
        datasets_path,
        noisy_odom_params,
        dataset_class_params,
        run_params,
        vehicle_params_list,
        environment_params_list  # NEW: Environment parameters for road friction, wind, etc.
    )
    run_and_gather_results.run_simulation()
    logging.info("Results Gathering is Done!")
