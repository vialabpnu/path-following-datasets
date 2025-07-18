import argparse
import subprocess
import sys
import os
import time
import logging
import datetime

import yaml

from typing import Dict, List, Any

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
    def __init__(self, config_file: str, current_workspace_dir: str, current_dataset_dir: str, noisy_odom_params: Dict[str, Any]) -> None:
        self.mpc_sampling_params = ['fsmpc', 'uniform']
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

    def run_simulation(self):
        for noise_param in self.noisy_odom_params:
            noisy_odom_cmd = self.build_rosrun_cmd(noise_param)
            self.logger.info(f"Running noisy odometry node with command: {noisy_odom_cmd}")
            # Start the noisy odom node as a subprocess
            noisy_proc = subprocess.Popen(noisy_odom_cmd, shell=True, executable="/bin/bash")
            # Create a folder based on the evaluation results path target and count of the noisy odom params
            self.eval_folder_count += 1
            self.eval_results_path_target_run = os.path.join(self.eval_results_path_target, 'run_' + str(self.eval_folder_count))
            if not os.path.exists(self.eval_results_path_target_run):
                os.mkdir(self.eval_results_path_target_run)

            for i, path_file in enumerate(self.path_files_list):
                self.logger.info(f"Running the simulation for path file: {path_file}")

                for sampling_param in self.mpc_sampling_params:
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
    # Set the noisy odometry parameters
    # Test the noise only on the x and y position to see the effect on the performance
    noisy_odom_params = [{'enable_x': True, 'enable_y': True, 'enable_heading': False, 'enable_speed': False, 'x_stddev': 0.01, 'y_stddev': 0.01},
                         {'enable_x': True, 'enable_y': True, 'enable_heading': False, 'enable_speed': False, 'x_stddev': 0.05, 'y_stddev': 0.05},
                         {'enable_x': True, 'enable_y': True, 'enable_heading': False, 'enable_speed': False, 'x_stddev': 0.25, 'y_stddev': 0.25}]
    run_and_gather_results = RunandGatherResults(config_file, car_ws_path, datasets_path, noisy_odom_params)
    run_and_gather_results.run_simulation()
    logging.info("Results Gathering is Done!")
    