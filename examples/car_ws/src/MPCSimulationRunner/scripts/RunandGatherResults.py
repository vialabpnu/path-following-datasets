import argparse
import subprocess
import sys
import os
import time
import atexit
import signal
import logging
import datetime

import yaml


def shutdown_sim(process_list, gracetime_s=0.1):
    """
    """
    for process in process_list:
        process.poll()
        if process.poll() is not None:
            logging.debug(
                "ROS process %d was shut down already with returncode %d.",
                process.pid,
                process.returncode,
            )
            continue
        process.terminate()
        try:
            process.wait(gracetime_s)
        except subprocess.TimeoutExpired as _exc:
            logging.warning(
                "ROS process %d did not shut down gracefully, sennding kill.",
                process.pid,
            )
            process.kill()
        assert (
            process.poll() and process.returncode is not None
        ), "ROS could not be killed."
    return 


class RunandGatherResults:
    def __init__(self, config_file):
        self.mpc_sampling_params = ['fsmpc', 'uniform']
        # self.mpc_sampling_params = ['fsmpc']
        # self.mpc_sampling_params = ['nonuni_sparse_var']
        # self.mpc_sampling_params = ['uniform']
        self.run_command = config_file['run_command']
        self.run_command_mpc_server = self.run_command['mpc_server']
        self.run_command_mpc_node = self.run_command['mpc_node']
        self.run_command_motion_planner = self.run_command['motion_planner']
        self.run_unpause_physics = self.run_command['unpause_physics']
        # Timeout for the MPC server
        self.timeout = config_file['timeout']
        # Path to the path files
        self.path_files = config_file['path_files']
        self.user_prefix = config_file['user_prefix_path']
        self.workspace_list = config_file['workspace_list'][0]
        self.sleep_list = config_file['sleep_list']
        self.path_files_list = os.listdir(os.path.join(self.user_prefix, self.workspace_list, self.path_files))
        ## Hnadling missing path files
        # Filter out the files that are not .csv
        self.path_files_list = [path_file for path_file in self.path_files_list if path_file.endswith('.csv')]
        # self.current_uniform_data_path = '/home/vialab/car_ws/src/MPC_Results_Gather/data/eval_test/20241124_1551_5.0_5.0_withLastHorizon'
        # # Get lists of successful paths
        # csv_file_list = os.listdir(self.current_uniform_data_path)
        # # Filter only start with uniform
        # csv_file_list = [csv_file for csv_file in csv_file_list if csv_file.startswith('uniform')]
        # # Get the path name (uniform_verdict_Diff_PathName.csv) -> Diff_PathName.csv
        # # Filter only successful paths  (uniform_verdict_Diff_PathName.csv)
        # csv_file_list = [csv_file for csv_file in csv_file_list if csv_file.split('_')[1] == 'success']
        # self.path_files_list = [csv_file.split('uniform_success_')[1] for csv_file in csv_file_list]
        # self.path_files_list.append('H_Path5317.csv')
        ## Handling missing path files
        # Filter path to only start with H_Path
        # self.path_files_list = [path_file for path_file in self.path_files_list if path_file.startswith('H_Path')]
        # self.path_files_list = ["E_Path7009_PB.csv", "E_Path7055_PB.csv", "E_Path7097_PB.csv", "E_Path7104_PB.csv", "E_Path7114_PB.csv", "E_Path7135_PB.csv", "E_Path7177_PB.csv"]       
        self.count_path_files = len(self.path_files_list)
        self.eval_results_path = config_file['eval_results_path']
        self.current_time = datetime.datetime.now().strftime("%Y%m%d_%H%M")
        self.eval_results_path = os.path.join(self.user_prefix, self.workspace_list, self.eval_results_path)
        self.eval_results_path_target = os.path.join(self.eval_results_path, self.current_time)
        self.process_list = []
        self.gazebo_reset_command = "rosservice call /gazebo/reset_simulation {}"
        # Create the evaluation results folder
        if not os.path.exists(self.eval_results_path_target):
            os.mkdir(self.eval_results_path_target)
        # Console handler logging
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
        for i, path_file in enumerate(self.path_files_list):
            self.logger.info(f"Running the simulation for path file: {path_file}")
            for sampling_param in self.mpc_sampling_params:
                self.process_list = []
                run_command_dict = {
                    'mpc_node': self.run_command_mpc_node,
                    'motion_planner': self.run_command_motion_planner,
                    'mpc_server': self.run_command_mpc_server + ' --horizon_type ' + sampling_param + ' --file_path_name ' + path_file,
                }
                idx = 0
                
                def shutdown_handler(signum=None, stackframe=None):
                    """
                    Ensure that gazebo and ROS always gets shut down on python exit.

                    This is implemented as a local function on purpose.
                    There could be more than one roslaunch in one python process.
                    So calling atexit.unregister(shutdown__ros) could cause leaks.
                    """
                    shutdown_sim(self.process_list)
                    if signum is not None:
                        sys.exit()
                time.sleep(3)
                for key, value in run_command_dict.items():
                    if key == 'mpc_server':
                        command = ["cd ~/car_ws",
                                    ". ~/miniconda3/etc/profile.d/conda.sh && conda activate lab",
                                   str(value)]
                    elif key == 'mpc_node':         
                        # command = ["source /home/vialab/" + self.workspace_list + "/devel/setup.bash",
                        #     f"sleep {self.sleep_list[idx]}",
                        #     str(value)
                        # ] # MPC Node with ROSLAUNCH
                        command = ["source /home/vialab/" + self.workspace_list + "/devel/setup.bash",
                            f"sleep {self.sleep_list[idx]}",
                            str(value) + ' --eval_mode True' + ' --file_path_name ' + path_file + ' --file_path_dir ' + self.eval_results_path_target \
                                + ' --horizon_type ' + sampling_param
                        ] # MPC Node with ROSRUN
                    elif key == 'motion_planner':
                        command = ["cd ",
                            "source /home/vialab/" + self.workspace_list + "/devel/setup.bash",
                            f"sleep {self.sleep_list[idx]}",
                            str(value) + ' eval_mode:=true' + ' file_path:=' + path_file
                        ]
                    else:
                        command = ["cd ~/car_ws",
                            f"sleep {self.sleep_list[idx]}",
                            str(value)
                        ]
                    # print_output = True if key == 'mpc_server' else False
                    print_output = True
                    stdout = sys.stdout if print_output else subprocess.DEVNULL
                    logging.info(f"Running command: {';'.join(command)}")
                    # time.sleep(sleep_list[i])
                    process = subprocess.Popen((';'.join(command)), shell=True, executable="/bin/bash")
                    # process = subprocess.Popen(command, stdout=stdout, shell=False, executable="/bin/bash")
                    self.process_list.append(process)
                    idx += 1
                # Wait for the process to finish or timeout (The MPC server should finish first)
                # Check timeout
                subprocess.run(self.run_unpause_physics, shell=True)
                start_time = time.time()
                mpc_server_done = False
                what_to_kill_in_ros = "rosnode kill motion_planner_node"
                what_to_kill_in_ros_mpc_node = "rosnode kill mpc_node"
                fuse_udp = "fuser -k 12345/udp"
                while not mpc_server_done:
                    if time.time() - start_time < self.timeout:
                        if self.process_list[2].poll() is not None:
                            mpc_server_done = True
                            self.process_list[2].terminate()
                            self.logger.info("MPC server is done!")
                            # Kill the ros processes
                            subprocess.run(fuse_udp, shell=True)
                            subprocess.run(self.gazebo_reset_command, shell=True)
                            subprocess.run(what_to_kill_in_ros, shell=True)
                            subprocess.run(what_to_kill_in_ros_mpc_node, shell=True)
                            time.sleep(1)

                    else:
                        # Create a file to indicate that the timeout has occurred and it will be read by the MPC server
                        with open(os.path.join(self.eval_results_path, 'timeout.txt'), 'w') as f:
                            f.write('Timeout has occurred!')
                        time.sleep(1)
                        self.logger.info("Timeout! Killing the processes")
                        self.process_list[2].terminate()
                        # shutdown_sim(self.process_list)
                        subprocess.run(fuse_udp, shell=True)
                        subprocess.run(self.gazebo_reset_command, shell=True)
                        subprocess.run(what_to_kill_in_ros, shell=True)
                        subprocess.run(what_to_kill_in_ros_mpc_node, shell=True)
                        mpc_server_done = True
                        time.sleep(1)

                # Register the shutdown handler                    
                # atexit.register(shutdown_handler)
                # signal.signal(signal.SIGTERM, shutdown_handler)
                #time.sleep(self.timeout)
                #for i, process in enumerate(process_list):
                #    if i == 2:
                #        # Terminate the MPC server
                #        os.kill(process.pid, signal.SIGTERM)
                #       logging.info("Terminating the MPC server")
                #       time.sleep(5)
                #   else:
                #        os.kill(process.pid, signal.SIGKILL)
                #       sys.stdout.flush()
                 #        logging.info("Killing the other processes")
                # If timeout or the MPC server is done, kill the other processes
                # Check if the MPC server is still running
                # mpc_server_done = False
                # start_time = time.time()
                # while not mpc_server_done or time.time() - start_time > self.timeout:
                #     if process_list[2].poll() is not None:
                #         # Kill the other processes
                #         for i, process in enumerate(process_list):
                #             if i != 2:
                #                 os.killpg(os.getpgid(process.pid), signal.SIGTERM)
                #                 logging.info("Killing the other processes")
                #         mpc_server_done = True
                #     time.sleep(0.05)
                # logging.info("MPC server is done!")
                # # Wait for the other processes to finish
                # for process in process_list:
                #     process.wait()
                # logging.info("All processes are done!")
                # Wait for the other processes to finish
                time.sleep(4)
                # Delete the timeout file
                if os.path.exists(os.path.join(self.eval_results_path, 'timeout.txt')):
                    os.remove(os.path.join(self.eval_results_path, 'timeout.txt'))
        # Move the results with *.csv to the evaluation folder
        self.logger.info("Moving the results to the evaluation folder")
        get_list_files = os.listdir(self.eval_results_path)
        for file in get_list_files:
            if file.endswith('.csv'):
                os.rename(os.path.join(self.eval_results_path, file), os.path.join(self.eval_results_path_target, file))
        # Export the path dir to csv to tell which directory the results are from
        with open(os.path.join(self.eval_results_path_target, 'path_dir.csv'), 'w') as f:
            f.write(self.path_files)
        
        self.logger.info("Finished moving the results to the evaluation folder")        
        self.logger.info("Finished running the results gathering!")
        
if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    default_path = '/home/vialab/car_ws/src/MPC_Results_Gather/config/config_run_gather.yaml'
    parser.add_argument('--config_file', type=str, default=default_path, help='Path to the configuration file', required=False)
    config_file_path = parser.parse_args().config_file
    config_file = yaml.safe_load(open(config_file_path, 'r'))
    run_and_gather_results = RunandGatherResults(config_file)
    run_and_gather_results.run_simulation()
    logging.info("Results Gathering is Done!")
