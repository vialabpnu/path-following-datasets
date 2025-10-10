#!/usr/bin/env python3
"""
Refactored RunandGatherResults.py
- Uses TmuxManager for ROS process management
- Supports experiment configurations with vehicle and environment parameters
- Single entry point for running multiple experiments
"""

import argparse
import subprocess
import sys
import os
import time
import logging
import datetime
import yaml
import shutil
from typing import Dict, List, Any, Optional

# Add path to scripts
script_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.append(script_dir)

# Add path to config
config_path = os.path.join(script_dir, '..', '..', '..', 'config')
sys.path.append(config_path)

# Import our new modules
from tmux_manager import TmuxManager


class RunandGatherResults:
    """
    Master controller for running simulation experiments
    Manages experiments with different vehicle and environment parameters
    """

    def __init__(self,
                 experiment_configs: List[Dict[str, Any]],
                 car_ws_path: str,
                 datasets_path: str,
                 config_file: Dict[str, Any]) -> None:
        """
        Initialize with list of experiment configurations

        Args:
            experiment_configs: List of experiment configuration dictionaries
            car_ws_path: Path to workspace
            datasets_path: Path to dataset directory
            config_file: General configuration from YAML
        """
        self.experiment_configs = experiment_configs
        self.car_ws_path = car_ws_path
        self.datasets_path = datasets_path
        self.config_file = config_file

        # Extract config parameters
        self.run_command = config_file['run_command']
        self.timeout = float(config_file['timeout'])
        self.path_files = config_file['path_files']
        self.eval_results_path = config_file['eval_results_path']
        self.sleep_list = config_file['sleep_list']

        # Paths
        self.path_files_dir = os.path.join(self.datasets_path, self.path_files)
        self.path_files_list = [f for f in os.listdir(self.path_files_dir) if f.endswith('.csv')]
        self.eval_results_base = os.path.join(self.car_ws_path, self.eval_results_path)

        # Initialize TmuxManager
        self.tmux = TmuxManager("mpc_experiment_session")

        # Logging
        self._setup_logging()

    def _setup_logging(self):
        """Setup logging configuration"""
        console_handler = logging.StreamHandler()
        console_handler.setLevel(logging.INFO)
        formatter = logging.Formatter('%(asctime)s - %(levelname)s - %(message)s')
        console_handler.setFormatter(formatter)
        self.logger = logging.getLogger(__name__)
        self.logger.setLevel(logging.INFO)
        self.logger.addHandler(console_handler)
        self.logger.info("="*60)
        self.logger.info("RunandGatherResults - Experiment Controller")
        self.logger.info("="*60)
        self.logger.info(f"Total experiments: {len(self.experiment_configs)}")
        self.logger.info(f"Total path files available: {len(self.path_files_list)}")

    def run_experiment_loop(self):
        """
        Main loop: Iterate through all experiments
        """
        self.logger.info("\n" + "="*60)
        self.logger.info("STARTING EXPERIMENT LOOP")
        self.logger.info("="*60)

        for exp_idx, experiment_config in enumerate(self.experiment_configs, 1):
            self.logger.info(f"\n{'='*60}")
            self.logger.info(f"EXPERIMENT {exp_idx}/{len(self.experiment_configs)}: {experiment_config.get('name', 'Unnamed')}")
            self.logger.info(f"{'='*60}")

            # Setup experiment (update YAML files, run update script)
            if not self._setup_experiment(experiment_config):
                self.logger.error(f"Failed to setup experiment {exp_idx}, skipping...")
                continue

            # Launch ROS stack with TmuxManager
            if not self._launch_ros_stack():
                self.logger.error(f"Failed to launch ROS stack for experiment {exp_idx}, skipping...")
                self._cleanup_experiment()
                continue

            # Execute path following tests
            self._execute_path_tests(experiment_config)

            # Collect and organize results
            self._collect_results(experiment_config['name'])

            # Cleanup (kill tmux session, etc.)
            self._cleanup_experiment()

        self.logger.info("\n" + "="*60)
        self.logger.info("ALL EXPERIMENTS COMPLETED")
        self.logger.info("="*60)

    def _setup_experiment(self, config: Dict[str, Any]) -> bool:
        """
        Setup experiment: update YAML files and run update script

        Args:
            config: Experiment configuration dictionary

        Returns:
            bool: True if successful, False otherwise
        """
        self.logger.info("Setting up experiment...")

        try:
            # 1. Update vehicle_params.yaml if specified
            if 'vehicle_params' in config:
                vehicle_params_path = os.path.join(self.car_ws_path, 'config', 'vehicle_params.yaml')
                self.logger.info(f"Updating vehicle parameters: {config['vehicle_params']}")
                with open(vehicle_params_path, 'w') as f:
                    yaml.dump(config['vehicle_params'], f)

            # 2. Update environment_params.yaml if specified
            if 'environment' in config:
                environment_params_path = os.path.join(self.car_ws_path, 'config', 'environment_params.yaml')
                env_params = yaml.safe_load(open(environment_params_path, 'r'))

                # Update active road condition if specified
                if 'road_condition' in config['environment']:
                    env_params['active_road_condition'] = config['environment']['road_condition']
                    self.logger.info(f"Setting road condition: {config['environment']['road_condition']}")

                # Update wind parameters if specified
                if 'wind_speed' in config['environment']:
                    env_params['wind']['mean_speed_mps'] = config['environment']['wind_speed']
                    self.logger.info(f"Setting wind speed: {config['environment']['wind_speed']} m/s")

                with open(environment_params_path, 'w') as f:
                    yaml.dump(env_params, f)

            # 3. Run update_simulation_from_yaml.py to apply changes
            self.logger.info("Running update_simulation_from_yaml.py...")
            update_script_path = os.path.join(self.car_ws_path, 'config', 'update_simulation_from_yaml.py')
            result = subprocess.run(['python', update_script_path], capture_output=True, text=True)

            if result.returncode != 0:
                self.logger.error(f"Failed to run update script: {result.stderr}")
                return False

            self.logger.info("Experiment setup completed successfully")
            return True

        except Exception as e:
            self.logger.error(f"Error setting up experiment: {e}")
            return False

    def _launch_ros_stack(self) -> bool:
        """
        Launch ROS stack using TmuxManager

        Returns:
            bool: True if successful, False otherwise
        """
        self.logger.info("Launching ROS stack with TmuxManager...")

        try:
            # Start tmux session
            if not self.tmux.start_session():
                return False

            # Note: We assume roscore and Gazebo are already running
            # This script focuses on launching test nodes

            self.logger.info("ROS stack launch completed")
            return True

        except Exception as e:
            self.logger.error(f"Error launching ROS stack: {e}")
            return False

    def _execute_path_tests(self, config: Dict[str, Any]):
        """
        Execute path following tests based on configuration

        Args:
            config: Experiment configuration
        """
        self.logger.info("Executing path following tests...")

        # Get configuration parameters
        noise_params = config.get('noise_params', {
            'enable_x': False, 'enable_y': False,
            'enable_heading': False, 'enable_speed': False,
            'x_stddev': 0.0, 'y_stddev': 0.0,
            'heading_stddev': 0.0, 'speed_stddev': 0.0
        })

        dataset_params = config.get('datasets', {
            'enable_easy': True,
            'enable_moderate': True,
            'enable_hard': False
        })

        mpc_types = config.get('mpc_types', ['nonuni_sparse_var'])

        # Filter path files based on dataset difficulty
        selected_paths = []
        if dataset_params.get('enable_easy', False):
            selected_paths.extend([p for p in self.path_files_list if p.startswith('E_')])
        if dataset_params.get('enable_moderate', False):
            selected_paths.extend([p for p in self.path_files_list if p.startswith('M_')])
        if dataset_params.get('enable_hard', False):
            selected_paths.extend([p for p in self.path_files_list if p.startswith('H_')])

        if not selected_paths:
            self.logger.warning("No paths selected for this configuration")
            return

        self.logger.info(f"Selected {len(selected_paths)} path files")
        self.logger.info(f"MPC types: {mpc_types}")

        # Create results folder
        current_time = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        results_folder = os.path.join(self.eval_results_base, config['name'], current_time)
        os.makedirs(results_folder, exist_ok=True)

        # Start noisy odom node
        noisy_odom_cmd = self._build_noisy_odom_cmd(noise_params)
        noisy_proc = subprocess.Popen(noisy_odom_cmd, shell=True, executable="/bin/bash")
        time.sleep(2)

        # Execute tests for each path and MPC type
        for path_file in selected_paths:
            for mpc_type in mpc_types:
                self.logger.info(f"Testing: {path_file} with {mpc_type}")
                self._run_single_test(path_file, mpc_type, results_folder, current_time)
                time.sleep(3)

        # Terminate noisy odom node
        noisy_proc.terminate()
        try:
            noisy_proc.wait(timeout=5)
        except subprocess.TimeoutExpired:
            noisy_proc.kill()

    def _run_single_test(self, path_file: str, mpc_type: str, results_folder: str, eval_folder: str):
        """
        Run a single path following test

        Args:
            path_file: Name of path CSV file
            mpc_type: MPC horizon type
            results_folder: Where to save results
            eval_folder: Evaluation folder name
        """
        # Build commands
        mpc_server_cmd = (
            f"cd {self.car_ws_path} && "
            f". ~/anaconda3/etc/profile.d/conda.sh && conda activate mpc-gen && "
            f"{self.run_command['mpc_server']} --horizon_type {mpc_type} "
            f"--file_path_name {path_file} --eval_path_folder {eval_folder}"
        )

        motion_planner_cmd = (
            f"source {self.car_ws_path}/devel/setup.bash && "
            f"{self.run_command['motion_planner']} eval_mode:=true file_path:={path_file}"
        )

        mpc_node_cmd = (
            f"source {self.car_ws_path}/devel/setup.bash && "
            f"{self.run_command['mpc_node']} --eval_mode True "
            f"--file_path_name {path_file} --file_path_dir {results_folder} "
            f"--horizon_type {mpc_type}"
        )

        # Create panes and send commands
        self.tmux.create_pane("mpc_server", split_from="main", split_direction='h')
        self.tmux.create_pane("motion_planner", split_from="main", split_direction='v')
        self.tmux.create_pane("mpc_node", split_from="motion_planner", split_direction='h')

        self.tmux.send_command("mpc_server", mpc_server_cmd)
        time.sleep(2)
        self.tmux.send_command("motion_planner", motion_planner_cmd)
        time.sleep(1)
        self.tmux.send_command("mpc_node", mpc_node_cmd)

        # Unpause Gazebo physics
        subprocess.run(self.run_command['unpause_physics'], shell=True)

        # Wait for completion or timeout
        start_time = time.time()
        while (time.time() - start_time) < self.timeout:
            # Check if test is complete (implement completion check logic)
            time.sleep(1)
            # TODO: Add proper completion detection
            pass

        # Cleanup test
        subprocess.run("rosnode kill motion_planner_node", shell=True)
        subprocess.run("rosnode kill mpc_node", shell=True)
        subprocess.run("fuser -k 12345/udp", shell=True)
        subprocess.run("rosservice call /gazebo/reset_simulation {}", shell=True)

        # Kill panes
        self.tmux.send_keys("mpc_server", "C-c")
        time.sleep(1)

    def _build_noisy_odom_cmd(self, params: Dict[str, Any]) -> str:
        """Build rosrun command for noisy odometry node"""
        cmd = (
            f"source {self.car_ws_path}/devel/setup.bash && "
            f"rosrun rbcar_localization noisy_odom_generator.py "
            f"--input_topic /INS/odom_raw --output_topic /INS/odom"
        )
        if params.get('enable_x', False):
            cmd += f" --enable_x --x_stddev {params['x_stddev']}"
        if params.get('enable_y', False):
            cmd += f" --enable_y --y_stddev {params['y_stddev']}"
        if params.get('enable_heading', False):
            cmd += f" --enable_heading --heading_stddev {params['heading_stddev']}"
        if params.get('enable_speed', False):
            cmd += f" --enable_speed --speed_stddev {params['speed_stddev']}"
        return cmd

    def _collect_results(self, experiment_name: str):
        """
        Collect and organize results from experiment

        Args:
            experiment_name: Name of the experiment
        """
        self.logger.info(f"Collecting results for: {experiment_name}")
        # TODO: Implement result collection logic

    def _cleanup_experiment(self):
        """Cleanup after experiment"""
        self.logger.info("Cleaning up experiment...")
        self.tmux.kill_session()
        time.sleep(2)


def main():
    """Main entry point"""
    parser = argparse.ArgumentParser(description='Run simulation experiments')
    parser.add_argument('--config', type=str, help='Path to configuration file')
    parser.add_argument('--experiments', nargs='+', help='Specific experiments to run')
    args = parser.parse_args()

    # Get workspace paths
    car_ws_path = os.environ.get('CAR_WS_PATH', '/home/ubuntu/path-following-datasets/path_following_simulator')
    datasets_path = os.environ.get('DATASET_PATH', '/home/ubuntu/path-following-datasets/datasets')

    # Load general config
    config_file_path = args.config or os.path.join(car_ws_path, 'src/MPCSimulationRunner/config/config_run_gather.yaml')
    config_file = yaml.safe_load(open(config_file_path, 'r'))

    # Define experiment configurations
    experiment_configs = [
        {
            'name': 'baseline_dry_golfcart',
            'vehicle_params': {
                'wheelbase': 2.48,
                'weight': 600,
                'length': 3.74,
                'width': 1.20,
                'height': 0.2,
                'steering_angle_limit_rad': 0.444,
                'steering_angle_rate_limit_rad_s': 0.14
            },
            'environment': {
                'road_condition': 'dry',
                'wind_speed': 0.0
            },
            'datasets': {
                'enable_easy': True,
                'enable_moderate': True,
                'enable_hard': False
            },
            'mpc_types': ['nonuni_sparse_var'],
            'noise_params': {
                'enable_x': False,
                'enable_y': False,
                'enable_heading': False,
                'enable_speed': False,
                'x_stddev': 0.0,
                'y_stddev': 0.0,
                'heading_stddev': 0.0,
                'speed_stddev': 0.0
            }
        },
        {
            'name': 'wet_road_golfcart',
            'environment': {
                'road_condition': 'wet'
            },
            'datasets': {
                'enable_easy': True,
                'enable_moderate': False,
                'enable_hard': False
            },
            'mpc_types': ['nonuni_sparse_var']
        },
        {
            'name': 'icy_road_golfcart',
            'environment': {
                'road_condition': 'icy'
            },
            'datasets': {
                'enable_easy': True,
                'enable_moderate': False,
                'enable_hard': False
            },
            'mpc_types': ['nonuni_sparse_var']
        }
    ]

    # Filter experiments if specified
    if args.experiments:
        experiment_configs = [e for e in experiment_configs if e['name'] in args.experiments]

    # Create runner and execute
    runner = RunandGatherResults(
        experiment_configs=experiment_configs,
        car_ws_path=car_ws_path,
        datasets_path=datasets_path,
        config_file=config_file
    )

    runner.run_experiment_loop()
    logging.info("All experiments completed!")


if __name__ == '__main__':
    main()
