# Path-Following Dataset with Multi-Level Difficulty for Autonomous Driving in Unstructured Roads

![UBUNTU](https://img.shields.io/badge/UBUNTU-18.04-orange?style=plastic&logo=ubuntu)
![python](https://img.shields.io/badge/Python-2.7-blue?style=plastic&logo=python)
![ROS2](https://img.shields.io/badge/ROS-Melodic-white?style=plastic&logo=ros)

This repository contains a dataset for control algorithm benchmarking in unstructured environments, specifically targeting scenarios like parking lots where precise low-speed maneuvering is crucial. This dataset, based on detailed 2D HD maps from two distinct areas at Pusan National University (PNU), was created to address the need for specialized data to evaluate the performance of control algorithms for autonomous vehicles and robots navigating in such complex, real-world scenarios.

## Background

While there are existing datasets for autonomous driving, they often focus on structured environments like highways and urban roads. These datasets are valuable for high-speed navigation and scenarios with well-defined lanes and traffic rules. However, they do not adequately capture the unique challenges of Unstructured environments like parking lots, where:

* **Maneuvering is predominantly at low speeds**, requiring precise control and responsiveness to navigate tight spaces and avoid collisions.
* **Paths are often complex and irregular**, with sharp turns, varying curvature, and obstacles that necessitate sophisticated path-following algorithms.
* **Elevation changes can significantly impact vehicle dynamics**, demanding control algorithms that can maintain stability and trajectory accuracy on slopes.

This dataset aims to fill this gap by providing a dedicated resource for evaluating and improving control algorithms specifically for Unstructured environments, enabling the development of more robust and reliable autonomous navigation systems. This data can be used for benchmarking both classical and learning-based control algorithms, facilitating the development of advanced techniques for autonomous driving in challenging scenarios. The dataset is classified into three difficulty levels, providing a diverse range of paths for comprehensive evaluation and benchmarking.


## Dataset Description

This dataset includes:

*   **1200 paths:** Generated based on the HD maps of two distinct areas at PNU: the flat parking area in front of PNU Electrical Engineering building and the sloped road area near PNU Museum. The PNU Museum area features a sloped terrain, adding another dimension of complexity compared to the Electrical Engineering parking lot. 
*   **Annotations:** Each path is annotated with a difficulty level (easy, moderate, or hard) based on its curvature characteristics. This allows for targeted evaluation of control algorithms on different difficulty levels.

**Example use cases:**

*   **Control algorithm benchmarking:** Comparing the performance of different control algorithms (e.g., PID, MPC, pure pursuit) in tracking the generated paths, analyzing their ability to handle varying curvature and difficulty levels.
*   **Unstructured environment navigation:** This data can be used for evaluating the performance of autonomous driving systems in navigating complex, unstructured environments like parking lots.
*   **Developing learning-based control algorithms:** Training and evaluating reinforcement learning agents or other learning-based controllers to navigate in unstructured environments.
*   **Testing path-following algorithms in the presence of disturbances:** Evaluating the robustness of control algorithms to disturbances like wind, tire slip, or sensor noise.
*   **Analyzing the impact of different vehicle parameters on control performance:** Studying how factors like vehicle dimensions, mass, and tire characteristics affect path-following accuracy.

## Data Collection

The paths in this dataset were generated using the Hybrid A* algorithm. This algorithm combines the benefits of A* search with continuous state space exploration, making it suitable for finding optimal paths in complex environments with kinodynamic constraints, such as those encountered by vehicles. By incorporating heuristics and continuous state transitions, Hybrid A* efficiently explores the search space and generates smooth, drivable paths.

The algorithm was applied to the HD maps of the flat parking area in front of PNU Electrical Engineering building and the sloped road area near PNU Museum to generate a diverse set of paths with varying difficulty levels.

## Dataset Format
The dataset is organized in the following folder structure:
```
path_datasets
├── train
│   ├── E_Path388_EE.csv
│   ├── E_Path395_EE.csv
│   └── ...
└── test
    ├── M_Path996_M.csv
    ├── H_Path1047_M.csv
    └── ...
```

The dataset is provided in CSV file format with its metadata. Each CSV file represents a single path and contains the following columns information:

* **ref_x:**  The x-coordinate of the reference path (in meters).
* **ref_y:** The y-coordinate of the reference path (in meters).
* **ref_yaw:** The yaw angle of the reference path (in radians).
* **ref_z:** The height information of the reference path (in meters).

The difficulty level and the map used for generating the path are encoded in the filename. For example:

* `E_Path120_M.csv` indicates an easy path (E) generated from the Museum (M) map.
* `M_Path200_EE.csv` indicates a moderate path (M) generated from the EE building (EE) map.
* `H_Path300_EE.csv` indicates a hard path (H) generated from the EE parking lot (EE) map.

**Dataset Split and Proportions:**

* The dataset is split into `train` and `test` sets with a 3:2 ratio. This means that approximately 60% of the paths are in the `train` set and 40% are in the `test` set.
* Each map (e.g., Museum, EE building) has the same proportion of paths in the overall dataset.
* Each difficulty level (easy, moderate, hard) also has the same proportion of paths in the overall dataset.

This ensures a balanced representation of maps and difficulty levels in both the training and testing sets.

## Usage Examples of Dataset

This section provides examples of how to use the path-following datasets.

**Prerequisites:**

* **Conda:** Make sure you have Conda installed on your system. If not, you can download and install Miniconda from [here](https://docs.conda.io/en/latest/miniconda.html).
* **ROS Melodic:** This example requires ROS Melodic to be installed. You can find installation instructions on the [ROS website](http://wiki.ros.org/melodic/Installation).
* **Ubuntu 18.04 or later:** This example is designed for Ubuntu 18.04 or later versions.

**Running with ROS:**

```bash
# Clone the repository
cd ~
git clone git@github.com:vialabpnu/path-following-datasets.git

cd path-following-datasets

# Install ROS dependencies
./install_dependencies_ros_melodic.sh

# Navigate to the examples directory
cd examples/car_ws

# Build the Workspace
catkin build

# Run the simulator
cd ..
./runSimulator.sh
```

* After running these commands, the simulation saves its logs to the ```examples/car_ws/src/MPCSimulationRunner/data/eval_test``` directory.

## Running with Docker

We provide a Docker image with all the necessary dependencies pre-installed.

```bash
# Change to the project directory (where the Dockerfile is located)
cd path-following-datasets

# Build the Docker image
docker build -t path-following-datasets:melodic .

# Run the Docker container
docker run -it --network bridge path-following-datasets:melodic
```

Once inside the container, you can run the simulator:

```bash
cd path-following-datasets
./examples/runSimulator.sh
```

## Generating Custom Paths and Difficulty Classification

To generate your own paths and classify their difficulty based on different vehicle parameters or vehicle models:

- **Edit Configuration Files:**  
  Adjust the settings in both `path_generation/generation_config.yaml` and `vehicle_params.yaml` to match your desired vehicle model, parameters, or path generation preferences.

- **Generate Paths in MATLAB:**  
  Open MATLAB and run `PathGenerator.m`  
  This script will use the parameters from both configuration files to generate new paths.

- **Automatic Difficulty Classification:**  
  The generated paths will be automatically classified into different difficulty levels (easy, moderate, hard) based on their curvature and the specified configuration and vehicle parameters.

By modifying these configuration files and regenerating the dataset, you can benchmark control algorithms for a variety of vehicle types and scenarios with your own custom map.

<!-- ## Citation

If you use this dataset in your research, please cite the following paper:

@article{amiruddin2024path,
  title={Path-Following Dataset with Multi-Level Difficulty for Autonomous Driving in Unstructured Roads},
  author={Amiruddin, Brilian Putra and Kim, Sungil and Jeong, Han-You},
  journal={}, 
  year={2025},
  volume={XX},
  number={XX},
  pages={XX--XX},
  doi={10.XXXX/XXXX.2025.XXXXXXX},  % Add actual DOI if available
  url={https://github.com/vialabpnu/path-following-datasets},  % Add repo URL
  note={Dataset available at: \url{https://github.com/vialabpnu/path-following-datasets}}
} -->

## License

This dataset is released under the **GPL-3.0 license**.

## Acknowledgements

This golf cart simulator utilizes Robotnik Automation's Gazebo-based Ackermann steering model (found in the repositories below), adapted and configured to match our specific golfcart vehicle platform.

* [rbcar_sim](https://github.com/RobotnikAutomation/rbcar_sim/tree/melodic-devel-namespace) - Provides the core simulation packages.
* [rbcar_common](https://github.com/RobotnikAutomation/rbcar_common/tree/melodic-devel-sim) - Contains common packages and utilities used in the simulation.

We extend our gratitude to the developers and contributors of these repositories for their valuable work.
