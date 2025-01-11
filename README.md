# A Path-Following Dataset in Less structured Environments for Autonomous Driving

![UBUNTU](https://img.shields.io/badge/UBUNTU-18.04-orange?style=plastic&logo=ubuntu)
![python](https://img.shields.io/badge/Python-2.7-blue?style=plastic&logo=python)
![ROS2](https://img.shields.io/badge/ROS-Melodic-white?style=plastic&logo=ros)

This repository contains a dataset for control algorithm benchmarking in Less structured environments, specifically targeting scenarios like parking lots where precise low-speed maneuvering is crucial. This dataset, based on a map and slope data from Pusan National University (PNU), was created to address the need for specialized data to evaluate the performance of control algorithms for autonomous vehicles and robots navigating in such complex, real-world scenarios.

## Background

While there are existing datasets for autonomous driving, they often focus on structured environments like highways and urban roads. These datasets are valuable for high-speed navigation and scenarios with well-defined lanes and traffic rules. However, they do not adequately capture the unique challenges of Less structured environments like parking lots, where:

* **Maneuvering is predominantly at low speeds**, requiring precise control and responsiveness to navigate tight spaces and avoid collisions.
* **Paths are often complex and irregular**, with sharp turns, varying curvature, and obstacles that necessitate sophisticated path-following algorithms.
* **Elevation changes can significantly impact vehicle dynamics**, demanding control algorithms that can maintain stability and trajectory accuracy on slopes.

This dataset aims to fill this gap by providing a dedicated resource for evaluating and improving control algorithms specifically for Less structured environments, enabling the development of more robust and reliable autonomous navigation systems. This data can be used for benchmarking both classical and learning-based control algorithms, facilitating the development of advanced techniques for autonomous driving in challenging scenarios. The dataset is classified into three difficulty levels, providing a diverse range of paths for comprehensive evaluation and benchmarking.


## Dataset Description

This dataset includes:

* **1000+ paths:**  Generated based on the HD map of PNU's parking lot and a slope map built in the Physics Building of PNU. These paths incorporate elevation changes and the complex layout of the parking lot.
* **Annotations:** Each path is annotated with a difficulty level (easy, moderate, or hard) based on its curvature characteristics. This allows for targeted evaluation of control algorithms on different difficulty levels.

**Example use cases:**

* **Control algorithm benchmarking:** Comparing the performance of different control algorithms (e.g., PID, MPC, pure pursuit) in tracking the generated paths, analyzing their ability to handle varying curvature and difficulty levels.
* **Parking lot navigation:** This data can be used for evaluating the performance of autonomous valet parking (AVP) systems in navigating complex parking lot scenarios.
* **Developing learning-based control algorithms:** Training and evaluating reinforcement learning agents or other learning-based controllers to navigate in Less structured environments.
* **Testing path-following algorithms in the presence of disturbances:** Evaluating the robustness of control algorithms to disturbances like wind, tire slip, or sensor noise.
* **Analyzing the impact of different vehicle parameters on control performance:** Studying how factors like vehicle dimensions, mass, and tire characteristics affect path-following accuracy.

## Data Collection

The paths in this dataset were generated using the Hybrid A* algorithm. This algorithm combines the benefits of A* search with continuous state space exploration, making it suitable for finding optimal paths in complex environments with kinodynamic constraints, such as those encountered by vehicles. By incorporating heuristics and continuous state transitions, Hybrid A* efficiently explores the search space and generates smooth, drivable paths.

The algorithm was applied to the HD map of PNU's parking lot and a slope map built in the Physics Building to generate a diverse set of paths with varying difficulty levels.

## Data Format

The dataset is provided in CSV file format. Each CSV file represents a single path and contains the following four columns:

* **ref_x:**  The x-coordinate of the reference path (in meters).
* **ref_y:** The y-coordinate of the reference path (in meters).
* **ref_yaw:** The yaw angle of the reference path (in radians).
* **ref_z:** The z-coordinate (height) of the reference path (in meters).

The difficulty level and the map used for generating the path are encoded in the filename. For example:

* `E_Path120_PB.csv` indicates an easy path (E) generated from the Physics Building (PB) map.
* `M_Path200_EE.csv` indicates a moderate path (M) generated from the EE building (EE) map.
* `H_Path300_EE.csv` indicates a hard path (H) generated from the EE parking lot (EE) map.

## Usage Instructions

[Provide clear instructions on how to use the dataset, including any necessary software or dependencies.]

## Citation

If you use this dataset in your research, please cite the following paper:

[Your paper citation details]

## License

This dataset is released under the **GPL-3.0 license**.

## Acknowledgements

[Acknowledge any individuals or organizations that supported the creation of this dataset.]
