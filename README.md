# A Path-Planning Dataset in Unstructured Environments

This repository contains a dataset for control algorithm benchmarking in unstructured environments, with a focus on a parking lot map and slope map from Pusan National University (PNU). This dataset was created to address the need for specialized data to evaluate the performance of control algorithms for autonomous vehicles and robots navigating in complex, real-world scenarios.

## Background

Traditional control algorithms are often evaluated in structured environments with well-defined paths and predictable conditions. However, unstructured environments present unique challenges for control algorithms:

* **Complex paths:** Unstructured environments often require navigating complex paths with varying curvature, elevation changes, and obstacles.
* **Limited space:** Maneuvering in tight spaces demands precise and responsive control.
* **Low-speed navigation:** Compared to structured environments where higher speeds are common, unstructured environments often involve lower speeds due to the increased complexity and need for careful maneuvering. This necessitates accurate control at lower speeds.

Existing datasets for autonomous driving often focus on road driving and may not adequately capture the specific challenges of unstructured environments. This project aims to bridge this gap by providing a dedicated dataset for evaluating control algorithms in such scenarios.

## Dataset Description

This dataset includes:

* **1000+ paths:**  Generated based on the HD map of PNU's parking lot and a slope map built in the Physics Building of PNU. These paths incorporate elevation changes and the complex layout of the parking lot.
* **Annotations:** Each path is annotated with a difficulty level (easy, moderate, or hard) based on its curvature characteristics. This allows for targeted evaluation of control algorithms on different difficulty levels.

**Example use cases:**

* **Parking lot navigation:** This data can be used for evaluating the performance of autonomous valet parking (AVP) systems in navigating complex parking lot scenarios.
* **Control algorithm benchmarking:** Comparing the performance of different control algorithms in tracking the generated paths, analyzing their ability to handle varying curvature and difficulty levels.

## Data Collection

The paths in this dataset were generated using the Hybrid A* algorithm. This algorithm combines the benefits of A* search with continuous state space exploration, making it suitable for finding optimal paths in complex environments with kinodynamic constraints, such as those encountered by vehicles. By incorporating heuristics and continuous state transitions, Hybrid A* efficiently explores the search space and generates smooth, drivable paths.

The algorithm was applied to the HD map of PNU's parking lot and a slope map built in the Physics Building to generate a diverse set of paths with varying difficulty levels.

## Data Format

[Explain the format of the data, including file organization, data structures, and any relevant metadata. Specify how the paths are represented, e.g., as a sequence of waypoints, etc. **Include how the difficulty annotations are encoded in the data.**]

## Usage Instructions

[Provide clear instructions on how to use the dataset, including any necessary software or dependencies.]

## Citation

If you use this dataset in your research, please cite the following paper:

[Your paper citation details]

## License

This dataset is released under the **GPL-3.0 license**.

## Acknowledgements

[Acknowledge any individuals or organizations that supported the creation of this dataset.]
