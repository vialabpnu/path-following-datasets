# A Path-Planning Datasets in Unstructured Environments for Autonomous Driving

This repository contains datasets for path planning in unstructured environments, with one example being a parking lot map and slope map from Pusan National University (PNU). These datasets were created to address the need for specialized data to develop and evaluate path-planning algorithms for autonomous vehicles and robots navigating in complex, real-world scenarios.

## Background

Traditional path-planning algorithms often rely on structured environments with well-defined rules and predictable obstacles. However, unstructured environments present unique challenges:

* **Complex layouts:** Unstructured environments can have irregular shapes, varying terrain, and diverse obstacles.
* **Limited space:** Maneuvering in tight spaces requires precise and efficient path planning.

Existing datasets for autonomous driving often focus on road driving and may not adequately capture the specific challenges of unstructured environments. This project aims to bridge this gap by providing dedicated datasets for path planning in such scenarios, specifically focusing on **static** environments.

## Dataset Description

This dataset includes:

* **600+ paths:** Collected from various unstructured environments, including:
    * **PNU parking lot and slope map:** This dataset features paths collected within PNU's parking lots, incorporating elevation changes and challenging terrain.
    * [Describe other environments included in the dataset, if any.]
* **Sensor data:**  [Specify the types of sensor data included, e.g., camera, lidar, radar, etc.]
* **Ground truth:** Accurate vehicle position, path, and surrounding environment information.
* **Annotations:**  [Describe the types of annotations provided, e.g., object labels, lane markings, etc.]
* **Static environment:**  The dataset focuses on static obstacles and does not include dynamic objects like pedestrians or moving vehicles.

**Example use cases:**

* **Autonomous navigation in challenging environments:** Developing and evaluating autonomous navigation systems for robots and vehicles in complex, off-road scenarios with static obstacles.
* **Parking lot navigation:**  While not the sole focus, the PNU parking lot data can be used for developing and evaluating autonomous valet parking (AVP) systems in scenarios without dynamic obstacles.
* **Algorithm benchmarking:** Comparing the performance of different path-planning algorithms in unstructured, static environments.

## Data Collection

[Describe the data collection process, including the equipment used, locations, and any specific considerations. **Mention that data was collected in static environments without dynamic obstacles.**]

## Data Format

[Explain the format of the data, including file organization, data structures, and any relevant metadata.]

## Usage Instructions

[Provide clear instructions on how to use the dataset, including any necessary software or dependencies.]

## Citation

If you use this dataset in your research, please cite the following paper:

[Your paper citation details]

## License

[Specify the license under which the dataset is released, e.g., MIT License, Creative Commons, etc.]

## Contributing

Contributions to this dataset are welcome. Please contact [your contact information] for more details.

## Acknowledgements

[Acknowledge any individuals or organizations that supported the creation of this dataset.]
