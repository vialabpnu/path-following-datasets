# Vehicle Parameter Synchronization System

## Overview
This system ensures consistency between `vehicle_params.yaml` and the Gazebo simulation xacro files. It synchronizes vehicle parameters from the YAML configuration to the URDF xacro files that define the robot's physical properties.

## Files Updated

### 1. Created: `update_xacro_from_yaml.py`
**Location:** `config/update_xacro_from_yaml.py`

**Purpose:** Automatically synchronizes vehicle parameters from YAML to xacro files

**Parameter Mappings:**
```
vehicle_params.yaml              →  Target Xacro Files
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
weight (600 kg)                  →  rbcar_base.urdf.xacro::mass
length, width, height            →  rbcar_base.urdf.xacro::inertia (calculated)
wheelbase (2.48 m)               →  suspension_wheel.urdf.xacro::wheelbase
steering_angle_limit_rad (0.444) →  suspension_wheel.urdf.xacro::steer_limit
steering_angle_rate_limit_rad_s  →  suspension_wheel.urdf.xacro::servo_no_load_speed
                                     (0.14 rad/s)
```

**Usage:**
```bash
cd path_following_simulator/config
python update_xacro_from_yaml.py
```

**Output Example:**
```
Reading vehicle parameters from: .../vehicle_params.yaml
Parameters: wheelbase=2.48, weight=600, steering_limit=0.444, steering_rate=0.14

Updating xacro files...
Updated rbcar_base.urdf.xacro: mass=600, ixx=216.5, iyy=843.9, izz=771.4
Updated suspension_wheel.urdf.xacro: wheelbase=2.48, steer_limit=0.444, servo_rate=0.14

[SUCCESS] Updated all xacro files from vehicle_params.yaml
Run this script before launching Gazebo to ensure parameter consistency.
```

### 2. Updated: `noisy_odom_generator.py`
**Location:** `rbcar_localization/scripts/noisy_odom_generator.py`

**Changes:**
- Enhanced covariance update logic (lines 71-108)
- Updates pose covariance indices: 0 (x), 7 (y), 35 (yaw)
- Updates twist covariance index: 0 (linear x velocity)
- Python 2.7 compatible (ROS Melodic)

**Features:**
- Adds Gaussian noise to odometry measurements (x, y, heading, speed)
- Updates covariance matrices to reflect added uncertainty
- Configurable noise parameters via command-line arguments

### 3. Updated: `RunandGatherResults.py`
**Location:** `MPCSimulationRunner/scripts/RunandGatherResults.py`

**Enhancements:**
- **Dataset Filtering:** Filter paths by difficulty (Easy/Moderate/Hard)
  - Prefix-based filtering: `E_` (Easy), `M_` (Moderate), `H_` (Hard)
- **Configurable MPC Types:** Run different MPC algorithms per evaluation
  - Supports: `uniform`, `fsmpc`, and custom types
- **Enhanced Parameters:**
  - `dataset_class_params`: Control which difficulty levels to run
  - `run_params`: Specify which MPC types to use per run

**Example Configuration:**
```python
dataset_class_params = [
    {'enable_easy': True, 'enable_moderate': False, 'enable_hard': False},
    {'enable_easy': False, 'enable_moderate': True, 'enable_hard': False},
    {'enable_easy': False, 'enable_moderate': False, 'enable_hard': True}
]

run_params = [
    {'which_mpc': ['uniform', 'fsmpc']},
    {'which_mpc': ['uniform', 'fsmpc']},
    {'which_mpc': ['uniform', 'fsmpc']}
]
```

## Workflow

### Before Running Simulations:
1. Edit `vehicle_params.yaml` with desired vehicle parameters
2. Run `python update_xacro_from_yaml.py` to sync xacro files
3. Launch Gazebo simulation (parameters will be consistent)

### For Automated Evaluation:
The `RunandGatherResults.py` script automatically:
1. Launches noisy odometry node with configured noise parameters
2. Filters datasets by difficulty level
3. Runs specified MPC algorithms on selected paths
4. Collects and organizes results

## Key Benefits

1. **Single Source of Truth:** `vehicle_params.yaml` is the authoritative source
2. **Consistency:** MPC controller and Gazebo simulation use identical parameters
3. **Easy Updates:** Change parameters in one place, sync with one command
4. **Proper Inertia:** Automatically calculates correct inertia tensor from dimensions
5. **Enhanced Testing:** Run comprehensive evaluations with dataset filtering

## Important Notes

- **Run sync script before every simulation** if you've changed `vehicle_params.yaml`
- Inertia is calculated assuming a rectangular box model
- All updated files maintain Python 2.7 compatibility for ROS Melodic
- Covariance updates in odometry provide better sensor modeling (useful for future EKF/UKF integration)

## Files Modified Summary

```
✓ config/update_xacro_from_yaml.py (CREATED)
✓ config/vehicle_params.yaml (REFERENCE SOURCE)
✓ rbcar_description/urdf/bases/rbcar_base.urdf.xacro (AUTO-UPDATED)
✓ rbcar_description/urdf/wheels/suspension_wheel.urdf.xacro (AUTO-UPDATED)
✓ rbcar_localization/scripts/noisy_odom_generator.py (ENHANCED)
✓ MPCSimulationRunner/scripts/RunandGatherResults.py (ENHANCED)
```
