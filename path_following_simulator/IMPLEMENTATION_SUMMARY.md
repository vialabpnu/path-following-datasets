# Implementation Summary: Environment Parameters & Experiment Management System

## Overview
This document summarizes the implementation of the environment parameters system and refactored experiment management for the path-following simulator.

---

## What Was Implemented

### Phase 1: Environment Parameters System ✅

#### 1. Created `config/environment_params.yaml`
**Purpose:** Single source of truth for environment conditions (road friction, wind)

**Features:**
- Three road friction levels (professor's specifications):
  - **Icy:** mu=0.1, mu2=0.1
  - **Wet:** mu=0.55, mu2=0.55
  - **Dry:** mu=0.85, mu2=0.85 (default)
- Wind disturbance parameters (for future implementation)
- Active road condition tracking

**Usage:**
```yaml
# Set active condition
active_road_condition: "dry"  # or "wet" or "icy"
```

#### 2. Fixed Xacro Syntax Errors
**File:** `rbcar_description/urdf/wheels/suspension_wheel.urdf.xacro`

**Fixed:**
- Line 100: `{suspension_joint_friction}` → `${suspension_joint_friction}`
- Line 142: `{wheel_joint_friction}` → `${wheel_joint_friction}`

#### 3. Created `config/update_simulation_from_yaml.py`
**Purpose:** Unified script to update BOTH vehicle and environment parameters

**Replaces:** `update_xacro_from_yaml.py` (vehicle params only)

**Features:**
- Updates xacro files from `vehicle_params.yaml` (existing functionality)
- Updates world file friction from `environment_params.yaml` (NEW)
- Command-line options:
  ```bash
  python update_simulation_from_yaml.py --road-condition wet
  ```

**How it works:**
1. Reads `vehicle_params.yaml` → updates xacro files (mass, inertia, wheelbase, steering limits)
2. Reads `environment_params.yaml` → updates world file (ground friction mu/mu2)
3. Single command updates entire simulation configuration

---

### Phase 2: TmuxManager Class ✅

#### Created `MPCSimulationRunner/scripts/tmux_manager.py`
**Purpose:** Clean interface for managing tmux sessions in experiments

**Key Features:**
- Session management (create, kill, cleanup)
- Pane creation and command execution
- Pattern matching (wait for specific output)
- Output monitoring

**Methods:**
```python
tmux = TmuxManager("session_name")
tmux.start_session()                    # Create new session
tmux.create_pane("pane_name")           # Create pane
tmux.send_command("pane_name", "cmd")   # Execute command
tmux.wait_for_pattern("pane", "pattern")  # Wait for output
tmux.kill_session()                     # Cleanup
```

**Use Case:** Automated launching of ROS nodes in separate panes for experiments

---

### Phase 3: Refactored Experiment Controller ✅

#### Created `MPCSimulationRunner/scripts/RunandGatherResults_refactored.py`
**Purpose:** Master controller for running multiple experiments with different parameters

**Key Improvements:**
- **Experiment Configuration System:** Define experiments in structured dictionaries
- **TmuxManager Integration:** Clean process management
- **Environment Parameter Support:** Road conditions per experiment
- **Modular Design:** Separate methods for setup, execution, cleanup

**Example Experiment Config:**
```python
{
    'name': 'wet_road_golfcart',
    'vehicle_params': {
        'wheelbase': 2.48,
        'weight': 600,
        # ... other params
    },
    'environment': {
        'road_condition': 'wet',  # icy/wet/dry
        'wind_speed': 0.0
    },
    'datasets': {
        'enable_easy': True,
        'enable_moderate': False,
        'enable_hard': False
    },
    'mpc_types': ['nonuni_sparse_var'],
    'noise_params': {
        'enable_x': False,
        # ... odometry noise settings
    }
}
```

**Workflow:**
1. Setup experiment (update YAMLs, run update script)
2. Launch ROS stack (TmuxManager)
3. Execute path tests (filtered by difficulty)
4. Collect results
5. Cleanup (kill tmux session)

---

### Phase 4: Updated runSimulator.sh ✅

**Changes:**
- Now calls `update_simulation_from_yaml.py` instead of `update_xacro_from_yaml.py`
- Exports `CAR_WS_PATH` and `DATASET_PATH` environment variables
- Better error handling

**Usage:** (Same as before)
```bash
./runSimulator.sh
```

---

## File Structure

```
path_following_simulator/
├── config/
│   ├── vehicle_params.yaml              # Vehicle properties
│   ├── environment_params.yaml          # Environment properties (NEW)
│   ├── update_xacro_from_yaml.py       # Old (vehicle only)
│   └── update_simulation_from_yaml.py  # New (vehicle + environment)
│
├── src/
│   ├── ackerman_ros_robot_gazebo_simulation/
│   │   ├── rbcar_common/rbcar_description/urdf/
│   │   │   ├── bases/rbcar_base.urdf.xacro       # Updated by script
│   │   │   └── wheels/suspension_wheel.urdf.xacro # Fixed syntax, updated by script
│   │   └── rbcar_sim/rbcar_gazebo/worlds/
│   │       └── new_asphalt_friction_noobstacle.world  # Updated by script
│   │
│   └── MPCSimulationRunner/scripts/
│       ├── RunandGatherResults.py            # Original (preserved)
│       ├── RunandGatherResults_refactored.py # New experiment controller
│       └── tmux_manager.py                   # New tmux interface
│
└── runSimulator.sh                          # Updated to use new script
```

---

## How to Use

### Running Experiments with Different Road Conditions

**Option 1: Manual (for single tests)**
```bash
# Set road condition
python config/update_simulation_from_yaml.py --road-condition wet

# Launch simulation
./runSimulator.sh
```

**Option 2: Automated (for multiple experiments)**
```bash
# Edit RunandGatherResults_refactored.py to define experiments
# Then run:
python src/MPCSimulationRunner/scripts/RunandGatherResults_refactored.py
```

**Option 3: Run specific experiments only**
```bash
python src/MPCSimulationRunner/scripts/RunandGatherResults_refactored.py \
    --experiments wet_road_golfcart icy_road_golfcart
```

### Testing Environment Parameters

**Test friction update:**
```bash
# Dry asphalt
python config/update_simulation_from_yaml.py --road-condition dry
cat src/ackerman_ros_robot_gazebo_simulation/rbcar_sim/rbcar_gazebo/worlds/new_asphalt_friction_noobstacle.world | grep -A2 "mu>"
# Should show: <mu>0.85</mu> <mu2>0.85</mu2>

# Wet asphalt
python config/update_simulation_from_yaml.py --road-condition wet
cat src/ackerman_ros_robot_gazebo_simulation/rbcar_sim/rbcar_gazebo/worlds/new_asphalt_friction_noobstacle.world | grep -A2 "mu>"
# Should show: <mu>0.55</mu> <mu2>0.55</mu2>

# Icy road
python config/update_simulation_from_yaml.py --road-condition icy
cat src/ackerman_ros_robot_gazebo_simulation/rbcar_sim/rbcar_gazebo/worlds/new_asphalt_friction_noobstacle.world | grep -A2 "mu>"
# Should show: <mu>0.1</mu> <mu2>0.1</mu2>
```

---

## Key Distinctions

### Vehicle vs Environment Parameters

**Vehicle Parameters** (xacro files):
- Wheelbase, mass, inertia
- Steering limits and rates
- Joint friction (0.7) - mechanical property

**Environment Parameters** (world file):
- Road surface friction (0.1, 0.55, 0.85)
- Wind disturbance
- External conditions

---

## Migration Guide

### For Existing Scripts
If you have existing scripts using `update_xacro_from_yaml.py`:

**Before:**
```bash
python config/update_xacro_from_yaml.py
```

**After:**
```bash
python config/update_simulation_from_yaml.py
```

The new script is backward compatible - it does everything the old one did, plus environment updates.

### For Experiment Runners
If using `RunandGatherResults.py`:

**Option 1:** Keep using the original (still works)
**Option 2:** Migrate to `RunandGatherResults_refactored.py` for:
- Multi-experiment support
- Environment parameter control
- Better process management

---

## Testing Checklist

- [ ] Test friction updates (dry/wet/icy)
- [ ] Test vehicle parameter updates
- [ ] Test TmuxManager in container
- [ ] Run single experiment end-to-end
- [ ] Run multi-experiment sequence
- [ ] Verify no process leaks
- [ ] Check results organization

---

## Future Enhancements

1. **Wind Disturbance Implementation**
   - Add Gazebo wind plugin to `rbcar_base.gazebo.xacro`
   - Enable/disable wind in `environment_params.yaml`

2. **Wheel Surface Friction**
   - Add `<gazebo>` friction tags to wheel collision elements
   - Link to environment parameters

3. **Configuration Validation**
   - Add schema validation for experiment configs
   - Automatic parameter range checking

4. **Result Analysis Tools**
   - Automated comparison across experiments
   - Visualization of friction effects

---

## Summary

✅ **Completed:**
1. Environment parameters system with 3 friction levels
2. Fixed xacro syntax errors
3. Unified update script (vehicle + environment)
4. TmuxManager for process management
5. Refactored experiment controller
6. Updated runSimulator.sh

✅ **Benefits:**
- Single source of truth for environment conditions
- Easy testing of different road conditions
- Automated multi-experiment execution
- Clean process management
- Maintainable, extensible codebase

✅ **Ready for:**
- Testing different friction coefficients
- Running experiment matrices
- Further environment parameter additions (wind, etc.)
