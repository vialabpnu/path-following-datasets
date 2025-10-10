#!/usr/bin/env python
"""
Update simulation files from YAML configuration
- Updates xacro files from vehicle_params.yaml (vehicle properties)
- Updates world files from environment_params.yaml (environment properties)
Ensures consistency between parameters and simulation physics
"""

import yaml
import os
import re
import argparse

def read_yaml(yaml_path):
    """Read YAML file"""
    with open(yaml_path, 'r', encoding='utf-8') as f:
        return yaml.safe_load(f)

def write_yaml(yaml_path, data):
    """Write YAML file"""
    with open(yaml_path, 'w', encoding='utf-8') as f:
        yaml.dump(data, f, default_flow_style=False)

def calculate_inertia(mass, length, width, height):
    """
    Calculate inertia tensor for a rectangular box
    Using standard formulas: I = (1/12) * m * (h^2 + w^2) for rotation about length axis
    """
    ixx = (1.0/12.0) * mass * (height**2 + width**2)
    iyy = (1.0/12.0) * mass * (height**2 + length**2)
    izz = (1.0/12.0) * mass * (width**2 + length**2)
    return ixx, iyy, izz

# ===== Vehicle Parameter Updates (Xacro Files) =====

def update_rbcar_base_xacro(xacro_path, params):
    """Update rbcar_base.urdf.xacro with mass and inertia values"""
    with open(xacro_path, 'r') as f:
        content = f.read()

    # Calculate inertia based on vehicle dimensions
    mass = params['weight']
    length = params['length']
    width = params['width']
    height = params['height']
    ixx, iyy, izz = calculate_inertia(mass, length, width, height)

    # Update mass value
    content = re.sub(
        r'<mass value="[\d.]+"\s*/?>',
        '<mass value="{}" />'.format(mass),
        content
    )

    # Update inertia values
    content = re.sub(
        r'<inertia\s+ixx="[\d.]+".*?izz="[\d.]+"\s*/?>',
        '<inertia  ixx="{:.1f}" ixy="0.0"  ixz="0.0"  iyy="{:.1f}"  iyz="0.0"  izz="{:.1f}" />'.format(ixx, iyy, izz),
        content
    )

    with open(xacro_path, 'w') as f:
        f.write(content)

    print("Updated {}: mass={}, ixx={:.1f}, iyy={:.1f}, izz={:.1f}".format(
        os.path.basename(xacro_path), mass, ixx, iyy, izz))

def update_suspension_wheel_xacro(xacro_path, params):
    """Update suspension_wheel.urdf.xacro with wheelbase, steering limits, and rates"""
    with open(xacro_path, 'r') as f:
        content = f.read()

    wheelbase = params['wheelbase']
    steer_limit = params['steering_angle_limit_rad']
    steer_rate = params['steering_angle_rate_limit_rad_s']

    # Update wheelbase property
    content = re.sub(
        r'<xacro:property name="wheelbase" value="[\d.]+"\s*/?>',
        '<xacro:property name="wheelbase" value="{}"/>'.format(wheelbase),
        content
    )

    # Check if steer_limit property exists, if not add it after degrees_45
    if 'name="steer_limit"' not in content:
        # Add steer_limit property after degrees_45
        content = re.sub(
            r'(<xacro:property name="degrees_45" value="[\d.]+"\s*/>)',
            r'\1\n  <xacro:property name="steer_limit" value="{}"/>'.format(steer_limit),
            content
        )
    else:
        # Update existing steer_limit
        content = re.sub(
            r'<xacro:property name="steer_limit" value="[\d.]+"\s*/?>',
            '<xacro:property name="steer_limit" value="{}"/>'.format(steer_limit),
            content
        )

    # Update servo_no_load_speed
    content = re.sub(
        r'<xacro:property name="servo_no_load_speed" value="[\d.]+"\s*/?>',
        '<xacro:property name="servo_no_load_speed" value="{}"/>'.format(steer_rate),
        content
    )

    # Replace degrees_45 with steer_limit in steering joint limits
    # Handle both formats: ${-degrees_45} and -${degrees_45}
    content = re.sub(
        r'<limit lower="\$\{-degrees_45\}" upper="\$\{degrees_45\}"',
        '<limit lower="-${steer_limit}" upper="${steer_limit}"',
        content
    )
    content = re.sub(
        r'<limit lower="-\$\{degrees_45\}" upper="\$\{degrees_45\}"',
        '<limit lower="-${steer_limit}" upper="${steer_limit}"',
        content
    )

    with open(xacro_path, 'w') as f:
        f.write(content)

    print("Updated {}: wheelbase={}, steer_limit={}, servo_rate={}".format(
        os.path.basename(xacro_path), wheelbase, steer_limit, steer_rate))

# ===== Environment Parameter Updates (World Files) =====

def update_world_friction(world_path, env_params):
    """Update world file with road friction coefficients"""
    with open(world_path, 'r') as f:
        content = f.read()

    # Get active road condition
    active_condition = env_params.get('active_road_condition', 'dry')
    road_conditions = env_params['road_conditions']

    if active_condition not in road_conditions:
        print("Warning: Active road condition '{}' not found, using 'dry'".format(active_condition))
        active_condition = 'dry'

    friction_params = road_conditions[active_condition]
    mu = friction_params['mu']
    mu2 = friction_params['mu2']

    # Update friction values in asphalt plane section
    # Look for the friction section with comment "Friction of Dry Asphalt"
    friction_pattern = r'(<!-- Friction of Dry Asphalt \(Change the friction here\) -->.*?<friction>.*?<ode>.*?)<mu>([\d.]+)</mu>(.*?)<mu2>([\d.]+)</mu2>'

    def replace_friction(match):
        return '{}<mu>{}</mu>{}<mu2>{}</mu2>'.format(
            match.group(1), mu, match.group(3), mu2
        )

    content = re.sub(friction_pattern, replace_friction, content, flags=re.DOTALL)

    with open(world_path, 'w') as f:
        f.write(content)

    print("Updated {}: road_condition={}, mu={}, mu2={}".format(
        os.path.basename(world_path), active_condition, mu, mu2))

def update_vehicle_params(vehicle_params_path, vehicle_params_update):
    """Update vehicle_params.yaml with new values"""
    params = read_yaml(vehicle_params_path)
    params.update(vehicle_params_update)
    write_yaml(vehicle_params_path, params)
    print("Updated vehicle_params.yaml")

def update_environment_params(environment_params_path, road_condition=None):
    """Update environment_params.yaml with new road condition"""
    params = read_yaml(environment_params_path)

    if road_condition:
        if road_condition not in params['road_conditions']:
            print("Error: Unknown road condition '{}'".format(road_condition))
            print("Available: {}".format(list(params['road_conditions'].keys())))
            return params
        params['active_road_condition'] = road_condition
        write_yaml(environment_params_path, params)
        print("Updated environment_params.yaml: active_road_condition={}".format(road_condition))

    return params

# ===== Main Function =====

def main():
    parser = argparse.ArgumentParser(description='Update simulation files from YAML parameters')
    parser.add_argument('--road-condition', type=str, choices=['icy', 'wet', 'dry'],
                       help='Set road friction condition')
    parser.add_argument('--vehicle-params', type=str,
                       help='Path to vehicle_params.yaml (default: config/vehicle_params.yaml)')
    parser.add_argument('--environment-params', type=str,
                       help='Path to environment_params.yaml (default: config/environment_params.yaml)')

    args = parser.parse_args()

    # Get script directory
    script_dir = os.path.dirname(os.path.abspath(__file__))

    # Paths to YAML files
    vehicle_params_path = args.vehicle_params or os.path.join(script_dir, 'vehicle_params.yaml')
    environment_params_path = args.environment_params or os.path.join(script_dir, 'environment_params.yaml')

    # Paths to xacro and world files
    simulation_dir = os.path.join(script_dir, '..', 'src', 'ackerman_ros_robot_gazebo_simulation')
    rbcar_description_dir = os.path.join(simulation_dir, 'rbcar_common', 'rbcar_description')
    worlds_dir = os.path.join(simulation_dir, 'rbcar_sim', 'rbcar_gazebo', 'worlds')

    rbcar_base_xacro = os.path.join(rbcar_description_dir, 'urdf', 'bases', 'rbcar_base.urdf.xacro')
    suspension_wheel_xacro = os.path.join(rbcar_description_dir, 'urdf', 'wheels', 'suspension_wheel.urdf.xacro')
    world_file = os.path.join(worlds_dir, 'new_asphalt_friction_noobstacle.world')

    # ===== Update Vehicle Parameters =====
    print("=" * 60)
    print("UPDATING VEHICLE PARAMETERS (Xacro Files)")
    print("=" * 60)
    print("Reading vehicle parameters from: {}".format(vehicle_params_path))
    vehicle_params = read_yaml(vehicle_params_path)

    print("Parameters: wheelbase={}, weight={}, steering_limit={}, steering_rate={}".format(
        vehicle_params['wheelbase'], vehicle_params['weight'],
        vehicle_params['steering_angle_limit_rad'], vehicle_params['steering_angle_rate_limit_rad_s']))

    print("\nUpdating xacro files...")
    update_rbcar_base_xacro(rbcar_base_xacro, vehicle_params)
    update_suspension_wheel_xacro(suspension_wheel_xacro, vehicle_params)

    # ===== Update Environment Parameters =====
    print("\n" + "=" * 60)
    print("UPDATING ENVIRONMENT PARAMETERS (World File)")
    print("=" * 60)
    print("Reading environment parameters from: {}".format(environment_params_path))

    # Update road condition if specified
    env_params = update_environment_params(environment_params_path, args.road_condition)

    print("\nUpdating world file...")
    update_world_friction(world_file, env_params)

    print("\n" + "=" * 60)
    print("[SUCCESS] Updated all simulation files")
    print("=" * 60)
    print("Vehicle: xacro files updated with vehicle_params.yaml")
    print("Environment: world file updated with environment_params.yaml")
    print("Active road condition: {}".format(env_params['active_road_condition']))
    print("\nRun this script before launching Gazebo to ensure parameter consistency.")

if __name__ == '__main__':
    main()
