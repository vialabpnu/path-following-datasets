#!/usr/bin/env python
"""
Update xacro files from vehicle_params.yaml
Ensures consistency between MPC controller parameters and Gazebo simulation physics
"""

import yaml
import os
import re

def read_vehicle_params(yaml_path):
    """Read vehicle parameters from YAML file"""
    with open(yaml_path, 'r', encoding='utf-8') as f:
        return yaml.safe_load(f)

def calculate_inertia(mass, length, width, height):
    """
    Calculate inertia tensor for a rectangular box
    Using standard formulas: I = (1/12) * m * (h^2 + w^2) for rotation about length axis
    """
    ixx = (1.0/12.0) * mass * (height**2 + width**2)
    iyy = (1.0/12.0) * mass * (height**2 + length**2)
    izz = (1.0/12.0) * mass * (width**2 + length**2)
    return ixx, iyy, izz

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

def main():
    # Get script directory
    script_dir = os.path.dirname(os.path.abspath(__file__))

    # Path to vehicle_params.yaml
    yaml_path = os.path.join(script_dir, 'vehicle_params.yaml')

    # Paths to xacro files
    rbcar_description_dir = os.path.join(
        script_dir, '..', 'src', 'ackerman_ros_robot_gazebo_simulation',
        'rbcar_common', 'rbcar_description'
    )

    rbcar_base_xacro = os.path.join(rbcar_description_dir, 'urdf', 'bases', 'rbcar_base.urdf.xacro')
    suspension_wheel_xacro = os.path.join(rbcar_description_dir, 'urdf', 'wheels', 'suspension_wheel.urdf.xacro')

    # Read vehicle parameters
    print("Reading vehicle parameters from: {}".format(yaml_path))
    params = read_vehicle_params(yaml_path)
    print("Parameters: wheelbase={}, weight={}, steering_limit={}, steering_rate={}".format(
        params['wheelbase'], params['weight'],
        params['steering_angle_limit_rad'], params['steering_angle_rate_limit_rad_s']))

    # Update xacro files
    print("\nUpdating xacro files...")
    update_rbcar_base_xacro(rbcar_base_xacro, params)
    update_suspension_wheel_xacro(suspension_wheel_xacro, params)

    print("\n[SUCCESS] Updated all xacro files from vehicle_params.yaml")
    print("Run this script before launching Gazebo to ensure parameter consistency.")

if __name__ == '__main__':
    main()
