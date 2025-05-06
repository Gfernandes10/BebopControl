# BebopControl Workspace

## Overview

This repository is associated with the following publication:

**Gabriel Fernandes, Andre G. S. Conceicao, Humberto X. De Araújo.** "Control of an Underactuated Quadrotor Using LQR and Visual Markers." *2025 IEEE International Conference on Autonomous Robot Systems and Competitions (ICARSC)*, pp. 58-63, April 2025. DOI: [10.1109/ICARSC65809.2025.10970126](https://doi.org/10.1109/ICARSC65809.2025.10970126)

### Abstract
In this study, a robust Linear Quadratic Regulator (LQR) controller was developed and evaluated for the Bebop 2 quadrotor drone. Polytopic uncertainties in the dynamic model were taken into account in the control design, ensuring robustness. The drone's localization was achieved using fiducial markers and a Kalman Filter for state estimation, integrating visual and positional data. The controller design utilized Linear Matrix Inequalities (LMIs) to optimize performance while accounting for system constraints. A circular trajectory in three-dimensional space was employed to assess the controller's effectiveness in simulation. Results demonstrated precise trajectory tracking, stability, and robust performance despite parameter uncertainties. These findings highlight the applicability of the proposed framework for real-world scenarios involving underactuated UAVs.

### Repository Overview

This repository contains the implementation of the robust LQR controller and the associated tools for the Bebop 2 quadrotor drone. It includes:
- **Controller Design**: Implementation of the LQR controller with robustness to uncertainties.
- **State Estimation**: Kalman Filter for integrating visual and positional data.
- **Simulation Tools**: Scripts and configurations for simulating the drone's behavior.
- **Workspace Structure**: This repository is designed to function as a ROS workspace, making it easy to build and run the provided packages.

## Installation Steps

Follow these steps to set up the workspace:

```bash
# Clone the repository
git clone https://github.com/Gfernandes10/BebopControl.git

# Navigate to the workspace
cd BebopControl

# Initialize and update the submodules:
git submodule update --init --recursive

# Add environment variables to your bashrc
echo 'export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:~/BebopControl/devel/lib/parrot_arsdk' >> ~/.bashrc
echo 'export MY_WORKSPACE_NAME="BebopControl"' >> ~/.bashrc

# Update workspace dependencies
wstool update -t src

# Update system packages
sudo apt update

# Install dependencies using rosdep
rosdep install --from-paths src --ignore-src -r -y

# Source the updated bashrc
source ~/.bashrc

# Build the workspace
catkin build

# Source the workspace setup file
source devel/setup.bash
```

## Running the Workspace

To launch the workspace and start the packages, use the following command:

For a clearer view of the command logs, it is recommended to execute the following commands in separate terminals:

1. In the first terminal, launch the Gazebo simulation:
   ```bash
   roslaunch interface_bebop bebopgazeboaruco.launch
   ```

2. In the second terminal, launch the drone control interface:
   ```bash
   roslaunch interface_bebop sim_bebop_control.launch
   ```

Use the second terminal (where `sim_bebop_control.launch` is running) to control the drone. The first terminal, which starts Gazebo, generates an excessive amount of log messages, making it less suitable for monitoring commands.

## Available Commands

After launching the workspace, the following commands are available:

- **w**: Move forward
- **s**: Move backward
- **a**: Move left
- **d**: Move right
- **t**: Takeoff
- **l**: Land
- **1**: Start control identification for z-axis
- **2**: Start control identification for y-axis
- **3**: Start control identification for x-axis
- **4**: Start control identification for yaw-axis
- **c**: Enable controller
- **h**: Show help commands
- **CTRL+C**: Exit program

## Usage Notes

- It is recommended to first issue the **takeoff** command (`t`) to lift the drone.
- After takeoff, activate the controller by pressing **c**.
- Any manual movement command (e.g., `w`, `s`, `a`, `d`) will automatically disable the position controller.

## Circular Trajectory Service

To execute a circular trajectory with the tag centralized and reproduce the results mentioned in the associated publication, you can call the following service:

```bash
rosservice call /SetDiagCircPath "radius: 0.3
angular_velocity: 0.05
height: 1.0"
```

This will command the drone to follow a circular path with a radius of 0.3 meters, an angular velocity of 0.05 rad/s, and a height of 1.0 meter.

## Fixed Position Service

To set the drone to a fixed position, you can call the following service:

```bash
rosservice call /SetReferencePose "x: <value>
y: <value>
z: <value>
heading: <value>"
```

Replace `<value>` with the desired coordinates (`x`, `y`, `z`) and heading angle. For example:

```bash
rosservice call /SetReferencePose "x: 0.0
y: 0.3
z: 1.0
heading: 0.0"
```

This will command the drone to move to the specified position and orientation.

## Log Storage

Some CSV files containing logs of the drone's operation are saved in the `csvLogs` folder within the workspace. These logs can be useful for analyzing the drone's performance and behavior during operation.

## Citation

If you use this repository in your research or projects, please cite the following publication:

**Gabriel Fernandes, Andre G. S. Conceicao, Humberto X. De Araújo.** "Control of an Underactuated Quadrotor Using LQR and Visual Markers." *2025 IEEE International Conference on Autonomous Robot Systems and Competitions (ICARSC)*, pp. 58-63, April 2025. DOI: [10.1109/ICARSC65809.2025.10970126](https://doi.org/10.1109/ICARSC65809.2025.10970126)


