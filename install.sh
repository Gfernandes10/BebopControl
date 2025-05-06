#!/bin/bash
sudo apt install python3-pip -y

# Initialize and update the submodules
git submodule update --init --recursive

# Check and add environment variables to bashrc if not already present
if ! grep -q 'export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:~/BebopControl/devel/lib/parrot_arsdk' ~/.bashrc; then
  echo 'export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:~/BebopControl/devel/lib/parrot_arsdk' >> ~/.bashrc
fi

if ! grep -q 'export MY_WORKSPACE_NAME="BebopControl"' ~/.bashrc; then
  echo 'export MY_WORKSPACE_NAME="BebopControl"' >> ~/.bashrc
fi

echo 'source ~/BebopControl/devel/setup.bash' >> ~/.bashrc

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

# Source the workspace
source devel/setup.bash
