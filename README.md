# BebopControl Workspace


## Installation Steps

Follow these steps to set up the workspace:

```bash
# Clone the repository
git clone https://github.com/Gfernandes10/BebopControl.git

# Navigate to the workspace
cd BebopControl

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

