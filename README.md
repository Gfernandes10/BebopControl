# Meu Workspace ROS

## Dependências de Sistema

Certifique-se de que as seguintes dependências estejam instaladas no seu sistema antes de compilar o workspace:

```bash
git clone https://github.com/Gfernandes10/BebopControl.git
cd BebopControl
sudo apt update
sudo apt install build-essential python3-rosdep python3-catkin-tools
sudo apt install libusb-dev python3-osrf-pycommon libspnav-dev libbluetooth-dev libcwiid-dev libgoogle-glog-dev
sudo apt install ros-noetic-mavros ros-noetic-octomap-ros
rosdep install --from-paths src --ignore-src -r -y
catkin build
source devel/setup.bash
```

