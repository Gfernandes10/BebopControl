# Meu Workspace ROS

## Dependências de Sistema

Certifique-se de que as seguintes dependências estejam instaladas no seu sistema antes de compilar o workspace:


Dependências 
```bash
sudo apt update
sudo apt install build-essential python3-rosdep python3-catkin-tools
sudo apt install libusb-dev python3-osrf-pycommon libspnav-dev libbluetooth-dev libcwiid-dev libgoogle-glog-dev
sudo apt install libavahi-client-dev
sudo apt install ros-noetic-mavros ros-noetic-octomap-ros ros-noetic-joy ros-noetic-joy-teleop 
sudo apt install ros-noetic-teleop-twist-joy
sudo ln -s /usr/bin/python3 /usr/bin/python
```
Add this line in your ~/.bashrc :
```bash
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:~/BebopControl/devel/lib/parrot_arsdk
```


```bash
git clone https://github.com/Gfernandes10/BebopControl.git
cd BebopControl
rosdep install --from-paths src --ignore-src -r -y
catkin build
source devel/setup.bash
```

