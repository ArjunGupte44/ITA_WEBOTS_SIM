## Setup
1. Install [ROS2 Humble](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)
```
locale  # check for UTF-8

sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

locale  # verify settings

sudo apt install software-properties-common
sudo add-apt-repository universe

sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update
sudo apt upgrade

sudo apt install ros-humble-desktop
```

2. Install [Webots v2023a](https://github.com/cyberbotics/webots/releases?q=2023a&expanded=true)

## Visualized Testing with the Webots Simulation
- Configure the number of agents in the simulation.
  To facilitate the use of this simulation platform in various scenarios, we created a convenient file-based plugin for input/output. This makes writing/reading simulation input/output to be easily customized and automated.  
  1. Modify the simulation input parameters in `webots_ros2_mavic/resource/numSimAgents.txt`. The items in the file are listed in the following order:
     - Number of Safe Points of Interest (POIs)
     - Number of Threat Points of Interest (POIs)
     - Number of Humans
     - Number of UAVs
     - Number of UGVs
  2. Update the Webots simulation world file to reflect the changes from above by entering the `webots_ros2_mavic/webots_ros2_mavic/` directory, and running 
  ```
  python main.py
  ```
  
- Build the ROS2 package and launch the simulation from the repository root directory, by running
  ```
  colcon build --packages-select webots_ros2_mavic; source install/setup.bash; ros2 launch webots_ros2_mavic robot_launch.py
  ```
- View a sample task allocation policy output in `webots_ros2_mavic/resource/poiAssignments.txt` that is structured in the following format:
  ```
  (POI_x, POI_y, POI_z) [Robot used to reach POI, Agent assigned to control the robot, Agent assigned to classify the POI]
  ```
**Note: When launching the simulation, there may be a number of "File Not Found" errors. This is due to various segments of code accessing files using their absolute paths, which are specific to the developer's environment setup. To resolve this, modify the problematic absolute paths to match your environment setup.**

## Citation
If you find the code or the paper useful for your research, please cite the following papers:
```
@misc{wang2023initial,
      title={Initial Task Assignment in Multi-Human Multi-Robot Teams: An Attention-enhanced Hierarchical Reinforcement Learning Approach}, 
      author={Ruiqi Wang and Dezhong Zhao and Arjun Gupte and Byung-Cheol Min},
      year={2023},
      eprint={2310.04979},
      archivePrefix={arXiv},
      primaryClass={cs.RO}
}
```
