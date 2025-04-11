# ITA_AeHRL
This repository contains the source code for our paper: "Initial Task Assignment in Multi-Human Multi-Robot Teams: An Attention-enhanced Hierarchical Reinforcement Learning Approach".
For more details, please refer to [our project website](https://sites.google.com/view/ita-aehrl).


## Abstract
Multi-human multi-robot teams (MH-MR) obtain tremendous potential in tackling intricate and massive missions by merging distinct strengths and expertise of individual members. The inherent heterogeneity of these teams necessitates advanced initial task allocation (ITA) methods that align tasks with the intrinsic capabilities of team members from the outset. While existing reinforcement learning approaches show encouraging results, they might fall short in addressing the nuances of long-horizon ITA problems, particularly in settings with large-scale MH-MR teams or multifaceted tasks. To bridge this gap, we propose an attention-enhanced hierarchical reinforcement learning approach that decomposes the complex ITA problem into structured sub-problems, facilitating more efficient allocations. To bolster sub-policy learning, we introduce a hierarchical cross-attribute attention (HCA) mechanism, encouraging each sub-policy within the hierarchy to discern and leverage the specific nuances in the state space that are crucial for its respective decision-making phase. Through an extensive environmental surveillance case study, we demonstrate the benefits of our model and the HCA inside.

## Overview

### A diagrammatic representation of the formulated hierarchical contextual multi-attribute decision-making process
<p align="center">
<img src="/figures/concept.png" width="450" />
</p>

### An illustration of the proposed AeHRL framework with an example of a hierarchy with two options.
<p align="center">
<img src="/figures/Framework_v4_00.jpg" width="950" />
</p>



## Setup

1. In an environment with Python 3.x, install the required Python package
```
pip install -r requirements.txt
```
2. Install [OpenAI Baselines](https://github.com/openai/baselines#installation) 
```
git clone https://github.com/openai/baselines.git
cd baselines
pip install -e.
```
3. Install [ROS2 Humble](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)
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

4. Install [Webots v2023a](https://github.com/cyberbotics/webots/releases?q=2023a&expanded=true)

## Usage

Note: As we stated in the paper, We have uploaded the code for the AeHrl_3 experimental setup, and you could change the number of sub_policy and the modal of attention if you want to add
or subtract the number of policies in `rl` and `env_sim`:

The repository is organized in four parts: 
- `env_config/` folder contains configurations and policies used in the simulator.
- `env_sim/` folder contains the simulation environment.
- `webots_sim/` folder contains the Webots simulation environment.
- `rl/` contains the code for the RL policy networks, attention network, and ppo algorithm. 
- `trained_models/` Store your trained model.

### Training
- Modify the configurations.
  1. Environment configurations: Modify `env_config/configs/config.py`. 


  2. PPO and network configurations: modify `arguments.py`

- After you change the configurations, run
  ```
  python train.py 
  ```
- The checkpoints and configuration files will be saved to the folder specified by `output_dir` in `arguments.py`.

### Testing
Please modify the test arguments in lines 20-33 of `test.py` (**Don't set the argument values in terminal!**), and run   
```
python test.py 
```
Note that the `config.py` and `arguments.py` in the testing folder will be loaded, instead of those in the root directory.  
The testing results are logged in `trained_models/your_output_dir/test/` folder, and are also printed on terminal.

### Plot the training curves
```
python plot.py
```



### Visualized Testing with the Webots Simulation
- Configure the number of agents in the simulation.
  To facilitate the use of this simulation platform in various scenarios, we created a convenient file-based plugin for input/output. This makes writing/reading simulation input/output to be easily customized and automated.  
  1. Modify the simulation input parameters in `webots_sim/webots_ros2_mavic/resource/numSimAgents.txt`. The items in the file are listed in the following order:
     - Number of Safe Points of Interest (POIs)
     - Number of Threat Points of Interest (POIs)
     - Number of Humans
     - Number of UAVs
     - Number of UGVs
  2. Update the Webots simulation world file to reflect the changes from above by entering the `webots_sim/webots_ros2_mavic/webots_ros2_mavic/` directory, and running 
  ```
  python main.py
  ```
  
- Build the ROS2 package and launch the simulation from the `webots_sim/` directory, by running
  ```
  colcon build --packages-select webots_ros2_mavic; source install/setup.bash; ros2 launch webots_ros2_mavic robot_launch.py
  ```
- View a sample task allocation policy output in `webots_sim/webots_ros2_mavic/resource/poiAssignments.txt` that is structured in the following format:
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
