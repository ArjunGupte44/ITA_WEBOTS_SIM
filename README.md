Visualized Testing with the Webots Simulation

Configure the number of agents in the simulation. To facilitate the use of this simulation platform in various scenarios, we created a convenient file-based plugin for input/output. This makes writing/reading simulation input/output to be easily customized and automated.

    Modify the simulation input parameters in webots_sim/webots_ros2_mavic/resource/numSimAgents.txt. The items in the file are listed in the following order:
        Number of Safe Points of Interest (POIs)
        Number of Threat Points of Interest (POIs)
        Number of Humans
        Number of UAVs
        Number of UGVs
    Update the Webots simulation world file to reflect the changes from above by entering the webots_sim/webots_ros2_mavic/webots_ros2_mavic/ directory, and running

python main.py

Build the ROS2 package and launch the simulation from the webots_sim/ directory, by running

colcon build --packages-select webots_ros2_mavic; source install/setup.bash; ros2 launch webots_ros2_mavic robot_launch.py

View a sample task allocation policy output in webots_sim/webots_ros2_mavic/resource/poiAssignments.txt that is structured in the following format:

(POI_x, POI_y, POI_z) [Robot used to reach POI, Agent assigned to control the robot, Agent assigned to classify the POI]

Note: When launching the simulation, there may be a number of "File Not Found" errors. This is due to various segments of code accessing files using their absolute paths, which are specific to the developer's environment setup. To resolve this, modify the problematic absolute paths to match your environment setup.
