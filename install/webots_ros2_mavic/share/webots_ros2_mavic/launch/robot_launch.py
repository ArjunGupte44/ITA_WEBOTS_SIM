#!/usr/bin/env python

# Copyright 1996-2023 Cyberbotics Ltd.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Launch Webots Mavic 2 Pro driver."""

import os
import pathlib
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch.substitutions.path_join_substitution import PathJoinSubstitution
from launch import LaunchDescription
from launch_ros.actions import Node
import launch
from ament_index_python.packages import get_package_share_directory, get_packages_with_prefixes
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from webots_ros2_driver.webots_launcher import WebotsLauncher
from webots_ros2_driver.wait_for_controller_connection import WaitForControllerConnection
from webots_ros2_driver.utils import controller_url_prefix
import xacro
import random


import sys
sys.path.insert(0, '/home/arjun/SMART-LAB-ITAP-WEBOTS/webots_ros2_mavic/src')
print(sys.path)
from main import itapSim

def get_ros2_nodes(*args):
    package_dir_mavic = get_package_share_directory('webots_ros2_mavic')

    #robot_description_mavic = pathlib.Path(os.path.join(package_dir_mavic, 'resource', 'mavic_webots.urdf.xacro')).read_text()

    package_dir_turtle = get_package_share_directory('webots_ros2_turtlebot')
    use_nav = LaunchConfiguration('nav', default=False)
    use_slam = LaunchConfiguration('slam', default=False)
    robot_description_turtle = pathlib.Path(os.path.join(package_dir_turtle, 'resource', 'turtlebot_webots.urdf')).read_text()
    ros2_control_params = os.path.join(package_dir_turtle, 'resource', 'ros2control.yaml')
    nav2_params = os.path.join(package_dir_turtle, 'resource', 'nav2_params.yaml')
    nav2_map = os.path.join(package_dir_turtle, 'resource', 'turtlebot3_burger_example_map.yaml')
    use_sim_time = LaunchConfiguration('use_sim_time', default=True)

    launchList = []

    #Get num robot info from object of class
    numUAVs = itapSim.getNumUAVs()
    numUGVs = itapSim.getNumUGVs()
    numHumans = itapSim.getNumHumans()

    #Get POIs assigned to UAVs
    uavWaypoints = 'uavCoords.txt'
    ugvWaypoints = 'ugvCoords.txt'

    #Launch all UAVs
    for i in range(numUAVs):
        robot_description_raw = xacro.process_file(pathlib.Path(os.path.join(package_dir_mavic, 'resource', 'mavic_webots.urdf')), 
                                                   mappings={'nspace': '/mavic_' + str(i) + "/", 'waypointsFile': uavWaypoints})
        robot_description_mavic = robot_description_raw.toprettyxml(indent='  ')
    
        mavic_driver = Node(
            package='webots_ros2_driver',
            executable='driver',
            output='screen',
            namespace='mavic_' + str(i),
            additional_env={'WEBOTS_CONTROLLER_URL': controller_url_prefix() + 'Mavic_2_PRO_' + str(i)},
            parameters=[
                {'robot_description': robot_description_mavic},
            ]
        )
        launchList.append(mavic_driver)

        mavicManager = Node(
            package='webots_ros2_mavic',
            executable='UAVManager.py',
            output='screen',
            arguments=['-name', 'mavic_' + str(i), '-assignedPOIs', '0'] #replace 0 with uavPOIs[i]
        )
        launchList.append(mavicManager)

    #Launch all Meese
    #moose_robot_description = pathlib.Path(os.path.join(package_dir_mavic, 'resource', 'moose_webots.urdf')).read_text()
    moose_robot_description = xacro.process_file(pathlib.Path(os.path.join(package_dir_mavic, 'resource', 'moose_webots.urdf')), 
                                                mappings={'waypointsFile': ugvWaypoints})
    moose_robot_description = moose_robot_description.toprettyxml(indent='  ')
    for i in range(numUGVs):
        mooseDriver = Node(
            package='webots_ros2_driver',
            executable='driver',
            output='screen',
            additional_env={'WEBOTS_CONTROLLER_URL': controller_url_prefix() + 'moose' + str(i)},
            parameters=[
                {'robot_description': moose_robot_description},
            ]
        )
        launchList.append(mooseDriver)
    
    for i in range(numHumans):
        humanManager = Node(
            package='webots_ros2_mavic',
            executable='OperatorHub.py',
            output='screen',
            arguments=['-humanAttributes', '0', '-poiAttributes', '0', '-humanPoiAssignments', '0'] #replace 0 with humanAttributes[i], poiAttributes[i]
        )
        launchList.append(humanManager)


    return launchList

def generate_launch_description():
    package_dir_mavic = get_package_share_directory('webots_ros2_mavic') #'/home/arjun/SMART-LAB-ITAP-WEBOTS/webots_ros2_mavic/' 
    #package_dir_turtle = get_package_share_directory('webots_ros2_turtlebot')
    world = LaunchConfiguration('world')
    mode = LaunchConfiguration('mode')

    webots = WebotsLauncher(
        world=PathJoinSubstitution([package_dir_mavic, 'worlds', world]), #'/home/arjun/SMART-LAB-ITAP-WEBOTS/webots_ros2_mavic/worlds/mavic_world.wbt', #P,
        mode=mode,
        ros2_supervisor=True
    )

    # The following line is important!
    # This event handler respawns the ROS 2 nodes on simulation reset (supervisor process ends).
    reset_handler = launch.actions.RegisterEventHandler(
        event_handler=launch.event_handlers.OnProcessExit(
            target_action=webots._supervisor,
            on_exit=get_ros2_nodes,
        )
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'world',
            default_value='mavic_world.wbt',
            description='Choose one of the world files from `/webots_ros2_mavic/worlds` directory'
        ),
        DeclareLaunchArgument(
            'mode',
            default_value='realtime',
            description='Webots startup mode'
        ),
        webots,
        webots._supervisor,

        # This action will kill all nodes once the Webots simulation has exited
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=webots,
                on_exit=[
                    launch.actions.UnregisterEventHandler(
                        event_handler=reset_handler.event_handler
                    ),
                    launch.actions.EmitEvent(event=launch.events.Shutdown())
                ],
            )
        ),

        # Add the reset event handler
        reset_handler
    ] + get_ros2_nodes())

