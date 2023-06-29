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

def get_ros2_nodes(*args):
    package_dir_mavic = get_package_share_directory('webots_ros2_mavic')
    package_dir_turtle = get_package_share_directory('webots_ros2_turtlebot')

    #robot_description_mavic = pathlib.Path(os.path.join(package_dir_mavic, 'resource', 'mavic_webots.urdf.xacro')).read_text()
    robot_description_turtle = pathlib.Path(os.path.join(package_dir_turtle, 'resource', 'turtlebot_webots.urdf')).read_text()
    ros2_control_params = os.path.join(package_dir_turtle, 'resource', 'ros2control.yml')

    use_sim_time = LaunchConfiguration('use_sim_time', default=True)

    robot_description_raw = xacro.process_file(pathlib.Path(os.path.join(package_dir_mavic, 'resource', 'mavic_webots.urdf')), mappings={'nspace': '/mavic1/'})
    robot_description_mavic = robot_description_raw.toprettyxml(indent='  ')
    
    mavic_driver1 = Node(
        package='webots_ros2_driver',
        executable='driver',
        output='screen',
        namespace='mavic1',
        additional_env={'WEBOTS_CONTROLLER_URL': controller_url_prefix() + 'Mavic_2_PRO1'},
        parameters=[
            {'robot_description': robot_description_mavic},
        ]
    )

    robot_description_raw = xacro.process_file(pathlib.Path(os.path.join(package_dir_mavic, 'resource', 'mavic_webots.urdf')), mappings={'nspace': '/mavic2/'})
    robot_description_mavic = robot_description_raw.toprettyxml(indent='  ')

    mavic_driver2 = Node(
        package='webots_ros2_driver',
        executable='driver',
        output='screen',
        namespace='mavic2',
        additional_env={'WEBOTS_CONTROLLER_URL': controller_url_prefix() + 'Mavic_2_PRO2'},
        parameters=[
            {'robot_description': robot_description_mavic},
        ]
    )
    
    # TODO: Revert once the https://github.com/ros-controls/ros2_control/pull/444 PR gets into the release
    # ROS control spawners
    controller_manager_timeout = ['--controller-manager-timeout', '50']
    controller_manager_prefix = 'python.exe' if os.name == 'nt' else ''
    diffdrive_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        output='screen',
        prefix=controller_manager_prefix,
        arguments=['diffdrive_controller'] + controller_manager_timeout,
    )
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        output='screen',
        prefix=controller_manager_prefix,
        arguments=['joint_state_broadcaster'] + controller_manager_timeout,
    )
    ros_control_spawners = [diffdrive_controller_spawner, joint_state_broadcaster_spawner]

    mappings = [('/diffdrive_controller/cmd_vel_unstamped', 'turtle/cmd_vel')]
    if 'ROS_DISTRO' in os.environ and os.environ['ROS_DISTRO'] in ['humble', 'rolling']:
        mappings.append(('/diffdrive_controller/odom', 'turtle/odom'))

    turtlebot_driver = Node(
        package='webots_ros2_driver',
        executable='driver',
        output='screen',
        additional_env={'WEBOTS_CONTROLLER_URL': controller_url_prefix() + 'TurtleBot3Burger'},
        parameters=[
            {'robot_description': robot_description_turtle,
             'use_sim_time': use_sim_time,
             'set_robot_state_publisher': True},
            ros2_control_params
        ],
        remappings=mappings
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': '<robot name=""><link name=""/></robot>'
        }],
    )

    footprint_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        output='screen',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'base_footprint'],
    )

    return [
        mavic_driver1, mavic_driver2,
        diffdrive_controller_spawner, joint_state_broadcaster_spawner, turtlebot_driver, robot_state_publisher, footprint_publisher
    ]

def get_static_object_nodes():
    launchList = []

    #POI Parameters
    numPOIs = 50
    numSafe = 25
    numThreats = 25
    poiManager = Node(
                package='webots_ros2_mavic',
                namespace='poiManager',
                executable='poiManager.py',
                arguments=['-threats', str(numThreats), '-safe', str(numSafe)],
                output='screen')
    launchList.append(poiManager)
    
    #Human Operator Parameters
    numHumans = random.randint(1, 10)
    print(numHumans)
    humanManager = Node(
                    package='webots_ros2_mavic',
                    namespace='humanManager',
                    executable='humanManager.py',
                    arguments=['-humans', str(numHumans)],
                    output='screen')
    launchList.append(humanManager)
    print("HERE")

    
    return launchList

def generate_launch_description():
    package_dir_mavic = get_package_share_directory('webots_ros2_mavic') #'/home/arjun/SMART-LAB-ITAP-WEBOTS/webots_ros2_mavic/' 
    package_dir_turtle = get_package_share_directory('webots_ros2_turtlebot')
    world = LaunchConfiguration('world')

    webots = WebotsLauncher(
        world=PathJoinSubstitution([package_dir_mavic, 'worlds', world]), #'/home/arjun/SMART-LAB-ITAP-WEBOTS/webots_ros2_mavic/worlds/mavic_world.wbt', #P,
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

