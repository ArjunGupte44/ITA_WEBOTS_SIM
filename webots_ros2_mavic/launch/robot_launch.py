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

#from sim_configurator.main import itapSim
import sys
sys.path.insert(0, '/home/arjun/SMART-LAB-ITAP-WEBOTS/sim_configurator')

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
    numUAVs = 7 #itapSim.getNumUAVs()
    numUGVs = 3 #itapSim.getNumUGVs()

    #Launch all UAVs
    for i in range(numUAVs):
        robot_description_raw = xacro.process_file(pathlib.Path(os.path.join(package_dir_mavic, 'resource', 'mavic_webots.urdf')), mappings={'nspace': '/mavic_' + str(i) + "/"})
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

    #Launch all UGVs
    controller_manager_timeout = ['--controller-manager-timeout', '200']
    controller_manager_prefix = 'python.exe' if os.name == 'nt' else ''
    ros_control_spawners = []

    for i in range(numUGVs):
        diffdrive_controller_spawner = Node(
            package='controller_manager',
            executable='spawner',
            output='screen',
            prefix=controller_manager_prefix,
            arguments=['diffdrive_controller'] + controller_manager_timeout + ['--name', 'turtle_' + str(i)],
            #parameters=[{'name': 'turtle_' + str(i)}],
            #namespace='turtle_' + str(i)
        )
        launchList.append(diffdrive_controller_spawner)
        #ros_control_spawners.append(diffdrive_controller_spawner)

        joint_state_broadcaster_spawner = Node(
            package='controller_manager',
            executable='spawner',
            output='screen',
            prefix=controller_manager_prefix,
            arguments=['joint_state_broadcaster'] + controller_manager_timeout + ['--name', 'turtle_' + str(i)],
            #parameters=[{'name': 'turtle_' + str(i)}],
            #namespace='turtle_' + str(i)
        )
        launchList.append(joint_state_broadcaster_spawner)
        #ros_control_spawners.append(joint_state_broadcaster_spawner)

        mappings = [('/diffdrive_controller/cmd_vel_unstamped', 'turtle_' + str(i) + '/cmd_vel')]
        if 'ROS_DISTRO' in os.environ and os.environ['ROS_DISTRO'] in ['humble', 'rolling']:
            mappings.append(('/diffdrive_controller/odom', 'turtle_' + str(i) + '/odom'))

        turtlebot_driver = Node(
            package='webots_ros2_driver',
            executable='driver',
            output='screen',
            additional_env={'WEBOTS_CONTROLLER_URL': controller_url_prefix() + 'TurtleBot3Burger_' + str(i)},
            parameters=[
                {'robot_description': robot_description_turtle,
                'use_sim_time': use_sim_time,
                'set_robot_state_publisher': True,
                #'name': 'turtle_' + str(i)},
                },
                ros2_control_params
            ],
            remappings=mappings,
            #namespace='turtle_' + str(i)
        )
        launchList.append(turtlebot_driver)

        robot_state_publisher = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': '<robot name=""><link name=""/></robot>',
                'name': 'turtle_' + str(i)
            }],
            namespace="turtle_" + str(i)
        )
        launchList.append(robot_state_publisher)

        footprint_publisher = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            output='screen',
            arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'base_footprint'],
            parameters=[{'name': 'turtle_' + str(i)}],
            namespace="turtle_" + str(i)
        )
        launchList.append(footprint_publisher)

        # Navigation
        navigation_nodes = []
        os.environ['TURTLEBOT3_MODEL'] = 'burger'
        if 'turtlebot3_navigation2' in get_packages_with_prefixes():
            turtlebot_navigation = IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(
                    get_package_share_directory('turtlebot3_navigation2'), 'launch', 'navigation2.launch.py')),
                launch_arguments=[
                    ('map', nav2_map),
                    ('params_file', nav2_params),
                    ('use_sim_time', use_sim_time),
                ],
                condition=launch.conditions.IfCondition(use_nav))
            navigation_nodes.append(turtlebot_navigation)

        # SLAM
        if 'turtlebot3_cartographer' in get_packages_with_prefixes():
            turtlebot_slam = IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(
                    get_package_share_directory('turtlebot3_cartographer'), 'launch', 'cartographer.launch.py')),
                launch_arguments=[
                    ('use_sim_time', use_sim_time),
                ],
                condition=launch.conditions.IfCondition(use_slam))
            navigation_nodes.append(turtlebot_slam)

        # Wait for the simulation to be ready to start navigation nodes
        waiting_nodes = WaitForControllerConnection(
            target_driver=turtlebot_driver,
            nodes_to_start=ros_control_spawners
        )
        launchList.append(waiting_nodes)

    return launchList

def generate_launch_description():
    package_dir_mavic = get_package_share_directory('webots_ros2_mavic') #'/home/arjun/SMART-LAB-ITAP-WEBOTS/webots_ros2_mavic/' 
    package_dir_turtle = get_package_share_directory('webots_ros2_turtlebot')
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

