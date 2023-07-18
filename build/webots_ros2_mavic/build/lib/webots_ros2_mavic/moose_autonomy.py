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

"""ROS2 Mavic 2 Pro driver."""

from math import pow, atan2, sqrt, radians, atan, degrees
import rclpy
import pathlib
import os
import numpy as np
import time
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from webots_ros2_msgs.msg import FloatStamped
from ament_index_python.packages import get_package_share_directory


HALF_DISTANCE_BETWEEN_WHEELS = 0.045
WHEEL_RADIUS = 0.025

def clamp(value, value_min, value_max):
    return min(max(value, value_min), value_max)


class MooseAutonomy:
    def init(self, webots_node, properties):
        self.__robot = webots_node.robot
        self.__timestep = int(self.__robot.getBasicTimeStep())
        self.__package_dir = get_package_share_directory('webots_ros2_mavic')
        self.__properties = properties

        # Sensors
        self.__gps = self.__robot.getDevice('gps')
        # self.__gyro = self.__robot.getDevice('gyro')
        #self.__imu = self.__robot.getDevice('inertial unit')

        # Motors
        self.__motors = [
            self.__robot.getDevice('left motor 1'),
            self.__robot.getDevice('left motor 2'),
            self.__robot.getDevice('left motor 3'),
            self.__robot.getDevice('left motor 4'),
            self.__robot.getDevice('right motor 1'),
            self.__robot.getDevice('right motor 2'),
            self.__robot.getDevice('right motor 3'),
            self.__robot.getDevice('right motor 4')
        ]

        for motor in self.__motors:
            motor.setPosition(float('inf'))
            motor.setVelocity(0)

        # State
        self.__target_twist = Twist()
        self.__linear_x_integral = 0
        self.__linear_y_integral = 0

        # autonomy on or off (path following)
        self.__path_follow = True
        self.__path_file = None
        self.__waypoints = []
        self.__target_position = [0, 0]
        self.__target_index = 0
        self.__current_pose = 5 * [0]
        self.__target_precision = 5
        self.__waypointsFile = self.__properties['waypointsPath']
        self.__index = 0
        self.__startTime = 0
        self.__justReachedPOI = False
        self.__defaultLinearSpeed = 0.5
        self.__visitedAllWaypoints = False
        self.__mooseNumber = self.__robot.getName()[len(self.__robot.getName()) - 1]

        # ROS interface
        rclpy.init(args=None)
        self.__node = rclpy.create_node(self.__robot.getName() + '_driver')
        #self.__node.create_subscription(Twist, self.__robot.getName() + '/cmd_vel', self.__cmd_vel_callback, 1)
        self.__node.create_subscription(FloatStamped, self.__robot.getName() + '/compass/bearing', self.__getPosition, 10)
        self.__path_follow_callback('ugvCoords.txt')

    def __path_follow_callback(self, fileName):
        self.__path_follow = True
        #file = pathlib.Path(os.path.join(self.__package_dir, 'resource', fileName))
        file = open("/home/arjun/SMART-LAB-ITAP-WEBOTS/webots_ros2_mavic/resource/ugvCoords.txt", "r")
        contents = file.readlines()
        self.__path_file = contents[int(self.__robot.getName()[len(self.__robot.getName()) - 1])]
        
        point_list = []
        line = [x.strip() for x in self.__path_file.strip().split(",")]
        for line_el in line:
            line_el = line_el.strip().split()
            point_list.append([float(line_el[0]), float(line_el[1]), float(line_el[2])])
        number_of_waypoints = len(point_list)
        self.__waypoints = []
        for i in range(0, number_of_waypoints):
            self.__waypoints.append([])
            self.__waypoints[i].append(point_list[i][0])
            self.__waypoints[i].append(point_list[i][1])
            self.__waypoints[i].append(point_list[i][2]) # z
        
        #self.__node.get_logger().info(f"waypoints: {self.__waypoints}")
        
    def __cmd_vel_callback(self, twist):
        self.__target_twist = twist

    def __getPosition(self, floatStamped):
        x, y, z = self.__gps.getValues()
        #roll, pitch, yaw = self.__imu.getRollPitchYaw()
        self.heading = floatStamped

        #adjust angle by subtracting 270 degrees first
        heading = self.heading.data - 270
        heading = radians(heading) #in radians now
        self.__current_pose = [x, y, z, heading]
        #self.__node.get_logger().info(f"pos: {self.__current_pose}")


    def __updatedTargetWaypoint(self):
        # Check if the turtle is close enough to the target
        distance = self.euclidean_distance(self.__waypoints[self.__index])
        #self.__node.get_logger().info(f"dist: {distance}")

        if self.euclidean_distance(self.__waypoints[self.__index]) <= self.__target_precision:

            # Move to the next house
            if self.__index < len(self.__waypoints) - 1:
                self.__index += 1
                self.__node.get_logger().info(f"Moose {str(self.__mooseNumber)} headed to {self.__waypoints[self.__index]}")
            else:
                # If it's the last house, stop the robot and shut down the node
                self.__node.get_logger().info(f"Moose {str(self.__mooseNumber)} task complete!")
                self.__visitedAllWaypoints = True


    def euclidean_distance(self, goal_pose):
        """Euclidean distance between current pose and the goal."""
        return sqrt(pow((goal_pose[0] - self.__current_pose[0]), 2) +
                    pow((goal_pose[1] - self.__current_pose[1]), 2))

    def __adjustVelocities(self, linearVelocity, angularVelocity, goalPose):
        if self.__visitedAllWaypoints:
            return 0.0, 0.0

        distanceToTarget = self.euclidean_distance(goalPose)
        if distanceToTarget <= 5 and self.__justReachedPOI == False:
            self.__startTime = time.time()
            self.__justReachedPOI = True
            self.__node.get_logger().info(f"Moose {str(self.__mooseNumber)} reached {self.__waypoints[self.__index]}")
            return 0.0, 0.0
        
        if distanceToTarget <= 5 and self.__justReachedPOI == True:
            elapsedTime = time.time() - self.__startTime
            if elapsedTime <= 5:
                return 0.0, 0.0
            else:
                self.__updatedTargetWaypoint()
                self.__startTime = 0.0
                self.__justReachedPOI = False
                return linearVelocity, angularVelocity
        
        else:
            return linearVelocity, angularVelocity

    def steering_angle(self, goal_pose):
        targetAngle = atan2(goal_pose[0] - self.__current_pose[0], goal_pose[1] - self.__current_pose[1])

        # This is now in ]-2pi;2pi[
        angle_left = targetAngle - self.__current_pose[3]
        #self.__node.get_logger().info(f"CURR: {self.__current_pose[3]}")
        # Normalize turn angle to ]-pi;pi]
        angle_left = (angle_left + 2*np.pi) % (2*np.pi)
        if (angle_left > np.pi):
            angle_left -= 2*np.pi

        return angle_left

    def angular_vel(self, goal_pose, constant=1):
        error = -self.steering_angle(goal_pose) #negative because for ugv, turning right needs negative ang vel and turning left needs positive ang vel
        angularVel = constant * error
        return angularVel

    def __move_to_target(self):
        targetPOI = self.__waypoints[self.__index]
        linearVelocity, angularVelocity = self.__adjustVelocities(self.__defaultLinearSpeed, self.angular_vel(targetPOI), targetPOI)

        # Linear velocity in the x-axis.
        self.__target_twist.linear.x = linearVelocity #JUST USE fixed constant since error term is too large to set as linear velocity
        self.__target_twist.linear.y = float(0)
        self.__target_twist.linear.z = float(0)
        

        # Angular velocity in the z-axis.
        self.__target_twist.angular.x = float(0)
        self.__target_twist.angular.y = float(0)
        self.__target_twist.angular.z = angularVelocity
                

    def step(self):
        rclpy.spin_once(self.__node, timeout_sec=0)
        self.__move_to_target()

        forward_speed = self.__target_twist.linear.x #clamp(self.__target_twist.linear.x, -1, 1) 
        #self.__node.get_logger().info(f"fwd: {forward_speed}")
        #forward_speed = clamp(forward_speed, -1, 1)

        angular_speed = self.__target_twist.angular.z 
        #angular_speed = clamp(angular_speed, -1, 1)
        
        #self.__node.get_logger().info(f"angular: {angular_speed}")
        

        command_motor_left = (forward_speed - angular_speed * HALF_DISTANCE_BETWEEN_WHEELS) / WHEEL_RADIUS
        #command_motor_left = clamp(command_motor_left, -2, 2)

        command_motor_right = (forward_speed + angular_speed * HALF_DISTANCE_BETWEEN_WHEELS) / WHEEL_RADIUS
        #command_motor_right = clamp(command_motor_right, -2, 2)
        
        # Apply control
        self.__motors[0].setVelocity(command_motor_left)
        self.__motors[1].setVelocity(command_motor_left)
        self.__motors[2].setVelocity(command_motor_left)
        self.__motors[3].setVelocity(command_motor_left)

        self.__motors[4].setVelocity(command_motor_right)
        self.__motors[5].setVelocity(command_motor_right)
        self.__motors[6].setVelocity(command_motor_right)
        self.__motors[7].setVelocity(command_motor_right)