# import sys
# import math
# import rclpy
# import pathlib
# import os
# import numpy as np
# import time
# from geometry_msgs.msg import Twist
# from std_msgs.msg import String
# from ament_index_python.packages import get_package_share_directory

# from robot_interfaces.msg import DiverseArray


# K_VERTICAL_THRUST = 68.5    # with this thrust, the drone lifts.
# K_VERTICAL_P = 3.0          # P constant of the vertical PID.
# K_ROLL_P = 50.0             # P constant of the roll PID.
# K_PITCH_P = 1.0 #30.0       # P constant of the pitch PID.
# K_YAW_P = 2.0
# K_VERTICAL_OFFSET = 0.6
# K_X_VELOCITY_P = 1
# K_Y_VELOCITY_P = 1
# K_X_VELOCITY_I = 0.01
# K_Y_VELOCITY_I = 0.01
# LIFT_HEIGHT = 1
# MAX_YAW_DISTURBANCE = 2
# MAX_PITCH_DISTURBANCE = -2

# # New velocity PID constants
# K_VELOCITY_X_P = 1.0
# K_VELOCITY_X_I = 0.1
# K_VELOCITY_X_D = 0.1
# K_VELOCITY_Y_P = 1.0
# K_VELOCITY_Y_I = 0.1
# K_VELOCITY_Y_D = 0.1



# def clamp(value, value_min, value_max):
#     return min(max(value, value_min), value_max)


# class MavicAutonomy:
#     def init(self, webots_node, properties):
#         self.__systemStartTime = time.time()
#         self.__robot = webots_node.robot
#         self.__timestep = int(self.__robot.getBasicTimeStep())
#         self.__package_dir = get_package_share_directory('webots_ros2_mavic')
#         self.__properties = properties

#         # Sensors
#         self.__gps = self.__robot.getDevice('gps')
#         self.__gyro = self.__robot.getDevice('gyro')
#         self.__imu = self.__robot.getDevice('inertial unit')

#         # Propellers
#         self.__propellers = [
#             self.__robot.getDevice('front right propeller'),
#             self.__robot.getDevice('front left propeller'),
#             self.__robot.getDevice('rear right propeller'),
#             self.__robot.getDevice('rear left propeller')
#         ]
#         for propeller in self.__propellers:
#             propeller.setPosition(float('inf'))
#             propeller.setVelocity(0)

#         # State
#         self.__target_twist = Twist()
#         self.__vertical_ref = LIFT_HEIGHT
#         self.pitch_ref = 0
#         self.roll_ref = 0
#         self.__linear_x_integral = 0
#         self.__linear_y_integral = 0

#         # Velocity PID variables
#         self.velocity_x_error_previous = 0
#         self.velocity_x_integral = 0
#         self.velocity_y_error_previous = 0
#         self.velocity_y_integral = 0


#         # autonomy on or off (path following)
#         self.__path_follow = True
#         self.__path_file = None
#         self.__waypoints = []
#         self.__target_altitudes = []
#         self.__target_position = [0, 0]
#         self.__target_index = 0
#         self.__current_pose = 6*[0]
#         self.__target_precision = 0.5
#         self.__target_altitude = 10
#         self.__waypointsFile = self.__properties['waypointsPath']
#         self.__justReachedPOI = True
#         self.__startTime = 0.0
#         self.__mavicNumber = self.__robot.getName()[len(self.__robot.getName()) - 1]
#         self.__launchTime = time.time()

#         #Poi visit info
#         #self.__visitInfo = DiverseArray()

#         # ROS interface
#         rclpy.init(args=None)
#         self.__node = rclpy.create_node(self.__robot.getName() + '_autonomy')
#         self.__node.create_subscription(Twist, self.__robot.getName() + '/cmd_vel', self.__cmd_vel_callback, 1)
#         self.__node.create_subscription(String, self.__robot.getName() + '/path_file', self.__path_follow_callback, 1)
#         self.__poiPublisher = self.__node.create_publisher(DiverseArray, 'poiVisits', 10)

    
#     def __publishVisitInfo(self, poiCoords, timeToVisitPOI):
#         visitInfo = DiverseArray()
#         visitInfo.robot_name = self.__robot.getName()
#         visitInfo.poi_x = poiCoords[0]
#         visitInfo.poi_y = poiCoords[1]
#         visitInfo.poi_z = poiCoords[2]
#         visitInfo.arrival_time = timeToVisitPOI
#         self.__poiPublisher.publish(visitInfo)

#     def __set_position(self, pos):    
#         self.__current_pose = pos
    
#     def __cmd_vel_callback(self, twist):
#         self.__target_twist = twist

#     def __path_follow_callback(self, fileName):
#         self.__path_follow = True
#         file = pathlib.Path(os.path.join(self.__package_dir, 'resource',fileName))

#         with open("/home/arjun/SMART-LAB-ITAP-WEBOTS/install/webots_ros2_mavic/share/webots_ros2_mavic/resource/uavCoords.txt", 'r') as file:
#             contents = file.readlines()
            
#             self.__path_file = contents[int(self.__robot.getName()[len(self.__robot.getName()) - 1])]
        
#         point_list = []
#         line = [x.strip() for x in self.__path_file.strip().split(",")]
#         for line_el in line:
#             line_el = line_el.strip().split()
#             point_list.append([float(line_el[0]), float(line_el[1]), float(line_el[2]), float(line_el[3])])
#         number_of_waypoints = len(point_list)
#         self.__waypoints = []
#         for i in range(0, number_of_waypoints):
#             self.__waypoints.append([])
#             self.__waypoints[i].append(point_list[i][0])
#             self.__waypoints[i].append(point_list[i][1])
#             self.__waypoints[i].append(point_list[i][2]) # z
#             self.__waypoints[i].append(point_list[i][3]) # yaw
#         self.__target_altitudes = [float(z.strip().split()[2]) for z in line]


    
    
#     def __move_to_target(self):
#         self.__path_follow_callback('uavCoords.txt')
#         if self.__target_position == [0, 0]:  # Initialization
#             self.__target_position = self.__waypoints[0][:2]
#             self.__target_altitude = self.__waypoints[0][2]

#         # Calculate distance to target
#         dx = self.__target_position[0] - self.__current_pose[0]
#         dy = self.__target_position[1] - self.__current_pose[1]
#         distance_to_target = np.sqrt(dx**2 + dy**2)

#         # Desired velocity (assuming a constant desired velocity for simplicity)
#         desired_velocity = 22.0  # m/s

#         # Compute desired velocities based on distance
#         desired_velocity_x = desired_velocity * (dx / distance_to_target)
#         desired_velocity_y = desired_velocity * (dy / distance_to_target)

#         # Calculate current velocities
#         if (abs(self.__current_pose[3]) + abs(self.__current_pose[4])) == 0:
#             velocity_x = 0
#             velocity_y = 0
#         else:
#             velocity_x = (self.__current_pose[4] / (abs(self.__current_pose[3]) + abs(self.__current_pose[4]))) * distance_to_target / self.__timestep
#             velocity_y = (-self.__current_pose[3] / (abs(self.__current_pose[3]) + abs(self.__current_pose[4]))) * distance_to_target / self.__timestep

#         # Compute errors
#         velocity_x_error = desired_velocity_x - velocity_x
#         velocity_y_error = desired_velocity_y - velocity_y

#         # Integrate errors
#         self.__linear_x_integral += velocity_x_error
#         self.__linear_y_integral += velocity_y_error

#         # PID control for velocities
#         pitch_control = K_VELOCITY_X_P * velocity_x_error + K_VELOCITY_X_I * self.__linear_x_integral
#         roll_control = K_VELOCITY_Y_P * velocity_y_error + K_VELOCITY_Y_I * self.__linear_y_integral

#         # Existing angle logic for direction
#         angle = np.arctan2(dy, dx)
#         angle_left = angle - self.__current_pose[5]
#         angle_left = (angle_left + 2 * np.pi) % (2 * np.pi)
#         if angle_left > np.pi:
#             angle_left -= 2 * np.pi

#         # Turn the robot to the left or to the right according to the value and the sign of angle_left
#         yaw_disturbance = MAX_YAW_DISTURBANCE * angle_left / (2 * np.pi)
#         pitch_disturbance = clamp(np.log10(abs(angle_left)), MAX_PITCH_DISTURBANCE, 0.1)

#         return yaw_disturbance, pitch_control + pitch_disturbance, roll_control, self.__target_altitude
#     def step(self):
#         rclpy.spin_once(self.__node, timeout_sec=0)
#         elapsedTime = time.time() - self.__launchTime

#         if elapsedTime < 5 * int(self.__mavicNumber):            
#             pass
#         else:
#             # Read sensors
#             roll, pitch, yaw = self.__imu.getRollPitchYaw()
#             x, y, vertical = self.__gps.getValues()
#             roll_velocity, pitch_velocity, twist_yaw = self.__gyro.getValues()
#             velocity = self.__gps.getSpeed()
#             if math.isnan(velocity):
#                 return
#             self.__set_position([x, y, vertical, roll, pitch, yaw])
            
#             # Get control values from move_to_target
#             yaw_disturbance, pitch_control, roll_control, target_altitude = self.__move_to_target()

#             # Update pitch_ref and roll_ref with control values
#             self.pitch_ref += pitch_control
#             self.roll_ref += roll_control


#             # if not self.__path_follow:
#             #     # Allow high level control once the drone is lifted
#             #     if vertical > 0.2:
#             #         # Calculate velocity
#             #         velocity_x = (pitch / (abs(roll) + abs(pitch))) * velocity
#             #         velocity_y = - (roll / (abs(roll) + abs(pitch))) * velocity

#             #         # High level controller (linear velocity)
#             #         linear_y_error = self.__target_twist.linear.y - velocity_y
#             #         linear_x_error = self.__target_twist.linear.x - velocity_x
#             #         self.__linear_x_integral += linear_x_error
#             #         self.__linear_y_integral += linear_y_error
#             #         roll_ref = K_Y_VELOCITY_P * linear_y_error + K_Y_VELOCITY_I * self.__linear_y_integral
#             #         pitch_ref = - K_X_VELOCITY_P * linear_x_error - K_X_VELOCITY_I * self.__linear_x_integral
#             #         self.__vertical_ref = clamp(
#             #             self.__vertical_ref + self.__target_twist.linear.z * (self.__timestep / 1000),
#             #             max(vertical - 0.5, LIFT_HEIGHT),
#             #             vertical + 0.5
#             #         )
#             #     vertical_input = K_VERTICAL_P * (self.__vertical_ref - vertical)

#             #     # Low level controller (roll, pitch, yaw)
#             #     yaw_ref = self.__target_twist.angular.z
#             #     yaw_input = K_YAW_P * (yaw_ref - twist_yaw)

#             # # follow waypoints
#             # else:
#             #     yaw_disturbance, pitch_ref, target_altitude = self.__move_to_target()
#             #     yaw_input = yaw_disturbance
#             #     clamped_difference_altitude = clamp(target_altitude - (vertical) + K_VERTICAL_OFFSET, -1, 1)
#             #     vertical_input = K_VERTICAL_P * pow(clamped_difference_altitude, 3.0)


#             roll_input = K_ROLL_P * clamp(roll, -1, 1) + roll_velocity + self.roll_ref
#             pitch_input = K_PITCH_P * clamp(pitch, -1, 1) + pitch_velocity + self.pitch_ref
#             yaw_input = yaw_disturbance
#             clamped_difference_altitude = clamp(target_altitude - (vertical) + K_VERTICAL_OFFSET, -1, 1)
#             vertical_input = K_VERTICAL_P * pow(clamped_difference_altitude, 3.0)

#             m1 = K_VERTICAL_THRUST + vertical_input + yaw_input + pitch_input + roll_input
#             m2 = K_VERTICAL_THRUST + vertical_input - yaw_input + pitch_input - roll_input
#             m3 = K_VERTICAL_THRUST + vertical_input - yaw_input - pitch_input + roll_input
#             m4 = K_VERTICAL_THRUST + vertical_input + yaw_input - pitch_input - roll_input

#             #self.__node.get_logger().info(f"Motor cmd: {-m1} {m2} {m3} {-m4}")
#             # Apply control
#             self.__propellers[0].setVelocity(-m1)
#             self.__propellers[1].setVelocity(m2)
#             self.__propellers[2].setVelocity(m3)
#             self.__propellers[3].setVelocity(-m4)












import sys
import math
import rclpy
import pathlib
import os
import numpy as np
import time
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from ament_index_python.packages import get_package_share_directory

from robot_interfaces.msg import DiverseArray


K_VERTICAL_THRUST = 68.5    # with 68.5 thrust, the drone lifts.
K_VERTICAL_P = 3.0          #Original: 3.0      # P constant of the vertical PID.

K_ROLL_P = 50.0             #Original: 50.0  P constant of the roll PID.
K_ROLL_I = 0
K_ROLL_D = 1

K_PITCH_P = 30.0            #Original: 30.0 P constant of the pitch PID.
K_PITCH_I = 0
K_PITCH_D = 1

K_YAW_P = 2.0


K_VERTICAL_OFFSET = 0.6     #Original: 0.6
LIFT_HEIGHT = 1             #Original: 1

K_X_VELOCITY_P = 1          #Original: 1
K_Y_VELOCITY_P = 1          #Original: 1

K_X_VELOCITY_I = 0.01       #Original: 0.01
K_Y_VELOCITY_I = 0.01       #Original: 0.01

MAX_YAW_DISTURBANCE = 2
MAX_PITCH_DISTURBANCE = -2

SPEED_CONSTANT = 1.0


def clamp(value, value_min, value_max):
    return min(max(value, value_min), value_max)


class MavicAutonomy:
    def init(self, webots_node, properties):
        self.__systemStartTime = time.time()
        self.__robot = webots_node.robot
        self.__timestep = int(self.__robot.getBasicTimeStep())
        self.__package_dir = get_package_share_directory('webots_ros2_mavic')
        self.__properties = properties

        # Sensors
        self.__gps = self.__robot.getDevice('gps')
        self.__gyro = self.__robot.getDevice('gyro')
        self.__imu = self.__robot.getDevice('inertial unit')

        # Propellers
        self.__propellers = [
            self.__robot.getDevice('front right propeller'),
            self.__robot.getDevice('front left propeller'),
            self.__robot.getDevice('rear right propeller'),
            self.__robot.getDevice('rear left propeller')
        ]
        for propeller in self.__propellers:
            propeller.setPosition(float('inf'))
            propeller.setVelocity(0)

        # State
        #Going left is +y velocity, Going right is -y velocity
        #Going forwards is +x velocity, Going backwards is -x velocity
        self.__target_twist = Twist()
        self.__vertical_ref = LIFT_HEIGHT
        self.__linear_x_integral = 0
        self.__linear_y_integral = 0

        # Additional state variables for PID control
        self.__previous_roll_error = 0
        self.__previous_pitch_error = 0
        self.__previous_yaw_error = 0
        self.__previous_vertical_error = 0
        
        self.__roll_integral = 0
        self.__pitch_integral = 0
        self.__yaw_integral = 0
        self.__vertical_integral = 0

        # autonomy on or off (path following)
        self.__path_follow = True #True means do waypoint following; False means do default speed controller
        self.__path_file = None
        self.__waypoints = []
        self.__target_altitudes = []
        self.__target_position = [0, 0]
        self.__target_index = 0
        self.__current_pose = 6*[0]
        self.__target_precision = 0.5
        self.__target_altitude = 10
        self.__waypointsFile = self.__properties['waypointsPath']
        self.__justReachedPOI = True
        self.__startTime = 0.0
        self.__mavicNumber = self.__robot.getName()[len(self.__robot.getName()) - 1]
        self.__launchTime = time.time()

        #Poi visit info
        #self.__visitInfo = DiverseArray()

        # ROS interface
        rclpy.init(args=None)
        self.__node = rclpy.create_node(self.__robot.getName() + '_autonomy')
        self.__node.create_subscription(Twist, self.__robot.getName() + '/cmd_vel', self.__cmd_vel_callback, 1)
        self.__node.create_subscription(String, self.__robot.getName() + '/path_file', self.__path_follow_callback, 1)
        self.__poiPublisher = self.__node.create_publisher(DiverseArray, 'poiVisits', 10)

    
    def __publishVisitInfo(self, poiCoords, timeToVisitPOI):
        visitInfo = DiverseArray()
        visitInfo.robot_name = self.__robot.getName()
        visitInfo.poi_x = poiCoords[0]
        visitInfo.poi_y = poiCoords[1]
        visitInfo.poi_z = poiCoords[2]
        visitInfo.arrival_time = timeToVisitPOI
        self.__poiPublisher.publish(visitInfo)

    def __set_position(self, pos):    
        self.__current_pose = pos
    
    def __cmd_vel_callback(self, twist):
        self.__target_twist = twist

    def __path_follow_callback(self, fileName):
        self.__path_follow = True
        file = pathlib.Path(os.path.join(self.__package_dir, 'resource',fileName))

        with open("/home/arjun/SMART-LAB-ITAP-WEBOTS/install/webots_ros2_mavic/share/webots_ros2_mavic/resource/uavCoords.txt", 'r') as file:
            contents = file.readlines()
            
            self.__path_file = contents[int(self.__robot.getName()[len(self.__robot.getName()) - 1])]
        
        point_list = []
        line = [x.strip() for x in self.__path_file.strip().split(",")]
        for line_el in line:
            line_el = line_el.strip().split()
            point_list.append([float(line_el[0]), float(line_el[1]), float(line_el[2]), float(line_el[3])])
        number_of_waypoints = len(point_list)
        self.__waypoints = []
        for i in range(0, number_of_waypoints):
            self.__waypoints.append([])
            self.__waypoints[i].append(point_list[i][0])
            self.__waypoints[i].append(point_list[i][1])
            self.__waypoints[i].append(point_list[i][2]) # z
            self.__waypoints[i].append(point_list[i][3]) # yaw
        self.__target_altitudes = [float(z.strip().split()[2]) for z in line]


    def __move_to_target(self):
        self.__path_follow_callback('uavCoords.txt')
        if self.__target_position == [0, 0]:  # Initialisation
            self.__target_position = self.__waypoints[0][:2]
            self.__target_altitude = self.__waypoints[0][2]

        # if the robot is at the position with a precision of target_precision
        if all([abs(x1 - x2) < self.__target_precision for (x1, x2) in zip(self.__target_position, self.__current_pose[0:2])]):
            if self.__justReachedPOI == True:
                timeToVisitPOI = time.time() - self.__systemStartTime
                self.__startTime = time.time()
                self.__justReachedPOI = False
                poiCoords = self.__waypoints[self.__target_index]
                self.__node.get_logger().info(f"Mavic {str(self.__mavicNumber)} reached AD at {poiCoords}")
                self.__publishVisitInfo(poiCoords, timeToVisitPOI)
            
            elapsedTime = time.time() - self.__startTime
            if elapsedTime > 1:
                self.__target_index += 1
                self.__justReachedPOI = True
                if self.__target_index > len(self.__waypoints)-1:
                    self.__target_index -= 1
                    self.__path_follow = False
                    self.__node.get_logger().info(f"Mavic {str(self.__mavicNumber)} task complete!\n")
                else:
                    self.__node.get_logger().info(f"Mavic {str(self.__mavicNumber)} headed to AD at {self.__waypoints[self.__target_index]}")

            self.__target_position = self.__waypoints[self.__target_index][:2]
            self.__target_altitude = self.__waypoints[self.__target_index][2]
            
        angle = np.arctan2(self.__target_position[1] - self.__current_pose[1], self.__target_position[0] - self.__current_pose[0])
        # This is now in ]-2pi;2pi[
        angle_left = angle - self.__current_pose[5]
        # Normalize turn angle to ]-pi;pi]
        angle_left = (angle_left + 2*np.pi) % (2*np.pi)
        if (angle_left > np.pi):
            angle_left -= 2*np.pi

        # Turn the robot to the left or to the right according the value and the sign of angle_left
        yaw_disturbance = MAX_YAW_DISTURBANCE*angle_left/(2*np.pi)
        # non proportional and decruising function
        pitch_disturbance = clamp(np.log10(abs(angle_left)), MAX_PITCH_DISTURBANCE, 0.1)
        #pitch_disturbance = SPEED_CONSTANT * np.log10(abs(angle_left))

        return yaw_disturbance, pitch_disturbance, self.__target_altitude
        
    def step(self):
        rclpy.spin_once(self.__node, timeout_sec=0)
        elapsedTime = time.time() - self.__launchTime

        if elapsedTime < 5 * int(self.__mavicNumber):
            pass
        else:
            roll_ref = 0
            pitch_ref = 0

            # Read sensors
            roll, pitch, yaw = self.__imu.getRollPitchYaw()
            x, y, vertical = self.__gps.getValues()
            roll_velocity, pitch_velocity, twist_yaw = self.__gyro.getValues()
            velocity = self.__gps.getSpeed()
            self.__set_position([x, y, vertical, roll, pitch, yaw])

            if math.isnan(velocity):
                return

            if not self.__path_follow:
                # Allow high level control once the drone is lifted
                if vertical > 0.2:
                    # Calculate velocity
                    velocity_x = (pitch / (abs(roll) + abs(pitch))) * velocity
                    velocity_y = - (roll / (abs(roll) + abs(pitch))) * velocity

                    # High level controller (linear velocity)
                    linear_y_error = self.__target_twist.linear.y - velocity_y
                    linear_x_error = self.__target_twist.linear.x - velocity_x
                    self.__linear_x_integral += linear_x_error
                    self.__linear_y_integral += linear_y_error
                    roll_ref = K_Y_VELOCITY_P * linear_y_error + K_Y_VELOCITY_I * self.__linear_y_integral
                    pitch_ref = - K_X_VELOCITY_P * linear_x_error - K_X_VELOCITY_I * self.__linear_x_integral
                    self.__vertical_ref = clamp(
                        self.__vertical_ref + self.__target_twist.linear.z * (self.__timestep / 1000),
                        max(vertical - 0.5, LIFT_HEIGHT),
                        vertical + 0.5
                    )
                vertical_input = K_VERTICAL_P * (self.__vertical_ref - vertical)

                # Low level controller (roll, pitch, yaw)
                yaw_ref = self.__target_twist.angular.z
                yaw_input = K_YAW_P * (yaw_ref - twist_yaw)

            # follow waypoints
            else:
                yaw_disturbance, pitch_ref, target_altitude = self.__move_to_target()
                yaw_input = yaw_disturbance
                clamped_difference_altitude = clamp(target_altitude - (vertical) + K_VERTICAL_OFFSET, -1, 1)
                vertical_input = K_VERTICAL_P * pow(clamped_difference_altitude, 3.0)


            roll_input = K_ROLL_P * clamp(roll, -1, 1) + roll_velocity + roll_ref
            pitch_input = K_PITCH_P * clamp(pitch, -1, 1) + pitch_velocity + pitch_ref

            # Calculate roll PID control
            # roll_error = roll
            # self.__roll_integral += roll_error
            # roll_derivative = roll_error - self.__previous_roll_error
            # roll_input = K_ROLL_P * roll_error + K_ROLL_I * self.__roll_integral + K_ROLL_D * roll_derivative
            # self.__previous_roll_error = roll_error

            # Calculate pitch PID control
            # pitch_error = pitch
            # self.__pitch_integral += pitch_error
            # pitch_derivative = pitch_error - self.__previous_pitch_error
            # pitch_input = K_PITCH_P * pitch_error + K_PITCH_I * self.__pitch_integral + K_PITCH_D * pitch_derivative
            # self.__previous_pitch_error = pitch_error
           
           
            m1 = SPEED_CONSTANT * (K_VERTICAL_THRUST + vertical_input + yaw_input + pitch_input + roll_input)
            m2 = SPEED_CONSTANT * (K_VERTICAL_THRUST + vertical_input - yaw_input + pitch_input - roll_input)
            m3 = SPEED_CONSTANT * (K_VERTICAL_THRUST + vertical_input - yaw_input - pitch_input + roll_input)
            m4 = SPEED_CONSTANT * (K_VERTICAL_THRUST + vertical_input + yaw_input - pitch_input - roll_input)
            

            #self.__node.get_logger().info(f"Motor cmd: {-m1} {m2} {m3} {-m4}")
            # Apply control
            self.__propellers[0].setVelocity(-m1)
            self.__propellers[1].setVelocity(m2)
            self.__propellers[2].setVelocity(m3)
            self.__propellers[3].setVelocity(-m4)
 