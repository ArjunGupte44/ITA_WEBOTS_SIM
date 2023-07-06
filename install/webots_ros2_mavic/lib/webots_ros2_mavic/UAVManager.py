#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from example_interfaces.msg import String
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3
from gazebo_msgs.msg import ModelStates
import sys
import numpy as np
import random
import re

class UAVManager(Node):
    def __init__(self, uavName, assignedPOIs):
        self.name = uavName
        self.assignedPOIs = assignedPOIs
        super().__init__(self.name + "_Manager")
        self.publisher = self.create_publisher(Twist, "/" + self.name + "/cmd_vel", 10)
        self.subscriber = self.create_subscription(Float64MultiArray, "/" + self.name + "/Mavic_2_PRO_" + self.name[len(self.name) - 1] + "/gps", self.subCallback, 10)
        self.pubTimer = self.create_timer(1.0/5.0, self.pubCallback)
        self.subTimer = self.create_timer(1.0/5.0, self.subCallback)
        
        self.linearVelocity = [0.0, 0.0, 0.0]
        self.angularVelocity = [0.0, 0.0, 0.0]
        self.debug = False

    def subCallback(self):
        #Add waypoint following code given current gps position and the list of assigned pois
        #self.linearVelocity = ...
        #self.angularVelocity = ...
        pass

    def pubCallback(self):
        msg = Twist()
        msg.linear.x = self.linearVelocity[0]
        msg.linear.y = self.linearVelocity[1]
        msg.linear.z = self.linearVelocity[2]

        msg.angular.x = self.angularVelocity[0]
        msg.angular.y = self.angularVelocity[1]
        msg.angular.z = self.angularVelocity[2]

        if self.debug:
            print("SENT:")
            print(msg)
        self.publisher.publish(msg)


def main(argv):
    rclpy.init(args=argv)
    uavManager = UAVManager(argv[2], argv[4])
    rclpy.spin(uavManager)
    rclpy.shutdown()

if __name__ == "__main__":
    if (len(sys.argv) > 1):
        main(sys.argv)
    else:
        main()