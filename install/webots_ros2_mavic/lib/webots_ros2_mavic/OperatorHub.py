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
import math as m
import random

from robot_interfaces.msg import DiverseArray

class OperatorHub(Node):
    def __init__(self, humanAttributes, poiAttributes):
        super().__init__("OperatorHub")
        self.humanAttributes = humanAttributes
        self.poiAttributes = poiAttributes
        self.subscriber = self.create_subscription(DiverseArray, 'poiVisits', self.subCallback, 10)
        #self.subTimer = self.create_timer(1.0/5.0, self.subCallback)
        
        self.counter = 0
        self.finalScore = 0
        self.debug = False

    def subCallback(self, msg):
        robotName = msg.robot_name
        visitedPoiCoords = msg.poi_coords
        arrivalTime = msg.arrival_time
        self.get_logger().info(f" {robotName}, {visitedPoiCoords}, {arrivalTime}")
        #Add math code to update running score as a new poi image was captured by robot and sent to human to analyze
        
        #while self.counter < len(self.assignedPOIs)
            #fatigueFactor = ... Ff
            #workloadFactor = ... Fw
            #taskFactor = ... Fs

            #predictionProbability = 0.5 + (fatigueFactor * workloadFactor * taskFactor * self.humanAttributes[self.number][3] * self.humanAttributes[self.number][4])
            #correct = 1 if random.uniform(0, 1) <= predictionProbability else -1
            #if self.poiAttributes[xyz][4] == 1:
                #self.finalScore += correct * 10
            #elif self.poiAttributes[xyz][4] == 2:
                #self.finalScore += correct * 20
            #else:
                #self.finalScore += correct * 30
        
            #if self.counter == len(self.assignedPOIs) - 1:
                #self.pubCallback()
            
            #counter += 1

    """
    def pubCallback(self):
        msg = str(self.finalScore)
        if self.debug:
            print("SENT:")
            print(msg)
        self.publisher.publish(msg)
    """

def main(argv):
    rclpy.init(args=argv)
    operatorManager = OperatorHub(argv[2], argv[4])
    rclpy.spin(operatorManager)
    rclpy.shutdown()

if __name__ == "__main__":
    if (len(sys.argv) > 1):
        main(sys.argv)
    else:
        main()