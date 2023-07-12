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

class SimPerformance(Node):
    def __init__(self, numHumans):
        super().__init__("Sim Performance")
        self.numHumans = numHumans
        self.publisher = self.create_publisher(String, "simPerformance", 10)
        self.subscribers = []
        for i in range(self.numHumans):
            self.subscribers.append(self.create_subscription(String, "/" + "human_" + str(i) + "/finalScore", qos_profile=10))
        self.subTimer = self.create_timer(1.0/5.0, self.subCallback)
        
        self.counter = 0
        self.finalScore = 0
        self.debug = False

    def subCallback(self):
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

        pass

    def pubCallback(self):
        msg = str(self.finalScore)
        if self.debug:
            print("SENT:")
            print(msg)
        self.publisher.publish(msg)


def main(argv):
    rclpy.init(args=argv)
    simPerformance = SimPerformance()
    rclpy.spin(simPerformance)
    rclpy.shutdown()

if __name__ == "__main__":
    if (len(sys.argv) > 1):
        main(sys.argv)
    else:
        main()