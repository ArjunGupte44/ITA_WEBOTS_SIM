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
    def __init__(self, humanAttributes, poiAttributes, humanPoiAssignments):
        super().__init__("OperatorHub")
        self.humanAttributes = humanAttributes
        self.poiAttributes = poiAttributes
        self.humanPoiAssignments = humanPoiAssignments
        self.subscriber = self.create_subscription(DiverseArray, 'poiVisits', self.subCallback, 10)
        self.numParams = 6 #tBar, Fs, Ff, Fw, Pr, Correct(1)/Wrong(-1)        
        self.operatorMetrics = []
        self.initializeOperatorMetrics()
    
    def initializeOperatorMetrics(self):
        for i in range(len(self.humanAttributes)):
            self.operatorMetrics.append([])
            for param in range(self.numParams):
                self.operatorMetrics[i].append([])

    def subCallback(self, msg):
        robotName = msg.robot_name
        visitedPoiCoords = [msg.poi_x, msg.poi_y, msg.poi_z]
        arrivalTime = msg.arrival_time
        self.get_logger().info(f" {robotName}, {visitedPoiCoords}, {arrivalTime}")

        #Find the operator assigned to this poi based on its xyz location
        foundAssignedHuman = False
        row = 0
        assignedOperator = -1
        for i, row in enumerate(self.humanPoiAssignments):
            if visitedPoiCoords in row:
                assignedOperator = i
                break
        
        #Determine the poi difficulty and calculate t-bar value from the table II
        tBar = 10 if 'moose' in robotName else 20
        poiDifficulty = -1
        for i in range(len(self.poiAttributes)):
            if self.poiAttributes[i][0] == visitedPoiCoords[0]:
                poiDifficulty = self.poiAttributes[i][4]
                break
        
        if poiDifficulty == 2:
            tBar *= 3
        elif poiDifficulty == 3:
            tBar *= 9
        self.operatorMetrics[assignedOperator][0].append(tBar)
        
        #Calculate value of Fs - this param is not a cumulative value from all the previously visited pois
        Fs = 1 / (1 + (m.exp(0.05 * (tBar - 150))))
        self.operatorMetrics[assignedOperator][1].append(Fs)

        #All the following parameters need a cumulative value of tBar starting from the very first poi visited
        tBarCumulative = 0
        for val in self.operatorMetrics[assignedOperator][0]:
            tBarCumulative += val

        #Calculate value of Ff - this param is a cumulative value so look through previous values in tBar list for a given operator
        if tBarCumulative >= 0 and tBarCumulative < 3600: #between 0 and 1 hour in seconds
            Ff = 1
        elif tBarCumulative >= 3600 and tBarCumulative <= 28800: #between 1 and 8 hours in seconds
            Ff = -0.12 * tBarCumulative + 1.12
        self.operatorMetrics[assignedOperator][2].append(Ff)

        #Calculate value of Fw - this param is a cumulative value so look through previous values in tBar list for a given operator
        Fw = 0
        self.operatorMetrics[assignedOperator][3].append(Fw)

        #Calculate probability of operator predicting a given poi correctly
        predictionProbability = 0.5 + (Fs * Ff * Fw * m.sin(self.humanAttributes[assignedOperator][3]) * m.sin(self.humanAttributes[assignedOperator][4]))
        self.operatorMetrics[assignedOperator][4].append(predictionProbability)

        #Use flip function to convert probability into correct/wrong prediction
        binaryResult = 1 if random.uniform(0, 1) <= predictionProbability else -1
        self.operatorMetrics[assignedOperator][5].append(binaryResult)
        self.get_logger().info(f"result: {binaryResult}")




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
    operatorManager = OperatorHub(argv[2], argv[4], argv[6])
    rclpy.spin(operatorManager)
    rclpy.shutdown()

if __name__ == "__main__":
    if (len(sys.argv) > 1):
        main(sys.argv)
    else:
        main()