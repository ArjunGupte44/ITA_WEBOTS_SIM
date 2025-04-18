#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from example_interfaces.msg import String
from std_msgs.msg import Int32MultiArray
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
from src.main import itapSim, humanPoiAssignments
print(f"OPER: {itapSim.getPOIs()}")
FIVE_MINS_IN_SECONDS = 300
ONE_HOUR_IN_SECONDS = 3600
EIGHT_HOURS_IN_SECONDS = 28800

class OperatorHub(Node):
    def __init__(self):
        super().__init__("OperatorHub")
        self.humanAttributes = itapSim.getHumanAttributes()
        self.poiAttributes = itapSim.getPOIAttributes()
        self.humanPoiAssignments = humanPoiAssignments
        self.subscriber = self.create_subscription(DiverseArray, 'poiVisits', self.subCallback, 10)
        self.numParams = 6 #tBar, Fs, Ff, Fw, Pr, Correct(1)/Wrong(-1)        
        self.operatorMetrics = []
        self.numPois = len(self.poiAttributes)
        self.poiVisitTimes = np.zeros((self.numPois, 1))
        self.currentTimes = []
        self.pictureTakingTime = 3 #seconds
        self.poisVisited = 0

        self.initializeNestedLists(self.operatorMetrics, len(self.humanAttributes), self.numParams)
        self.initializeNestedLists(self.currentTimes, len(self.humanAttributes), 0)

    def initializeNestedLists(self, array, outerParam, innerParam):
        for i in range(outerParam):
            array.append([])
            for param in range(innerParam):
                array[i].append([])

    def subCallback(self, msg):
        self.poisVisited += 1
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
                self.poiVisitTimes[i][0] = arrivalTime #add arrival time to poi to poi visit times column vector
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
        if tBarCumulative >= 0 and tBarCumulative < ONE_HOUR_IN_SECONDS: #between 0 and 1 hour in seconds
            Ff = 1
        elif tBarCumulative >= ONE_HOUR_IN_SECONDS and tBarCumulative <= EIGHT_HOURS_IN_SECONDS: #between 1 and 8 hours in seconds
            Ff = -0.12 * tBarCumulative + 1.12
        self.operatorMetrics[assignedOperator][2].append(Ff)

        #Calculate value of Fw - this param is a cumulative value so look through previous values in tBar list for a given operator
        if len(self.operatorMetrics[assignedOperator][0]) == 0 or arrivalTime >= self.currentTimes[assignedOperator][-1]:
            currentTime = arrivalTime + self.pictureTakingTime + tBar
        else:
            currentTime = self.currentTimes[assignedOperator][-1] + tBar
        self.currentTimes[assignedOperator].append(currentTime)

        fiveMinCutoff = max(0, currentTime - FIVE_MINS_IN_SECONDS) #if more than 5 mins have passed by, all good ; if not then cutoff is simply t = 0s
        workingTime = 0 #seconds
        for i, time in reversed(enumerate(self.currentTimes[assignedOperator])):
            if time > fiveMinCutoff:
                workingTime += self.operatorMetrics[assignedOperator][0][i] #append the values of tBar for the current operator while traversing in reverse
            else:
                break #just break to avoid performing more for loop iterations than necessary once a time <= cutoff is found
        
        #Apply formula to calculate Fw and get result
        utilization = workingTime / FIVE_MINS_IN_SECONDS
        if utilization >= 0 and utilization < 0.45:
            Fw = -2.47 * m.pow(utilization, 2) + 2.22 * utilization + 0.5
        elif utilization >= 0.45 and utilization < 0.65:
            Fw = 1
        elif utilization >= 0.65 and utilization <= 1:
            Fw = -4.08 * m.pow(utilization, 2) + 5.31 * utilization - 0.724
        self.operatorMetrics[assignedOperator][3].append(Fw)

        #Calculate probability of operator predicting a given poi correctly
        predictionProbability = 0.5 + (Fs * Ff * Fw * m.sin(self.humanAttributes[assignedOperator][3]) * m.sin(self.humanAttributes[assignedOperator][4]))
        self.operatorMetrics[assignedOperator][4].append(predictionProbability)

        #Use flip function to convert probability into correct/wrong prediction
        binaryResult = (poiDifficulty * 10) if random.uniform(0, 1) <= predictionProbability else (poiDifficulty * -10)
        self.operatorMetrics[assignedOperator][5].append(binaryResult)

        if self.poisVisited == len(self.poiAttributes):
            self.getSimScore()

    
    def getSimScore(self):
        operatorNetScores = np.zeros((1, len(self.humanAttributes))) 
        for i, metrics in enumerate(self.operatorMetrics):
            netScore = 0
            for score in metrics[5]:
                netScore += score
            operatorNetScores[0][i] = netScore

        finalSimScore = np.mean(operatorNetScores)
        self.get_logger().info(f"Final Simulation Score: {finalSimScore}")
        

    

def main():
    rclpy.init(args=None)
    operatorManager = OperatorHub()
    rclpy.spin(operatorManager)
    rclpy.shutdown()

if __name__ == "__main__":
    if (len(sys.argv) > 1):
        main(sys.argv)
    else:
        main()