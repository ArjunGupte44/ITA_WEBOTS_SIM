#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from example_interfaces.msg import String
from std_msgs.msg import Int32MultiArray
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3
import sys
import numpy as np
import random
import re
import math as m
import random

from robot_interfaces.msg import DiverseArray
sys.path.insert(0, '/home/arjun/SMART-LAB-ITAP-WEBOTS/webots_ros2_mavic/webots_ros2_mavic')

NUM_HUMAN_ATTRIBUTES = 5
NUM_POI_ATTRIBUTES = 5

HUMAN_ATTRIBUTES_FILE = '/home/arjun/SMART-LAB-ITAP-WEBOTS/webots_ros2_mavic/resource/humanAttributes'
POI_ATTRIBUTES_FILE = '/home/arjun/SMART-LAB-ITAP-WEBOTS/webots_ros2_mavic/resource/poiAttributes'

POIS_FILE = '/home/arjun/SMART-LAB-ITAP-WEBOTS/webots_ros2_mavic/resource/pois'
NUM_AGENTS_FILE = '/home/arjun/SMART-LAB-ITAP-WEBOTS/webots_ros2_mavic/resource/numSimAgents'

FIVE_MINS_IN_SECONDS = 300
ONE_HOUR_IN_SECONDS = 3600
TWO_HOURS_IN_SECONDS = 7200

class OperatorHub(Node):
    def __init__(self):
        super().__init__("OperatorHub")
        self.humanAttributes = self.getAgentAttributes('human', NUM_HUMAN_ATTRIBUTES)
        self.poiAttributes = self.getAgentAttributes('poi', NUM_POI_ATTRIBUTES)

        self.humanPoiAssignments = self.getPoiAssignments()
        self.robotPoiAssignments = [] #from RL MODEL

        self.subscriber = self.create_subscription(DiverseArray, 'poiVisits', self.subCallback, 10)

        self.numParams = 6 #tBar, Fs, Ff, Fw, Pr, Correct(1)/Wrong(-1)        
        self.operatorMetrics = []
        
        #0: Safe POIs, 1: Threat POIs, 2: Humans, 3: UAVs, 4: UGVs
        self.numUAVs = self.getNumAgents(3)
        self.numUGVs = self.getNumAgents(4)
        self.uavMetrics = []
        self.ugvMetrics = []

        self.numPois = len(self.poiAttributes)
        self.poiVisitTimes = np.zeros((self.numPois, 1))
        self.currentTimes = []
        self.pictureTakingTime = 3 #seconds
        self.poisVisited = 0
        
        self.imageQualities = ["low", "medium", "upper-medium", "high"]
        self.tBarLUT = {"low": [45, 125, 245], "medium": [35, 95, 195], "upper-medium": [25, 65, 145], "high": [15, 35, 95]}
        self.robotProbabilitiesLUT = {"low": [0.5, 0.3, 0.1], "medium": [0.6, 0.4, 0.2], "upper-medium": [0.7, 0.5, 0.3], "high": [0.8, 0.6, 0.4]}

        self.initializeNestedLists(self.operatorMetrics, len(self.humanAttributes), self.numParams)
        self.initializeNestedLists(self.uavMetrics, self.numUAVs, 2)
        self.initializeNestedLists(self.ugvMetrics, self.numUGVs, 2)
        self.initializeNestedLists(self.currentTimes, len(self.humanAttributes), 0)

    def getAgentAttributes(self, agentType, numAttributes):
        attributesArray = []
        if agentType == 'human':
            f = open(HUMAN_ATTRIBUTES_FILE, '+r')
        else:
            f = open(POI_ATTRIBUTES_FILE, '+r')
        lines = f.readlines()

        currAgentAttributes = []
        for i, line in enumerate(lines):    
            if i == 0 or i % numAttributes != 0:
                currAgentAttributes.append(float(line[0:len(line) - 2]))
            else:
                attributesArray.append(currAgentAttributes)
                currAgentAttributes = []
                currAgentAttributes.append(float(line[0:len(line) - 2]))
        attributesArray.append(currAgentAttributes)
        
        return attributesArray

    def getPoiAssignments(self):
        f = open(POIS_FILE, '+r')
        pois = f.readlines()
    
        humanPoiAssignments = []
        for i in range(len(self.humanAttributes)):
            humanPoiAssignments.append([])

        index = 0
        counter = 0
        currPoi = []
        for coord in pois:
            if index < len(self.humanAttributes):
                if counter == 0 or counter % 3 != 0:
                    currPoi.append(round(float(coord[0:len(coord) - 2]), 2))
                    counter += 1
                else:
                    humanPoiAssignments[index].append(currPoi)
                    index += 1
                    currPoi = []
                    currPoi.append(round(float(coord[0:len(coord) - 2]), 2))
                    counter = 1
            else:
                index = 0
                currPoi.append(round(float(coord[0:len(coord) - 2]), 2))
                counter += 1
        humanPoiAssignments[index].append(currPoi)

        return humanPoiAssignments

    def getNumAgents(self, i):
        f = open(NUM_AGENTS_FILE, '+r')
        return int(f.readline(i))

    def initializeNestedLists(self, array, outerParam, innerParam):
        for i in range(outerParam):
            array.append([])
            for param in range(innerParam):
                array[i].append([])

    def subCallback(self, msg):
        self.poisVisited += 1
        robotName = msg.robot_name
        visitedPoiCoords = [round(msg.poi_x, 2), round(msg.poi_y, 2), round(msg.poi_z, 2)]
        arrivalTime = msg.arrival_time
    
        #Determine the poi difficulty level
        poiDifficulty = -1
        for i in range(len(self.poiAttributes)):
            if self.poiAttributes[i][0] == visitedPoiCoords[0]:
                poiDifficulty = self.poiAttributes[i][4]
                self.poiVisitTimes[i][0] = arrivalTime #add arrival time to poi to "poi visit times" column vector
                break
        
        #Determine how the robot got to the poi - robot means autonomous, operator means sharedMode
        #If sharedMode then adjust image quality based on operator skill level in TAKING PICTURE
        navigatingAgent = "robot" #or "operator"
        if navigatingAgent == "robot":
            imageQuality = self.getImageQuality(robotName, assignedOperator, sharedMode=False)
        else:
            assignedOperator = 0 #Determine which operator was assigned to drive the robot to this poi
            imageQuality = self.getImageQuality(robotName, assignedOperator, sharedMode=True)

        #Now work on determining which agent has been assigned to classify the poi
        #First try to find the robot assigned to this poi based on its xyz location
        foundAssignedRobot = False
        row = 0
        assignedRobot = -1
        for i, row in enumerate(self.robotPoiAssignments):
            for coord in row:
                if visitedPoiCoords[0] == coord[0] and visitedPoiCoords[1] == coord[1]:
                    assignedRobot = i
                    break
        
        #POI was actually assigned to a robot
        if assignedRobot != -1:
            self.get_logger().info(f"AD assigned to {assignedRobot}")
            robotNumber = re.findall(r"\d+", assignedRobot)[0] #extract the number of the robot from string
            robotClassificationAccuracy = self.robotProbabilitiesLUT[imageQuality][poiDifficulty - 1]
            robotPredictionResult = self.flipFunction(robotClassificationAccuracy, poiDifficulty)
            
            if "moose" in assignedRobot:
                self.mooseMetrics[robotNumber][0].append(robotClassificationAccuracy)
                self.mooseMetrics[robotNumber][1].append(robotPredictionResult)
            else:
                self.mavicMetrics[robotNumber][0].append(robotClassificationAccuracy)
                self.mavicMetrics[robotNumber][1].append(robotPredictionResult)

        #POI was not assigned to a robot -> it was assigned to an operator instead
        else:
            #Find the operator assigned to this poi based on its xyz location
            foundAssignedHuman = False
            row = 0
            assignedOperator = -1
            for i, row in enumerate(self.humanPoiAssignments):
                for coord in row:
                    if visitedPoiCoords[0] == coord[0] and visitedPoiCoords[1] == coord[1]:
                        assignedOperator = i
                        break
            self.get_logger().info(f"AD assigned to operator {assignedOperator}")
            
            #THE 2 LINES BELOW ARE NOT NEEDED AS TBAR INDEPENDENT OF CLASSIFICATION SKILL LEVEL
            #Adjust image quality one more time - this time to adjust/account for operator skill level in CLASSIFYING POI
            #imageQuality = self.getImageQuality(robotName, assignedOperator, sharedMode=True)
            
            #Assign the t Bar value based on final image quality and poi difficulty
            tBar = self.tBarLUT[imageQuality][poiDifficulty - 1]
            self.operatorMetrics[assignedOperator][0].append(tBar)
            
            #Calculate value of Fs - this param is not a cumulative value from all the previously visited pois
            Fs = 1 / (1 + (m.exp(0.04 * (tBar - 180))))
            self.operatorMetrics[assignedOperator][1].append(Fs)

            #All the following parameters need a cumulative value of tBar starting from the very first poi visited
            tBarCumulative = 0
            for val in self.operatorMetrics[assignedOperator][0]:
                tBarCumulative += val

            #Calculate value of Ff - this param is a cumulative value so look through previous values in tBar list for a given operator
            if tBarCumulative >= 0 and tBarCumulative < ONE_HOUR_IN_SECONDS: #between 0 and 1 hour in seconds
                Ff = 1
            elif tBarCumulative >= ONE_HOUR_IN_SECONDS and tBarCumulative <= TWO_HOURS_IN_SECONDS: #between 1 and 2 hours in seconds
                Ff = -0.66 * tBarCumulative + 1.33
            self.operatorMetrics[assignedOperator][2].append(Ff)

            #Calculate value of Fw - this param is a cumulative value so look through previous values in tBar list for a given operator
            if len(self.operatorMetrics[assignedOperator][0]) == 0 or len(self.currentTimes[assignedOperator]) == 0 or (arrivalTime + self.pictureTakingTime >= self.currentTimes[assignedOperator][-1]):
                currentTime = arrivalTime + self.pictureTakingTime + tBar
            else:
                currentTime = self.currentTimes[assignedOperator][-1] + tBar
            self.currentTimes[assignedOperator].append(currentTime)

            fiveMinCutoff = max(0, currentTime - FIVE_MINS_IN_SECONDS) #if more than 5 mins have passed by, all good ; if not then cutoff is simply t = 0s
            workingTime = 0 #seconds
            for i, time in reversed(list(enumerate(self.currentTimes[assignedOperator]))):
                if time > fiveMinCutoff:
                    workingTime += self.operatorMetrics[assignedOperator][0][i] #append the values of tBar for the current operator while traversing in reverse
                else:
                    break #just break to avoid performing more for loop iterations than necessary once a time <= cutoff is found
            
            #Apply formula to calculate Fw and get result
            #If a current time is within the 5 min cutoff, we automatically include the entire tBar value, not just the fraction of the tBar that is within the cutoff
            #Therefore there is a chance that the total working time within the last 5 minutes is actually greater than 300 seconds (5 mins)
            #Therefore if this is the case simple cap the utilization to its maximum value of 1
            utilization = min(1, workingTime / FIVE_MINS_IN_SECONDS) 
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
            binaryResult = self.flipFunction(predictionProbability, poiDifficulty)
            self.operatorMetrics[assignedOperator][5].append(binaryResult)
            if binaryResult >= 0:
                self.get_logger().info(f"Operator {assignedOperator} identified AD correctly!")
            else:
                self.get_logger().info(f"Operator {assignedOperator} identified AD incorrectly!")

            currentOperatorScore = round(np.mean(self.operatorMetrics[assignedOperator][5]), 2)
            self.get_logger().info(f"Operator {assignedOperator} current average score is {currentOperatorScore}")

        if self.poisVisited == len(self.poiAttributes):
            self.getSimScore()


    def getImageQuality(self, robotName, assignedOperator, sharedMode):
        #1. Determine the base image quality based on the type of robot that took the image - UAV (medium) and UGV (upper-medium)
        imageQuality = "upper-medium" if "moose" in robotName else "medium"

        if sharedMode == True:
            #2. Upgrade or downgrade the image quality by 1 based on the skill level of the human operator assigned
            #Downgrade image quality if skill level is LOW
            if self.humanAttributes[assignedOperator][4] > 0 and self.humanAttributes[assignedOperator][4] < m.pi / 12:
                imageQuality = self.imageQualities[self.imageQualities.index(imageQuality) - 1]
                
            #Upgrade image quality if skill level is high
            elif self.humanAttributes[assignedOperator][4] > m.pi / 6 and self.humanAttributes[assignedOperator][4] < m.pi / 4:
                imageQuality = self.imageQualities[self.imageQualities.index(imageQuality) + 1]
        
        return imageQuality


    def flipFunction(self, predictionProbability, poiDifficulty):
        return (poiDifficulty * 10) if random.uniform(0, 1) <= predictionProbability else (poiDifficulty * -10)


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
        print("HEHEHE: ", __name__)