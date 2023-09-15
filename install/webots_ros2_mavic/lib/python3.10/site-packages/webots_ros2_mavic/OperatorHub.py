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

UAV_COORDS_FILE = '/home/arjun/SMART-LAB-ITAP-WEBOTS/install/webots_ros2_mavic/share/webots_ros2_mavic/resource/uavCoords.txt'
UGV_COORDS_FILE = '/home/arjun/SMART-LAB-ITAP-WEBOTS/install/webots_ros2_mavic/share/webots_ros2_mavic/resource/ugvCoords.txt'

FIVE_MINS_IN_SECONDS = 300
ONE_HOUR_IN_SECONDS = 3600
TWO_HOURS_IN_SECONDS = 7200

class OperatorHub(Node):
    def __init__(self):
        super().__init__("OperatorHub")
        #0: Safe POIs, 1: Threat POIs, 2: Humans, 3: UAVs, 4: UGVs
        self.numPois = self.getNumAgents(0) + self.getNumAgents(1)
        self.numHumans = self.getNumAgents(2)
        self.numUAVs = self.getNumAgents(3)
        self.numUGVs = self.getNumAgents(4)
        self.numTotal = self.numPois + self.numHumans + self.numUAVs + self.numUGVs
        self.uavMetrics = []
        self.ugvMetrics = []

        #Get list of all Pois
        self.pois = self.createPoiList()

        #Create assignment structures
        self.humanAttributes = self.getAgentAttributes('human', NUM_HUMAN_ATTRIBUTES)
        self.poiAttributes = self.getAgentAttributes('poi', NUM_POI_ATTRIBUTES)

        #Make the following variables have this structure: [{"mavic1": [poi1, poi2,...], "moose1": [poi3, poi4,...]}]
        self.masterPoiDict = {}
        self.masterPoiDict = self.performAllAssignments()
        self.get_logger().info(f"{self.masterPoiDict}")

        #Original structures for assignments
        self.humanPoiAssignments = self.getPoiAssignments()
        self.robotPoiAssignments = [] #from RL MODEL
        self.navigatorPoiAssignments = [] #from RL MODEL

        self.subscriber = self.create_subscription(DiverseArray, 'poiVisits', self.subCallback, 10)
        self.publisher = self.create_publisher(String, 'speedMode', 10)

        self.numParams = 6 #tBar, Fs, Ff, Fw, Pr, Correct(1)/Wrong(-1)        
        self.operatorMetrics = []
        self.operatorArrivalTimes = []
        

        self.poiVisitTimes = np.zeros((self.numPois, 1))
        self.currentTimes = []
        self.uavArrivalTimes = []
        self.ugvArrivalTimes = []
        self.pictureTakingTime = 3 #seconds
        self.poisVisited = 0
        
        self.imageQualities = ["low", "medium", "upper-medium", "high"]
        self.tBarLUT = {"low": [45, 125, 245], "medium": [35, 95, 195], "upper-medium": [25, 65, 145], "high": [15, 35, 95]}
        self.robotProbabilitiesLUT = {"low": [0.5, 0.3, 0.1], "medium": [0.6, 0.4, 0.2], "upper-medium": [0.7, 0.5, 0.3], "high": [0.8, 0.6, 0.4]}

        self.initializeNestedStructure(self.operatorMetrics, len(self.humanAttributes), self.numParams, 'l')
        self.initializeNestedStructure(self.uavMetrics, self.numUAVs, 2, 'l')
        self.initializeNestedStructure(self.ugvMetrics, self.numUGVs, 2, 'l')
        self.initializeNestedStructure(self.currentTimes, len(self.humanAttributes), 0, 'l')
        self.initializeNestedStructure(self.uavArrivalTimes, self.numUAVs, 0, 'l')
        self.initializeNestedStructure(self.ugvArrivalTimes, self.numUGVs, 0, 'l')
        self.initializeNestedStructure(self.operatorArrivalTimes, len(self.humanAttributes), 0, 'd')

        self.setInititalUGVSpeeds()
        #repeat for uavs

    def getNumAgents(self, i):
        f = open(NUM_AGENTS_FILE, '+r')
        list = f.read().splitlines()
        return(int(list[i]))
        
    
    def createPoiList(self):
        f = open(POIS_FILE, '+r')
        pois = f.readlines()
        processedPois = list(zip(*[iter(pois)] * 3))
        return processedPois
    
    
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
    
    def performAllAssignments(self):
        localDict = {}
        paramsList = []

        for i in range(self.numPois):
            navigatingAgent = ""
            classifyingAgent = ""
            
            #PART 1
            #First perform navigation assignment - who is navigation mode: ROBOT (0) or HUMAN (1)
            navigationMode = random.randint(0, 1)
            
            #Next if navigation mode is robot, look through the assignments from k means to determine which robot navigated
            if navigationMode == 0:
                #Now search through UAVCoords and UGVCoords files to determine which robot was assigned to navigate to the poi
                #Search through UAV file:
                uavNumber = self.findAssignedRobot('uav', self.pois[i])
                
                #UAV did not take the robot to this poi, so it must be a UGV
                if uavNumber == -1:
                    ugvNumber = self.findAssignedRobot('ugv', self.pois[i])
                    navigatingAgent = "moose" + str(ugvNumber)
                else:
                    navigatingAgent = "mavic" + str(uavNumber)

            else:
                humanNavigator = random.randint(0, self.numHumans - 1)
                navigatingAgent = "human" + str(humanNavigator)
            
            #PART 2
            #First perform classification assignment - who is classifying image: ROBOT (0) or HUMAN (1)
            classificationMode = random.randint(0, 1)

            #If the classification mode = navigation mode then the agent doing classification must be same as agent doing navigation
            if classificationMode == navigationMode:
                classifyingAgent = navigatingAgent
            else:
                if classificationMode == 0:
                    #Determine if the robot doing the classification is a UAV (0) or UGV(1)
                    robotClassifierType = random.randint(0, 1)
                    if robotClassifierType == 0:
                        classifyingAgent = "mavic" + str(random.randint(0, self.numUAVs - 1))
                    else:
                        classifyingAgent = "moose" + str(random.randint(0, self.numUGVs - 1))
                else:
                    classifyingAgent = "human" + str(random.randint(0, self.numHumans - 1))
            
            #Add the navigating and classifying agents to the params list for this poi to be added to the master dict
            self.pois[i] = list(self.pois[i])
            for j, coord in enumerate(self.pois[i]):
                self.pois[i][j] = self.pois[i][j].replace("\n", "")

            localDict[tuple(self.pois[i])] = [navigatingAgent, classifyingAgent]

        return localDict

    def findAssignedRobot(self, mode, targetPoi):
        #Select correct file based on which robot we are checking
        file = UAV_COORDS_FILE if mode == 'uav' else UGV_COORDS_FILE
        f = open(file, '+r')
        allCoords = f.read().splitlines()
        
        #Pre-process targetPoi to first combine all coords into 1 string
        targetPoiString = targetPoi[0].replace("\n", "") + " " + targetPoi[1].replace("\n", "")

        #Get list of poi clusters in same format as clustersMatrix from main.py
        robotNumber = -1
        for i, cluster in enumerate(allCoords):
            clusterCoords = cluster.split(", ")
            #self.get_logger().info(f"CLUSTER: {clusterCoords}")
            #self.get_logger().info(f"TARGET: {targetPoiString}")

            for poi in clusterCoords:
                #self.get_logger().info(f"{targetPoiString}       {poi}")
                if targetPoiString in poi:
                    robotNumber = i
                    break
            
            if robotNumber != -1:
                break
        
        #targetPoi -> if found return robot number, else return -1
        return robotNumber
    

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



    def initializeNestedStructure(self, array, outerParam, innerParam, dType):
        for i in range(outerParam):
            array.append([]) if dType == 'l' else array.append({})
            for param in range(innerParam):
                array[i].append([]) if dType == 'l' else array.append({})
    
    def findAssignedHuman(self, visitedPoiCoords, mode):
        assignedHuman = -1
        searchList = self.humanPoiAssignments if mode == 'o' else self.navigatorPoiAssignments
        for i, row in enumerate(searchList):
            for coord in row:
                if visitedPoiCoords[0] == coord[0] and visitedPoiCoords[1] == coord[1]:
                    assignedHuman = i
                    break

        return assignedHuman
    
    def setInititalUGVSpeeds(self):
        for ugv in self.robotPoiAssignments:
            self.configureRobotSpeedMode(ugv)

    def subCallback(self, msg):
        self.poisVisited += 1
        robotName = msg.robot_name
        visitedPoiCoords = [round(msg.poi_x, 2), round(msg.poi_y, 2), round(msg.poi_z, 2)]
        arrivalTime = msg.arrival_time

        #Determine whether a uav/ugv arrived at poi and what its number was
        robotNumber = int(re.findall(r"\d+", robotName)[0])
        uavArrived = 1 if "mavic" in robotName else 0

        #Add arrival time to appropriate array 
        if "moose" in robotName:
            self.ugvArrivalTimes[robotNumber].append(arrivalTime)
        else:
            self.uavArrivalTimes[robotNumber].append(arrivalTime)

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
            imageQuality = self.getImageQuality(robotName, assignedNavigator=-1, sharedMode=False)
            
        else:
            assignedNavigator = 0 #Determine which operator was assigned to drive the robot to this poi
            imageQuality = self.getImageQuality(robotName, assignedNavigator, sharedMode=True)
            
            #Determine how long it took human to drive this robot from the previous poi to this one
            robotArrivalTimes = self.uavArrivalTimes if uavArrived else self.ugvArrivalTimes
            if len(robotArrivalTimes[robotNumber]) == 1:
                navigationTime = arrivalTime
            else:
                navigationTime = arrivalTime - robotArrivalTimes[robotNumber][-2] - self.pictureTakingTime #[-2] = arrival time for prev poi

            #Now append this navigation time to the dict: {arrival time: navigation time}
            self.operatorArrivalTimes[assignedNavigator][arrivalTime] = navigationTime

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
            assignedOperator = self.findAssignedHuman(visitedPoiCoords, 'o')
            self.get_logger().info(f"AD assigned to operator {assignedOperator}")
            
            #Assign the t Bar value based on final image quality and poi difficulty -> add human navigation time to tBar if human drove robot to poi
            self.get_logger().info(f"{self.imageQualities.index(imageQuality)}  {poiDifficulty - 1}")
            tBar = self.tBarLUT[imageQuality][int(poiDifficulty - 1)]
            self.operatorMetrics[assignedOperator][0].append(tBar)
            
            #Calculate value of Fs - this param is not a cumulative value from all the previously visited pois
            Fs = 1 / (1 + (m.exp(0.04 * (tBar - 180))))
            self.operatorMetrics[assignedOperator][1].append(Fs)

            #All the following parameters need a cumulative value of tBar starting from the very first poi visited
            tBarCumulative = 0
            for val in self.operatorMetrics[assignedOperator][0]:
                tBarCumulative += val

            #The following parameters also need a cumulative value of all the navigation times for the assigned operator
            navTimeCumulative = 0
            for arrivalTime in self.operatorArrivalTimes[assignedOperator].keys():
                navTimeCumulative += self.operatorArrivalTimes[assignedOperator][arrivalTime]
            
            cumulativeWorkingTime = tBarCumulative + navTimeCumulative

            #Calculate value of Ff - this param is a cumulative value so look through previous values in tBar + navTime lists for a given operator
            if cumulativeWorkingTime >= 0 and cumulativeWorkingTime < ONE_HOUR_IN_SECONDS: #between 0 and 1 hour in seconds
                Ff = 1
            elif cumulativeWorkingTime >= ONE_HOUR_IN_SECONDS and cumulativeWorkingTime <= TWO_HOURS_IN_SECONDS: #between 1 and 2 hours in seconds
                Ff = -0.66 * cumulativeWorkingTime + 1.33
            self.operatorMetrics[assignedOperator][2].append(Ff)

            #Calculate value of Fw - this param is a cumulative value so look through previous values in tBar list for a given operator
            if len(self.operatorMetrics[assignedOperator][0]) == 0 or len(self.currentTimes[assignedOperator]) == 0 or (arrivalTime + self.pictureTakingTime >= self.currentTimes[assignedOperator][-1]):
                currentTime = arrivalTime + self.pictureTakingTime + tBar
            else:
                currentTime = self.currentTimes[assignedOperator][-1] + tBar
            self.currentTimes[assignedOperator].append(currentTime)

            fiveMinCutoff = max(0, currentTime - FIVE_MINS_IN_SECONDS) #if more than 5 mins have passed by, all good ; if not then cutoff is simply t = 0s
            workingTime = 0 #seconds

            #append the values of tBar for the current operator while traversing in reverse
            for i, time in reversed(list(enumerate(self.currentTimes[assignedOperator]))):
                if time > fiveMinCutoff:
                    workingTime += self.operatorMetrics[assignedOperator][0][i] 
                else:
                    break #just break to avoid performing more for loop iterations than necessary once a time <= cutoff is found
            
            #append the values of navigation time for the current operator while traversing through dict in reverse
            for arrivalTime in reversed(self.operatorArrivalTimes[assignedOperator].keys()):
                if arrivalTime > fiveMinCutoff:
                    workingTime += self.operatorArrivalTimes[assignedOperator][arrivalTime]
                else:
                    break

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
        
        #As a last step in processing this visit to the poi...
        self.configureRobotSpeedMode(robotName, visitedPoiCoords)


    def getImageQuality(self, robotName, assignedNavigator, sharedMode):
        #1. Determine the base image quality based on the type of robot that took the image - UAV (medium) and UGV (upper-medium)
        imageQuality = "upper-medium" if "moose" in robotName else "medium"

        if sharedMode == True:
            #2. Upgrade or downgrade the image quality by 1 based on the skill level of the human operator that drove the robot
            #Downgrade image quality if skill level is LOW
            if self.humanAttributes[assignedNavigator][4] > 0 and self.humanAttributes[assignedNavigator][4] < m.pi / 12:
                imageQuality = self.imageQualities[self.imageQualities.index(imageQuality) - 1]
                
            #Upgrade image quality if skill level is high
            elif self.humanAttributes[assignedNavigator][4] > m.pi / 6 and self.humanAttributes[assignedNavigator][4] < m.pi / 4:
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
    

    def configureRobotSpeedMode(self, robotName, visitedPoiCoords=-1):
        #1. Determine the next poi the robot is going to - if there is one
        #-1 means we are at start of sim and havent visited anything yet => next poi is index 0 in robot poi assignments
        if visitedPoiCoords == -1:
            nextPoi = self.robotPoiAssignments[robotName][0]
            foundValidPoi = True
        else:
            poiIndex = self.robotPoiAssignments[robotName].index(visitedPoiCoords) 
            if poiIndex != len(self.robotPoiAssignments[robotName]) - 1:
                nextPoi = self.robotPoiAssignments[robotName][poiIndex + 1]
                foundValidPoi = True
            else:
                foundValidPoi = False
        
        if foundValidPoi:
            #2. Determine if a human operator will be controlling the robot or whether the robot will be autonomous
            if any(nextPoi in navigatorAssignments for navigatorAssignments in self.navigatorPoiAssignments):
            
                #3. If it is teleoperated, get the human skill level and publish a low/high speed mode message
                assignedNavigator = self.findAssignedHuman(nextPoi, 'n')
                #No operator assigned to the next poi which means the robot drives autonomously
                if assignedNavigator == -1:
                    self.publisher.publish(robotName + " medium")
                #An operator is assigned to the next poi so adjust speed based on skill level
                else:
                    navigatorSkillLevel = self.humanAttributes[assignedNavigator][4]
                    if navigatorSkillLevel > 0 and navigatorSkillLevel < m.pi / 12:
                        self.publisher.publish(robotName + " low")
                    elif navigatorSkillLevel >= m.pi / 12 and navigatorSkillLevel <= m.pi / 6:
                        self.publisher.publish(robotName + " medium")
                    else:
                        self.publisher.publish(robotName + " high")
        

    

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