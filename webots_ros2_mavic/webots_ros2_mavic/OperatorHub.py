#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import std_msgs
from std_msgs.msg import String
from std_msgs.msg import Int32MultiArray
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3
import sys
import numpy as np
import random
import re
import math as m
import random
from matplotlib import pyplot as plt
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

POI_ASSIGNMENTS_FILE = '/home/arjun/SMART-LAB-ITAP-WEBOTS/webots_ros2_mavic/resource/poiAssignments.txt'

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

        #Master POI Dict has following structure: {(x1, y1, z1): [robotUsed, navigatingAgent, classifyingAgent], (x2, y2, z2): [...], ...}
        #self.masterPoiDict = {(47.0, -12.0): ['mavic0', 'human1', 'human3'], (16.0, -24.0): ['mavic0', 'human3', 'mavic0'], (14.0, 21.0): ['mavic1', 'mavic1', 'mavic1'], (-3.0, 11.0): ['mavic1', 'mavic1', 'mavic1'], (1.0, 13.0): ['mavic1', 'mavic1', 'human1'], (11.0, -28.0): ['mavic0', 'mavic0', 'mavic0'], (-4.0, 10.0): ['mavic1', 'human5', 'human0'], (33.0, 12.0): ['mavic1', 'human0', 'mavic1'], (-26.0, -21.0): ['moose0', 'moose0', 'moose0'], (-33.0, -9.0): ['moose0', 'moose0', 'moose0'], (16.0, -23.0): ['mavic0', 'mavic0', 'human3'], (-12.0, 0.0): ['moose0', 'human0', 'human3'], (36.0, -31.0): ['mavic0', 'human5', 'mavic0'], (-41.0, 7.0): ['moose0', 'moose0', 'moose0'], (-23.0, 48.0): ['moose1', 'moose1', 'moose1'], (-20.0, -8.0): ['moose0', 'human2', 'moose0'], (46.0, 20.0): ['mavic1', 'mavic1', 'mavic1'], (-8.0, -36.0): ['mavic0', 'human0', 'human4'], (-23.0, -6.0): ['moose0', 'moose0', 'moose0'], (25.0, -12.0): ['mavic0', 'human4', 'human2'], (-1.0, 46.0): ['moose1', 'human1', 'moose1'], (11.0, 27.0): ['mavic1', 'human3', 'human0'], (-49.0, 41.0): ['moose1', 'moose1', 'moose1'], (5.0, 48.0): ['moose1', 'human3', 'moose1'], (-40.0, -20.0): ['moose0', 'moose0', 'human5'], (-44.0, 0.0): ['moose0', 'moose0', 'moose0'], (-28.0, 45.0): ['moose1', 'human4', 'moose1'], (-41.0, 47.0): ['moose1', 'human2', 'moose1'], (-10.0, 37.0): ['moose1', 'moose1', 'human5'], (-23.0, 0.0): ['moose0', 'human5', 'moose0'], (23.0, -2.0): ['mavic1', 'mavic1', 'human1'], (1.0, -12.0): ['mavic0', 'mavic0', 'human0'], (2.0, 26.0): ['mavic1', 'human1', 'human1'], (25.0, 20.0): ['mavic1', 'human4', 'human5'], (10.0, -33.0): ['mavic0', 'mavic0', 'human1']}
        self.masterPoiDict = {}
        self.masterPoiDict = self.performAllAssignments()
        self.writeAssignmentsToFile()
        self.get_logger().info(f"{self.masterPoiDict}")

        #Original structures for assignments
        self.humanPoiAssignments = self.getPoiAssignments()
        self.robotPoiAssignments = {} #from RL MODEL
        self.navigatorPoiAssignments = {} #from RL MODEL

        self.subscriber = self.create_subscription(DiverseArray, 'poiVisits', self.subCallback, 10)
        self.nextPoiSubscriber = self.create_subscription(DiverseArray, 'nextPoiLocation', self.nextPoiCallback, 10)
        self.publisher = self.create_publisher(String, 'speedMode', 10)

        self.numParams = 7 #tBar, Fs, Ff, Fw, Pr, Correct(1)/Wrong(-1), utilization fraction        
        self.operatorMetrics = []
        self.operatorArrivalTimes = []
        self.utilizationArray = []
        self.cognitiveFactorArray = []        
        self.taskDifficultyArray = []

        self.poiVisitTimes = np.zeros((self.numPois, 1))
        self.currentTimes = []
        self.uavArrivalTimes = []
        self.ugvArrivalTimes = []
        self.pictureTakingTime = 3 #seconds
        self.poisVisited = 0
        self.numCorrect = 0
        
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
        self.initializeNestedStructure(self.utilizationArray, self.numHumans, 0, 'l')   
        self.initializeNestedStructure(self.cognitiveFactorArray,self.numHumans, 0, 'l')
        self.initializeNestedStructure(self.taskDifficultyArray,self.numHumans, 0, 'l')


        self.setInitialRobotSpeeds()

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
            robotUsed = ""

            #PART 1
            #Determine which robot was used to get to the POI

            #Now search through UAVCoords and UGVCoords files to determine which robot was assigned to navigate to the poi
            #Search through UAV file:
            uavNumber = self.findAssignedRobot('uav', self.pois[i])
                
            #UAV did not take the robot to this poi, so it must be a UGV
            if uavNumber == -1:
                ugvNumber = self.findAssignedRobot('ugv', self.pois[i])
                robotUsed = "moose" + str(ugvNumber)
            else:
                robotUsed = "mavic" + str(uavNumber)


            #Next, perform navigation assignment - who is navigation mode: ROBOT (0) or HUMAN (1)
            navigationMode = random.randint(0, 1)
            
            #Next if navigation mode is robot, look through the assignments from k means to determine which robot navigated
            if navigationMode == 0:
                navigatingAgent = robotUsed
            else:
                humanNavigator = random.randint(0, self.numHumans - 1)
                navigatingAgent = "human" + str(humanNavigator)
            
            #PART 2
            #First perform classification assignment - who is classifying image: ROBOT (0) or HUMAN (1)
            classificationMode = random.randint(0, 1)

            #If the classification mode is ROBOT(0) then we check to see which robot was used to get to the POI and assign that to be classifying agent
            if classificationMode == 0:
                classifyingAgent = robotUsed
            
            #However, if the classification mode is HUMAN(1) then we randomly assign a human to be the classifying agent
            else:
                classifyingAgent = "human" + str(random.randint(0, self.numHumans - 1))
            
            #Add the navigating and classifying agents to the params list for this poi to be added to the master dict
            self.pois[i] = list(self.pois[i])
            for j, coord in enumerate(self.pois[i]):
                self.pois[i][j] = float(self.pois[i][j].replace("\n", ""))

            localDict[(self.pois[i][0], self.pois[i][1])] = [robotUsed, navigatingAgent, classifyingAgent]

        return localDict

    def writeAssignmentsToFile(self):
        #Format: (x, y) [robot used to reach POI, navigating agent, picture-taking agent]
        f = open(POI_ASSIGNMENTS_FILE, 'w')
        for item in self.masterPoiDict:
            f.write(str(item) + " " + str(self.masterPoiDict[item]) + "\n")


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
            for poi in clusterCoords:
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
    
    def setInitialRobotSpeeds(self):
        for i in range(self.numUAVs):
            self.configureRobotSpeedMode("Mavic_2_PRO_" + str(i), -1, 1) #-1 -> havent visited any POIs yet, 1 -> at start of sim
        
        for i in range(self.numUGVs):
            self.configureRobotSpeedMode("moose" + str(i), -1, 1) #-1 -> havent visited any POIs yet, 1 -> at start of sim

    def plotHumanUtilization(self):
        numPlots = 3 #self.numHumans
        fig, axs = plt.subplots(numPlots, constrained_layout=True)
        
        fig2, axs2 = plt.subplots(numPlots, constrained_layout=True)

        fig3, axs3 = plt.subplots(numPlots, constrained_layout=True)


        # humanArray = self.utilizationArray[0]
        # times = [coord[0] for coord in humanArray]
        # utilizations = [coord[1] for coord in humanArray]
        # axs[0].plot(times, utilizations)
        # axs[0].set_title('Human ' + str(0))
        # axs[0].set(xlabel='Time (minutes)', ylabel='Utilization')

        for i, cognitiveArray in enumerate(self.cognitiveFactorArray):
            if i <= 2:
                times = [coord[0] for coord in cognitiveArray]
                Ef = [coord[1] for coord in cognitiveArray]
                finalFactor = [self.humanAttributes[0][3] * element for element in Ef]
                axs[i].plot(times, finalFactor)
                axs[i].set_title('Human ' + str(i))
                axs[i].set(xlabel='Time (minutes)', ylabel='Cognitive Workload Factor')


        for i, humanArray in enumerate(self.utilizationArray):
            if i <= 2:
                #Separate times and utilizations
                times = [coord[0] for coord in humanArray]
                utilizations = [coord[1] for coord in humanArray]

                #Plot utilization fraction as a function of time in minutes
                axs2[i].plot(times, utilizations)
                axs2[i].set_title('Human ' + str(i))
                axs2[i].set(xlabel='Time (minutes)', ylabel='Utilization')
        
        for i, taskArray in enumerate(self.taskDifficultyArray):
            if i <= 2:
                #Separate times and utilizations
                times = [coord[0] for coord in taskArray]
                Es = [coord[1] for coord in taskArray]
                finalFactor = [self.humanAttributes[0][4] * element for element in Es]
                axs3[i].scatter(times, finalFactor)
                axs3[i].set_title('Human ' + str(i))
                axs3[i].set(xlabel='Time (minutes)', ylabel='Task Difficulty Factor')
        
        plt.show()

    def subCallback(self, msg):
        self.poisVisited += 1
        robotName = msg.robot_name #if UAV: Mavic_2_Pro_# but if UGV: moose#
        visitedPoiCoords = (round(msg.poi_x, 1), round(msg.poi_y, 1))
        arrivalTime = msg.arrival_time
        self.get_logger().info(f"arrived at: {arrivalTime}")
        #Determine whether a uav/ugv arrived at poi and what its number was
        robotNumber = int(re.findall(r"\d+", robotName[::-1])[0])
        uavArrived = 1 if "Mavic" in robotName else 0
        
        #Add arrival time to appropriate array 
        if "moose" in robotName:
            self.ugvArrivalTimes[robotNumber].append(arrivalTime)
        else:
            self.uavArrivalTimes[robotNumber].append(arrivalTime)

        #Determine the poi difficulty level
        poiDifficulty = -1
        for i in range(len(self.poiAttributes)):
            if self.poiAttributes[i][0] == visitedPoiCoords[0]:
                poiDifficulty = int(self.poiAttributes[i][4])
                self.poiVisitTimes[i][0] = arrivalTime #add arrival time to poi to "poi visit times" column vector
                break
        
        #Determine how the robot got to the poi - robot means autonomous, operator means sharedMode
        #If sharedMode then adjust image quality based on operator skill level in TAKING PICTURE
        navigatingAgent = self.masterPoiDict[visitedPoiCoords][1]
        navigatingAgentNumber = int(re.findall(r"\d+", navigatingAgent)[0])

        #If the robot was driven autonomously to the POI
        if "human" not in navigatingAgent:
            imageQuality = self.getImageQuality(robotName, assignedNavigator=-1, sharedMode=False)
        
        #If a human drove the robot to the POI
        else:
            imageQuality = self.getImageQuality(robotName, navigatingAgentNumber, sharedMode=True)
            
            #Determine how long it took human to drive this robot from the previous poi to this one
            robotArrivalTimes = self.uavArrivalTimes if uavArrived else self.ugvArrivalTimes
            if len(robotArrivalTimes[robotNumber]) == 1:
                navigationTime = arrivalTime
            else:
                navigationTime = arrivalTime - robotArrivalTimes[robotNumber][-2] - self.pictureTakingTime #[-2] = arrival time for prev poi

            #Now append this navigation time to the dict: {arrival time: navigation time}
            self.operatorArrivalTimes[navigatingAgentNumber][arrivalTime] = navigationTime

        ###  At this point, we have identified how the robot got to the POI (robot controlled or human controlled). ###
        ###  We also determined the image quality based on how the robot got to the POI. ###
        ###  It is now time to determine who has been assigned to classify the POI and calculate the prediction probability score accordingly  ###

        classifyingAgent = self.masterPoiDict[visitedPoiCoords][2]
        classifyingAgentNumber = int(re.findall(r"\d+", classifyingAgent)[0])

        #Robot will be classifying POI
        if "human" not in classifyingAgent:
            self.get_logger().info(f"AD assigned to {classifyingAgent}")
            robotNumber = classifyingAgentNumber
            robotClassificationAccuracy = self.robotProbabilitiesLUT[imageQuality][poiDifficulty - 1]
            robotPredictionResult = self.flipFunction(robotClassificationAccuracy, poiDifficulty)
            
            if robotPredictionResult > 0:
                self.numCorrect += 1
                self.get_logger().info(f"{classifyingAgent} identified AD correctly!")
            else:
                self.get_logger().info(f"{classifyingAgent} identified AD incorrectly!")

            if "moose" in classifyingAgent:
                self.ugvMetrics[robotNumber][0].append(robotClassificationAccuracy)
                self.ugvMetrics[robotNumber][1].append(robotPredictionResult)
            else:
                self.uavMetrics[robotNumber][0].append(robotClassificationAccuracy)
                self.uavMetrics[robotNumber][1].append(robotPredictionResult)

        #Human will be classifying POI
        else:
            row = 0
            assignedOperator = classifyingAgentNumber
            self.get_logger().info(f"AD assigned to {classifyingAgent}")
            
            #Assign the t Bar value based on final image quality and poi difficulty -> add human navigation time to tBar if human drove robot to poi
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
            
            cumulativeWorkingTime = (tBarCumulative + navTimeCumulative) / ONE_HOUR_IN_SECONDS #in hours

            #Calculate value of Ff - this param is a cumulative value so look through previous values in tBar + navTime lists for a given operator
            if cumulativeWorkingTime >= 0 and cumulativeWorkingTime < 1: #between 0 and 1 hour
                Ff = 1
            elif cumulativeWorkingTime >= 1 and cumulativeWorkingTime <= 2: #between 1 and 2 hours
                Ff = -0.3 * cumulativeWorkingTime + 1.3
            self.operatorMetrics[assignedOperator][2].append(Ff)

            #Calculate value of Fw - this param is a cumulative value so look through previous values in tBar list for a given operator
            if len(self.operatorMetrics[assignedOperator][0]) == 0 or len(self.currentTimes[assignedOperator]) == 0 or (arrivalTime + self.pictureTakingTime >= self.currentTimes[assignedOperator][-1]):
                currentTime = arrivalTime + self.pictureTakingTime + tBar
            else:
                currentTime = self.currentTimes[assignedOperator][-1] + tBar
            self.currentTimes[assignedOperator].append(currentTime)

            
            #Append the Fs value for the final plots
            self.taskDifficultyArray[assignedOperator].append((currentTime / 60, Fs))


            #The following code interpolates between the current time and the previous POI's current time to fill in all intermediate utilization values
            #If there is only one element in currentTimes array ATM, then the starting value should be t = 0
            #Else, it should be the current time of the previous POI
            initValue = 1 if len(self.currentTimes[assignedOperator]) == 1 else int(self.currentTimes[assignedOperator][-2]) + 2

            #Sample at 1 second intervals
            self.get_logger().info(f"init: {initValue}  currenttime: {int(currentTime)}")
            for iter, interpCurrentTime in enumerate(range(initValue, int(currentTime) + 2, 1)):
                fiveMinCutoff = max(0, interpCurrentTime - FIVE_MINS_IN_SECONDS) #if more than 5 mins have passed by, all good ; if not then cutoff is simply t = 0s
                workingTime = 0 #seconds

                #append the values of tBar for the current operator while traversing in reverse
                for i, time in reversed(list(enumerate(self.currentTimes[assignedOperator]))):
                    classifyingTime = self.operatorMetrics[assignedOperator][0][i] 
                    if time > fiveMinCutoff: #and (time < interpCurrentTime if interpCurrentTime != 0 else True):
                        if time > interpCurrentTime:
                            if interpCurrentTime > time - classifyingTime:
                                workingTime += (interpCurrentTime - (time - classifyingTime))
                        else:
                            workingTime += classifyingTime
                    else:
                        break #just break to avoid performing more for loop iterations than necessary once a time <= cutoff is found

                #append the values of navigation time for the current operator while traversing through dict in reverse
                for arrivalTime in reversed(self.operatorArrivalTimes[assignedOperator].keys()):
                    navTime = self.operatorArrivalTimes[assignedOperator][arrivalTime]
                    if arrivalTime > fiveMinCutoff:
                        if arrivalTime > interpCurrentTime:
                            if interpCurrentTime > arrivalTime - navTime:
                                workingTime += (interpCurrentTime - (arrivalTime - navTime))
                        else:
                            workingTime += (navTime if (arrivalTime - navTime > fiveMinCutoff) else arrivalTime - fiveMinCutoff)
                    else:
                        break

                #Apply formula to calculate Fw and get result
                #If a current time is within the 5 min cutoff, we automatically include the entire tBar value, not just the fraction of the tBar that is within the cutoff
                #Therefore there is a chance that the total working time within the last 5 minutes is actually greater than 300 seconds (5 mins)
                #Therefore if this is the case simple cap the utilization to its maximum value of 1
                if interpCurrentTime < FIVE_MINS_IN_SECONDS:
                    utilization = min(1, workingTime / (workingTime if interpCurrentTime == 0 else interpCurrentTime))
                else:
                    utilization = min(1, workingTime / FIVE_MINS_IN_SECONDS)
                self.utilizationArray[assignedOperator].append((interpCurrentTime / 60, utilization))
                self.cognitiveFactorArray[assignedOperator].append((interpCurrentTime / 60, self.calculateWorkloadFactor(utilization)))


            self.get_logger().info(f"Entry: {(round(currentTime / 60, 2), utilization)}")
            #The following code for Fw and appending to array is only done for the real currentTime value corresponding to the POI visit, not the interps.
            self.operatorMetrics[assignedOperator][6].append(utilization) 
            Fw = self.calculateWorkloadFactor(utilization)
            self.operatorMetrics[assignedOperator][3].append(Fw)
            
            #Calculate probability of operator predicting a given poi correctly
            predictionProbability = 0.5 + (Fs * Ff * Fw * m.sin(self.humanAttributes[assignedOperator][3]) * m.sin(self.humanAttributes[assignedOperator][4]))
            self.operatorMetrics[assignedOperator][4].append(predictionProbability)

            #Use flip function to convert probability into correct/wrong prediction
            binaryResult = self.flipFunction(predictionProbability, poiDifficulty)
            self.operatorMetrics[assignedOperator][5].append(binaryResult)
            if binaryResult > 0:
                self.numCorrect += 1
                self.get_logger().info(f"{classifyingAgent} identified AD correctly!")
            else:
                self.get_logger().info(f"{classifyingAgent} identified AD incorrectly!")

            #OPTIONAL STATISTIC TO PRINT
            #currentOperatorScore = round(np.mean(self.operatorMetrics[assignedOperator][5]), 2)
            #self.get_logger().info(f"{classifyingAgent} current average score is {currentOperatorScore}")

        self.get_logger().info(f"Visited {self.poisVisited}/{self.numPois}")
        if self.poisVisited == len(self.poiAttributes):
            self.getSimScore()
        
            #Simulation has finished at this point (robots are returning or have returned home) -> Now graph utilization for humans
            #self.plotHumanUtilization()
        
        #As a last step in processing this visit to the poi...
        #self.configureRobotSpeedMode(robotName, visitedPoiCoords)


    def calculateWorkloadFactor(self, utilization):
        if utilization >= 0 and utilization < 0.45:
            Fw = -2.47 * m.pow(utilization, 2) + 2.22 * utilization + 0.5
        elif utilization >= 0.45 and utilization < 0.65:
            Fw = 1
        elif utilization >= 0.65 and utilization <= 1:
            Fw = -4.08 * m.pow(utilization, 2) + 5.31 * utilization - 0.724
        return Fw

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
        return (poiDifficulty * 10) if random.uniform(0, 1) <= predictionProbability else 0 #Used to be: (poiDifficulty * -10) if applying negative penalty

    def calculateAgentNetScore(self, agentType):
        if agentType == "human":
            numSlots = self.numHumans
            metricsArray = self.operatorMetrics
            index = 5
        elif agentType == "uav":
            numSlots = self.numUAVs
            metricsArray = self.uavMetrics
            index = 1
        else:
            numSlots = self.numUGVs
            metricsArray = self.ugvMetrics
            index = 1

        netScores = np.zeros((1, numSlots))
        for i, metrics in enumerate(metricsArray):
            agentTotalScore = np.sum(metrics[index])
            netScores[0][i] = agentTotalScore
        agentTypeNetScore = np.mean(netScores[0])
        return agentTypeNetScore

    def getSimScore(self):
        humansNetScore = self.calculateAgentNetScore("human")
        ugvsNetScore = self.calculateAgentNetScore("ugv")
        uavsNetScore = self.calculateAgentNetScore("uav")

        #Get final sim score
        finalSimScore = round((humansNetScore + ugvsNetScore + uavsNetScore) / 3, 2)
        self.get_logger().info(f"Final Simulation Score: {finalSimScore}")

        #Get final team accuracy percentage
        finalTeamAccuracy = 100 * round(self.numCorrect / self.numPois, 2)
        self.get_logger().info(f"Final Team Accuracy Percentage: {finalTeamAccuracy}%")
    

    def nextPoiCallback(self, msg):
        robotName = msg.robot_name #if UAV: Mavic# but if UGV: moose#
        nextPoiCoords = (round(msg.poi_x, 1), round(msg.poi_y, 1))
        self.configureRobotSpeedMode(robotName, nextPoiCoords, 0)


    def configureRobotSpeedMode(self, robotName, nextPoiCoords, atStartofSim):
        robotNumber = int(re.findall(r"\d+", robotName[::-1])[0])
        firstTime = False

        #if at start of sim and going to visit first POI
        if atStartofSim:
            firstTime = True
            if "Mavic" in robotName:
                file = UAV_COORDS_FILE
            else:
                file = UGV_COORDS_FILE

            f = open(file, '+r')
            allCoords = f.read().splitlines()

            #Examine the line in the file corresponding to the number of the robot
            robotPois = allCoords[robotNumber]

            #Extract the first POI coordinate from this line and store as tuple (x, y)
            #First split the whole line at each comma to separate out each POI coordinate
            splitAtCommas = robotPois.split(',')

            #Then split the first POI coordinate at each space to extract each x, y, z coordinate
            splitAtSpaces = splitAtCommas[0].split(' ')

            #Generate the POI coordinate tuple
            nextPoiCoords = (round(float(splitAtSpaces[0]), 2), round(float(splitAtSpaces[1]), 2))

        navigatingAgent = self.masterPoiDict[nextPoiCoords][1]
        navigatingAgentNumber = int(re.findall(r"\d+", navigatingAgent)[0])

        message = String()
        #Robot is driving autonomously to next POI so use MEDIUM speed
        if "human" not in navigatingAgent:
            message.data = robotName + " medium"
            if firstTime:
                subscriberCount = self.publisher.get_subscription_count() #should equal 2 after moose_autonomy and mavic_autonomy launch
                while subscriberCount < 2:
                    subscriberCount = self.publisher.get_subscription_count()    
            self.publisher.publish(message)
            self.get_logger().info(f"{robotName} driving AUTONOMOUSLY")

        
        #Human is driving to next POI to use operator skill level to determine speed
        else:
            navigatorSkillLevel = self.humanAttributes[navigatingAgentNumber][4]
            if navigatorSkillLevel > 0 and navigatorSkillLevel < m.pi / 12:
                message.data = robotName + " low"
            elif navigatorSkillLevel >= m.pi / 12 and navigatorSkillLevel <= m.pi / 6:
                message.data = robotName + " medium"
            else:
                message.data = robotName + " high"

            if firstTime:
                subscriberCount = self.publisher.get_subscription_count() #should equal 2 after moose_autonomy and mavic_autonomy launch
                while subscriberCount < 2:
                    subscriberCount = self.publisher.get_subscription_count()
            self.publisher.publish(message)
            self.get_logger().info(f"{robotName} controlled by human {navigatingAgentNumber}")
        self.get_logger().info(f"Current robot speed mode: {message.data}")    

        


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