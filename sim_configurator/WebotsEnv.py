import sys
import numpy as np
import random
import re
import math

class WebotsEnv:
    def __init__(self, numSafe, numThreats, numHumans, numUAVs, numUGVs):
        self.numSafe = numSafe
        self.numThreats = numThreats
        self.numPOIs = self.numSafe + self.numThreats
        self.numHumans = numHumans
        self.numUGVs = numUGVs
        self.numUAVs = numUAVs
        self.numRobots = self.numUGVs + self.numUAVs
    
        self.setupPOIs()
        self.setupHumans()
        self.setupRobots()
    

    def setupPOIs(self):
        #Store: x pos, y pos, z pos, threat (1)/safe (0), task difficulty (1 - easy to 3 - hard)
        self.poiAttributes = np.zeros((self.numPOIs, 5)) 

        #Assign xyz location
        for i in range(self.numSafe + self.numThreats):
            self.poiAttributes[i][0] = random.uniform(-1500, 1500)
            self.poiAttributes[i][1] = random.uniform(-1400, 1500) #-1400 to avoid overlapping with the robots and humans
            self.poiAttributes[i][2] = 0

        #Assign threat/safe
        self.poiAttributes[0:self.numSafe, 3] = 0
        self.poiAttributes[self.numSafe:, 3] = 1

        #Assign task difficulty
        for i in range(self.numPOIs):
            self.poiAttributes[i][4] = random.uniform(1,4)
        
        #Write to sim
        cutoff = 42
        f = open('/home/arjun/SMART-LAB-ITAP-WEBOTS/webots_ros2_mavic/worlds/mavic_world.wbt', "r+")
        lines = f.readlines()
        f.seek(0)
        f.truncate()
        f.writelines(lines[0:cutoff])
        f.close()

        f2 = open('/home/arjun/SMART-LAB-ITAP-WEBOTS/install/webots_ros2_mavic/share/webots_ros2_mavic/worlds/mavic_world.wbt', "r+")
        lines = f2.readlines()
        f2.seek(0)
        f2.truncate()
        f2.writelines(lines[0:cutoff])
        f2.close()

        f = open('/home/arjun/SMART-LAB-ITAP-WEBOTS/webots_ros2_mavic/worlds/mavic_world.wbt', "a")
        f2 = open('/home/arjun/SMART-LAB-ITAP-WEBOTS/install/webots_ros2_mavic/share/webots_ros2_mavic/worlds/mavic_world.wbt', "a")

        for i in range(self.numPOIs):
            xCoord = self.poiAttributes[i][0]
            yCoord = self.poiAttributes[i][1]
            if i < self.numSafe:
                poiDeclaration = "\nSolidBox {\n  translation " + str(xCoord) + " " + str(yCoord) + " 0\n  name \"safe_" + str(i) + "\"\n  size 13 13 8\n  appearance PBRAppearance {\n    baseColor 0 1 0\n    roughness 0.5\n    metalness 0\n  }\n}"
                f.write(poiDeclaration)
                f2.write(poiDeclaration)
            else:
                poiDeclaration = "\nSolidBox {\n  translation " + str(xCoord) + " " + str(yCoord) + " 0\n  name \"threat_" + str(i - self.numThreats) + "\"\n  size 13 13 8\n  appearance PBRAppearance {\n    baseColor 1 0 0\n    roughness 0.5\n    metalness 0\n  }\n}"
                f.write(poiDeclaration)
                f2.write(poiDeclaration)
        
        f.close()
        f2.close()
    

    def setupHumans(self):
        #Store: x pos, y pos, z pos, cognitive ability (low 0 to pi/12, med pi/12 to pi/6, high pi/6 to pi/4), skill (low 0 to pi/12, med pi/12 to pi/6, high pi/6 to pi/4)
        self.humanAttributes = np.zeros((self.numHumans, 5)) 

        #Assign xyz location
        for i in range(self.numHumans):
            self.humanAttributes[i][0] = (2 * i + 1) - 10
            self.humanAttributes[i][1] = -1493
            self.humanAttributes[i][2] = 1.28

        #Assign cognitive ability and operator skill level
        for i in range(self.numHumans):
            self.humanAttributes[i][3] = round(random.uniform(0, 1 + math.pi / 4), 4)
            self.humanAttributes[i][4] = round(random.uniform(0, 1 + math.pi / 4), 4)

        #Write to sim
        f = open('/home/arjun/SMART-LAB-ITAP-WEBOTS/webots_ros2_mavic/worlds/mavic_world.wbt', "a")
        f2 = open('/home/arjun/SMART-LAB-ITAP-WEBOTS/install/webots_ros2_mavic/share/webots_ros2_mavic/worlds/mavic_world.wbt', "a")
        
        for i in range(self.numHumans):
            xCoord = self.humanAttributes[i][0]
            yCoord = self.humanAttributes[i][1]
            zCoord = self.humanAttributes[i][2]
            if i < self.numHumans:
                humanDeclaration = "\nPedestrian {\n  translation " + str(xCoord) + " " + str(yCoord) + " " + str(zCoord) + "\n  rotation 0 0 1 " + str(math.pi / 2) + "\n  name \"human_" + str(i) + "\"\n}"
                f.write(humanDeclaration)
                f2.write(humanDeclaration)
        
        f.close()
        f2.close()
    
    def setupRobots(self):
        #Store: x pos, y pos, z pos, slow (0)/fast (1), low quality (0)/high quality (1)
        self.robotAttributes = np.zeros((self.numRobots, 5)) 

        #Assign xyz location
        for i in range(self.numUAVs):
            self.robotAttributes[i][0] = -(3 * i + 1)
            self.robotAttributes[i][1] = -1490
            self.robotAttributes[i][2] = 0.1
        
        for i in range(self.numUGVs):
            self.robotAttributes[i + self.numUAVs][0] = (3 * i + 1)
            self.robotAttributes[i + self.numUAVs][1] = -1490
            self.robotAttributes[i + self.numUAVs][2] = 0

        #Assign vehicle speed
        self.robotAttributes[0:self.numUAVs, 3] = 1
        self.robotAttributes[self.numUAVs:, 3] = 0

        #Assign image quality
        self.robotAttributes[0:self.numUAVs, 4] = 0
        self.robotAttributes[self.numUAVs:, 4] = 1

        #Write to sim
        f = open('/home/arjun/SMART-LAB-ITAP-WEBOTS/webots_ros2_mavic/worlds/mavic_world.wbt', "a")
        f2 = open('/home/arjun/SMART-LAB-ITAP-WEBOTS/install/webots_ros2_mavic/share/webots_ros2_mavic/worlds/mavic_world.wbt', "a")
        
        for i in range(self.numRobots):
            xCoord = self.robotAttributes[i][0]
            yCoord = self.robotAttributes[i][1]
            zCoord = self.robotAttributes[i][2]
            if i < self.numUAVs:
                robotDeclaration = "\nMavic2Pro {\n  translation " + str(xCoord) + " " + str(yCoord) + " " + str(zCoord) + "\n  rotation 0 0 1 " + str(math.pi / 2) + "\n  name \"Mavic_2_PRO_" + str(i) + "\"\n  controller \"<extern>\"\n  cameraSlot [\n    Camera {\n      width 400\n      height 240\n      near 0.2\n    }\n  ]\n}"
            else:
                robotDeclaration = "\nTurtleBot3Burger {\n  translation " + str(xCoord) + " " + str(yCoord) + " " + str(zCoord) + "\n  rotation 0 0 1 " + str(math.pi / 2) + "\n  name \"TurtleBot3Burger_" + str(i - self.numUAVs) + "\"\n  controller \"<extern>\"\n  controllerArgs [\n    \"\"\n  ]\n  extensionSlot [\n    Solid {\n      name \"imu_link\"\n    }\n    GPS {\n    }\n    InertialUnit {\n      name \"inertial_unit\"\n    }\n    RobotisLds01 {\n    }\n  ]\n}"
            
            f.write(robotDeclaration)
            f2.write(robotDeclaration)

        f.close()
        f2.close()
    
    def getNumSafe(self):
        return self.numSafe
    
    def getNumThreats(self):
        return self.numThreats
        
    def getNumHumans(self):
        return self.numHumans
    
    def getNumUAVs(self):
        return self.numUAVs

    def getNumUGVs(self):
        return self.numUGVs
    
    def getPOIs(self):
        poiList = []

        for i in range(self.numPOIs):
            poiList.append((self.poiAttributes[i][0], self.poiAttributes[i][1], self.poiAttributes[i][2]))
        
        return poiList