#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from example_interfaces.msg import String
from std_msgs.msg import Float64MultiArray
from gazebo_msgs.msg import ModelStates
import sys
import numpy as np
import random
import re

class PoiManager(Node):
    def __init__(self, numThreats, numSafe):
        super().__init__("poiManager")
        print("IN POI MANAGER")
        self.publisher_ = self.create_publisher(Float64MultiArray, "poiInfo", 10)
        self.timer_ = self.create_timer(1.0/5.0, self.pubCallback)

        self.numThreats = int(numThreats)
        self.numSafe = int(numSafe)
        self.numPOIs = self.numThreats + self.numSafe

        #Store: x pos, y pos, z pos, threat (1)/safe (0), task difficulty (1 - easy to 3 - hard)
        self.poiAttributes = np.zeros((self.numPOIs, 5)) 
        self.initializeAttributes()

        self.writeToSim()

        self.modelLocations = {}
        self.debug = False

    def initializeAttributes(self):
        #Assign name and xyz location
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

    def writeToSim(self):
        f = open('/home/arjun/SMART-LAB-ITAP-WEBOTS/webots_ros2_mavic/worlds/mavic_world.wbt', "a")

        for i in range(self.numPOIs):
            xCoord = self.poiAttributes[i][0]
            yCoord = self.poiAttributes[i][1]
            if i < self.numSafe:
                poiDeclaration = "SolidBox {\n  translation " + str(xCoord) + " " + str(yCoord) + " 0\n  name \"safe_" + str(i) + "\"\n  size 13 13 13\n  appearance PBRAppearance {\n    baseColor 0 1 0\n    roughness 0.5\n    metalness 0\n  }\n}"
                f.write(poiDeclaration)
            else:
                poiDeclaration = "SolidBox {\n  translation " + str(xCoord) + " " + str(yCoord) + " 0\n  name \"threat_" + str(i) + "\"\n  size 13 13 13\n  appearance PBRAppearance {\n    baseColor 1 0 0\n    roughness 0.5\n    metalness 0\n  }\n}"
                f.write(poiDeclaration)
        
        f.close()

    def pubCallback(self):
        msg = Float64MultiArray()
        message = list(np.concatenate(self.poiAttributes).flat)
        message = [i for i in message]
        msg.data = message
        if self.debug:
            print("SENT:")
            print(msg)
        self.publisher_.publish(msg)


def main(argv):
    rclpy.init(args=argv)
    poiManager = PoiManager(argv[2], argv[4])
    rclpy.spin(poiManager)
    rclpy.shutdown()

if __name__ == "__main__":
    if (len(sys.argv) > 1):
        main(sys.argv)
    else:
        main()