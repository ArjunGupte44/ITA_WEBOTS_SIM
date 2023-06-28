#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from example_interfaces.msg import String
from std_msgs.msg import Float64MultiArray
import sys
import numpy as np
import random
import math

class HumanManager(Node):
    def __init__(self, numHumans):
        super().__init__("humanManager")
        self.publisher_ = self.create_publisher(Float64MultiArray, "humanInfo", 10)
        self.timer_ = self.create_timer(1.0/5.0, self.pubCallback)
        self.numHumans = int(numHumans)

        #Store: x pos, y pos, z pos, cognitive ability (low 0 to pi/12, med pi/12 to pi/6, high pi/6 to pi/4), skill (low 0 to pi/12, med pi/12 to pi/6, high pi/6 to pi/4)
        self.humanAttributes = np.zeros((self.numHumans, 5)) 
        self.initializeAttributes()
        self.writeToSim()

    def initializeAttributes(self):
        #Assign xyz location
        for i in range(self.numHumans):
            self.humanAttributes[i][0] = 2 * i + 1
            self.humanAttributes[i][1] = -1510
            self.humanAttributes[i][2] = 1.37

        #Assign cognitive ability and operator skill level
        for i in range(self.numHumans):
            self.humanAttributes[i][3] = round(random.uniform(0, 1 + math.pi / 4), 4)
            self.humanAttributes[i][4] = round(random.uniform(0, 1 + math.pi / 4), 4)

    def writeToSim(self):
        f = open('/home/arjun/SMART-LAB-ITAP-WEBOTS/webots_ros2_mavic/worlds/mavic_world.wbt', "a")
        
        for i in range(self.numHumans):
            xCoord = self.humanAttributes[i][0]
            yCoord = self.humanAttributes[i][1]
            zCoord = self.humanAttributes[i][2]
            if i < self.numHumans:
                humanDeclaration = "\nPedestrian {\n  translation " + str(xCoord) + " " + str(yCoord) + " " + str(zCoord) + "\n  name \"human_" + str(i) + "\"\n}"
                f.write(humanDeclaration)
        
        f.close()

    def pubCallback(self):
        msg = Float64MultiArray()
        message = list(np.concatenate(self.humanAttributes).flat)
        message = [float(i) for i in message]
        msg.data = message
        self.publisher_.publish(msg)

def main(argv):
    rclpy.init(args=argv)
    humanManager = HumanManager(argv[2])
    rclpy.spin(humanManager)
    rclpy.shutdown()

if __name__ == "__main__":
    if (len(sys.argv) > 1):
        main(sys.argv)
    else:
        main()