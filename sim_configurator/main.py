#!/usr/bin/env python
from WebotsEnv import WebotsEnv
import numpy as np
from sklearn.cluster import KMeans
import math as m
import sys

def performKMeans(numRobots, pois):
    for r in pois:
        for val in r:
            print(val)
    kMeans = KMeans(n_clusters = numRobots)
    clusterIndexes = kMeans.fit_predict(pois)
    clustersMatrix = [] #Each element is a list corresponding to a single cluster - the inner lists contain tuples of the xyz coordinates of pois in that cluster
    poisInCluster = []
    minDist = sys.maxsize
    clustersDict = {}

    for i in range(len(clusterIndexes)):
        #If a member of the ith cluster has not been entered into the dict yet, check whether to append the centroid or the normal current point
        if clusterIndexes[i] not in clustersDict.keys():
            centroid = tuple(kMeans.cluster_centers_[clusterIndexes[i]])
            if centroid in pois:
                clustersDict[clusterIndexes[i]] = [centroid]
            else:
                clustersDict[clusterIndexes[i]] = [pois[i]]
        else:
            clustersDict[clusterIndexes[i]].append(pois[i])
    
    #For each key in the dict, sort the values list based on their closeness to the centroid
    for cluster in clustersDict.keys():
        poiList = clustersDict[cluster]
        if len(poiList) > 0:
            poisSorted = sorted(poiList, key=lambda d: m.dist(d, kMeans.cluster_centers_[cluster]))
            clustersMatrix.append(poisSorted)

    print("\n")
    count = 0
    for r in clustersMatrix:
        for point in r:
            print(point)
            if point not in pois:
                count += 1

    
    print(count)
    return clustersMatrix

def RL():
    pass

def writeToFile(clustersMatrix, numUGVs, numUAVs, uavHeight):
    f = open("/home/arjun/SMART-LAB-ITAP-WEBOTS/install/webots_ros2_mavic/share/webots_ros2_mavic/resource/uavCoords.txt", 'w')
    line = ""

    for i, row in enumerate(clustersMatrix[0: numUAVs]):
        for poi in row:
            line += str(poi[0]) + " " + str(poi[1]) + " " + str(uavHeight + 0.5 * i) + " 0.0, "
    
        line = line[:len(line) - 2] #delete ,SPACE at the end of the line
        f.write(line + "\n")
        line = ""
    f.close()

    f = open("/home/arjun/SMART-LAB-ITAP-WEBOTS/install/webots_ros2_mavic/share/webots_ros2_mavic/resource/ugvCoords.txt", 'w')
    for row in clustersMatrix[numUAVs:]:
        for poi in row:
            line += str(poi[0]) + " " + str(poi[1]) + " " + str(poi[2]) + ", "
    
        line = line[:len(line) - 2] #delete ,SPACE at the end of the line
        f.write(line + "\n")
        line = ""
    f.close()        


numSafe = 2
numThreats = 1
numHumans = 10
numUAVs = 0
numUGVs = 1
numRobots = numUAVs + numUGVs
uavHeight = 19

itapSim = WebotsEnv(numSafe, numThreats, numHumans, numUAVs, numUGVs)
pois = itapSim.getPOIs()
clustersMatrix = performKMeans(numRobots, pois)
writeToFile(clustersMatrix, numUGVs, numUAVs, uavHeight)
print(clustersMatrix)

#robotPoiAssignments, humanPoiAssignments = RL()
#writeToFile(robotPoiAssignments, humanPoiAssignments)

