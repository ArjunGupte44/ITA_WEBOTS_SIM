#!/usr/bin/env python3
import sys
sys.path.insert(0, '/home/arjun/SMART-LAB-ITAP-WEBOTS/webots_ros2_mavic/src')
from WebotsEnv import WebotsEnv
import numpy as np
from sklearn.cluster import KMeans
import math as m

UAV_COORDS_FILE = "/home/arjun/SMART-LAB-ITAP-WEBOTS/install/webots_ros2_mavic/share/webots_ros2_mavic/resource/uavCoords.txt"
UGV_COORDS_FILE = "/home/arjun/SMART-LAB-ITAP-WEBOTS/install/webots_ros2_mavic/share/webots_ros2_mavic/resource/ugvCoords.txt"
SIM_AGENT_NUM_FILE = '/home/arjun/SMART-LAB-ITAP-WEBOTS/webots_ros2_mavic/resource/numSimAgents'
HUMAN_ATTRIBUTES_FILE = '/home/arjun/SMART-LAB-ITAP-WEBOTS/webots_ros2_mavic/resource/humanAttributes'
POI_ATTRIBUTES_FILE = '/home/arjun/SMART-LAB-ITAP-WEBOTS/webots_ros2_mavic/resource/poiAttributes'
POIS_FILE = '/home/arjun/SMART-LAB-ITAP-WEBOTS/webots_ros2_mavic/resource/pois'


def main():
    def findClosestPoint(centroid, pois, clusterIndexes, targetIndex):
        minDistance = sys.maxsize
        for i, poi in enumerate(pois):
            if clusterIndexes[i] == targetIndex:
                distance = m.sqrt((centroid[0] - poi[0]) ** 2 + (centroid[1] - poi[1]) ** 2)
                if distance < minDistance:
                    minDistance = distance
                    closestPoi = poi
        return closestPoi


    def performKMeans(numRobots, pois):
        kMeans = KMeans(n_clusters = numRobots)
        clusterIndexes = kMeans.fit_predict(pois)
        clustersMatrix = [] #Each element is a list corresponding to a single cluster - the inner lists contain tuples of the xyz coordinates of pois in that cluster
        poisInCluster = []
        minDist = sys.maxsize
        clustersDict = {}
        centroids = []
        
        for i in range(len(clusterIndexes)):
            #If a member of the ith cluster has not been entered into the dict yet, check whether to append the centroid or the normal current point
            if clusterIndexes[i] not in clustersDict.keys():
                centroid = list(kMeans.cluster_centers_[clusterIndexes[i]])
                if centroid in pois:
                    clustersDict[clusterIndexes[i]] = [centroid]
                else:
                    closestPoint = findClosestPoint(centroid, pois, clusterIndexes, clusterIndexes[i])
                    clustersDict[clusterIndexes[i]] = [closestPoint]
                
                centroids.append(clustersDict[clusterIndexes[i]][0])

            if pois[i] not in centroids:
                clustersDict[clusterIndexes[i]].append(pois[i])
        
        
        #For each key in the dict, sort the values list based on their closeness to the centroid
        sum = 0
        for cluster in clustersDict.keys():
            poiList = clustersDict[cluster]
            sum += len(poiList)
            if len(poiList) > 0:
                poisSorted = sorted(poiList, key=lambda d: m.dist(d, poiList[0]))
                clustersMatrix.append(poisSorted)

        count = 0
        for r in clustersMatrix:
            for point in r:
                if point not in pois:
                    count += 1

        return clustersMatrix

    def getStartingCoords(itapSim):
        uavStartingCoords = []
        ugvStartingCoords = []

        #Initialize UAV coords first
        robotAttributes = itapSim.getRobotAttributes()
        for i in range(itapSim.getNumUAVs()):
            uavStartingCoords.append([robotAttributes[i][0], robotAttributes[i][1], robotAttributes[i][2]])
        
        #Initialize UGV coords first
        for i in range(itapSim.getNumUGVs()):
            ugvStartingCoords.append([robotAttributes[i + itapSim.getNumUAVs()][0], robotAttributes[i + itapSim.getNumUAVs()][1], robotAttributes[i + itapSim.getNumUAVs()][2]])

        return uavStartingCoords, ugvStartingCoords
    

    def writeWaypoints(clustersMatrix, numUGVs, numUAVs, uavHeight, uavStartingCoords, ugvStartingCoords):
        f = open(UAV_COORDS_FILE, 'w')
        line = ""

        for i, row in enumerate(clustersMatrix[0: numUAVs]):
            #Add all normal coords to visit
            for poi in row:
                line += str(poi[0]) + " " + str(poi[1]) + " " + str(uavHeight + 0.5 * i) + " 0.0, "

            #Add the starting coord at very end to return to home
            line += str(uavStartingCoords[i][0]) + " " + str(uavStartingCoords[i][1]) + " " + str(uavHeight + 0.5 * i) + " 0.0, "

            line = line[:len(line) - 2] #delete ,SPACE at the end of the line
            f.write(line + "\n")
            line = ""
        f.close()

        f = open(UGV_COORDS_FILE, 'w')
        for i, row in enumerate(clustersMatrix[numUAVs:]):
            #Add all normal coords to visit
            for poi in row:
                line += str(poi[0]) + " " + str(poi[1]) + " " + str(poi[2]) + ", "
        
            #Add the starting coord at very end to return to home
            line += str(ugvStartingCoords[i][0]) + " " + str(ugvStartingCoords[i][1]) + " " + str(3.0) + ", "
            line = line[:len(line) - 2] #delete ,SPACE at the end of the line
            f.write(line + "\n")
            line = ""
        f.close()        
    
    def writeToFile(array, file, mode='a'):
        if mode == 'a':
            f = open(file, '+w')
            for val in array:
                f.write(str(val) + "\n")
            f.close()
        elif mode == 'p':
            f = open(file, 'r')
            data = f.readlines()
            f.close()

            f = open(file, 'w')
            f.seek(0)
            
            for i, line in enumerate(data):
                if i >= 4 and (i + 1) % 5 == 0:
                    data[i] = str(array[i]) + "\n"
            
            f.writelines(data)
            f.close()


    numSafe = 25
    numThreats = 25  
    numHumans = 12
    numUAVs = 4
    numUGVs = 4
    numRobots = numUAVs + numUGVs
    uavHeight = 7

    writeToFile([numSafe, numThreats, numHumans, numUAVs, numUGVs], SIM_AGENT_NUM_FILE)

    itapSim = WebotsEnv(numSafe, numThreats, numHumans, numUAVs, numUGVs)
    pois = itapSim.getPOIs()
    poisFlattened = list(np.concatenate(pois).flat)
    writeToFile(poisFlattened, POIS_FILE)
    
    clustersMatrix = performKMeans(numRobots, pois) 
    uavStartingCoords, ugvStartingCoords = getStartingCoords(itapSim)
    writeWaypoints(clustersMatrix, numUGVs, numUAVs, uavHeight, uavStartingCoords, ugvStartingCoords)
    
    humanAttributes = itapSim.getHumanAttributes()
    humanAttrFlattened = list(np.concatenate(humanAttributes).flat)
    writeToFile(humanAttrFlattened, HUMAN_ATTRIBUTES_FILE)

    poiAttributes = itapSim.getPOIAttributes()
    poiAttrFlattened = list(np.concatenate(poiAttributes).flat)
    writeToFile(poiAttrFlattened, POI_ATTRIBUTES_FILE)

    return clustersMatrix


if __name__ == '__main__':
    main()