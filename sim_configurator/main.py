#!/usr/bin/env python
from WebotsEnv import WebotsEnv
import numpy as np
from sklearn.cluster import KMeans
import math as m
import sys

def performKMeans(numRobots, pois):
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

    return clustersMatrix

def RL():
    pass

numSafe = 25
numThreats = 25
numHumans = 10
numUAVs = 10
numUGVs = 3
numRobots = numUAVs + numUGVs

itapSim = WebotsEnv(numSafe, numThreats, numHumans, numUAVs, numUGVs)
pois = itapSim.getPOIs()
clustersMatrix = performKMeans(numRobots, pois)
#print(clustersMatrix)

#robotPoiAssignments, humanPoiAssignments = RL()

