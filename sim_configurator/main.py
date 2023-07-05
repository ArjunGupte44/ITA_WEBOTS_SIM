#!/usr/bin/env python
from WebotsEnv import WebotsEnv
import numpy as np
from sklearn.cluster import KMeans


def performKMeans(numRobots, pois):
    kMeans = KMeans(n_clusters = numRobots)
    clusterIndexes = kMeans.fit_predict(pois)
    clustersMatrix = [] #Each element is a list corresponding to a single cluster - the inner lists contain tuples of the xyz coordinates of pois in that cluster
    poisInCluster = []

    for i in range(numRobots):
        #The first element of each list (row) in clustersMatrix is the centroid of that cluster
        clustersMatrix.append([tuple(kMeans.cluster_centers_[i])])

        #Keep appending pois to the correct list based on the number of the cluster it belongs to
        for j in range(len(clusterIndexes)):
            if clusterIndexes[j] == i and tuple(pois[j]) != clustersMatrix[i][0]:
                clustersMatrix[i].append(tuple(pois[j]))

    return clustersMatrix



numSafe = 25
numThreats = 25
numHumans = 10
numUAVs = 7
numUGVs = 3
numRobots = numUAVs + numUGVs

itapSim = WebotsEnv(numSafe, numThreats, numHumans, numUAVs, numUGVs)
pois = itapSim.getPOIs()
clustersMatrix = performKMeans(numRobots, pois)

