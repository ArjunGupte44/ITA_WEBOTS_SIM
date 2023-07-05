import numpy as np
from sklearn.cluster import KMeans
from crowd_sim.envs.utils.state import ObservableState, FullState

class POI():
    def __init__(self, config):
        #super().__init__(config)
        self.isObstacle = False # whether the human is a static obstacle (part of wall) or a moving agent
        self.id = None # the human's ID, used for collecting traj data
        self.observed_id = -1 # if it is observed, give it a tracking ID
        self.all = config.POI.number
        self.threat_num = int(self.all/2)
        self.non_threat_num = self.all - self.threat_num
        self.n = config.robot.num
        self.poi_list = None
        self.poi_attr = None
    # ob: a list of observable states

    def poi_set(self):
        # define poi list without threats 
        poi_set_list = []
        for i in range(self.all):
            x = int(np.random.randint(0,200,1))
            y = int(np.random.randint(0,200,1))
            poi_set_list.append((x,y,i))
        #print(poi_set_list)
        #poi_set_list.append((0,0,3))
        self.poi_list = np.array(poi_set_list)
        self.poi_attr = np.random.randint(0,3,50)
        return self.poi_list,self.poi_attr
    
    def k_mean(self):
        poi,attr = self.poi_set()
        kmeans = KMeans(n_clusters=self.n)
        kmeans.fit(poi)
        y_kmeans = kmeans.predict(poi)# clusters index
        #print('kkkkkk:  ', y_kmeans,poi)
        centers = kmeans.cluster_centers_# cluster centers
        return centers,y_kmeans
    
    def threat(self):
        threat_list = np.random.choice(50,self.threat_num,replace=False)
        non_threat_list = np.array([i for i in range(50) if i not in threat_list])
        return threat_list,non_threat_list
    
    def poi_index_list (self):
        centers,y_kmeans = self.k_mean()
        #print('kkkkk: ',y_kmeans)
        poi_index = []
        for i in range(self.n):
            poi_index.append(list(np.where(y_kmeans == i)[0]))
        return poi_index
    
    def poi_sort(self,poi,center,poi_set_list):
        dis = [self.all]
        new_poi = poi[:]
        new_center = center[:]
        for i in range(len(poi)):
            distance_ = []
            for j in range(len(new_poi)):
                distance = np.sqrt((poi_set_list[new_poi[j]][0]-new_center[0])**2 + (poi_set_list[new_poi[j]][1]-new_center[1])**2)
                distance_.append(distance)
            distance_i = np.array(distance_).argsort()[0]
            dis.append(new_poi[distance_i])
            new_center = poi_set_list[new_poi[distance_i]]
            new_poi.pop(distance_i)
        return dis

    def neighbor (self): 
        # Get the poi workflow in each cluster, and use the poi index to represent it
        centers,y_kmeans = self.k_mean()
        poi_index = self.poi_index_list()
        poi_set_list,attr = self.poi_set()
        poi_dis_index = []
        for i in range(len(poi_index)):
            poi_i = poi_index[i]
            center_i = centers[i]
            distance_ = []
            distance_i = self.poi_sort(poi_i,center_i,poi_set_list)#return sorted distances index of poi_i
            poi_dis_index.append(distance_i)
        #print(poi_dis_index)
        return poi_dis_index
    
    def get_observable_state(self):#应重写
        return ObservableState(self.poi_list,self.poi_attr)

    def get_observable_state_list(self):
        return [self.poi_list,self.poi_attr]