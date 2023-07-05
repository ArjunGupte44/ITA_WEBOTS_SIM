import numpy as np
from numpy.linalg import norm
import abc
import logging
from crowd_nav.policy.policy_factory import policy_factory
from crowd_sim.envs.utils.action import ActionXY, ActionRot,Actiondz
from crowd_sim.envs.utils.state import ObservableState, FullState
from .poi import POI
# agent : UGV,UAV, don't contain human
class Agent(object):
    def __init__(self, config, section):
        """
        Base class for UAV and UGV. Have the physical attributes of an agent.

        """
        subconfig = config.UAV if section == 'UAV' else config.UGV
        self.agent = 'UAV' if section == 'UAV' else 'UGV'
        #self.visible = subconfig.visible
        self.v_pref = subconfig.v_pref
        #self.radius = subconfig.radius
        # randomize neighbor_dist of ORCA

        '''if config.env.randomize_attributes:
            config.orca.neighbor_dist = np.random.uniform(5, 10)'''#这是个什么玩意

        self.policy = policy_factory[config.robot.policy](config)
        #self.sensor = subconfig.sensor
        #self.FOV = np.pi * subconfig.FOV
        # for agents: we depend on config
        #self.kinematics = config.action_space.kinematics
        # px,py -> position (x,y)
        self.px = 0
        self.py = 0
        # gx,gy -> goal position (x,y)
        self.gx = None
        self.gy = None
        # vx,vy -> velocity (x,y)
        self.vx = None
        self.vy = None
        #self.poi = POI()

        self.poi_index = None

        self.theta = None
        self.time_step = config.env.time_step
        #self.policy.time_step = config.env.time_step
        self.robot_poi = 0
        self.max_poi_num = None
        self.robot_poi_list = None
        # get poi list from poi.py to each robot
        self.poi_list = None
        self.stop_time_step = 0
        self.photo_if = False
        self.sec = section
        self.num_poi = 0
        self.now_poi_index = None
        self.sleep = False
    def print_info(self):
        logging.info('Agent is {} and has {} kinematic constraint'.format(
            'visible' if self.visible else 'invisible', self.kinematics))


    def take_photo(self):#dz
        #获取拍摄图片信息(图片质量，分类难度)
        self.photo_if = True
        cls_diff = np.random.randint(0,3,1)
        if norm((self.gx-self.px,self.gy-self.py)) <= 5 :
            self.stop_time_step += 1

        if self.stop_time_step == 3/self.time_step:
            if self.sec == 'UAV':
                img = (0,cls_diff,self.poi_list[self.num_poi+1])
            elif self.sec == 'UGV':
                img = (1,cls_diff,self.poi_list[self.num_poi+1])
            #self.stop_time_step = 0
            return img

            
    
    def return_origin(self,now_poi):#dz
        #print(now_poi)
        delta_x = 0 - now_poi[0]
        delta_y = 0 - now_poi[1]
        angle = np.arctan2(delta_y,delta_x)
        #print(angle)
        self.vx = np.cos(angle) * self.v_pref
        self.vy = np.sin(angle) * self.v_pref
        self.gx = 0
        self.gy = 0
        if norm((self.gx-self.px,self.gy-self.py)) <= 5:
            self.sleep = True
            #print('This robot return origin')
        return Actiondz(self.vx,self.vy)
    
    def robot_sleep(self):
        if self.sleep:
            return Actiondz(0,0)

    def set(self):
        '''self.px = 0
        self.py = 0
        self.gx = 0 
        self.gy = 0'''
        #self.v_pref = self.subconfig.v_pref
        #self.photo_if = photo_if


    def act(self,now_poi,next_poi,photo_if = False):#dz
        #print(now_poi,next_poi)
        #print(self.poi_list)
        if photo_if:
            self.vx = 0
            self.vy = 0
            self.photo_if = photo_if
        else:
            delta_x = next_poi[0] - now_poi[0]
            delta_y = next_poi[1] - now_poi[1]
            angle = np.arctan2(delta_y,delta_x)
            self.vx = np.cos(angle) * self.v_pref
            self.vy = np.sin(angle) * self.v_pref
            self.gx = next_poi[0]
            self.gy = next_poi[1]
            self.photo_if = photo_if
        #print(self.px, self.py, self.gx, self.gy, self.v_pref, self.photo_if)
        return Actiondz(self.vx,self.vy)

    # self.px, self.py, self.vx, self.vy, self.radius, self.gx, self.gy, self.v_pref, self.theta
    # px, py, gx, gy, v, if_photograph

    #def get_observable_state(self):#应重写
        #return ObservableState(self.px, self.py, self.gx, self.gy, self.v_pref, self.photo_if)

    def get_observable_state_list(self):
        return [self.px, self.py, self.gx, self.gy, self.v_pref, self.photo_if]

        # return [self.px, self.py, self.radius, self.gx, self.gy, self.v_pref]

    def get_position(self):
        return self.px, self.py

    def set_position(self, position):
        self.px = position[0]
        self.py = position[1]

    def get_goal_position(self):
        return self.gx, self.gy

    def get_velocity(self):
        return self.vx, self.vy


    def set_velocity(self, velocity):
        self.vx = velocity[0]
        self.vy = velocity[1]


    '''@abc.abstractmethod
    def act(self, ob):
        """
        Compute state using received observation and pass it to policy

        """
        return'''


    def step(self, action):
        """
        Perform an action and update the state
        """
        
        self.px = self.px + action.vx * self.time_step
        self.py = self.py + action.vy * self.time_step
        self.vx = action.vx
        self.vy = action.vy



