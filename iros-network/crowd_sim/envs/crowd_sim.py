import logging
import gym
import numpy as np
#import rvo2
#import random
#import copy
from crowd_sim.envs.utils.poi import POI
from numpy.linalg import norm
from crowd_sim.envs.utils.human import Human
from crowd_sim.envs.utils.robot import Robot
from crowd_sim.envs.utils.info import *
#from crowd_nav.policy.orca import ORCA
from crowd_sim.envs.utils.state import *
from crowd_sim.envs.utils.action import ActionRot, ActionXY,Actiondz
from crowd_sim.envs.utils.recorder import Recoder
from crowd_nav.policy.policy_factory import policy_factory



class CrowdSim(gym.Env):
    """
    A base environment
    treat it as an abstract class, all other environments inherit from this one
    """
    def __init__(self):
        """
        Movement simulation for n agents
        Agent can either be UAV or UGV.
        humans are controlled by a unknown and fixed policy.
        robot is controlled by a known and learnable policy.
        """
        self.time_limit = None
        self.time_step = None
        self.robot = None # a Robot instance representing the robot
        self.humans = None # a list of Human instances, representing all humans in the environment
        self.global_time = None
        self.step_counter=0

        # reward function
        self.success_reward = None
        self.collision_penalty = None
        self.discomfort_dist = None
        self.discomfort_penalty_factor = None
        # simulation configuration
        self.config = None
        self.case_capacity = None
        self.case_size = None
        self.case_counter = None
        self.randomize_attributes = None

        #self.circle_radius = None
        self.human_num = None


        self.action_space=None
        self.observation_space=None

        #seed
        self.thisSeed=None # the seed will be set when the env is created

        #nenv
        self.nenv=None # the number of env will be set when the env is created.

        self.phase=None # set the phase to be train, val or test
        self.test_case=None # the test case ID, which will be used to calculate a seed to generate a human crossing case

        # for render
        self.render_axis=None

        self.humans = []

        self.potential = None

        ##########################################
        self.all_poi_is_done = False
        self.poi_list = None
        self.poi_attr = None
        self.poi_workflow = None
        self.robot_num = None
        self.robots = []
        self.photo_if = False
        self.poi_imgs = []
        self.human_workflow = None
        self.sleep_man = 0 
        self.img_copy = []

    def configure(self, config):
        """ read the config to the environment variables """

        self.config = config
        self.POI = POI(config)
        self.time_limit = config.env.time_limit
        self.time_step = config.env.time_step

        self.case_capacity = {'train': np.iinfo(np.uint32).max - 2000, 'val': 1000, 'test': 1000}
        self.case_size = {'train': np.iinfo(np.uint32).max - 2000, 'val': self.config.env.val_size,
                          'test': self.config.env.test_size}
        #self.circle_radius = config.sim.circle_radius
        self.human_num = config.sim.human_num
        self.robot_num = config.robot.num
        #self.arena_size = config.sim.arena_size

        self.case_counter = {'train': 0, 'test': 0, 'val': 0}

        self.last_human_states = np.zeros((self.human_num, 4))
        self.last_robot_states = np.zeros((self.robot_num, 6))


        # set robot for this envs
        #rob_RL = Robot(config, 'robot')
        self.set_robot(self.robots)
        ##########################################
        #self.robot_num = config.robot.num
        
        self.policy = policy_factory[self.config.robot.policy](self.config)
        self.human_workflow = np.random.choice(50,50,replace=False).reshape(self.human_num,int(self.config.POI.number/self.human_num))


    def generate_poi(self):
        self.poi_list,self.poi_attr = self.POI.poi_set()
        self.poi_workflow = self.POI.neighbor()
        #print(self.poi_workflow)
        self.threat_list,self.non_threat_list = self.POI.threat()
        #self.human_workflow = np.random.choice(50,50,replace=False).reshape(self.human_num,int(self.config.POI.number/self.human_num))
        for h,human in enumerate(self.humans):
            human.human_poi_list = self.human_workflow[h]

    def set_robot(self, robot):#添加机器人信息
        #self.robot = robot

        # we set the max and min of action/observation space as inf
        # clip the action and observation as you need

        d={}
        # robot node: px, py, gx, gy, v, photo_if
        d['robot_node'] = gym.spaces.Box(low=-np.inf, high=np.inf, shape=(self.robot_num,6,), dtype = np.float32)
        d['task_attribute'] = gym.spaces.Box(low=-np.inf, high=np.inf, shape=(self.config.POI.number,3,), dtype = np.float32)
        d['human_factor'] = gym.spaces.Box(low=-np.inf, high=np.inf, shape=(self.human_num,4,), dtype = np.float32)
        self.observation_space = gym.spaces.Dict(d)

        high = np.inf * np.ones([2, ])
        self.action_space = gym.spaces.Box(low=-np.inf, high=np.inf, shape=(self.robot_num,6,), dtype = np.float32)

    def generate_human_list(self, human_num): # DONE
        """
        Calls generate_circle_crossing_human function to generate a certain number of random humans
        :param human_num: the total number of humans to be generated
        :return: None
        """
        # initial min separation distance to avoid danger penalty at beginning
        for i in range(human_num):
            self.humans.append(self.generate_human())

    def generate_human(self):#Done
        cognitive_ability = np.random.randint(0,3,1) # level: 0,1,2 -> low,medium,high
        skill_level = np.random.randint(0,3,1)       # level: 0,1,2 -> low,medium,high
        human = Human(self.config, 'humans')
        human.set(cognitive_ability,skill_level,self.global_time)#要加传入参数,认知能力，技能水平
        human.max_poi_num = int(self.config.POI.number/self.config.sim.human_num)
        return human

    def generate_robot_list(self,robot_num):#Done
        UAV_num = np.random.randint(0,robot_num-1,1)
        UGV_num = robot_num - UAV_num
        for i in range(robot_num):
            if i <= UAV_num:
                self.robots.append(self.generate_robot('UAV'))
            else:
                self.robots.append(self.generate_robot('UGV'))

    def generate_robot(self,section):#Done
        robot = Robot(self.config,section)
        #robot.set(self.orig, self.poi_workflow[0], photo_if = self.photo_if)
        robot.set()
        return robot 
    
    def generate_robot_humans(self, robot_num = None, human_num=None):#Done
        # generate robots
        self.generate_robot_list(robot_num = robot_num)
        # generate humans
        self.generate_human_list(human_num = human_num)
    ### generate n human and m robots in env
    
    def update_last_human_states(self,reset):# Done
        # human_state ：cognitive ability and operational skill level,time,if work
        for i in range(self.human_num):
            if reset:
                cognitive_ability = np.random.randint(0,3,1)
                skill_level = np.random.randint(0,3,1)
                time = np.zeros(1)
                if_work = np.zeros(1)
                humanS = np.array([cognitive_ability,skill_level,time,if_work]).reshape(1,4)
                self.last_human_states[i, :] = humanS

            else:
                cognitive_ability,skill_level,time,if_work = self.last_human_states[i, :]
                time = time + self.time_step
                if_work = self.humans[i].if_work
                self.last_human_states[i, :] = np.array([cognitive_ability,skill_level,time,if_work])

    def update_last_robot_states(self,reset):# Done
        # human_state ：cognitive ability and operational skill level,time,if work
        for i,robot in enumerate(self.robots):
            if reset:
                #px, py, gx, gy, v, photo_if = robot.px,robot.py,self.poi_list[self.poi_workflow[i][0]][0],self.poi_list[self.poi_workflow[i][0]][1],robot.v_pref,False
                #px, py, gx, gy, v, photo_if = robot.px,robot.py,0,0,robot.v_pref,False
                if self.poi_workflow[i][0] == 50:
                    px, py, gx, gy, v, photo_if = robot.px,robot.py,0,0,robot.v_pref,False
                else:
                    px, py, gx, gy, v, photo_if = robot.px,robot.py,self.poi_list[self.poi_workflow[i][0]][0],self.poi_list[self.poi_workflow[i][0]][1],robot.v_pref,False
                robotS = np.array([px, py, gx, gy, v, photo_if]).reshape(1,6)
                self.last_robot_states[i, :] = robotS

            else:
                px, py, gx, gy, v, photo_if = self.last_robot_states[i, :]
                photo_if = self.robots[i].photo_if
                self.last_robot_states[i, :] = np.array([px, py, gx, gy, v, photo_if])


    # return the ground truth locations of all humans
    def get_true_human_states(self):#Done,用这个检查human是否处于工作状态
        #检查human是否处于working
        true_human_states = np.zeros((self.human_num, 1))
        for i in range(self.human_num):
            humanS = np.array(self.humans[i].get_observable_state_list())
            true_human_states[i, :] = humanS[-1]
        return true_human_states

    def task_allocate(self):

        for h,human in enumerate(self.humans):
            #human.human_poi_list = self.human_workflow[h]
            #print('human_poi_list  ',human.human_poi_list)
            for i,p in enumerate(self.poi_imgs):
                if p[-1] in human.human_poi_list and human.if_work == False:
                    human.get_img(p)
                    human.human_poi_list = np.delete(human.human_poi_list,np.where(human.human_poi_list == p[-1]))
                    self.poi_imgs.remove(p)
                else:
                    continue
            #print('human_poi_list  ',human.human_poi_list)

    def reset(self, phase='train', test_case=None):# Done
        """
        Reset the environment
        :return:
        """

        if self.phase is not None:
            phase = self.phase
        if self.test_case is not None:
            test_case=self.test_case

        '''if self.robots is None:
            raise AttributeError('robots has to be set!')'''
        assert phase in ['train', 'val', 'test']
        if test_case is not None:
            self.case_counter[phase] = test_case # test case is passed in to calculate specific seed to generate case
        self.global_time = 0
        self.step_counter=0
        self.sleep_man = 0
        self.poi_imgs = []
        self.all_poi_is_done = False
        self.humans = []
        self.robots = []
        self.img_copy = []
        counter_offset = {'train': self.case_capacity['val'] + self.case_capacity['test'],
                          'val': 0, 'test': self.case_capacity['val']}

        np.random.seed(counter_offset[phase] + self.case_counter[phase] + self.thisSeed)

        self.generate_robot_humans(robot_num=self.robot_num , human_num= self.human_num)
        self.generate_poi()

        for i,agent in enumerate(self.robots):
            agent.time_step = self.time_step
            #agent.policy.time_step = self.time_step
            agent.poi_list = self.poi_workflow[i]
            agent.max_poi_num = len(self.poi_workflow[i])

        for human in self.humans:
            human.time_step = self.time_step

        # case size is used to make sure that the case_counter is always between 0 and case_size[phase]
        self.case_counter[phase] = (self.case_counter[phase] + int(1*self.nenv)) % self.case_size[phase]
        # get current observation
        ob = self.generate_ob(reset=True)
        # initialize potential
        #self.potential = -abs(np.linalg.norm(np.array([self.robot.px, self.robot.py]) - np.array([self.robot.gx, self.robot.gy])))#应该要改
        return ob


    def updata_robot_poi(self,robot,num_poi):#Done
        '''robot_copy = []
        for r in self.robots:
            if r != robot:
                robot_copy.append(r)

        now_poi_index = num_poi - 1'''
        robot.gx = self.poi_list[robot.poi_list[num_poi]][0]
        robot.gy = self.poi_list[robot.poi_list[num_poi]][1] 
        robot.photo_if = False
        robot.stop_time_step = 0
        return

    def updata_human_task(self,human):# Done
        human.if_work = False
        

    def last_human_states_obj(self):#Done
        '''
        convert self.last_human_states to a list of observable state objects for old algorithms to use
        '''
        humans = []
        for i in range(self.human_num):
            h = ObservableState(*self.last_human_states[i])
            humans.append(h)
        return humans
    def last_robot_states_obj(self):#Done
        '''
        convert self.last_human_states to a list of observable state objects for old algorithms to use
        '''
        robots = []
        for i in range(self.robot_num):
            h = self.last_robot_states[i]
            robots.append(h)
        return robots
    
    def last_poi_states_obj(self):#Done
        '''
        convert self.last_human_states to a list of observable state objects for old algorithms to use
        '''
        pois = []
        for i in range(self.config.POI.number):
            gx,gy = self.poi_list[i][0],self.poi_list[i][1]
            attr = self.poi_attr[i]
            pois.append((gx,gy,attr))
        poi_s = np.array(pois)
        return poi_s

    def calc_reward(self, actions):#Done
        #poi,poi_attr = POI.poi_set()
        done = False
        reward = 0
        episode_info = Nothing()
        reward_time = 0
        if self.global_time >= self.time_limit - 1:
            reward = 0
            done = True
            episode_info = Timeout()
        elif self.all_poi_is_done:
            print('done')
            reward = 0
            done = True
            episode_info = Over()
        else:
            for i, human_action in enumerate(actions):
                if human_action != None:
                    human_result_i,poi_attr = human_action
                    if human_result_i == 0 and poi_attr == 0:
                        reward = self.config.reward.success_reward_low
                        done = False
                        episode_info = Nothing()
                    elif human_result_i == 0 and poi_attr == 1 :
                        reward = self.config.reward.success_reward_medium
                        done = False
                        episode_info = Nothing()
                    elif human_result_i == 0 and poi_attr == 2 :
                        reward = self.config.reward.success_reward_high
                        done = False
                        episode_info = Nothing()
                    elif human_result_i == 1 and poi_attr == 0 :
                        reward = self.config.reward.penalty_reward_low
                        done = False
                        episode_info = Nothing()
                    elif human_result_i == 1 and poi_attr == 1 :
                        reward = self.config.reward.penalty_reward_medium
                        done = False
                        episode_info = Nothing()
                    elif human_result_i == 1 and poi_attr == 2 :
                        reward = self.config.reward.penalty_reward_high
                        done = False
                        episode_info = Nothing()
                    else:
                        reward = 0
                        done = False
                        episode_info = Nothing()
                reward_time += reward
        return reward_time, done, episode_info

    # compute the observation
    def generate_ob(self, reset):# Done
        #visible_human_states, num_visible_humans, human_visibility = self.get_num_human_in_fov()
        #num_visible_humans = self.human_num
        #human_visibility = [True,True,True,True,True].
        ob = {}
        self.update_last_human_states(reset= reset)
        self.update_last_robot_states(reset= reset)
        #self.update_last_poi_states(reset= reset)

        ob['human_factor']= self.last_human_states_obj()#(i,4)
        ob['task_attribute'] = self.last_poi_states_obj()#(n,3)
        ob['robot_node'] = self.last_robot_states_obj()#(r,6)
        return ob

    def get_human_actions(self):#Done
        # step all humans
        human_actions = []  # a list of all humans' actions

        for i, human in enumerate(self.humans):
            # observation for humans is always coordinates
            ob = human.get_observable_state()

            human_actions.append(human.human_act(ob))#根据human_num决定的list，内容为actionxy(vx,vy),应改为自己的human_act,内容为一组bool值的list

        return human_actions
    
    def get_robot_actions(self):#Done
        # step all humans
        robot_actions = []  # a list of all humans' actions

        for i, robot in enumerate(self.robots):
            # observation for humans is always coordinates
            #ob = robot.get_observable_state()
            if robot.num_poi + 1 < robot.max_poi_num:
                if robot.poi_list[robot.num_poi] == 50:
                    robot_actions.append(robot.act((0,0,3),self.poi_list[robot.poi_list[robot.num_poi+1]],robot.photo_if))
                else:
                    robot_actions.append(robot.act(self.poi_list[robot.poi_list[robot.num_poi]],self.poi_list[robot.poi_list[robot.num_poi+1]],robot.photo_if))
            else:
                if robot.sleep == False:
                    robot_actions.append(robot.return_origin(self.poi_list[robot.poi_list[robot.num_poi]]))
                else:
                    robot_actions.append(robot.robot_sleep())
                    #self.robots.remove(robot)
                    #self.robot_num = self.robot_num - 1 
                    #print(self.robots)
                
                #根据human_num决定的list，内容为actionxy(vx,vy),应改为自己的human_act,内容为一组bool值的list

        return robot_actions

    def step(self, action, update=True):#改robot_policy 后 Done
        """
        Compute actions for all agents, detect collision, update environment and return (ob, reward, done, info)
        """
        '''ppo的作用对象、多robot和多human均需要分别setp(),robot和human是否应放在同一观测空间'''
        # input_action = [n 个 robot的action]
        # clip the action to obey robot's constraint
        #robot_actions = self.robot.policy.clip_action(action, self.robot.v_pref)#之后看这个policy要调整吗
        # !!!!!! 这句重点考虑
        #UGV,UAV action = self.robot.xxx()
        #sleep_man = 0
        
        robot_actions = self.get_robot_actions()
        human_actions = self.get_human_actions()
        #workflow_action = self.policy.clip_action(self.human_workflow)

        if self.sleep_man == self.human_num:
            #print('it is time done')
            self.all_poi_is_done = True

        # compute reward and episode info
        reward, done, episode_info = self.calc_reward(human_actions) 
        #print(self.last_human_states_obj()[0][0])
        #if done: 
            #self.human_workflow = self.policy.clip_action(self.human_workflow,self.last_human_states_obj(),self.poi_list)
        #self.human_workflow = self.policy.clip_action(self.human_workflow,self.last_human_states_obj(),self.last_poi_states_obj())
        for i,robot_action in enumerate(robot_actions):
            self.robots[i].step(robot_action)
        # apply action and update all agents
        #self.robot.step(action)
        #for i, human_action in enumerate(human_actions):
            #self.humans[i].step(human_action)

        self.global_time += self.time_step # max episode length=time_limit/time_step
        self.step_counter=self.step_counter+1

        ##### compute_ob goes here!!!!!
        

        info={'info':episode_info}
        #info = episode_info
        '''if self.robot.policy.name in ['srnn']:
            #policy_name to atrl
            info={'info':episode_info}
        else: # for orca and sf
            info=episode_info'''

        for robot in self.robots:
            if norm((robot.gx-robot.px,robot.gy-robot.py)) <= 5 and (robot.gx,robot.gy) != (0,0):
                img = robot.take_photo()
                if robot.stop_time_step == 12:
                    #self.poi_imgs = list(filter(None,self.poi_imgs.append(img)))
                    self.poi_imgs.append(img)
                    self.img_copy.append(img)
                    robot.num_poi = robot.num_poi +1
                    self.updata_robot_poi(robot,robot.num_poi)
        #print('len: ',len(self.img_copy))
        self.task_allocate()
        # Update a specific robot's goal once its reached its original goal
        for i,human in enumerate(self.humans):
            human.global_time = self.global_time
            human.utilization()
            if human.if_work :
                human.keep_working()
                human.work_if_end()
            if human.if_work == False and human.worked_num == human.max_poi_num:
                self.sleep_man = self.sleep_man + 1
                #print('I am sleep' ,self.sleep_man)
                human.max_poi_num = 100
        #print()
        
        

        ob = self.generate_ob(reset=False)
        return ob, reward, done, info

    def render(self, mode='human'):#Done
        """ Render the current status of the environment using matplotlib """
        import matplotlib.pyplot as plt

        plt.rcParams['animation.ffmpeg_path'] = '/usr/bin/ffmpeg'

        robot_color = 'black'
        human_sleep_color = 'grey'
        human_work_color = 'blue'
        threat_color = 'red'
        non_threat_color = 'green'

        ax=self.render_axis
        artists=[]

        # add pois
        #print(self.poi_list)
        poi = [plt.Circle((poi[0],poi[1]), 3, fill=True) for poi in self.poi_list]
        #print('poi',len(poi))
        for i in range(len(self.poi_list)):
            if self.poi_list[i][2] in self.threat_list:
                poi[i].set_color(c=threat_color)
            else:
                poi[i].set_color(c=non_threat_color)
            ax.add_artist(poi[i])
            artists.append(poi[i])
            #plt.text(self.poi_list[i][0], self.poi_list[i][1], i, color='black', fontsize=12)

        # add robots
        robots = [plt.Circle((robot.px,robot.py), 5, fill=False , color = robot_color) for robot in self.robots]
        for i in range(len(self.robots)):
            ax.add_artist(robots[i])
            artists.append(robots[i])

        # add humans
        humans = [plt.Circle((0,(10+40*i)), 10, fill=True) for i,human in enumerate(self.humans)]
        for i in range(len(self.humans)):
            ax.add_artist(humans[i])
            artists.append(humans[i])

            if self.humans[i].if_work:
                humans[i].set_color(c = human_work_color)
            else:
                humans[i].set_color(c = human_sleep_color)

        plt.pause(0.1)
        for item in artists:
            item.remove() # there should be a better way to do this. For example,


