from crowd_sim.envs.utils.agent import Agent
from crowd_sim.envs.utils.state import JointState
from crowd_sim.envs.utils.state import ObservableState
from .preference import preference
import numpy as np
class Human(object):
    # see Agent class in agent.py for details!!!
    def __init__(self, config, section):
        #super().__init__(config, section)
        self.isObstacle = False # whether the human is a static obstacle (part of wall) or a moving agent
        self.id = None # the human's ID, used for collecting traj data
        self.observed_id = -1 # if it is observed, give it a tracking ID
        self.human_preference = preference(config)
        self.cognitive_ability = None
        self.skill_level = None
        self.time = None
        self.if_work = False
        self.time_step = config.env.time_step
        self.work_time_step = 0
        self.work_end = False
        self.global_time = 0
        self.total_time = 300
        self.len_util = int(self.total_time / self.time_step)
        self.dyn_array = np.zeros(self.len_util)
        self.img = None
        self.uti = None
        self.human_poi_list = None
        self.worked_num = 0
        self.max_poi_num = 0
    # ob: (n个person,len(ob))

    
    def get_img(self,poi_img):
        self.img = poi_img
        self.if_work = True

    def keep_working(self):
        if self.if_work :
            self.work_time_step += 1 


    def work_if_end(self):
        #根据图片质量和分类难度决定工作时间
        img_q , cls_diff,poi_index = self.img
        if img_q == 0 and cls_diff == 0:
            t = 2
        elif img_q == 0 and cls_diff == 1:
            t = 6
        elif img_q == 0 and cls_diff == 2:
            t = 18
        elif img_q == 1 and cls_diff == 0:
            t = 1
        elif img_q == 1 and cls_diff == 1:
            t = 3
        elif img_q == 1 and cls_diff == 2:
            t = 9
        total_time_step = t / self.time_step

        if total_time_step == self.work_time_step:
            self.work_end = True
            self.work_time_step = 0
            self.if_work = False
            self.worked_num = self.worked_num + 1
            #print('worked_num:  ',self.worked_num)

    def utilization(self):
        #计算五分钟内的工作饱和度
        index = int((self.global_time/self.time_step) % self.len_util)
        #print('global: ', self.global_time, 'time_step: ', self.time_step, index)
        if self.if_work:
            self.dyn_array[index] = 1
        else :
            self.dyn_array[index] = 0

        if 0 < self.global_time <= 300 :
            len_util = int(self.global_time / self.time_step)
            dyn_array = self.dyn_array[:len_util]
        else:
            len_util = self.len_util
            dyn_array = self.dyn_array

        self.uti = np.sum(dyn_array) / len_util
        #print('uti')
        #print('uti',self.uti)
 
        return 

    def set(self,cognitive_ability,skill_level,global_time):
        #在下一个reset前应只被调用一次或一个loop，生成human_list后不应变化
        self.cognitive_ability = cognitive_ability
        self.skill_level = skill_level
        self.global_time = global_time

    def human_act(self,ob):
        # question: ob is a 2d array,but I need a 1d array to human
        result_ = np.array((0,1))

        if self.img :
            img_q , cls_diff,poi_index = self.img
            f_value = self.human_preference.function_f(self.global_time/3600)
            w_value = self.human_preference.function_w(self.uti)
            s_value = self.human_preference.function_s(img_q,cls_diff)
            human_prob = self.human_preference.p_rc(self.cognitive_ability,self.skill_level,s_value,w_value,f_value)#传入参数未定义
            prob = np.array((human_prob,(1-human_prob)))
            result = np.random.choice(a = result_, size= 1 ,replace= False, p= prob)
            return result,cls_diff
        else:
            return None
    
    def get_observable_state_list(self):
        return [self.cognitive_ability, self.skill_level, self.time, self.if_work]
    
    def get_observable_state(self):#应重写
        return ObservableState(self.cognitive_ability, self.skill_level, self.time, self.if_work)
