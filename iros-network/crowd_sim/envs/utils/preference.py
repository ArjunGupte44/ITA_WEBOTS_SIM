import numpy as np
import torch
import torch.nn as nn
class preference (nn.Module):
    def __init__(self,config) :
        super(preference,self).__init__()
        #self.high = 
        # 起始时间，当前时间
        # 前5min的工作饱和度
        # 图片质量(根据机器人性质决定)，分类难度(随机指标)
    def h_c_s (self,level):
        # define human’s cognitive ability or operational skill, 
        # both of them are randomly simpled from level.
        if level == 0:
            h_ = np.random.uniform(0,1/12) 
        elif level == 1:
            h_ = np.random.uniform(1/12,1/6)
        elif level == 2:
            h_ = np.random.uniform(1/6,1/4)
        h = np.sin(h_ * np.pi)
        return h
    
    def function_f (self,t):
        #t: work_time,[0-8]hours
        if 0 <= t < 1:
            value = 1
        elif 1 <= t <= 8:
            value = 1.12 - 0.12 * np.power(t,2)

        return value
    
    def function_w (self,u):
        # u: utilization, [0-1]%
        # u denotes a human agent’s utilization, 
        # which is the percentage of time that an operator is busy performing tasks,
        # measured over atrailing 5-minute period.
        if 0 <= u  < 0.45:
            value = 0.5 + 2.22*u - 2.47*np.power(u,2)
        elif 0.45 <= u < 0.65:
            value = 1
        elif 0.65 <= u <= 1:
            value = -0.724 + 5.31*u - 4.08*np.power(u,2)
        else:
            raise ValueError("Utilization Error! ")
        return value
    
    def function_s (self,img_q , cls_diff):
        # img_q: image quality -> low = 0, high = 1
        # cls_diff: classification difficulty -> easy = 0, medium = 1, high = 2
        # t: indicator of task difficulty, time of work
        if img_q == 0 and cls_diff == 0:
            t = 20
        elif img_q == 0 and cls_diff == 1:
            t = 60
        elif img_q == 0 and cls_diff == 2:
            t = 180
        elif img_q == 1 and cls_diff == 0:
            t = 10
        elif img_q == 1 and cls_diff == 1:
            t = 30
        elif img_q == 1 and cls_diff == 2:
            t = 90
        else:
            raise ValueError('tuple(image_quality,classification_diffculty) Error! ')
        
        value = 1/(1 + np.exp(0.05*(t-150)))
        return value
    
    def p_rc (self,cognitive_ability,skill_level,s_value,w_value,f_value):
        # gamma and xi in (0,sqrt(2)/2)
        gamma = self.h_c_s(cognitive_ability)
        xi = self.h_c_s(skill_level)
        value = 0.5 + gamma*f_value*w_value*xi*s_value
        return value