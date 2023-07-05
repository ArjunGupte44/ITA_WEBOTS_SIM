from crowd_sim.envs.utils.agent import Agent
from crowd_sim.envs.utils.state import JointState


class Robot(Agent):
    def __init__(self, config,section):
        super().__init__(config,section)
        #self.sensor_range = config.robot.sensor_range

    '''def act(self, ob):
        # 机器人act应为它下一步的目标点
        if self.policy is None:
            raise AttributeError('Policy attribute has to be set!')
        # 这个fullstate怎么定义
        state = JointState(self.get_full_state(), ob)
        action = self.policy.predict(state)
        return action

# 机器人工作状态判断： 如果机器人下一个time_step的(gx,gy) = next poi 则返回停留3s，3s结束时返回task信息

    def actWithJointState(self,ob):
        action = self.policy.predict(ob)
        return action'''
    
    