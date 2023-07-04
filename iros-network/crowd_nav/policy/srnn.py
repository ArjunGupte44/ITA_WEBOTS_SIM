
from crowd_nav.policy.policy import Policy
import numpy as np
from crowd_sim.envs.utils.action import ActionRot, ActionXY, Actiondz


class SRNN(Policy):
	def __init__(self, config):
		super().__init__(config)
		self.time_step = self.config.env.time_step # Todo: is this needed?
		self.name = 'srnn'
		self.trainable = True
		self.multiagent_training = True


	# clip the self.raw_action and return the clipped action
	def clip_action(self, actions, humans,pois):
		"""
        Input state is the joint state of robot concatenated by the observable state of other agents

        To predict the best action, agent samples actions and propagates one step to see how good the next state is
        thus the reward function is needed

        """
		# quantize the action
		print(pois)
		new_actions = actions[:]
		new_poi = pois.copy()
		#poi = np.array(pois)
		'''for i in range(len(actions)):
			for j in range(len(actions[i])):
				index = actions[i][j]
				if np.sqrt((pois[index][-1]-humans[i][0])**2 + (pois[index][-1]-humans[i][1])**2) <= 2:'''
		
		for j in range(len(humans)):
			human_j = []
			if j <= len(humans)-1 :
				for i in range(len(new_poi)):
					print(new_poi[i])
					print(humans[j][0])
					print(humans[j][1])
					if np.sqrt((new_poi[i][-1]-humans[j][0])**2 + (new_poi[i][-1] - humans[j][1])**2) <= 2:
						human_j.append(np.where(pois == new_poi[i]))
						print(np.where(pois == new_poi[i]))
						new_poi = np.delete(new_poi,i)
						if len(human_j) == len(actions[0]):
							break
			else:
				for i in range(len(new_poi)):
					human_j.append(np.where(pois == new_poi[i]))

			new_actions[j] = human_j
			

		return  new_actions



class atrl(SRNN):
	def __init__(self, config):
		super().__init__(config)
		self.name = 'selfAttn_merge_srnn'
