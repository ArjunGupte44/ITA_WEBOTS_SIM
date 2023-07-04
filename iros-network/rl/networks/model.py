import torch
import torch.nn as nn


from rl.networks.distributions import Bernoulli, Categorical, DiagGaussian
from .srnn_model import SRNN
from .selfAttn_srnn_temp_node import selfAttn_merge_SRNN
from .atrl import Atrl
class Flatten(nn.Module):
    def forward(self, x):
        return x.view(x.size(0), -1)


class Policy(nn.Module):
    """ Class for a robot policy network """
    def __init__(self, obs_shape, action_space, base=None, base_kwargs=None):
        super(Policy, self).__init__()
        if base_kwargs is None:
            base_kwargs = {}

        if base == 'atrl':
            base = Atrl
        else:
            raise NotImplementedError

        self.base = base(obs_shape, base_kwargs)
        self.srnn = True
        #print(action_space.__class__.__name__)
        if action_space.__class__.__name__ == "Discrete":
            num_outputs = action_space.n
            self.dist = Categorical(self.base.output_size, num_outputs)
        elif action_space.__class__.__name__ == "Box":
            num_outputs = action_space.shape[0]

            self.dist = DiagGaussian(self.base.output_size, num_outputs)
        elif action_space.__class__.__name__ == "MultiBinary":
            num_outputs = action_space.shape[0]
            self.dist = Bernoulli(self.base.output_size, num_outputs)
        else:
            raise NotImplementedError

    @property
    def is_recurrent(self):
        return self.base.is_recurrent

    @property
    def recurrent_hidden_state_size(self):
        """Size of rnn_hx."""
        return self.base.recurrent_hidden_state_size

    def forward(self, inputs):
        raise NotImplementedError

    def act(self, inputs,  deterministic=False):
        #通过调整selfAttn_senn_temp_node.py进行调整
        value, actor_features = self.base(inputs)
        #print(value.size(),actor_features.size())
        dist = self.dist(actor_features)
        if deterministic:
            action = dist.mode()
        else:
            action = dist.sample()
        #print(action.size())
        action_log_probs = dist.log_probs(action)
        dist_entropy = dist.entropy().mean()
        #print(action_log_probs.size())
        return value, action, action_log_probs

    def get_value(self, inputs):

        value, _ = self.base(inputs)

        return value

    def evaluate_actions(self, inputs, action):
        value, actor_features = self.base(inputs,)

        dist = self.dist(actor_features)

        action_log_probs = dist.log_probs(action)
        dist_entropy = dist.entropy().mean()

        return value, action_log_probs, dist_entropy



