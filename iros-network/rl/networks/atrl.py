import torch
import sys
import torch.nn.functional as F
from torch import nn
from .srnn_model import *
from torch.nn import Parameter
from .transformer import TransformerEncoder

class Atrl (nn.Module):
    def __init__(self,obs_shape,args) :
        super(Atrl,self).__init__()
        #self.rnn = nn.RNN()
        self.args = args
        self.d = 10
        self.num_heads = 2
        self.layers = 5
        self.attn_dropout = 0.05
        self.relu_dropout = 0.1
        self.res_dropout = 0.1
        self.out_dropout = 0.0
        self.embed_dropout = 0.1
        self.attn_mask = True
        self.output_size = 2
        self.mp = nn.AvgPool1d(2,2)
        self.gru = nn.GRU(10,10)
        self.is_recurrent =True
        self.human_global = self.get_network(self_type='human',layers=5)
        self.robot_global = self.get_network(self_type='robot',layers=5)
        self.task_global = self.get_network(self_type= 'task',layers=5)

        self.trans_final = self.get_network(self_type='policy', layers=5)
        self.dim_linear = nn.Linear(10,1)
        self.cha_linear = nn.Linear(31,1)
        self.pol_linear = nn.Linear(10,2)
        self.policy_linear = nn.Linear(31,1)

    def get_network(self, self_type='l', layers=-1):
        if self_type in ['human','robot','task','policy']:
            embed_dim, attn_dropout = self.d, self.attn_dropout
        else:
            raise ValueError("Unknown network type")
        
        return TransformerEncoder(embed_dim=embed_dim,
                                  num_heads=self.num_heads,
                                  layers=max(self.layers, layers),
                                  attn_dropout=attn_dropout,
                                  relu_dropout=self.relu_dropout,
                                  res_dropout=self.res_dropout,
                                  embed_dropout=self.embed_dropout,
                                  attn_mask=self.attn_mask)
    def dim_rnn(self,inputs):
        input_dim = inputs.size()[-1]
        if self.args.cuda:
            rnn = nn.RNN(input_dim,self.d).cuda()
        else:
            rnn = nn.RNN(input_dim,self.d).cpu()
        outputs,_ = rnn(inputs)
        return outputs.reshape(inputs.size(0),inputs.size(1),self.d)
    
    def forward(self,input):
        humans = input['human_factor']#(b,i,4)
        tasks = input['task_attribute']#(b,n,2)
        robots = input['robot_node']#(b,j,6)

        humans = self.dim_rnn(humans).permute(1,0,2)#(i,b,d)
        tasks = self.dim_rnn(tasks).permute(1,0,2)#(n,b,d)
        robots = self.dim_rnn(robots).permute(1,0,2)#(j,b,d)

        global_context_l = torch.cat([humans,tasks,robots],dim = 0)#(c,b,d)

        human_global = self.human_global(humans,global_context_l,global_context_l)
        robot_global = self.robot_global(robots,global_context_l,global_context_l)
        task_global = self.task_global(tasks,global_context_l,global_context_l)

        global_context_h_ = torch.cat([human_global,task_global,robot_global],dim = 0).permute(1,2,0)#(b,d,c)

        global_context_h = self.mp(global_context_h_).permute(0,2,1)#(b,c/2,d)
        #print('global:',global_context_h.size())
        value_, policy_ = self.gru(global_context_h) #(b,c/2,d)  (1,c/2,d)
        #print('gru:  ',value_.size(),policy_.size())
        #torch.Size([b, 31, 10]) torch.Size([1, 31, 10])
        value = self.cha_linear(self.dim_linear(value_).permute(0,2,1))#(1,1,1)
        policy = self.policy_linear(self.pol_linear(value_).permute(0,2,1)).permute(0,2,1)#(1,j,2)
        #print('atrl:  ', value.size(),policy.size())
        return value.squeeze(1),policy.squeeze(1)
