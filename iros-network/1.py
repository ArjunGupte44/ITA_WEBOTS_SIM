import numpy as np
import torch
import torch.nn as nn
from numpy.linalg import norm
'''a = torch.rand((3,10,62))
mp = nn.AvgPool1d(2,2)
e = mp(a)
print(e.size())
a1 = e.permute(0,2,1)
gru = nn.GRU(10,10)
b,c = gru(a1)
print(b.size(),c.size())'''

a = []
for i in range(3):
    
    for j in range(2):
        a[i][j] = i+j

print(a)