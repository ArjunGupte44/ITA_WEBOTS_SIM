from math import atan, atan2, radians
import numpy as np


goal_pose = [-650, -1475]
__current_pose = [5, -1485, 0, 182]
__current_pose[3] -= 270
__current_pose[3] = radians(__current_pose[3])


angle = atan2(goal_pose[0] - __current_pose[0], goal_pose[1] - __current_pose[1])

# This is now in ]-2pi;2pi[
angle_left = angle - __current_pose[3]
# Normalize turn angle to ]-pi;pi]
angle_left = (angle_left + 2*np.pi) % (2*np.pi)
if (angle_left > np.pi):
    angle_left -= 2*np.pi


print(angle_left) #this is the error - how much to turn by (NOT the current/goal headings individually)