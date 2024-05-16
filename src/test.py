#!/usr/bin/env python3.8

import rospy
import numpy as np

list = np.linspace(1, 3, 20)
# print(list)

for i in range(1, 5):
    print(i)

# path = np.array(
#     [
#         [0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1],
#         [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
#     ]
# )
path = np.array([np.linspace(1, 3, 20), np.linspace(1, -2, 20)])
# path[0] = list
print(path)
