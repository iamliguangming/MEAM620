#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sun Jan 26 22:43:25 2020

@author: yupengli
"""
import matplotlib as mpl
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np

c = 0.25
mpl.rcParams['legend.fontsize'] =10
fig = plt.figure()  # an empty figure with no axes
ax = fig.gca(projection = '3d')
t = np.linspace(0,20,1000)
z = np.sin(t)
x = np.cos(t) + c*np.cos(10*t)
y = np.sin(t) + c*np.sin(10*t)

x1 = np.cos(t)
y1 = np.sin(t)
z1 = np.sin(t)

ax.plot(x, y, z, label='Point Position')
ax.plot(x1, y1, z1, label='Displacement of Robot')
ax.set_title('Location of Robot')

ax.legend()
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
plt.xlabel('X')
plt.ylabel('Y')
plt.zlabel('Z')
plt.show()