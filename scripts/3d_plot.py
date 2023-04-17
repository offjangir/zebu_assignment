
# importing mplot3d toolkits, numpy and matplotlib
from mpl_toolkits import mplot3d
import numpy as np
import matplotlib.pyplot as plt
import math
 
fig = plt.figure()
 
# syntax for 3-D projection
ax = plt.axes(projection ='3d')
r = 2
v = 1

# defining all 3 axis
t = 0
X = []
Y = []
Z = []
for i in range(1000):
    x = r * math.sin(math.pi*t)
    y = v*t
    z = 5 + r * math.cos(math.pi*t)
    X.append(x)
    Y.append(y)
    Z.append(z)
    t += 1/20
    

X = np.array(X)
Y = np.array(Y)
Z = np.array(Z)

 
# plotting
ax.plot3D(X, Y, Z, 'green')
ax.set_title('3D Sprial Plot for Drone')
plt.show()