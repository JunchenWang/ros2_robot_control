import matplotlib.pyplot as plt
import numpy as np

data = np.loadtxt('data.txt')
t = data[:,0]
x = data[:,1]
y = data[:,2]
vx = data[:,3]
vy = data[:,4]
ax = data[:,5]
ay = data[:,6]
plt.plot(t,x,t,y,t,vx,t,vy)
plt.show()