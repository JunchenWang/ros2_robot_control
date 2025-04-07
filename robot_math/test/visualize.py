import matplotlib.pyplot as plt
import numpy as np

data = np.loadtxt('data.txt')
t = data[:,0]
x = data[:,1]
y = data[:,2]
v = data[:,3]
a = data[:,4]
plt.plot(t, x)
# plt.axis('equal')
plt.show()