from numpy import *
from matplotlib import pyplot as plt

data = loadtxt("build/timingResolutions.dat",delimiter=",")
plt.plot(data[:,0],data[:,3])
plt.ylabel("Search Time QuadTree (ns)")
plt.xlabel("Resolution")
plt.figure()
plt.plot(data[:,0],data[:,2])
plt.ylabel("Build Time QuadTree (ns)")
plt.xlabel("Resolution")
plt.show()