import matplotlib.pyplot as plt
import numpy as np

subExp = [0.0812, 0.455, 0.9656, 1.2234234, 1.525465, 1.877754, 1.9946]
coA = [0.091, 0.5552, 1.212, 2.55651, 2.75555, 3.265465, 3.65464654]
x = np.linspace(1, 7, 7)
print(x)
y = np.array(subExp)

# first plot with X and Y data
plt.plot(x, y, color='blue')
plt.scatter(x, y, marker='v', color='blue')
y1 = np.array(coA)

# second plot with x1 and y1 data
plt.plot(x, y1, color='red')
plt.scatter(x, y1, marker='v', color='red')

plt.xlabel("Number of robots")
plt.ylabel("Solution Time(sec)")
plt.title('Coupled A* vs M* convergence time plot')
plt.legend(["M*", "Coupled-A*"])
plt.grid()
plt.show()
