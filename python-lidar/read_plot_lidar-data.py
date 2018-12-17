from serial import Serial
import numpy as np
import matplotlib.pyplot as plt
import pandas as pd

# Port to read serial data from and baudrate
ser = Serial('COM5', 9600)

# Size of dots in plot should vary with loop length
plotdot_tweak_size = 0.5
loop_length = 2000

x = []
y = []

for i in range(loop_length):
    try:
        xval, yval = ser.readline().decode('utf8', 'ignore').rstrip().split(",")
        x.append(float(xval))
        y.append(int(yval))
    except ValueError:
        print("error found, unless really spamming it keeps going")

plotX = np.array(x)
plotY = np.array(y)

df = pd.DataFrame(plotX)
df.to_csv('x1.csv', header=None, index=None)

df = pd.DataFrame(plotY)
df.to_csv('y1.csv', header=None, index=None)


plt.title('LIDAR point-cloud')
plt.xlabel('Angle')
plt.ylabel('Distance')

plt.scatter(plotX, plotY, c=plotY, s=plotdot_tweak_size, cmap='viridis')
plt.show()
