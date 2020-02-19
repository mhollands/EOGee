import serial
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

s = serial.Serial("/dev/tty.usbmodem2050316A41501")

fig, ax = plt.subplots()
ydata = []
ln, = plt.plot([], [], 'r-')

def init():
    ax.set_xlim(0, 2*np.pi)
    ax.set_ylim(-1, 1)
    return ln,

def update(frame):
	global ydata
	num_bytes_available = int(np.floor(s.in_waiting/2.0)*2)
	points_8bit = s.read(num_bytes_available)
	points_16bit = [points_8bit[i] * 256 + points_8bit[i+1] for i in range(0,len(points_8bit), 2)]
	[ydata.append(x) for x in points_16bit]
	if(len(ydata) > 1000):
		ydata = ydata[-1000:]
	ax.set_ylim(min(ydata), max(ydata))
	ax.set_xlim(0, len(ydata))
	ln.set_data(range(0,len(ydata)), ydata)
	return ln,

ani = FuncAnimation(fig, update, init_func=init)
plt.show()