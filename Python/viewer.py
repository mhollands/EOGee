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
	point = s.read(s.in_waiting)
	[ydata.append(x) for x in point]
	ax.set_ylim(min(ydata), max(ydata))
	ax.set_xlim(0, len(ydata))
	ln.set_data(range(0,len(ydata)), ydata)
	return ln,

ani = FuncAnimation(fig, update, init_func=init)
plt.show()