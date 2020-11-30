import serial
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import time

s = serial.Serial("/dev/tty.usbmodem2050316A41501")

buffer_len = 1000
sampling_rate = 48000000/65336

def init():
    return

def update(frame):
	global ydata
	global fdata

	timestamps.append((time.time(), len(ydata)))
	num_bytes_available = int(np.floor(s.in_waiting/2.0)*2)
	points_8bit = s.read(num_bytes_available)
	points_16bit = [points_8bit[i+1] * 256 + points_8bit[i] for i in range(0,len(points_8bit), 2)]
	points_16bit = [x & 0x0FFF for x in points_16bit]
	[ydata.append(x) for x in points_16bit]

	volts = np.array(ydata) * 3.3 / np.power(2,12)
	volts = volts[-buffer_len:]

	ax.set_ylim(np.min(volts), np.max(volts))
	ax.set_xlim(0, len(volts))
	ln.set_data(range(len(volts)), volts)
	return

timestamps = []
fig, ax = plt.subplots(1,1)
ydata = []
ln, = ax.plot([], [], 'r-')

ax.set_xlabel("Sample")
ax.set_ylabel("Voltage / V")

ani = FuncAnimation(fig, update, init_func=init)
plt.show()

if(len(ydata) > 0 and len(timestamps) > 0):
	save_data = {"ydata": ydata, "timestamps":timestamps}
	np.save("logger_data_{0}.npy".format(str(timestamps[0][0]).replace(".", "-")), save_data)