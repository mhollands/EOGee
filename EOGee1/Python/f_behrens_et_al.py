# This script streams data from EOGee1 and implements the saccade detection algorithm described in the paper
# "An improved algorithm for automatic detection of saccades in eye movement data and for calculating saccade parameters"
# by F. Behrens et al.

import serial
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import time
from typing import NamedTuple

def get_data(s):
	timestamp = None
	data = []
	if(type(s) == serial.Serial):
		timestamp = (time.time(), len(ydata))
		num_bytes_available = int(np.floor(s.in_waiting/2.0)*2)
		points_8bit = s.read(num_bytes_available)
		points_16bit = [points_8bit[i+1] * 256 + points_8bit[i] for i in range(0,len(points_8bit), 2)]
		data = points_16bit
	return [timestamp, data]

def update(frame, s):
	global ydata
	# Get data from source
	[timestamp, data] = get_data(s)

	# Save data
	timestamps.append(timestamp)
	[ydata.append(x) for x in data]

	N = 3.4
	acceleration_window = 200

	if(len(ydata) <= buffer_len + acceleration_window):
		print("Filling buffer {0}/{1}".format(len(ydata), buffer_len + acceleration_window))
		return

	# We want to end up with a certain number of time points
	final_y_num_samples = buffer_len
	# Each time we differentiate we get one point less
	final_dy_num_samples =  final_y_num_samples - 1
	final_d2y_num_samples = final_dy_num_samples - 1

	# We need enough data to show our buffer and also calculate NSigma over the acceleration window prior to the first point (but we include the first point in it's window so need one less point)
	processing_d2y_numsamples = final_d2y_num_samples + acceleration_window - 1
	# Differentiaton looses one point
	processing_dy_numsamples = processing_d2y_numsamples + 1
	# Differentiaton looses one point
	processing_y_numsamples = processing_dy_numsamples + 1

	# Select the y data
	y = ydata[-processing_y_numsamples:]
	# The last position point is t0
	ty = np.array(range(-processing_y_numsamples, 0)) + 1

	# Differentiate to get velocity
	dy = [y[i] - y[i-1] for i in range(1, len(y))]
	# The last velocity point is t0 - 0.5
	tdy = np.array(range(-processing_dy_numsamples, 0)) + 0.5

	# Differentiate to get acceleration
	d2y = [dy[i] - dy[i-1] for i in range(1, len(dy))]
	# The last acceleration point is t0 - 1
	td2y = np.array(range(-processing_d2y_numsamples, 0))

	# Calcualte NSigma
	NSigma = N * np.array([np.std(d2y[i-acceleration_window+1:i+1]) for i in range(acceleration_window-1, len(d2y))])

	y = y[-final_y_num_samples:]
	ty = ty[-final_y_num_samples:]
	dy = dy[-final_dy_num_samples:]
	tdy = tdy[-final_dy_num_samples:]
	d2y = d2y[-final_d2y_num_samples:]
	td2y = td2y[-final_d2y_num_samples:]

	# Update position plot
	ln_y.set_data(ty, y)
	ax_y.set_ylim(np.min(y), np.max(y))
	ax_y.set_xlim(np.min(ty), np.max(ty))

	# Update velocity plot
	ln_dy.set_data(tdy, dy)
	ax_dy.set_ylim(np.min(dy), np.max(dy))
	ax_d2y.set_xlim(np.min(tdy), np.max(tdy))	

	# Update acceleration plot
	ln_d2y.set_data(td2y, d2y)
	ln_nsigma_p.set_data(td2y, NSigma)
	ln_nsigma_n.set_data(td2y, -NSigma)
	ylim = [np.min([2*np.min(-NSigma), np.min(d2y)]),np.max([2*np.max(NSigma), np.max(d2y)])]
	ax_d2y.set_ylim(ylim)
	ax_d2y.set_xlim(np.min(td2y), np.max(td2y))
	
	return ln_y, ln_dy, ln_d2y

s = serial.Serial("/dev/tty.usbmodem2050316A41501")

buffer_len = 2500
sampling_rate = 48000000/65336


timestamps = []
fig, ax = plt.subplots(3,1, sharex=True)
ydata = []

ax_y = ax[0]
ln_y, = ax_y.plot([], [], 'r-')
ax_y.set_xlabel("Sample")
ax_y.set_ylabel("Voltage / V")

ax_dy = ax[1]
ln_dy, = ax_dy.plot([], [], 'r-')
ax_dy.set_xlabel("Sample")
ax_dy.set_ylabel("dV/dt / Vs^-1")

ax_d2y = ax[2]
ln_d2y, = ax_d2y.plot([], [], 'r-')
ln_nsigma_p, = ax_d2y.plot([], [], 'b--')
ln_nsigma_n, = ax_d2y.plot([], [], 'b--')
ax_d2y.set_xlabel("Sample")
ax_d2y.set_ylabel("dV2/d2yt / Vs^-2")

ani = FuncAnimation(fig, update, fargs=[s])
plt.show()