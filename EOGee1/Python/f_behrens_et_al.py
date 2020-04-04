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

	# Get the latest processed data point
	y_ptr = len(ydata)
	dy_ptr = len(dydata)
	d2y_ptr = len(d2ydata)

	# Save new data
	timestamps.append(timestamp)

	# Get latest processed sample
	t_latest = len(ydata)

	# Add new ydata
	ydata.extend(data)

	# For each new sample, update dy
	while(len(dydata) < len(ydata)):
		if(len(dydata) == 0):
			dydata.append(0)
		else:
			dydata.append(ydata[len(dydata)] - ydata[len(dydata)-1])

	# For each new sample, update dy
	while(len(d2ydata) < len(ydata) - 1):
		# If this is the first sample we can't calculate d2y or NSigma or MSF or F1 so put 0
		if(len(d2ydata) == 0):
			d2ydata.append(0)
			NSigmadata.append(0)
			MSFdata.append(0)
			F1data.append(0)
			continue

		# Calculate d2y
		d2ydata.append(ydata[len(d2ydata)+1] - 2*ydata[len(d2ydata)] + ydata[len(d2ydata)-1])
		# If we are before the acceleration window, we cannot calculate NSigma so we cannot calculate MSF, put 0
		if(len(d2ydata) < acceleration_window):
			NSigmadata.append(0)
			MSFdata.append(0)
			F1data.append(0)
			continue
		# Calcualte NSigma as N times the std of the previous samples in accerleation window
		NSigmadata.append(N*np.std(d2ydata[-acceleration_window:]))
		# If MSF is 0 and d2y crossed the NSigma threshold, set MSF to 1 or -1
		if(MSFdata[-1] == 0 and np.abs(d2ydata[-1]) > np.abs(NSigmadata[-1])):
			MSFdata.append(np.sign(d2ydata[-1]))
		else:
			# By default MSF should retain state
			MSFdata.append(MSFdata[-1])
		# If we are in a saccade and F1 crosses NSigma line
		if(MSFdata[-1] != 0 and F1data[-1] == 0 and np.sign(MSFdata[-1])*d2ydata[-1] < NSigmadata[-1]):
			F1data.append(1)
		else:
			F1data.append(F1data[-1])

	# Select the end samples
	y = ydata[-buffer_len:]
	dy = dydata[-buffer_len:]
	d2y = d2ydata[-buffer_len:]
	NSigma = NSigmadata[-buffer_len:]
	MSF = MSFdata[-buffer_len:]
	F1 = F1data[-buffer_len:]

	ty = range(len(ydata)-buffer_len, len(ydata))
	tdy = np.array(range(len(dydata)-buffer_len, len(dydata)))-0.5 #The time of each dy sample is actually 0.5 samples behind y
	td2y = range(len(d2ydata)-buffer_len, len(d2ydata))
	tNSigma = range(len(NSigmadata)-buffer_len, len(NSigmadata))
	tMSF = range(len(MSFdata)-buffer_len, len(MSFdata))
	tF1 = range(len(F1data)-buffer_len, len(F1data))

	# Make sure we have enough samples to plot y data
	if(len(y) < len(ty)):
		print("Waiting for y-buffer to fill, {0}/{1}".format(len(y), len(ty)))
		return

	# Update position plot
	ln_y.set_data(ty, y)
	ax_y.set_ylim(np.min(y), np.max(y))
	ax_y.set_xlim(np.min(ty), np.max(ty))

	# Make sure we have enough samples to plot dy data
	if(len(dy) < len(tdy)):
		print("Waiting for dy-buffer to fill, {0}/{1}".format(len(dy), len(tdy)))
		return

	# Update velocity plot
	ln_dy.set_data(tdy, dy)
	ax_dy.set_ylim(np.min(dy), np.max(dy))
	ax_dy.set_xlim(np.min(tdy), np.max(tdy))	

	# Make sure we have enough samples to plot dy data
	if(len(d2y) < len(td2y)):
		print("Waiting for d2y-buffer to fill, {0}/{1}".format(len(d2y), len(td2y)))
		return
	
	# Update acceleration plot
	ln_d2y.set_data(td2y, d2y)
	ylim = [np.min([-2*np.max(NSigma), np.min(d2y)]),np.max([2*np.max(NSigma), np.max(d2y)])]
	# ylim = [np.min([0, np.min(d2y)]),np.max([0, np.max(d2y)])]
	ax_d2y.set_ylim(ylim)
	ax_d2y.set_xlim(np.min(td2y), np.max(td2y))

	# Make sure we have enough samples to plot NSigma data
	if(len(NSigma) < len(tNSigma)):
		print("Waiting for NSigma-buffer to fill, {0}/{1}".format(len(NSigma), len(tNSigma)))
		return
	ln_nsigma_p.set_data(tNSigma, NSigma)
	ln_nsigma_n.set_data(tNSigma, -np.array(NSigma))
	
	# Make sure we have enough samples to plot MSF data
	if(len(MSF) < len(tMSF)):
		print("Waiting for MSF-buffer to fill, {0}/{1}".format(len(MSF), len(tMSF)))
		return
	ln_MSF.set_data(tMSF, MSF)
	ln_F1.set_data(tF1, F1)
	ax_Flags.set_xlim(np.min(tMSF), np.max(tMSF))	
	return

buffer_len = 2500
acceleration_window = 200
sampling_rate = 48000000/65336
N = 3.4

s = serial.Serial("/dev/tty.usbmodem2050316A41501")

timestamps = []
fig, ax = plt.subplots(4,1, sharex=True)
ydata = []
dydata = []
d2ydata = []
NSigmadata = []
MSFdata = []
F1data = []

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

ax_Flags = ax[3]
ln_MSF, = ax_Flags.plot([], [], 'r-')
ln_F1, = ax_Flags.plot([], [], 'r--')
ax_Flags.set_xlabel("Sample")
ax_Flags.set_ylabel("Flags")
ax_Flags.set_ylim(-1.5,1.5)

ani = FuncAnimation(fig, update, fargs=[s])
plt.show()