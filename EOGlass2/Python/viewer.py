import serial
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import scipy.signal
import scipy.stats
import time
import glob

buffer_len = 2000
sampling_rate = 30000/32

def connect_to_usb():
	available_devices = []
	print("Looking for usb device...")
	while(len(available_devices) < 1):
		available_devices = glob.glob("/dev/tty.usbmodem*")
	print("Connecting to {0}".format(available_devices[0]))
	s = serial.Serial(available_devices[0])	
	return s

def plot_filter(b,a,fs):
	freq, h = scipy.signal.freqz(b, a, fs=fs)
	# Plot
	fig, ax = plt.subplots(2, 1, figsize=(8, 6))
	ax[0].plot(freq, 20*np.log10(abs(h)), color='blue')
	ax[0].set_title("Frequency Response")
	ax[0].set_ylabel("Amplitude (dB)", color='blue')
	ax[0].grid()
	ax[1].plot(freq, np.unwrap(np.angle(h))*180/np.pi, color='green')
	ax[1].set_ylabel("Angle (degrees)", color='green')
	ax[1].set_xlabel("Frequency (Hz)")
	ax[1].grid()
	plt.show()
	return

def init():
    return

def update(frame):
	global ydata
	global fdata
	global s

	try:
		num_bytes_available = int(np.floor(s.in_waiting/2.0)*2)
		points_8bit = s.read(num_bytes_available)
	except:
  		print("Unable to communicate with device...")
  		s = connect_to_usb()
  		return


	points_16bit = [points_8bit[i+1] * 256 + points_8bit[i] for i in range(0,len(points_8bit), 2)]

	[ydata.append(x & 0x0FFF) for x in points_16bit if (x >> 12) == 0x8]

	demoddata = []
	[demoddata.append(x & 0x0FFF) for x in points_16bit if (x >> 12) == 0x2]
	[print("Demod: {0}".format(x)) for x in demoddata]

	y = ydata[-buffer_len:]
	ax_t.set_ylim(np.min(y)-10, np.max(y)+10)
	ax_t.set_xlim(0, len(y))
	ln_y.set_data(range(len(y)), y)

	ydata_filt = scipy.signal.lfilter(filt_b, filt_a, ydata)
	y_filt = ydata_filt[-buffer_len:]
	ln_y_filt.set_data(range(len(y_filt)), y_filt)

	# Take fourier transform of data with mean value removed
	fdata = np.abs(np.fft.fft(y - np.mean(y)))
	freqs = np.arange(0,int(len(fdata)/2)) * sampling_rate / len(fdata)
	ax_f.set_ylim(np.min(fdata), np.max(fdata))
	ax_f.set_xlim(0, np.max(freqs))
	ln_f.set_data(freqs, fdata[0:int(len(fdata)/2)])
	return

s = connect_to_usb()

filt_b, filt_a = scipy.signal.iirnotch(60, 3, fs=sampling_rate)
# plot_filter(filt_b, filt_a, sampling_rate)

fig, [ax_t, ax_f] = plt.subplots(2,1)
ydata = []

ln_y, = ax_t.plot([], [], 'r-')
ln_y_filt, = ax_t.plot([], [], 'b-')

ln_f, = ax_f.plot([], [], 'r-')

ax_t.set_xlabel("Sample")
ax_t.set_ylabel("ADC Code")

ani = FuncAnimation(fig, update, init_func=init)
plt.show()

# if(len(ydata) > 0):
# 	save_data = {"ydata": ydata}
# 	filename = "logger_data_{0}.npy".format(str(time.time()).replace(".", "-"))
# 	np.save(filename, save_data)
# 	print("Saved as {0}".format(filename))