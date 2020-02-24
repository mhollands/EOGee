import serial
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import scipy.signal

s = serial.Serial("/dev/tty.usbmodem2050316A41501")

buffer_len = 1000
sampling_rate = 48000000/65336

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

	num_bytes_available = int(np.floor(s.in_waiting/2.0)*2)
	points_8bit = s.read(num_bytes_available)
	points_16bit = [points_8bit[i+1] * 256 + points_8bit[i] for i in range(0,len(points_8bit), 2)]
	[ydata.append(x) for x in points_16bit]
	if(len(ydata) > buffer_len):
		ydata = ydata[-buffer_len:]

	volts = np.array(ydata) * 3.3 / np.power(2,12)

	volts_filt = scipy.signal.lfilter(filt_b, filt_a, volts)

	ax_t.set_ylim(np.min(volts), np.max(volts))
	ax_t.set_xlim(0, len(volts))
	ln_t.set_data(range(len(volts)), volts)
	ln_tf.set_data(range(len(volts_filt)), volts_filt)

	# Take fourier transform of data with mean value removed
	fdata = np.abs(np.fft.fft(volts - np.mean(volts)))
	fdata_filt = np.abs(np.fft.fft(volts_filt - np.mean(volts_filt)))
	freqs = np.arange(0,int(len(fdata)/2)) * sampling_rate / len(fdata)
	ax_f.set_ylim(min(fdata), max(fdata_filt))
	ax_f.set_xlim(0, np.max(freqs))
	ln_f.set_data(freqs, fdata[0:int(len(fdata)/2)])
	ln_ff.set_data(freqs, fdata_filt[0:int(len(fdata)/2)])
	return

filt_b, filt_a = scipy.signal.iirnotch(60, 3, fs=sampling_rate)
plot_filter(filt_b, filt_a, sampling_rate)

fig, [ax_t, ax_f] = plt.subplots(2,1)
ydata = []
fdata = []
ln_t, = ax_t.plot([], [], 'r-')
ln_tf, = ax_t.plot([], [], 'b-')
ln_f, = ax_f.plot([], [], 'r-')
ln_ff, = ax_f.plot([], [], 'b-')

ax_t.set_xlabel("Sample")
ax_t.set_ylabel("Voltage / V")

ani = FuncAnimation(fig, update, init_func=init)
plt.show()