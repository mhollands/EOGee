import numpy as np
import matplotlib.pyplot as plt

file = "sample2.npy"

d = np.load(file, allow_pickle=True).item()

ydata = d["ydata"]

# ax1.plot(ydata - np.array(d["offsetdata"])*30.126222239066166)
# ax2.plot(d["demoddata"])

# plt.show()

sampling_rate = 30000/32
sample_length = 500
mags = []
angles = []
for i in range(int(sample_length/2),len(ydata)-int(sample_length/2), 25):
	sample = ydata[i-int(sample_length/2):i+int(sample_length/2)]
	fdata = np.fft.fft(sample - np.mean(sample))
	freqs = np.arange(0,int(len(fdata)/2)) * sampling_rate / len(fdata)
	freq = freqs[160]
	mag = np.abs(fdata[160])
	angle = (np.angle(fdata[160]))# - 2*np.pi*((i*300/sampling_rate)%1))%(2*np.pi) - np.pi
	print("Frequency {0} has magnitude {1} and angle {2}".format(freq, mag, angle))
	mags.append(mag)
	angles.append(angle)

fig, [ax1, ax2, ax3, ax4] = plt.subplots(4,1, sharex=True)
ydata = ydata - np.array(d["offsetdata"])*30.126222239066166
ax1.plot(ydata)
ax2.plot(np.linspace(0,len(ydata), len(d["demoddata"])), d["demoddata"])
ax3.plot(np.linspace(0,len(ydata), len(mags)), mags)
ax4.plot(np.linspace(0,len(ydata), len(angles)), angles)

s = 100
ydata_smooth = np.convolve(ydata, np.ones(s)/s)
angles_smooth = np.convolve(angles, np.ones(s)/s)
demoddata_smooth = np.convolve(d["demoddata"], np.ones(s)/s)
mags_smooth = np.convolve(mags, np.ones(s)/s)

ax1.plot(ydata_smooth)
ax2.plot(np.linspace(0,len(ydata_smooth), len(demoddata_smooth)), demoddata_smooth)
ax3.plot(np.linspace(0,len(ydata_smooth), len(mags_smooth)), mags_smooth)
ax4.plot(np.linspace(0,len(ydata_smooth), len(angles_smooth)), angles_smooth)

plt.show()