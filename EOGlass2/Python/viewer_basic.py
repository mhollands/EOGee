import serial
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import scipy.signal
import scipy.stats
import time
import glob

buffer_len = 2000
sampling_rate = 48000000/65336
adc_per_dac = 22;

def connect_to_usb():
	available_devices = []
	print("Looking for usb device...")
	while(len(available_devices) < 1):
		available_devices = glob.glob("/dev/tty.usbmodem*")
	print("Connecting to {0}".format(available_devices[0]))
	s = serial.Serial(available_devices[0])
	return s

s = connect_to_usb()

while(1==1):
	try:
		num_bytes_available = int(np.floor(s.in_waiting/2.0)*2)
		points_8bit = s.read(num_bytes_available)
	except:
		print("Unable to communicate with device...")
		s = connect_to_usb()
		continue
	points_16bit = [points_8bit[i+1] * 256 + points_8bit[i] for i in range(0,len(points_8bit), 2)]
	if(len(points_16bit) > 0):
		print(points_16bit)