import matplotlib.pyplot as plt
import numpy as np
import matplotlib.animation as animation
import time
import serial 
import glob

def connect_to_usb():
	available_devices = []
	print("Looking for usb device...")
	while(len(available_devices) < 1):
		available_devices = glob.glob("/dev/tty.usbmodem*")
	print("Connecting to {0}".format(available_devices[0]))
	s = serial.Serial(available_devices[0])
	return s

def get_usb_data(s):
	try:
		num_bytes_available = int(np.floor(s.in_waiting/2.0)*2)
		points_8bit = s.read(num_bytes_available)
	except:
  		print("Unable to communicate with device...")
  		s = connect_to_usb()
  		return
	points_16bit = [points_8bit[i+1] * 256 + points_8bit[i] for i in range(0,len(points_8bit), 2)]
	return points_16bit

def get_dots(i, path):
	dots = path[i%len(path)]
	return dots

def update(i, scat, scat_next, path, s):
	# Beep
	if(int(i/rate) != int((i-1)/rate)):
		print("\a")
	# Add timestamp
	timestamps.append(time.time())
	# Store where the point transitioned to
	points.append(get_dots(int((i-1)/rate),path))
	# Store eog data
	data_frame = get_usb_data(s)
	if data_frame is None: data_frame = [] 
	data.append(data_frame)
	#Update the dots
	scat.set_offsets(get_dots(int(i/rate),path))
	scat_next.set_offsets(get_dots(int((i/rate)+1),path))
	pass

# Our path to trace
path = [[-0.9,0], [0.9,0], [0,0]]
interval = 100
rate = 10;
timestamps = []
points = []
data = []
temp_data = []
source = "eogee2_agagcl_unshielded"

# Connect to usb
s = connect_to_usb()

# Set up animations
fig, ax = plt.subplots()
ax.set_xlim([-1,1])
ax.set_ylim([-1,1])
plt.tight_layout()
scat = ax.scatter([],[], c="red")
scat_next = ax.scatter([],[], c="gray")
update(0, scat, scat_next, path, None)

anim = animation.FuncAnimation(fig, update, interval=interval, frames=1000, fargs=(scat, scat_next, path, s))
plt.show(block=True)

if(len(timestamps)) > 0:
	save_data = {"timestamps":timestamps, "data":data, "points":points, "path":path, "interval":interval, "rate":10, "source":source}
	np.save("pingpong_data_{0}.npy".format(str(timestamps[0]).replace(".", "-")), save_data)
s.close()