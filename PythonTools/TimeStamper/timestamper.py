import numpy as np
import matplotlib.pyplot as plt
import seaborn as sns

sns.set_palette(sns.color_palette("hls", 8))

logger_file = "./logger_data_1585372676-819437.npy"
pingpong_file = "./pingpong_data_1585372676-573356.npy"

logger_data = np.load(logger_file, allow_pickle=True).item()
pingpong_data = np.load(pingpong_file, allow_pickle=True).item()


fig,ax = plt.subplots()
ax.plot(logger_data["ydata"])

for p in pingpong_data["timestamps"]:
	# Find all logger timestamps on either side of this pingpongtimestamp
	before_neighbours = [l for l in logger_data["timestamps"] if p > l[0]]
	after_neighbours = [l for l in logger_data["timestamps"] if p < l[0]]

	# We need upper and lower timestamps to interpolate event position
	if(len(before_neighbours) < 1 or len(after_neighbours) < 1):
		continue

	# Find the timestamps eitherside
	before_neighbour = max(before_neighbours)
	after_neighbour = min(after_neighbours)

	sample = ((p - before_neighbour[0])/(after_neighbour[0] - before_neighbour[0])) * (after_neighbour[1]-before_neighbour[1]) + before_neighbour[1]
	ax.axvline(sample, linestyle='--', color="lightgrey")

plt.show()