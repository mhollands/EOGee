import numpy as np
import matplotlib.pyplot as plt

data_file = "calibration_data.npy"
edge_threshold = 0.5
settling_time = 20
midscale = 2048
valid_range = [1980,2105]
data = np.load(data_file, allow_pickle=True).item()

y = data["ydata"]
o = data["offsetdata"]

edges = (np.abs(np.diff(o)) > edge_threshold)
edge_beginnings = [i for i in range(len(edges)-1) if edges[i] == 0 and edges[i+1] == 1]
edge_endings = [i for i in range(len(edges)-1) if edges[i] == 1 and edges[i+1] == 0]

if(edge_beginnings[0] < edge_endings[0]):
	edge_beginnings.pop(0)

yvals = []
ovals = []
for i in range(len(edge_endings)-1):
	ypoints = y[edge_endings[i]+settling_time:edge_beginnings[i]]
	diff_points = np.diff(y)[edge_endings[i]+settling_time:edge_beginnings[i]]
	opoints = o[edge_endings[i]+settling_time:edge_beginnings[i]]
	plt.plot(ypoints)
	yvals.append(np.mean(ypoints))
	ovals.append(np.mean(opoints))

plt.figure()
plt.scatter(ovals,yvals)

ovals = np.array(ovals)
yvals = np.array(yvals)

lookup = dict()
diffs = []
for i in range(valid_range[0], valid_range[1]):
	idx = (ovals == i)
	diffs.extend(np.diff(yvals)[idx[:-1]])
	lookup[i] = np.mean(yvals[idx]) - midscale

plt.figure()
plt.plot(y)
plt.plot(o)

average_step = np.mean(diffs)
print("Average Step: {0}".format(average_step))
np.save("calibration_table_new.npy", lookup)

plt.show()