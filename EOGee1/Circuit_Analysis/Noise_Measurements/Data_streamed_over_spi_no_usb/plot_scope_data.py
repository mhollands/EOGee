import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import scipy.stats

file_list = ["BUS{0:02d}.CSV".format(x) for x in range(1,44)]

all_data = []
for file in file_list:
	df = pd.read_csv(file,header=1)
	df = df[df["State"] == "OK"]
	for i,row in df.iterrows():
		data_text = row["Data"]
		data = [int(data_text[i+2:i+4]+data_text[i:i+2], 16) for i in range(2, len(data_text)-3, 4)]
		all_data.extend(data)

mean = np.mean(all_data)
std = np.std(all_data)
pk = np.max(all_data)
tr = np.min(all_data)
print("Mean: {0}".format(mean))
print("Range: {0}-{1} = {2} over {3} samples".format(pk, tr, pk-tr, len(all_data)))
print("Std: {0}".format(std))
biggest_spike = np.max([pk-mean, mean-tr])
pdf = scipy.stats.norm(0, std).cdf(-biggest_spike)
print("Biggest spike: {0}".format(biggest_spike))
print("Probability of this spike or worse: {0}".format(pdf))

plt.plot(all_data)
plt.show()