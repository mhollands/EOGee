import numpy as np
import matplotlib.pyplot as plt
import glob
import matplotlib as mpl
mpl.style.use("seaborn-pastel")

folder = "./1M_experiment"
files= glob.glob("./1M_experiment/*.npy")

for file in files:
	d = np.load(file, allow_pickle=True).item()

	ydata = d["ydata"]
	ydata = d["ydata"]
	offsetdata = d["offsetdata"]
	adc_counts_per_dac_code = d["adc_counts_per_dac_code"]

	l = np.min([len(ydata), len(offsetdata)])
	y = ydata[:l] - (np.array(offsetdata)[:l]-2048)*adc_counts_per_dac_code
	plt.plot(y)
	break

plt.xlabel("Time /samples")
plt.ylabel("Signal")
plt.show()