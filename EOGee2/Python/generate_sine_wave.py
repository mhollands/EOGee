import numpy as np
import matplotlib.pyplot as plt

ns = 100
fsine = 300

nbits = 12;
fsample = ns * fsine

i = (np.sin(2*np.pi*np.arange(ns)/ns) * (2**(nbits-1)-1)).astype(int) + 2**(nbits-1)
q = (np.cos(2*np.pi*np.arange(ns)/ns) * (2**(nbits-1)-1)).astype(int) + 2**(nbits-1)
print(np.array2string(d, separator=',').replace("\n", ""))