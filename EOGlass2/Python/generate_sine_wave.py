import numpy as np
import matplotlib.pyplot as plt

ns = 100 # samples per period
fsine = 300 # frequency
nbits = 12; # number of bits

fsample = ns * fsine

i = ((np.sin(2*np.pi*np.arange(ns)/ns) * (2**(nbits)-1)/2) + (2**(nbits)-1)/2).round().astype(int)
q = ((np.cos(2*np.pi*np.arange(ns)/ns) * (2**(nbits)-1)/2) + (2**(nbits)-1)/2).round().astype(int)

print("in phase")
print(np.array2string(i, separator=',').replace("\n", ""))

print("quad phase")
print(np.array2string(q, separator=',').replace("\n", ""))