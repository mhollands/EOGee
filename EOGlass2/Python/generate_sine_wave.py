import numpy as np
import matplotlib.pyplot as plt

ns = 100 # samples per period
fsine = 300 # frequency
nbits = 12; # number of bits
periods_per_demod = 5; #how many periods are captured before demodulation

fsample = ns * fsine

i = np.floor(((np.sin(2*np.pi*np.arange(ns)/ns) * (2**(nbits)-1)/2))).astype(int)
q = np.floor(((np.cos(2*np.pi*np.arange(ns)/ns) * (2**(nbits)-1)/2))).astype(int)

print("in phase")
print(np.array2string(i, separator=',').replace("\n", ""))

print("quad phase")
print(np.array2string(q, separator=',').replace("\n", ""))