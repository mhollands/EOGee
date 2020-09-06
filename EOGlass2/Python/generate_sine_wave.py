import numpy as np
import matplotlib.pyplot as plt

ns = 100 # samples per period
fsine = 300 # frequency
nbits = 12; # number of bits
periods_per_demod = 5; #how many periods are captured before demodulation

fsample = ns * fsine

i = ((np.sin(2*np.pi*np.arange(ns)/ns) * (2**(nbits)-1)/2) + (2**(nbits)-1)/2).round().astype(int)
q = ((np.cos(2*np.pi*np.arange(ns)/ns) * (2**(nbits)-1)/2) + (2**(nbits)-1)/2).round().astype(int)

print("in phase")
print(np.array2string(i, separator=',').replace("\n", ""))

print("quad phase")
print(np.array2string(q, separator=',').replace("\n", ""))

# Calculate the necessary size of the i- and q-accumulator integer size in order to support the maximum possible input for all samples
max_input_val = np.power(2,nbits) - 1
acc_size = np.log2(((max_input_val) * i).sum() * periods_per_demod)
print("Max accumulator size is {0} bits".format(acc_size))