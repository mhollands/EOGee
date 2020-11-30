# This script is intended to simulate the received signal at the electrode sense ADC and simulate the decoding process
import numpy as np
import matplotlib.pyplot as plt

stim_freq = 300 # Stimming frequency is hertz
over_sample_rate = 10 # We sample at a higher frequency than the stim
num_sample_periods = 5; # How long do we sample for
signal_strength = 0

noise_freq = 59;
noise_strength = 3;

sample_freq = stim_freq * over_sample_rate # Calculate the sample rate

t = np.arange(num_sample_periods * sample_freq/stim_freq) * 1/sample_freq # Find sample times

i_wf = np.sin(2*np.pi*stim_freq*t) # Waveform for decoding in-phase component
q_wf = np.cos(2*np.pi*stim_freq*t) # Waveform for decoding quadrature phase component

phase = np.random.random(1)*2*np.pi # Generate a random phase
noise_phase = np.random.random(1)*2*np.pi # Generate a random phase for the noise

samples = signal_strength*np.sin(2*np.pi*stim_freq*t + phase) + noise_strength * np.sin(2*np.pi*noise_freq*t + noise_phase) # do ADC sampling

# IQ demodulation
i = np.sum(i_wf*samples)
q = np.sum(q_wf*samples)
print(i, q, np.sqrt(i**2+q**2))

plt.plot(t, samples)
plt.show()