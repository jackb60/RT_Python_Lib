import numpy as np
import pandas as pd
from scipy.io.wavfile import write

# Load CSV and skip the units row
df = pd.read_csv("scope_0.csv", skiprows=[1])

# Convert columns to numeric (handles scientific notation strings)
df["x-axis"] = pd.to_numeric(df["x-axis"])
df["1"] = pd.to_numeric(df["1"])

# Extract time and signal
time = df["x-axis"].values
signal = df["1"].values

# Normalize signal to 16-bit audio range
signal = signal / np.max(np.abs(signal))
signal_int16 = np.int16(signal * 32767)

# Compute sample rate from time spacing
dt = np.mean(np.diff(time))
sample_rate = int(1 / dt)

print("Sample rate:", sample_rate)

# Save WAV file
write("output.wav", sample_rate, signal_int16)

print("Saved as output.wav")