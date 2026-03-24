import numpy as np
from scipy.io.wavfile import write as wav_write

def time_voltage_to_wav(
    time_s: np.ndarray,
    voltage: np.ndarray,
    out_path: str,
    fs: int,
    normalize: bool = True,
    dtype=np.int16,
):
    """
    Convert time (seconds) + voltage samples to a mono WAV file.

    Parameters
    ----------
    time_s : (N,) array of timestamps in seconds (monotonic increasing preferred)
    voltage: (N,) array of samples (any units)
    out_path: output .wav path
    fs : sample rate to use. If None, inferred from median dt of time_s.
    normalize : if True, scale to full-scale (prevents clipping)
    dtype : np.int16 (common) or np.float32, etc.

    Returns
    -------
    fs_used, audio_samples_written (1D numpy array)
    """
    t = np.asarray(time_s).astype(np.float64).ravel()
    x = np.asarray(voltage).astype(np.float64).ravel()
    if t.shape != x.shape:
        raise ValueError(f"time_s and voltage must have same shape, got {t.shape} vs {x.shape}")
    if len(t) < 2:
        raise ValueError("Need at least 2 samples.")

    # sort by time (in case input isn't sorted)
    order = np.argsort(t)
    t = t[order]
    x = x[order]

    # infer sample rate if not provided
    dt = np.diff(t)
    dt_med = np.median(dt)
    if dt_med <= 0:
        raise ValueError("time_s must be strictly increasing (after sorting).")

    fs_inferred = int(round(1.0 / dt_med))
    fs_used = fs_inferred if fs is None else int(fs)

    # build uniform time grid for WAV samples
    t0, t1 = t[0], t[-1]
    n = int(np.floor((t1 - t0) * fs_used)) + 1
    tu = t0 + np.arange(n) / fs_used

    # interpolate onto uniform grid
    xu = np.interp(tu, t, x)

    # remove DC offset (optional but usually helpful)
    xu = xu - np.mean(xu)

    # normalize to avoid clipping
    if normalize:
        peak = np.max(np.abs(xu))
        if peak > 0:
            xu = xu / peak

    # write WAV
    if dtype == np.int16:
        y = np.clip(xu, -1.0, 1.0)
        y = (y * 32767.0).astype(np.int16)
        wav_write(out_path, fs_used, y)
        return fs_used, y
    elif dtype == np.float32:
        y = np.clip(xu, -1.0, 1.0).astype(np.float32)
        wav_write(out_path, fs_used, y)
        return fs_used, y
    else:
        raise ValueError("Use dtype=np.int16 (recommended) or dtype=np.float32.")












importedDat = np.array(np.genfromtxt("scope_6.csv",delimiter=",",skip_header=2)).T

fs = 1/np.mean([importedDat[0][i]-importedDat[0][i-1] for i in range(1,len(importedDat[0]))])
import matplotlib.pyplot as plt



# Example:
fs_used, y = time_voltage_to_wav(importedDat[0], importedDat[1], "../output2.wav", fs)
print("Wrote output.wav at", fs_used, "Hz")




from scipy.io import wavfile

fs, x = wavfile.read("../test.wav")
fs2, x2 = wavfile.read("../output2.wav")


plt.plot(range(len(x)),x)
plt.plot(range(len(x2)),x2)
plt.show()

















