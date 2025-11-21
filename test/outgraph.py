import numpy as np
import matplotlib.pyplot as plt
import wave
import struct

# -----------------------------
# Settings
# -----------------------------
INPUT_FILE = "signal.txt"      # your text file
MAKE_WAV = True                # enable/disable WAV export
WAV_OUT = "output.wav"         # wav file name
AMPLITUDE_SCALE = 32767        # convert float → int16 amplitude
# -----------------------------


def read_signal_file(filename):
    times = []
    values = []

    with open(filename, "r") as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            
            # Expect: time,value
            try:
                t, v = line.split(",")
                times.append(float(t))
                values.append(float(v))
            except:
                print("Skipping malformed line:", line)

    return np.array(times), np.array(values)


def write_wav(times, values, wav_filename):
    # Infer sample rate from time differences
    dt = np.diff(times)
    avg_dt = np.mean(dt)

    sample_rate = int(1.0 / avg_dt)

    print(f"Detected sample rate: {sample_rate} Hz")

    # Convert values to int16
    int_samples = np.int16(values * AMPLITUDE_SCALE)

    wav = wave.open(wav_filename, 'w')
    wav.setnchannels(1)       # mono
    wav.setsampwidth(2)       # int16
    wav.setframerate(sample_rate)

    for sample in int_samples:
        wav.writeframes(struct.pack('<h', sample))

    wav.close()
    print(f"Exported WAV → {wav_filename}")


def plot_waveform(times, values):
    plt.figure(figsize=(12, 6))
    plt.plot(times, values, linewidth=1)
    plt.title("Waveform Viewer")
    plt.xlabel("Time (seconds)")
    plt.ylabel("Amplitude")
    plt.grid(True)

    # full interactive zoom/pan
    plt.tight_layout()
    plt.show()


# ----------------------------------------------
# Main Execution
# ----------------------------------------------
times, values = read_signal_file(INPUT_FILE)

print(f"Loaded {len(times)} samples.")

plot_waveform(times, values)

if MAKE_WAV:
    write_wav(times, values, WAV_OUT)
