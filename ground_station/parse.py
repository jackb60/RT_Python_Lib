import pandas as pd
import matplotlib.pyplot as plt
import ast

# For LaTeX-powered plotting:
from matplotlib import rcParams
rcParams['font.family'] = 'serif'
rcParams['text.usetex'] = True
rcParams['font.size'] = 14


# === CONFIG ===
filename = "/Users/mdn/Downloads/telemetry_xanthusflight3.csv"

headers = [
    "time", "pyros", "servos", "accelerometer", "barometer",
    "baro_filtered_alt", "baro_filtered_velocity", "temp", "gyro",
    "magnetometer", "heading", "gps_fix", "lat", "lon", "gps_alt",
    "pdop", "hdop", "vdop", "flight_time", "last_rec",
    "yaw_gyro_int", "pitch_gyro_int", "roll_gyro_int",
    "batt_voltage", "state", "pktnum", "rssi",
    "armed", "fired", "bad_packets", "rxrssi",
    "accel_integrated_velo", "baro_max_alt", "gps_max_alt"
]

# === LOAD & PREPARE ===
df = pd.read_csv(filename, names=headers,skiprows=1)

# Parse vector-like columns
for col in ["accelerometer", "gyro", "magnetometer", "servos"]:
    df[col] = df[col].apply(lambda x: ast.literal_eval(x) if isinstance(x, str) and x.startswith('[') else x)


# --- Add accelerometer components ---
df["accel_x"] = df["accelerometer"].apply(lambda a: a[0] if isinstance(a, list) else None)
df["accel_y"] = df["accelerometer"].apply(lambda a: a[1] if isinstance(a, list) else None)
df["accel_z"] = df["accelerometer"].apply(lambda a: a[2] if isinstance(a, list) else None)

# --- Add gyro components ---
df["gyro_x"] = df["gyro"].apply(lambda g: g[0] if isinstance(g, list) else None)
df["gyro_y"] = df["gyro"].apply(lambda g: g[1] if isinstance(g, list) else None)
df["gyro_z"] = df["gyro"].apply(lambda g: g[2] if isinstance(g, list) else None)

# --- Add servo channels (servo_0 .. servo_7) ---
for i in range(8):
    df[f"servo_{i}"] = df["servos"].apply(lambda s: s[i] if isinstance(s, list) and len(s) > i else None)

# Normalize time
df["time"] = df["time"] - df["time"].iloc[0]

#RSSI FIX
df["rssi"] = df["rssi"].apply(lambda a: a if a < 0 else a - 256)
df["rxrssi"] = df["rxrssi"].apply(lambda a: a if a < 0 else a - 256)
# Convert numeric columns
for col in df.columns:
    df[col] = pd.to_numeric(df[col], errors="ignore")

# === INTERACTIVE USER SELECTION ===
numeric_cols = df.select_dtypes(include=["float64", "int64"]).columns.tolist()
print("\nAvailable numeric columns:\n")
for i, col in enumerate(numeric_cols):
    print(f"{i:2d}: {col}")

print("\nEnter the numbers of the stats you want to plot (comma-separated):")
user_input = input(">>> ")

try:
    selected_indices = [int(x.strip()) for x in user_input.split(",") if x.strip().isdigit()]
    selected_cols = [numeric_cols[i] for i in selected_indices if i < len(numeric_cols)]
except:
    selected_cols = []

if not selected_cols:
    print("\n⚠️ No valid selection made. Exiting.")
else:
    # === PLOTTING (one subplot per variable) ===
    fig, axes = plt.subplots(len(selected_cols), 1, figsize=(12, 4 * len(selected_cols)), sharex=True)
    if len(selected_cols) == 1:
        axes = [axes]

    # Create color map for states
    if "state" in df.columns:
        unique_states = list(df["state"].dropna().unique())
        colors = plt.cm.tab20.colors
        color_map = {s: colors[i % len(colors)] for i, s in enumerate(unique_states)}

    for ax, col in zip(axes, selected_cols):
        # Plot the data
        ax.plot(df["time"], df[col], label="\\verb|{}|".format(col), color="tab:blue")
        ax.set_ylabel("\\verb|{}|".format(col))
        ax.grid(True)
        ax.legend(loc="upper left")

        # Overlay state regions
        if "state" in df.columns:
            last_state = None
            start_time = df["time"].iloc[0]
            for i in range(len(df)):
                state = df["state"].iloc[i]
                if state != last_state and last_state is not None:
                    ax.axvspan(start_time, df["time"].iloc[i],
                               color=color_map.get(last_state, "gray"), alpha=0.15)
                    start_time = df["time"].iloc[i]
                last_state = state
            # Final span
            ax.axvspan(start_time, df["time"].iloc[-1],
                       color=color_map.get(last_state, "gray"), alpha=0.15)

        # Auto-scale Y per variable
        ax.relim()
        ax.autoscale()

    plt.xlabel("Time (s)")
    plt.suptitle("\\sc Telemetry Data (auto-scaled per variable, state overlay)", fontsize=14)
    plt.tight_layout()
    plt.show()