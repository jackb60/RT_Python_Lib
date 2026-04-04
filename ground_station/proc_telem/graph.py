import pandas as pd
import matplotlib.pyplot as plt
import ast

# === Load CSV ===
TELEM_FILE_TO_PROC = "telemetry_1775344644.020344.csv"
pathToTelems = "../telemetry/"

df = pd.read_csv(pathToTelems+TELEM_FILE_TO_PROC)

# Convert timestamp to relative time
df['time'] = df['timestamp'] - df['timestamp'].iloc[0]

# === Parse list-like columns ===
def try_parse_list(x):
    try:
        return ast.literal_eval(x)
    except:
        return x

for col in df.columns:
    if isinstance(df[col].iloc[0], str) and df[col].iloc[0].startswith('['):
        df[col] = df[col].apply(try_parse_list)

# === Clean state column ===
if 'state' in df.columns:
    df['state_clean'] = df['state'].astype(str).apply(lambda x: x.split('.')[-1])
else:
    df['state_clean'] = None

# === Show available columns ===
print("\nAvailable columns:")
for col in df.columns:
    print("-", col)

# === User input ===
print("\nEnter columns to plot (comma separated):")
cols = input(">> ").split(",")
cols = [c.strip() for c in cols]

plt.figure()

# === Plot data ===
for col in cols:
    if col not in df.columns:
        print(f"Column '{col}' not found, skipping.")
        continue

    if isinstance(df[col].iloc[0], list):
        length = len(df[col].iloc[0])
        print(f"\n'{col}' is a list with {length} elements.")
        idx_input = input(f"Enter index/indices (e.g. 0 or 0,1,2): ")
        indices = [int(i.strip()) for i in idx_input.split(",")]

        for idx in indices:
            y = df[col].apply(lambda x: x[idx])
            plt.plot(df['time'], y, label=f"{col}[{idx}]")
    else:
        plt.plot(df['time'], df[col], label=col)

# === STATE OVERLAY ===
if df['state_clean'] is not None:
    prev_state = df['state_clean'].iloc[0]

    for i in range(1, len(df)):
        curr_state = df['state_clean'].iloc[i]
        if curr_state != prev_state:
            t = df['time'].iloc[i]

            plt.axvline(x=t, linestyle='--')  # vertical line
            plt.text(t, plt.ylim()[1], curr_state,
                     rotation=90, verticalalignment='bottom')

            prev_state = curr_state

# === Final formatting ===
plt.xlabel("Time (s)")
plt.ylabel("Value")
plt.title("Data vs Time (with State Overlay)")
plt.legend()
plt.grid()

plt.show()