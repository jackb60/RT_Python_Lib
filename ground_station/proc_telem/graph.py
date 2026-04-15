import pandas as pd
import matplotlib.pyplot as plt
import ast

# === Load CSV ===
TELEM_FILE_TO_PROC = "ZEPH_TEST_FLIGHT_GS2.csv"
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

# === Build list of things to plot first ===
plot_items = []

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
            plot_items.append((col, idx))
    else:
        plot_items.append((col, None))

# === Create subplots ===
n = len(plot_items)
fig, axes = plt.subplots(n, 1, figsize=(10, 3*n), sharex=True)

# If only one plot, axes isn't a list
if n == 1:
    axes = [axes]

# === Plot each item ===
for ax, (col, idx) in zip(axes, plot_items):

    if idx is not None:
        y = df[col].apply(lambda x: x[idx])
        label = f"{col}[{idx}]"
    else:
        y = df[col]
        label = col

    ax.plot(df['time'], y, label=label)

    # === STATE OVERLAY ===
    if df['state_clean'] is not None:
        prev_state = df['state_clean'].iloc[0]

        for i in range(1, len(df)):
            curr_state = df['state_clean'].iloc[i]
            if curr_state != prev_state:
                t = df['time'].iloc[i]
                ax.axvline(x=t, linestyle='--')
                ax.text(t, ax.get_ylim()[1], curr_state,
                        rotation=90, verticalalignment='bottom')
                prev_state = curr_state

    ax.set_ylabel("Value")
    ax.set_title(label)
    ax.legend()
    ax.grid()

# === Shared X label ===
axes[-1].set_xlabel("Time (s)")

plt.tight_layout()
plt.show()