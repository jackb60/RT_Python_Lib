import pandas as pd

# Load your CSV
TELEM_FILE_TO_PROC = "ZEPH_TEST_FLIGHT_GS3.csv"
pathToTelems = "../telemetry/"

df = pd.read_csv(pathToTelems+TELEM_FILE_TO_PROC)

# Get unique packet numbers
pktnums = set(df["pktnum"])

# Define expected range (assuming packets should start at 0 or 1)
min_pkt = min(pktnums)
max_pkt = max(pktnums)

expected = set(range(min_pkt, max_pkt + 1))

# Find missing packets
missing = sorted(expected - pktnums)

# Percent of packets present
percent_present = 100 * len(pktnums) / len(expected)

print(f"Percent of packets present: {percent_present:.2f}%")

# First 100 missing
print("First 100 missing pktnums:")
print(missing[:100])