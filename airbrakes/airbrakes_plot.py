import pandas as pd
import matplotlib.pyplot as plt

# Read files
pred = pd.read_csv("c:/Users/jackb/RT_Python_Lib/airbrakes/dat/6225/airbrakes_input.csv")   # columns: Time, Predicted DP
actual = pd.read_csv("c:/Users/jackb/RT_Python_Lib/airbrakes/dat/6225/airbrakes_output.csv")    # columns: Time, Actual DP

# Plot
plt.figure()
plt.plot(pred["Time"], pred["Predicted DP"], label="Predicted DP")
plt.plot(actual["Time"], actual["Actual DP"], label="Actual DP")
plt.plot(pred["Time"], pred["Apogee"], label="Apogee", linestyle='--')

plt.xlabel("Time")
plt.ylabel("DP")
plt.legend()
plt.tight_layout()
plt.show()