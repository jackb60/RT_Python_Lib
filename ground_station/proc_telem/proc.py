# Import necessary libraries.
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import scipy.optimize as spopt
import ast

# For LaTeX-powered plotting:
from matplotlib import rcParams
rcParams['font.family'] = 'serif'
rcParams['text.usetex'] = True
rcParams['font.size'] = 10


blue = "#08357E"
red = "#7A0101"
lred = "#AB0000"
lblue = "#8AAEE8"
green = "#38761D"
orange = "#ff7f0e"
import matplotlib as mpl
rcParams['axes.prop_cycle'] = mpl.cycler(color=[blue,red,green,lblue,lred,orange]) 


TELEM_FILE_TO_PROC = "DUMMY_DATA_TEST.csv"
pathToTelems = "../telemetry/"

df = pd.read_csv(pathToTelems+TELEM_FILE_TO_PROC)

ts0 = np.array(df["flight_time"])*1e-3
ts0 -= ts0[0]


alt = np.array(df["barofilteredalt"])
apogeeInd = np.nanargmax(alt)
tenSecInd = 0

ts = ts0[tenSecInd:apogeeInd]
ts2 = ts0[apogeeInd:]


roll = np.array(df["roll_gyro_int"])[tenSecInd:apogeeInd]
roll2 = np.array(df["roll_gyro_int"])[apogeeInd:]
gyro_out = np.array(df["gyro"])
rollrate = -1*(np.array([ast.literal_eval(x) for x in gyro_out]).T)[0][tenSecInd:apogeeInd]
rollrate2 = -1*(np.array([ast.literal_eval(x) for x in gyro_out]).T)[0]



servos_out0 = df["servos_deg"]
servos_out = (np.array([ast.literal_eval(x) for x in servos_out0]).T)[tenSecInd:apogeeInd]
servos_out2 = (np.array([ast.literal_eval(x) for x in servos_out0]).T)

roll_out = servos_out[3][tenSecInd:apogeeInd]
roll_out2 = servos_out[3][apogeeInd:]
airbrakes_out = servos_out[0]
airbrakes_out2 = servos_out2[0]
airbrakes_out_plot = np.array([np.nan if np.abs(ri - airbrakes_out[0]) < 1 else ri for ri in airbrakes_out])[tenSecInd:apogeeInd]
airbrakes_out_plot2 = np.array([np.nan if np.abs(ri - airbrakes_out2[0]) < 1 else ri for ri in airbrakes_out2])

fig,axs = plt.subplots(nrows=3,sharex='col',figsize=(8.5,8.5))

ax = axs[0]
ax.plot(ts-10,roll_out,marker='x',color=blue,label="Roll")
ax.plot(ts2-10,roll_out2,color=lblue)
ax.plot(ts0-10,airbrakes_out2,color=lred,alpha=0.5)
ax.plot(ts-10,airbrakes_out[tenSecInd:apogeeInd],color=red,label="Airbrakes")
ax.vlines(ts0[apogeeInd]-10,*ax.get_ylim(),color='k',linestyle='dotted',label="Apogee")
ax.legend(ncol=3,loc='upper right')
ax.set_ylabel("Servo Position (deg)")
ax.set_ylim(-30,15)
ax.grid()


ax = axs[1]
ax.plot(ts-10,roll,color=orange)
ax.plot(ts2-10,roll2,color=orange,alpha=0.5)
ax.set_ylabel("Roll Angle (deg)")
ax.set_ylim(0,90)
ax.grid()
ax.vlines(ts0[apogeeInd]-10,*ax.get_ylim(),color='k',linestyle='dotted')


ax = axs[2]
ax.plot(ts-10,rollrate,color='purple')
ax.plot(ts2-10,rollrate2[apogeeInd:],color='purple',alpha=0.5)
ax.set_ylabel("Roll Rate (deg/s)")
ax.grid()
ax.set_ylim(-50,50)
ax.set_xlim(0,(ts[-1]-10)*1.05)
ax.vlines(ts0[apogeeInd]-10,*ax.get_ylim(),color='k',linestyle='dotted')


ax.set_xlabel("Flight Time (s)")
plt.tight_layout()
plt.savefig("plot_telemetry_{}.pdf".format(TELEM_FILE_TO_PROC))
plt.show()

#print(roll_out)


