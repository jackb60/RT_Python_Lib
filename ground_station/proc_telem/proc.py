# Import necessary libraries.
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import scipy.optimize as spopt
import ast
import getpass


mass = 34.15380231015455
rho = 0.736115423712237
airbrakesCd = 1.28
rocketCd = 0.4843927669074317
a_ref = 0.019289796351014733
a_max = 0.0066
fudge_factor = 3.2
fudge_factor_2 = 3.5
SIM_PREDICTED_ALTITUDE = 5046

EARLIEST_AIRBRAKES_PREP_TIME = 4.0
START_AIRBRAKES_PREP_VEL = 400.0
START_AIRBRAKES_PREPROC_TIME = 12.5
AIRBRAKES_TIME_DELAY = 1.0
AIRBRAKES_T_APOG_FUDGEDIFF = 1.5

roundToHowMuch = 100

t_apog = 35.5
coeffA = -0.0154397511
coeffB = -0.3379534959
g = 9.81

alt0 = 0.0
predictedAlt = 0.0
desiredDeltaX = 0.0

airbrakesCtrlStartTime = 1e10
A0_req = 0.0

Astar = 0.0
patchingAltitude = 0.0
velContribFudge = 1.0
cFudge = 0.825
K = 1

lastA = 0
lastDeltaA = 0
lastDeltaH = 0
lastHf = 0
lastI = 0


# For LaTeX-powered plotting:
from matplotlib import rcParams
rcParams['font.family'] = 'serif'
rcParams['text.usetex'] = True
rcParams['font.size'] = 10
if getpass.getuser() == 'mdn':
	rcParams['text.usetex'] = True
else:
	rcParams['text.usetex'] = False


blue = "#08357E"
red = "#7A0101"
lred = "#AB0000"
lblue = "#8AAEE8"
green = "#38761D"
orange = "#ff7f0e"
import matplotlib as mpl
rcParams['axes.prop_cycle'] = mpl.cycler(color=[blue,red,green,lblue,lred,orange]) 


TELEM_FILE_TO_PROC = "ZEPH_TEST_FLIGHT_GS1.csv"
pathToTelems = "../telemetry/"

df = pd.read_csv(pathToTelems+TELEM_FILE_TO_PROC)

ts0 = np.array(df["flight_time"])*1e-3
ts0 -= ts0[0]


alt = np.array(df["barofilteredalt"])
vel = np.array(df["accel_integrated_velo"])
apogeeInd = np.nanargmax(alt)
tenSecInd = 0

ts = ts0[tenSecInd:apogeeInd]-46-60
ts2 = ts0[apogeeInd:]-46-60


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

fig,axs = plt.subplots(nrows=2,sharex='col',figsize=(8.5,8.5))

ax = axs[0]


def calc_alt_prediction(A,index):
	h0 = alt[index]
	v0 = vel[index]
	m = mass
	c = rho * rocketCd * a_ref / 2.0
	c *= cFudge
	alpha = rho * airbrakesCd * A / 2.0

	hf = h0 + velContribFudge * m / (2.0 * (alpha + c)) * np.log((v0 * v0 * (alpha + c)) / g / m + 1.0)
	return hf - 13.0 + patchingAltitude

conrad = calc_alt_prediction(0,np.argmin((alt - 4284)**2))
patchingAltitude = SIM_PREDICTED_ALTITUDE-conrad



#ax.plot(ts-10,roll_out,marker='x',color=blue,label="Roll")
#ax.plot(ts2-10,roll_out2,color=lblue)
#ax.plot(ts0-10,airbrakes_out2,color=lred,alpha=0.5)
ax.plot(ts-10,airbrakes_out2[0]-airbrakes_out[tenSecInd:apogeeInd],color=red,label="Airbrakes")
ax.vlines(ts0[apogeeInd]-10,*ax.get_ylim(),color='k',linestyle='dotted',label="Apogee")
ax.legend(ncol=3,loc='upper right')
ax.set_ylabel("Servo Position (deg)")
ax.set_ylim(-5,55)
ax.grid()

coeff = 1/np.max(airbrakes_out2[0]-airbrakes_out[tenSecInd:apogeeInd])

ax = axs[1]
ax.plot(ts-10,alt[tenSecInd:apogeeInd],color=blue,label="True Altitude")
alt_prediction = [calc_alt_prediction((airbrakes_out2[0]-airbrakes_out[ind])*coeff*a_max,ind) for ind in range(tenSecInd,apogeeInd)]
print(alt_prediction[np.argmax([alt[ind] for ind in range(tenSecInd,apogeeInd)])])
#print(alt_prediction)
ax.plot(ts-10,alt_prediction,color=green,label="Airbrakes Algorithm Final Altitude Prediction")
#ax.plot(ts2-10,roll2,color=orange,alpha=0.5)
#ax.set_ylabel("Roll Angle (deg)")
ax.legend()
ax.set_ylim(4700,5600)
ax.grid()
ax.vlines(ts[-1]-10,*ax.get_ylim(),color='k',linestyle='dotted')
ax.set_ylabel("Altitude (m)")
"""
ax = axs[2]
ax.plot(ts-10,rollrate,color='purple')
ax.plot(ts2-10,rollrate2[apogeeInd:],color='purple',alpha=0.5)
ax.set_ylabel("Roll Rate (deg/s)")
ax.grid()
#ax.set_ylim(-50,50)

ax.vlines(ts0[apogeeInd]-10,*ax.get_ylim(),color='k',linestyle='dotted')
"""
ax.set_xlim(14,(ts[-1]-10)*1.05)
ax.set_xlabel("Flight Time (s)")
plt.tight_layout()
plt.savefig("plot_telemetry_{}.pdf".format(TELEM_FILE_TO_PROC))
plt.show()

#print(roll_out)


