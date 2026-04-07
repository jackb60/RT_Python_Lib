import numpy as np
import matplotlib.pyplot as plt
import scipy.optimize as spopt
plt.rcParams['text.usetex'] = True
import pandas as pd

dat = pd.read_csv('../telemetry/GYRO_CALCS.csv')
ts = dat['timestamp']
rs = np.array(dat["roll_gyro_int"])
ps = np.array(dat["pitch_gyro_int"])
ys = np.array(dat["yaw_gyro_int"])

def line(x,m,b):
	return m*x + b

optr,pcovr = spopt.curve_fit(line,ts,rs)
optp,pcovp = spopt.curve_fit(line,ts,ps)
opty,pcovy = spopt.curve_fit(line,ts,ys)
mr = optr[0]
mp = optp[0]
my = opty[0]
print("Slope for roll: {}".format(mr))
print("Slope for pitch: {}".format(mp))
print("Slope for yaw: {}".format(my))




plt.plot(ts,rs,color='red',label='Roll Data')
plt.plot(ts,line(ts,*optr),color='red',lw=5,alpha=0.2,label='Roll Fit')
plt.plot(ts,ps,color='blue',label='Pitch Data')
plt.plot(ts,line(ts,*optp),color='blue',lw=5,alpha=0.2,label='Pitch Fit')
plt.plot(ts,ys,color='green',label='Yaw Data')
plt.plot(ts,line(ts,*opty),color='green',lw=5,alpha=0.2,label='Yaw Fit')
plt.xlabel("Time (s)")


plt.ylabel("Value")
plt.title("\\sc Plot of Roll/Pitch/Yaw Linear Fits")
plt.savefig('roll_pitch_yaw_fit.pdf')

