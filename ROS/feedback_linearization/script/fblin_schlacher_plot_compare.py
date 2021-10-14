import sys
import rosbag
import numpy as np
from scipy.integrate import odeint
from scipy import interpolate
import matplotlib.pyplot as plt

# function that returns dy/dt
def integrator_chain(y,t):
    Vxp, dVyp, Vyp = y
    dydt = [w1(t), w2(t), dVyp]
    return dydt

# Read data from bag
bag = rosbag.Bag(sys.argv[1])

# State published
controllerState_time = []
controllerState_w1 = []
controllerState_w2 = []
controllerState_Fxr = []
controllerState_steer = []
controllerState_z1 = []
controllerState_z2 = []
vehicleState_time = []
vehicleState_x = []
vehicleState_y = []
vehicleState_theta = []
vehicleState_vx = []
vehicleState_vy = []
vehicleState_V = []
vehicleState_sideslip = []
vehicleState_yawrate = []

storeData = 0
for topic, msg, t in bag.read_messages():
    if topic == "/radio_cmd":
        if msg.state == 2:
            storeData = 1
        else:
            storeData = 0

    if storeData:
        if topic == "/feedback_linearization/controllerState":
            controllerState_time.append(msg.data[0])
            controllerState_w1.append(msg.data[1])
            controllerState_w2.append(msg.data[2])
            controllerState_Fxr.append(msg.data[3])
            controllerState_steer.append(msg.data[4])
            controllerState_z1.append(msg.data[5])
            controllerState_z2.append(msg.data[6])
        if topic == "/feedback_linearization/vehicleState":
            vehicleState_time.append(msg.data[0])
            vehicleState_x.append(msg.data[1])
            vehicleState_y.append(msg.data[2])
            vehicleState_theta.append(msg.data[3])
            vehicleState_vx.append(msg.data[4])
            vehicleState_vy.append(msg.data[5])
            vehicleState_V.append(pow(pow(msg.data[4],2.0)+pow(msg.data[5],2.0),0.5))
            vehicleState_sideslip.append(msg.data[6])
            vehicleState_yawrate.append(msg.data[7])

bag.close()

# initial condition
y0 = [vehicleState_vx[1], 0.0, 0.0]


# solve ODE
w1 = interpolate.interp1d(controllerState_time,controllerState_w1,kind='linear',axis=-1,copy=True,bounds_error=False,fill_value=(controllerState_w1[0],controllerState_w1[len(controllerState_w1)-1]))
w2 = interpolate.interp1d(controllerState_time,controllerState_w2,kind='linear',axis=-1,copy=True,bounds_error=False,fill_value=(controllerState_w2[0],controllerState_w2[len(controllerState_w2)-1]))
y = odeint(integrator_chain,y0,controllerState_time)

# plot results
plt.figure(1)
plt.plot(controllerState_time,y[:,0], label='nominal')
plt.plot(controllerState_time,controllerState_z1, label='actual', linestyle='dashed')
plt.xlabel('Time [s]')
plt.ylabel('Vxp')
plt.legend(loc='best')

plt.figure(2)
plt.plot(controllerState_time,y[:,2], label='nominal')
plt.plot(controllerState_time,controllerState_z2, label='actual', linestyle='dashed')
plt.xlabel('Time [s]')
plt.ylabel('Vyp')
plt.legend(loc='best')

plt.figure(3)
plt.plot(controllerState_time,controllerState_w1, label='w1')
plt.plot(controllerState_time,controllerState_w2, label='w2')
plt.xlabel('Time [s]')
plt.ylabel('Inputs')
plt.legend(loc='best')

plt.figure(4)
plt.subplot(211)
plt.plot(vehicleState_time,vehicleState_vx, label='vx')
plt.plot(vehicleState_time,vehicleState_vy, label='vy')
plt.xlabel('Time [s]')
plt.ylabel('Velocity [m/s]')
plt.legend(loc='best')
plt.subplot(212)
plt.plot(vehicleState_time,vehicleState_V)
plt.xlabel('Time [s]')
plt.ylabel('Absolute Velocity [m/s]')

plt.figure(5)
plt.subplot(211)
plt.plot(controllerState_time,controllerState_Fxr)
plt.xlabel('Time [s]')
plt.ylabel('Fxr [N]')
plt.subplot(212)
plt.plot(controllerState_time,controllerState_steer)
plt.xlabel('Time [s]')
plt.ylabel('Steer [rad]')

plt.figure(6)
plt.plot(vehicleState_time,vehicleState_yawrate)
plt.xlabel('Time [s]')
plt.ylabel('Yaw rate [rad/s]')

plt.figure(7)
plt.plot(vehicleState_time,vehicleState_sideslip)
plt.xlabel('Time [s]')
plt.ylabel('Sideslip [rad]')

plt.show()

