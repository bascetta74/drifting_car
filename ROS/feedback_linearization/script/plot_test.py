import sys
import rosbag
import numpy as np
from scipy.integrate import odeint
from scipy import interpolate
import matplotlib.pyplot as plt

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
controllerState_z1ref = []
controllerState_z2ref = []
vehicleState_time = []
vehicleState_x = []
vehicleState_y = []
vehicleState_theta = []
vehicleState_vx = []
vehicleState_vy = []
vehicleState_V = []
vehicleState_sideslip = []
vehicleState_yawrate = []

carState_time = []
carState_yawrate = []
carState_v = []
carState_sideslip = []

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
            controllerState_z1ref.append(msg.data[7])
            controllerState_z2ref.append(msg.data[8])
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
        if topic == "/car/state":
            carState_time.append(msg.data[0])
            carState_yawrate.append(msg.data[4])
            carState_v.append(msg.data[5])
            carState_sideslip.append(msg.data[8])

bag.close()


# plot results
plt.figure(1)
plt.subplot(2,1,1)
plt.plot(controllerState_time,controllerState_z1, label='actual')
plt.plot(controllerState_time,controllerState_z1ref, label='reference')
plt.xlabel('Time [s]')
plt.ylabel('vxp [m/s]')
plt.subplot(2,1,2)
plt.plot(controllerState_time,controllerState_z2, label='actual')
plt.plot(controllerState_time,controllerState_z2ref, label='reference')
plt.xlabel('Time [s]')
plt.ylabel('vyp [m/s]')
plt.legend(loc='best')

#plt.figure(1)
#plt.plot(vehicleState_time,vehicleState_V, label='fb')
#plt.plot(carState_time,carState_v, '--', label='sim')
#plt.xlabel('Time [s]')
#plt.ylabel('Velocity')
#plt.legend(loc='best')

#plt.figure(2)
#plt.plot(vehicleState_time,vehicleState_sideslip, label='fb')
#plt.plot(carState_time,carState_sideslip, '--', label='sim')
#plt.xlabel('Time [s]')
#plt.ylabel('Inputs')
#plt.legend(loc='best')

#plt.figure(3)
#plt.plot(vehicleState_time,vehicleState_yawrate, label='fb')
#plt.plot(carState_time,carState_yawrate, '--', label='sim')
#plt.xlabel('Time [s]')
#plt.ylabel('Yaw rate')
#plt.legend(loc='best')

#plt.figure(4)
#plt.subplot(2,1,1)
#plt.plot(controllerState_time,controllerState_Fxr)
#plt.xlabel('Time [s]')
#plt.ylabel('Fxr')
#plt.subplot(2,1,2)
#plt.plot(controllerState_time,controllerState_steer)
#plt.xlabel('Time [s]')
#plt.ylabel('Steer')

plt.show()

