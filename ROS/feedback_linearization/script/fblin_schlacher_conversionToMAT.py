import sys
import rosbag
import numpy as np
import scipy.io as sio
import matplotlib.pyplot as plt

# Extract file name from input
filename = sys.argv[1]
filename = filename[:-4]

# Read data from bag
bag = rosbag.Bag(filename+'.bag')

storeData = 0

vehicleState_time = []
vehicleState_x = []
vehicleState_y = []
vehicleState_theta = []
vehicleState_vx = []
vehicleState_vy = []
vehicleState_v = []
vehicleState_ay = []
vehicleState_sideslip = []
vehicleState_yawrate = []
controllerState_time = []
controllerState_w1 = []
controllerState_w2 = []
controllerState_Fxr = []
controllerState_steer = []
controllerState_z1 = []
controllerState_z2 = []
controllerState_vx = []
controllerState_vy = []
controllerState_sideslip = []
controllerState_yawrate = []

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
            controllerState_vx.append(msg.data[4])
            controllerState_vy.append(msg.data[5])
            controllerState_sideslip.append(msg.data[6])
            controllerState_yawrate.append(msg.data[7])
        if topic == "/car/state":
            vehicleState_time.append(msg.data[0])
            vehicleState_x.append(msg.data[1])
            vehicleState_y.append(msg.data[2])
            vehicleState_theta.append(msg.data[3])
            vehicleState_yawrate.append(msg.data[4])
            vehicleState_vx.append(msg.data[5])
            vehicleState_vy.append(msg.data[6])
            vehicleState_v.append(pow(pow(msg.data[4],2.0)+pow(msg.data[5],2.0),0.5))
            vehicleState_ay.append(msg.data[7])
            vehicleState_sideslip.append(msg.data[8])

bag.close()

# Save data on mat file
sio.savemat(filename+'.mat', {'car_t': vehicleState_time, 'car_x': vehicleState_x, 'car_y': vehicleState_y, 'car_theta': vehicleState_theta, 
    'car_yawrate': vehicleState_yawrate, 'car_vx': vehicleState_vx, 'car_vy': vehicleState_vy, 'car_v': vehicleState_v, 'car_ay': vehicleState_ay,
    'car_sideslip': vehicleState_sideslip, 'control_t': controllerState_time, 'control_w1': controllerState_w1, 'control_w2': controllerState_w2,
    'control_Fxr': controllerState_Fxr, 'control_steer': controllerState_steer, 'control_vx': controllerState_vx, 'control_vy': controllerState_vy,
    'control_sideslip': controllerState_sideslip, 'control_yawrate': controllerState_yawrate})

