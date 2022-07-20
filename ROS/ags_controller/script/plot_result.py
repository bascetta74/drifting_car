import sys
import rosbag
import matplotlib.pyplot as plt

from math import sin, cos
from scipy.io import savemat

# Constants
autostart_delay = 2.5

# Read data from bag
bag = rosbag.Bag(sys.argv[1])

# State published by the controller
controllerState_time = []
controllerState_x = []
controllerState_y = []
controllerState_theta = []
controllerState_vxAbs = []
controllerState_vyAbs = []
controllerState_sideslip = []
controllerState_yawrate = []
controllerState_longVel = []
controllerState_speedRef = []
controllerState_steerRef = []
controllerState_FyfRef = []

# State published by the simulator
carState_time = []
carState_x = []
carState_y = []
carState_theta = []
carState_vxCar = []
carState_vyCar = []
carState_ay = []
carState_sideslip = []
carState_yawrate = []
carState_slipFront = []
carState_slipRear = []
carState_FyFront = []
carState_FyRear = []
carState_speedAct = []
carState_steerAct = []
carState_vxAbs = []
carState_vyAbs = []

# Controller commands
carcmd_speed = []
carcmd_steer = []

# Vehicle measurements
carpose_x = []
carpose_y = []
carpose_theta = []
carimu_yawrate = []

storeData = 0
for topic, msg, t in bag.read_messages():
    if topic == "/radio_cmd":
        if msg.state == 2:
            storeData = 1
        else:
            storeData = 0

    if storeData:
        if topic == "/controllerState":
            controllerState_time.append(msg.data[0]+autostart_delay)
            controllerState_x.append(msg.data[1])
            controllerState_y.append(msg.data[2])
            controllerState_theta.append(msg.data[3])
            controllerState_vxAbs.append(msg.data[4])
            controllerState_vyAbs.append(msg.data[5])
            controllerState_sideslip.append(msg.data[6])
            controllerState_yawrate.append(msg.data[7])
            controllerState_longVel.append(msg.data[8])
            controllerState_speedRef.append(msg.data[9])
            controllerState_steerRef.append(msg.data[10])
            controllerState_FyfRef.append(msg.data[11])
        if topic == "/car/state":
            carState_time.append(msg.data[0])
            carState_x.append(msg.data[1])
            carState_y.append(msg.data[2])
            carState_theta.append(msg.data[3])
            carState_yawrate.append(msg.data[4])
            carState_vxCar.append(msg.data[5])
            carState_vyCar.append(msg.data[6])
            carState_ay.append(msg.data[7])
            carState_sideslip.append(msg.data[8])
            carState_slipFront.append(msg.data[9])
            carState_slipRear.append(msg.data[10])
            carState_FyFront.append(msg.data[11])
            carState_FyRear.append(msg.data[12])
            carState_speedAct.append(msg.data[13])
            carState_steerAct.append(msg.data[14])
            carState_vxAbs.append(cos(msg.data[3])*msg.data[5]-sin(msg.data[3])*msg.data[6])
            carState_vyAbs.append(sin(msg.data[3])*msg.data[5]+cos(msg.data[3])*msg.data[6])
        if topic == "/car/ground_pose":
            carpose_x.append(msg.x)
            carpose_y.append(msg.y)
            carpose_theta.append(msg.theta)
        if topic == "/controller_cmd":
            carcmd_steer.append(msg.steer_ref)
            carcmd_speed.append(msg.speed_ref)
        if topic == "/imu/data":
            carimu_yawrate.append(msg.angular_velocity.z)

bag.close()

# Plot data
plt.figure(1)
plt.plot(carpose_x,carpose_y)
plt.xlabel("x [m]")
plt.ylabel("y [m]")

plt.figure(2)
plt.subplot(211)
plt.plot(controllerState_time,controllerState_steerRef, label="ref")
plt.plot(carState_time,carState_steerAct,'--', label="act")
plt.xlabel("Time [s]")
plt.ylabel("Steer [rad]")
plt.legend()
plt.subplot(212)
plt.plot(controllerState_time,controllerState_speedRef, label="ref")
plt.plot(carState_time,carState_speedAct,'--', label="act")
plt.xlabel("Time [s]")
plt.ylabel("Speed [m/s]")
plt.legend()

plt.figure(3)
plt.plot(controllerState_time,controllerState_FyfRef, label="reference")
plt.plot(carState_time,carState_FyFront,'--', label="actual")
plt.xlabel("Time [s]")
plt.ylabel("Front lateral force [N]")
plt.legend()

plt.figure(4)
plt.subplot(311)
plt.plot(controllerState_time,controllerState_x, label="controller")
plt.plot(carState_time,carState_x,'--', label="simulator")
plt.xlabel("Time [s]")
plt.ylabel("x [m]")
plt.legend()
plt.subplot(312)
plt.plot(controllerState_time,controllerState_y, label="controller")
plt.plot(carState_time,carState_y,'--', label="simulator")
plt.xlabel("Time [s]")
plt.ylabel("y [m]")
plt.legend()
plt.subplot(313)
plt.plot(controllerState_time,controllerState_theta, label="controller")
plt.plot(carState_time,carState_theta,'--', label="simulator")
plt.xlabel("Time [s]")
plt.ylabel("Theta [rad]")
plt.legend()

plt.figure(5)
plt.subplot(211)
plt.plot(controllerState_time,controllerState_vxAbs, label="controller")
plt.plot(carState_time,carState_vxAbs,'--', label="simulator")
plt.xlabel("Time [s]")
plt.ylabel("Longitudinal velocity [m/s]")
plt.legend()
plt.subplot(212)
plt.plot(controllerState_time,controllerState_vyAbs, label="controller")
plt.plot(carState_time,carState_vyAbs,'--', label="simulator")
plt.xlabel("Time [s]")
plt.ylabel("Lateral velocity [m/s]")
plt.legend()

plt.figure(6)
plt.plot(carState_time,carState_yawrate)
plt.xlabel("Time [s]")
plt.ylabel("Yaw rate [rad/s]")

plt.figure(7)
plt.plot(controllerState_time,controllerState_sideslip, label="controller")
plt.plot(carState_time,carState_sideslip, label="simulator")
plt.xlabel("Time [s]")
plt.ylabel("Sideslip [rad]")
plt.legend()

plt.figure(8)
plt.plot(controllerState_time,controllerState_longVel, label="controller")
plt.plot(carState_time,carState_speedAct, label="simulator")
plt.xlabel("Time [s]")
plt.ylabel("Longitudinal velocity [m/s]")
plt.legend()

plt.show(block=False)
input("Press Enter to continue...")

# Write data as mat file
if (len(sys.argv)>2):
    savemat(sys.argv[1]+".mat",{"ctrl_t": controllerState_time, "ctrl_x": controllerState_x, "ctrl_y": controllerState_y, 
    "ctrl_theta": controllerState_theta, "ctrl_vxAbs": controllerState_vxAbs, "ctrl_vyAbs": controllerState_vyAbs, 
    "ctrl_beta": controllerState_sideslip, 
    "sim_t": carState_time, "sim_x": carState_x, "sim_y": carState_y, "sim_theta": carState_theta, "sim_vxCar": carState_vxCar,
    "sim_vyCar": carState_vyCar, "sim_vxAbs": carState_vxAbs, "sim_vyAbs": carState_vyAbs, "sim_beta": carState_sideslip});


