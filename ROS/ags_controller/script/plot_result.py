import sys
import rosbag
import matplotlib.pyplot as plt

# Read data from bag
bag = rosbag.Bag(sys.argv[1])


# State published by the controller
controllerState_time = []
controllerState_x = []
controllerState_y = []
controllerState_theta = []
controllerState_vx = []
controllerState_vy = []
controllerState_sideslip = []
controllerState_yawrate = []
controllerState_longVel = []
controllerState_speed = []
controllerState_steer = []

# State published by the simulator
carState_time = []
carState_x = []
carState_y = []
carState_theta = []
carState_vy = []
carState_ay = []
carState_sideslip = []
carState_yawrate = []
carState_slipFront = []
carState_slipRear = []
carState_forceFront = []
carState_forceRear = []
carState_speedAct = []
carState_steerAct = []

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
            controllerState_time.append(msg.data[0])
            controllerState_x.append(msg.data[1])
            controllerState_y.append(msg.data[2])
            controllerState_theta.append(msg.data[3])
            controllerState_vx.append(msg.data[4])
            controllerState_vy.append(msg.data[5])
            controllerState_sideslip.append(msg.data[6])
            controllerState_yawrate.append(msg.data[7])
            controllerState_longVel.append(msg.data[8])
            controllerState_speed.append(msg.data[9])
            controllerState_steer.append(msg.data[10])
        if topic == "/car/state":
            carState_time.append(msg.data[0])
            carState_x.append(msg.data[1])
            carState_y.append(msg.data[2])
            carState_theta.append(msg.data[3])
            carState_yawrate.append(msg.data[4])
            carState_vy.append(msg.data[5])
            carState_ay.append(msg.data[6])
            carState_sideslip.append(msg.data[7])
            carState_slipFront.append(msg.data[8])
            carState_slipRear.append(msg.data[9])
            carState_forceFront.append(msg.data[10])
            carState_forceRear.append(msg.data[11])
            carState_speedAct.append(msg.data[12])
            carState_steerAct.append(msg.data[13])
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
plt.plot(controllerState_time,controllerState_steer, label="ref")
plt.plot(carState_time,carState_steerAct, label="act")
plt.xlabel("Time [s]")
plt.ylabel("Steer [rad]")
plt.legend()
plt.subplot(212)
plt.plot(controllerState_time,controllerState_speed, label="ref")
plt.plot(carState_time,carState_speedAct, label="act")
plt.xlabel("Time [s]")
plt.ylabel("Speed ref [m/s]")
plt.legend()

plt.figure(3)
plt.plot(carState_time,carState_yawrate)
plt.xlabel("Time [s]")
plt.ylabel("Yaw rate [rad/s]")

plt.figure(4)
plt.plot(controllerState_time,controllerState_sideslip, label="controller")
plt.plot(carState_time,carState_sideslip, label="simulator")
plt.xlabel("Time [s]")
plt.ylabel("Sideslip [rad]")
plt.legend()

plt.figure(5)
plt.plot(controllerState_time,controllerState_longVel, label="controller")
plt.plot(carState_time,carState_speedAct, label="simulator")
plt.xlabel("Time [s]")
plt.ylabel("Longitudinal velocity [m/s]")
plt.legend()

plt.show()

