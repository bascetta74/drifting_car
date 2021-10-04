import sys
import rosbag
import matplotlib.pyplot as plt

# Read data from bag
bag = rosbag.Bag(sys.argv[1])

# State published by the controller
vehicleState_time = []
vehicleState_x = []
vehicleState_y = []
vehicleState_theta = []
vehicleState_vx = []
vehicleState_vy = []
vehicleState_ay = []
vehicleState_sideslip = []
vehicleState_slipFront = []
vehicleState_slipRear = []
vehicleState_forceFront = []
vehicleState_forceRear = []
vehicleState_yawrate = []
vehicleState_speed = []
vehicleState_steer = []

# Controller commands
carcmd_time = []
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
        if topic == "/car/state":
            vehicleState_time.append(msg.data[0])
            vehicleState_x.append(msg.data[1])
            vehicleState_y.append(msg.data[2])
            vehicleState_theta.append(msg.data[3])
            vehicleState_yawrate.append(msg.data[4])
            vehicleState_vx.append(msg.data[5])
            vehicleState_vy.append(msg.data[6])
            vehicleState_ay.append(msg.data[7])
            vehicleState_sideslip.append(msg.data[8])
            vehicleState_slipFront.append(msg.data[9])
            vehicleState_slipRear.append(msg.data[10])
            vehicleState_forceFront.append(msg.data[11])
            vehicleState_forceRear.append(msg.data[12])
            vehicleState_speed.append(msg.data[13])
            vehicleState_steer.append(msg.data[14])
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
plt.plot(vehicleState_time,vehicleState_steer)
plt.xlabel("Time [s]")
plt.ylabel("Steer act [rad]")
plt.subplot(212)
plt.plot(vehicleState_time,vehicleState_speed)
plt.xlabel("Time [s]")
plt.ylabel("Speed act [m/s]")

plt.figure(3)
plt.subplot(211)
plt.plot(carcmd_steer)
plt.xlabel("Time [s]")
plt.ylabel("Steer cmd [rad]")
plt.subplot(212)
plt.plot(carcmd_speed)
plt.xlabel("Time [s]")
plt.ylabel("Speed cmd [m/s]")

plt.show()

