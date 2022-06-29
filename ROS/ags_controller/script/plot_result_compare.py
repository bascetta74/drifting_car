import sys
import rosbag
import matplotlib.pyplot as plt

from math import sin, cos

# Constants
autostart_delay = 2.5

# Read data from bag
bag_1 = rosbag.Bag(sys.argv[1])
bag_2 = rosbag.Bag(sys.argv[2])

# State published by the controller
controllerState_time_1 = []
controllerState_x_1 = []
controllerState_y_1 = []
controllerState_theta_1 = []
controllerState_vxAbs_1 = []
controllerState_vyAbs_1 = []
controllerState_sideslip_1 = []
controllerState_yawrate_1 = []
controllerState_longVel_1 = []
controllerState_speedRef_1 = []
controllerState_steerRef_1 = []
controllerState_FyfRef_1 = []
controllerState_time_2 = []
controllerState_x_2 = []
controllerState_y_2 = []
controllerState_theta_2 = []
controllerState_vxAbs_2 = []
controllerState_vyAbs_2 = []
controllerState_sideslip_2 = []
controllerState_yawrate_2 = []
controllerState_longVel_2 = []
controllerState_speedRef_2 = []
controllerState_steerRef_2 = []
controllerState_FyfRef_2 = []

# State published by the simulator
carState_time_1 = []
carState_x_1 = []
carState_y_1 = []
carState_theta_1 = []
carState_vxCar_1 = []
carState_vyCar_1 = []
carState_ay_1 = []
carState_sideslip_1 = []
carState_yawrate_1 = []
carState_slipFront_1 = []
carState_slipRear_1 = []
carState_FyFront_1 = []
carState_FyRear_1 = []
carState_speedAct_1 = []
carState_steerAct_1 = []
carState_vxAbs_1 = []
carState_vyAbs_1 = []
carState_time_2 = []
carState_x_2 = []
carState_y_2 = []
carState_theta_2 = []
carState_vxCar_2 = []
carState_vyCar_2 = []
carState_ay_2 = []
carState_sideslip_2 = []
carState_yawrate_2 = []
carState_slipFront_2 = []
carState_slipRear_2 = []
carState_FyFront_2 = []
carState_FyRear_2 = []
carState_speedAct_2 = []
carState_steerAct_2 = []
carState_vxAbs_2 = []
carState_vyAbs_2 = []

# Controller commands
carcmd_speed_1 = []
carcmd_steer_1 = []
carcmd_speed_2 = []
carcmd_steer_2 = []

# Vehicle measurements
carpose_x_1= []
carpose_y_1= []
carpose_theta_1= []
carimu_yawrate_1 = []
carpose_x_2 = []
carpose_y_2 = []
carpose_theta_2 = []
carimu_yawrate_2 = []

storeData = 0
for topic, msg, t in bag_1.read_messages():
    if topic == "/radio_cmd":
        if msg.state == 2:
            storeData = 1
        else:
            storeData = 0

    if storeData:
        if topic == "/controllerState":
            controllerState_time_1.append(msg.data[0]+autostart_delay)
            controllerState_x_1.append(msg.data[1])
            controllerState_y_1.append(msg.data[2])
            controllerState_theta_1.append(msg.data[3])
            controllerState_vxAbs_1.append(msg.data[4])
            controllerState_vyAbs_1.append(msg.data[5])
            controllerState_sideslip_1.append(msg.data[6])
            controllerState_yawrate_1.append(msg.data[7])
            controllerState_longVel_1.append(msg.data[8])
            controllerState_speedRef_1.append(msg.data[9])
            controllerState_steerRef_1.append(msg.data[10])
            controllerState_FyfRef_1.append(msg.data[11])
        if topic == "/car/state":
            carState_time_1.append(msg.data[0])
            carState_x_1.append(msg.data[1])
            carState_y_1.append(msg.data[2])
            carState_theta_1.append(msg.data[3])
            carState_yawrate_1.append(msg.data[4])
            carState_vxCar_1.append(msg.data[5])
            carState_vyCar_1.append(msg.data[6])
            carState_ay_1.append(msg.data[7])
            carState_sideslip_1.append(msg.data[8])
            carState_slipFront_1.append(msg.data[9])
            carState_slipRear_1.append(msg.data[10])
            carState_FyFront_1.append(msg.data[11])
            carState_FyRear_1.append(msg.data[12])
            carState_speedAct_1.append(msg.data[13])
            carState_steerAct_1.append(msg.data[14])
            carState_vxAbs_1.append(cos(msg.data[3])*msg.data[5]-sin(msg.data[3])*msg.data[6])
            carState_vyAbs_1.append(sin(msg.data[3])*msg.data[5]+cos(msg.data[3])*msg.data[6])
        if topic == "/car/ground_pose":
            carpose_x_1.append(msg.x)
            carpose_y_1.append(msg.y)
            carpose_theta_1.append(msg.theta)
        if topic == "/controller_cmd":
            carcmd_steer_1.append(msg.steer_ref)
            carcmd_speed_1.append(msg.speed_ref)
        if topic == "/imu/data":
            carimu_yawrate_1.append(msg.angular_velocity.z)

bag_1.close()

storeData = 0
for topic, msg, t in bag_2.read_messages():
    if topic == "/radio_cmd":
        if msg.state == 2:
            storeData = 1
        else:
            storeData = 0

    if storeData:
        if topic == "/controllerState":
            controllerState_time_2.append(msg.data[0]+autostart_delay)
            controllerState_x_2.append(msg.data[1])
            controllerState_y_2.append(msg.data[2])
            controllerState_theta_2.append(msg.data[3])
            controllerState_vxAbs_2.append(msg.data[4])
            controllerState_vyAbs_2.append(msg.data[5])
            controllerState_sideslip_2.append(msg.data[6])
            controllerState_yawrate_2.append(msg.data[7])
            controllerState_longVel_2.append(msg.data[8])
            controllerState_speedRef_2.append(msg.data[9])
            controllerState_steerRef_2.append(msg.data[10])
            controllerState_FyfRef_2.append(msg.data[11])
        if topic == "/car/state":
            carState_time_2.append(msg.data[0])
            carState_x_2.append(msg.data[1])
            carState_y_2.append(msg.data[2])
            carState_theta_2.append(msg.data[3])
            carState_yawrate_2.append(msg.data[4])
            carState_vxCar_2.append(msg.data[5])
            carState_vyCar_2.append(msg.data[6])
            carState_ay_2.append(msg.data[7])
            carState_sideslip_2.append(msg.data[8])
            carState_slipFront_2.append(msg.data[9])
            carState_slipRear_2.append(msg.data[10])
            carState_FyFront_2.append(msg.data[11])
            carState_FyRear_2.append(msg.data[12])
            carState_speedAct_2.append(msg.data[13])
            carState_steerAct_2.append(msg.data[14])
            carState_vxAbs_2.append(cos(msg.data[3])*msg.data[5]-sin(msg.data[3])*msg.data[6])
            carState_vyAbs_2.append(sin(msg.data[3])*msg.data[5]+cos(msg.data[3])*msg.data[6])
        if topic == "/car/ground_pose":
            carpose_x_2.append(msg.x)
            carpose_y_2.append(msg.y)
            carpose_theta_2.append(msg.theta)
        if topic == "/controller_cmd":
            carcmd_steer_2.append(msg.steer_ref)
            carcmd_speed_2.append(msg.speed_ref)
        if topic == "/imu/data":
            carimu_yawrate_2.append(msg.angular_velocity.z)

bag_2.close()

# Plot data
plt.figure(1)
plt.plot(carpose_x_1,carpose_y_1, label="bag 1")
plt.plot(carpose_x_2,carpose_y_2,'--', label="bag 2")
plt.xlabel("x [m]")
plt.ylabel("y [m]")
plt.legend()

plt.figure(2)
plt.subplot(211)
plt.plot(controllerState_time_1,controllerState_steerRef_1, label="bag 1")
plt.plot(controllerState_time_2,controllerState_steerRef_2,'--', label="bag 2")
plt.xlabel("Time [s]")
plt.ylabel("Steer [rad]")
plt.legend()
plt.subplot(212)
plt.plot(controllerState_time_1,controllerState_speedRef_1, label="bag 1")
plt.plot(controllerState_time_1,controllerState_speedRef_1,'--', label="bag 2")
plt.xlabel("Time [s]")
plt.ylabel("Speed ref [m/s]")
plt.legend()

plt.figure(3)
plt.plot(controllerState_time_1,controllerState_FyfRef_1, label="reference")
plt.plot(carState_time_1,carState_FyFront_1, label="actual - bag 1")
plt.plot(carState_time_2,carState_FyFront_2,'--', label="actual - bag 2")
plt.xlabel("Time [s]")
plt.ylabel("Front lateral force [N]")
plt.legend()

plt.figure(4)
plt.subplot(311)
plt.plot(carState_time_1,carState_x_1, label="bag 1")
plt.plot(carState_time_2,carState_x_2,'--', label="bag 2")
plt.xlabel("Time [s]")
plt.ylabel("x [m]")
plt.legend()
plt.subplot(312)
plt.plot(carState_time_1,carState_y_1, label="bag 1")
plt.plot(carState_time_2,carState_y_2,'--', label="bag 2")
plt.xlabel("Time [s]")
plt.ylabel("y [m]")
plt.legend()
plt.subplot(313)
plt.plot(carState_time_1,carState_theta_1, label="bag 1")
plt.plot(carState_time_2,carState_theta_2,'--', label="bag 2")
plt.xlabel("Time [s]")
plt.ylabel("Theta [rad]")
plt.legend()

plt.figure(5)
plt.plot(carState_time_1,carState_yawrate_1, label="bag 1")
plt.plot(carState_time_2,carState_yawrate_2,'--', label="bag 2")
plt.xlabel("Time [s]")
plt.ylabel("Yaw rate [rad/s]")
plt.legend()

plt.figure(6)
plt.plot(carState_time_1,carState_sideslip_1, label="bag 1")
plt.plot(carState_time_2,carState_sideslip_2,'--', label="bag 2")
plt.xlabel("Time [s]")
plt.ylabel("Sideslip [rad]")
plt.legend()

plt.figure(7)
plt.plot(carState_time_1,carState_speedAct_1, label="bag 1")
plt.plot(carState_time_2,carState_speedAct_2,'--', label="bag 2")
plt.xlabel("Time [s]")
plt.ylabel("Longitudinal velocity [m/s]")
plt.legend()

plt.show(block=False)
raw_input("Press Enter to continue...")
