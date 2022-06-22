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
vehicleState_speedCmd = []
vehicleState_steerCmd = []

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
        vehicleState_speedCmd.append(msg.data[13])
        vehicleState_steerCmd.append(msg.data[14])
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

# Read data from txt
t = []
speed_ref = []
steer_ref = []
speed_cmd = []
steer_cmd = []
x = []
y = []
psi = []
ay = []
r = []
vy = []
beta = []
alphaf = []
alphar = []
Fyf = []
Fyr = []

f = open(sys.argv[2], 'r')

for line in f:
    line = line.strip()
    columns = line.split(";")
    
    t.append(float(columns[0]))
    speed_ref.append(float(columns[1]))
    steer_ref.append(float(columns[2]))
    speed_cmd.append(float(columns[3]))
    steer_cmd.append(float(columns[4]))
    x.append(float(columns[5]))
    y.append(float(columns[6]))
    psi.append(float(columns[7]))
    ay.append(float(columns[8]))
    r.append(float(columns[9]))
    vy.append(float(columns[10]))
    beta.append(float(columns[11]))
    alphaf.append(float(columns[12]))
    alphar.append(float(columns[13]))
    Fyf.append(float(columns[14]))
    Fyr.append(float(columns[15]))

f.close()


# Plot data
plt.figure(1)
plt.plot(carpose_x,carpose_y, label='ROS node')
plt.plot(x,y,'--', label='ODE sim')
plt.xlabel("x [m]")
plt.ylabel("y [m]")
plt.legend()

plt.figure(2)
plt.subplot(211)
plt.plot(vehicleState_time,vehicleState_speedCmd, label='ROS node')
plt.plot(t,speed_cmd,'--', label='ODE sim')
plt.xlabel("Time [s]")
plt.ylabel("Speed command [N]")
plt.legend()
plt.subplot(212)
plt.plot(vehicleState_time,vehicleState_steerCmd, label='ROS node')
plt.plot(t,steer_cmd,'--', label='ODE sim')
plt.xlabel("Time [s]")
plt.ylabel("Steer command [N]")
plt.legend()

plt.figure(3)
plt.plot(vehicleState_time,vehicleState_vy, label='ROS node')
plt.plot(t,vy,'--', label='ODE sim')
plt.xlabel("Time [s]")
plt.ylabel("Lateral velocity [m/s]")
plt.legend()

plt.figure(4)
plt.plot(vehicleState_time,vehicleState_yawrate, label='ROS node')
plt.plot(t,r,'--', label='ODE sim')
plt.xlabel("Time [s]")
plt.ylabel("Yaw rate [rad/s]")
plt.legend()

plt.figure(5)
plt.plot(vehicleState_time,vehicleState_sideslip, label='ROS node')
plt.plot(t,beta,'--', label='ODE sim')
plt.xlabel("Time [s]")
plt.ylabel("Sideslip [rad]")
plt.legend()

plt.figure(6)
plt.subplot(311)
plt.plot(vehicleState_time,vehicleState_x, label='ROS node')
plt.plot(t,x,'--', label='ODE sim')
plt.xlabel("Time [s]")
plt.ylabel("x [m]")
plt.legend()
plt.subplot(312)
plt.plot(vehicleState_time,vehicleState_y, label='ROS node')
plt.plot(t,y,'--', label='ODE sim')
plt.xlabel("Time [s]")
plt.ylabel("y [m]")
plt.legend()
plt.subplot(313)
plt.plot(vehicleState_time,vehicleState_theta, label='ROS node')
plt.plot(t,psi,'--', label='ODE sim')
plt.xlabel("Time [s]")
plt.ylabel("Heading [rad]")
plt.legend()

plt.figure(7)
plt.plot(vehicleState_time,vehicleState_ay, label='ROS node')
plt.plot(t,ay,'--', label='ODE sim')
plt.xlabel("Time [s]")
plt.ylabel("Lateral acceleration [m/s^2]")
plt.legend()

plt.figure(8)
plt.subplot(211)
plt.plot(vehicleState_time,vehicleState_slipFront, label='ROS node')
plt.plot(t,alphaf,'--', label='ODE sim')
plt.xlabel("Time [s]")
plt.ylabel("Front slip angle [rad]")
plt.legend()
plt.subplot(212)
plt.plot(vehicleState_time,vehicleState_slipRear, label='ROS node')
plt.plot(t,alphar,'--', label='ODE sim')
plt.xlabel("Time [s]")
plt.ylabel("Rear slip angle [rad]")
plt.legend()

plt.figure(9)
plt.subplot(211)
plt.plot(vehicleState_time,vehicleState_forceFront, label='ROS node')
plt.plot(t,Fyf,'--', label='ODE sim')
plt.xlabel("Time [s]")
plt.ylabel("Front lateral force [N]")
plt.legend()
plt.subplot(212)
plt.plot(vehicleState_time,vehicleState_forceRear, label='ROS node')
plt.plot(t,Fyr,'--', label='ODE sim')
plt.xlabel("Time [s]")
plt.ylabel("Rear lateral force [N]")
plt.legend()

plt.show(block=False)

raw_input("Press Enter to continue...")


