
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
import csv


create_scv = True
# reading the data from the file

X = []
Y = []
Z = []
X_TAR = []
Y_TAR = []
Z_TAR = []
VX = []
VY = []
VZ = []
VX_TAR = []
VY_TAR = []
VZ_TAR = []
ROLL = []
PITCH = []
YAW = []
ROLL_TAR = []
PITCH_TAR = []
YAW_TAR = []
THRUST = []
THRUST_TAR = []
THRUST_BASE = []
BATTERY = []


txt_file = "z_good_step2"
with open("/home/guillermoga/validation/Good/" + txt_file  + ".txt") as f:
    for line in f:
        values = line.split()
        values.pop(0)
        if "stateEstimate.x" in line:
            x = values[1].replace(',', '')
            y = values[3].replace(',', '')
            z = values[5].replace('}', '')
            X.append(float(x))
            Y.append(float(y))
            Z.append(float(z))

        if "stateEstimate.vx" in line:
            vx = values[1].replace(',', '')
            vy = values[3].replace(',', '')
            vz = values[5].replace('}', '')
            VX.append(float(vx))
            VY.append(float(vy))
            VZ.append(float(vz))

        if "stateEstimate.roll" in line:
            roll = values[1].replace(',', '')
            pitch = values[3].replace(',', '')
            yaw = values[5].replace('}', '')
            ROLL.append(float(roll))
            PITCH.append(float(pitch))
            YAW.append(float(yaw))


        if "controller.roll" in line and ("controller.rollRate" not in line):
            roll = values[1].replace(',', '')
            pitch = values[3].replace(',', '')
            yaw = values[5].replace('}', '')
            ROLL_TAR.append(float(roll))
            PITCH_TAR.append(float(pitch))
            YAW_TAR.append(float(yaw))

        if "posCtl.targetX" in line: #posCtl.targetX or ctrltarget.x
            x = values[1].replace(',', '')
            y = values[3].replace(',', '')
            z = values[5].replace('}', '')
            X_TAR.append(float(x))
            Y_TAR.append(float(y))
            Z_TAR.append(float(z))

        if "posCtl.targetVX" in line: #posCtl.targetX or ctrltarget.x
            vx = values[1].replace(',', '')
            vy = values[3].replace(',', '')
            vz = values[5].replace(',', '')
            t_base = values[7].replace('}', '')
            VX_TAR.append(float(vx))
            VY_TAR.append(float(vy))
            VZ_TAR.append(float(vz))
            THRUST_BASE.append(float(t_base))
        
        if "controller.actuatorThrust" in line:
            thrust = values[1].replace(',', '')
            thrust_tar = values[3].replace('}', '')
            THRUST.append(float(thrust))
            THRUST_TAR.append(float(thrust_tar))

        if "pm.vbat" in line:
            voltage = values[1].replace('}', '')
            BATTERY.append(float(voltage))


print(len(X), len(Y), len(Z), len(VX), len(VY), len(VZ), len(ROLL), len(PITCH), len(YAW))
print(len(X_TAR), len(Y_TAR), len(Z_TAR), len(VX_TAR), len(VY_TAR), len(VZ_TAR), len(ROLL_TAR), len(PITCH_TAR), len(YAW_TAR))
print(len(THRUST),len(BATTERY))

if create_scv:
    with open("/home/guillermoga/validation/Good/" + txt_file + ".csv", mode='w') as csv_file:
        writer = csv.writer(csv_file, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
        writer.writerow(["X", "Y", "Z", "VX", "VY", "VZ", "Roll", "Pitch", "Yaw", "Target X", "Target Y", "Target Z", "Target VX", "Target VY",
                        "Target VZ", "Target Roll", "Target Pitch", "Target Yaw", "Thrust", "Thrust base", "Target Thrust", "Battery"])

        for i in range(len(X_TAR)):
            writer.writerow([X[i], Y[i], Z[i], VX[i], VY[i], VZ[i], ROLL[i], PITCH[i], YAW[i], X_TAR[i], Y_TAR[i], Z_TAR[i],
                          VX_TAR[i], VY_TAR[i], VZ_TAR[i], ROLL_TAR[i], PITCH_TAR[i], YAW_TAR[i], THRUST[i], THRUST_BASE[i], THRUST_TAR[i], BATTERY[i]])
                         



'''
VX2 = []
VY2 = []
VZ2 = []

for i in range(1,len(X)):
    vx2 = (X[i] - X[i-1])/ 0.05
    VX2.append(vx2)

for i in range(1,len(Y)):
    vy2 = (Y[i] - Y[i-1])/ 0.05
    VY2.append(vy2)

for i in range(1,len(X)):
    vz2 = (Z[i] - Z[i-1])/ 0.05
    VZ2.append(vz2)
'''


fig, axs = plt.subplots(3, 3)
axs[0, 0].plot(X)
axs[0, 0].plot(X_TAR)
axs[0, 0].grid()
axs[0, 0].legend(["est", "cmd"])
axs[0, 0].set_title("Position X")
axs[1, 0].plot(Y)
axs[1, 0].plot(Y_TAR)
axs[1, 0].grid()
axs[1, 0].legend(["est", "cmd"])
axs[1, 0].set_title("Position Y")
axs[2, 0].plot(Z)
axs[2, 0].plot(Z_TAR)
axs[2, 0].grid()
axs[2, 0].legend(["est", "cmd"])
axs[2, 0].set_title("Position Z")
axs[0, 1].plot(VX)
axs[0, 1].plot(VX_TAR)
axs[0, 1].grid()
axs[0, 1].legend(["est", "cmd"])
axs[0, 1].set_title("Velocity X")
axs[1, 1].plot(VY)
axs[1, 1].plot(VY_TAR)
axs[1, 1].grid()
axs[1, 1].legend(["est", "cmd"])
axs[1, 1].set_title("Velocity Y")
axs[2, 1].plot(VZ)
axs[2, 1].plot(VZ_TAR)
axs[2, 1].grid()
axs[2, 1].legend(["est", "cmd"])
axs[2, 1].set_title("Velocity Z")
axs[0, 2].plot(PITCH)
axs[0, 2].plot(PITCH_TAR)
axs[0, 2].grid()
axs[0, 2].legend(["est", "cmd"])
axs[0, 2].set_title("Pitch")
axs[1, 2].plot(ROLL)
axs[1, 2].plot(ROLL_TAR)
axs[1, 2].grid()
axs[1, 2].legend(["est", "cmd"])
axs[1, 2].set_title("Roll")
axs[2, 2].plot(YAW)
axs[2, 2].plot(YAW_TAR)
axs[2, 2].grid()
axs[2, 2].legend(["est", "cmd"])
axs[2, 2].set_title("Yaw")


plt.figure(3)
plt.plot(BATTERY)          
plt.grid()

plt.figure(4)
plt.plot(X, Y)
plt.grid()

plt.figure(5)
plt.plot(THRUST)
plt.plot(THRUST_TAR)
plt.plot(THRUST_BASE)
plt.grid()
plt.legend(["est", "cmd"])

plt.figure(6)
plt.scatter(BATTERY, THRUST)


'''
fig2, axs2 = plt.subplots(1, 3)
axs2[0].plot(VX2)
axs2[0].plot(VX)
axs2[0].grid()
axs2[0].legend(["calculated","measured"])
axs2[0].set_title("Velocity X")
axs2[1].plot(VY2)
axs2[1].plot(VY)
axs2[1].grid()
axs2[1].legend(["calculated","measured"])
axs2[1].set_title("Velocity Y")
axs2[2].plot(VZ2)
axs2[2].plot(VZ)
axs2[2].grid()
axs2[2].legend(["calculated","measured"])
axs2[2].set_title("Velocity Z")
'''
plt.show()













