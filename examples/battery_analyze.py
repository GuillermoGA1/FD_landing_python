
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
BATTERY = []


txt_file = "battery_hover_20210503-160938"
with open("/home/guillermoga/office/" + txt_file  + ".txt") as f:
    for line in f:
        values = line.split()
        values.pop(0)        
        if "controller.actuatorThrust" in line:
            thrust = values[1].replace(',', '')
            thrust_tar = values[3].replace('}', '')
            THRUST.append(float(thrust))
            THRUST_TAR.append(float(thrust_tar))

        if "pm.vbat" in line:
            voltage = values[1].replace('}', '')
            BATTERY.append(float(voltage))

print(len(THRUST),len(THRUST_TAR), len(BATTERY))

BATTERY.pop(-1)
i = 4
while i<=len(THRUST):
    avg = (THRUST[i]+THRUST[i-1]+THRUST[i-2]+THRUST[i-3]+THRUST[i-4])/5
    avg2 = (THRUST_TAR[i]+THRUST_TAR[i-1]+THRUST_TAR[i-2]+THRUST_TAR[i-3]+THRUST_TAR[i-4])/5
    Y.append(avg)
    Z.append(avg2)
    i = i+5 
    
THRUST = Y
THRUST_TAR = Z

print(len(THRUST),len(THRUST_TAR), len(BATTERY), len(X))
#print(len(X), len(Y), len(Z), len(VX), len(VY), len(VZ), len(ROLL), len(PITCH), len(YAW))
#print(len(X_TAR), len(Y_TAR), len(Z_TAR), len(VX_TAR), len(VY_TAR), len(VZ_TAR), len(ROLL_TAR), len(PITCH_TAR), len(YAW_TAR))

if create_scv:
    with open("/home/guillermoga/Desktop/csv_files/" + txt_file + ".csv", mode='w') as csv_file:
        writer = csv.writer(csv_file, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
        writer.writerow(["Thrust", "Target Thrust", "Battery"])

        for i in range(len(THRUST)):
            writer.writerow([THRUST[i],THRUST_TAR[i], BATTERY[i]])
                         

plt.figure(3)
plt.plot(BATTERY)          
plt.grid()

plt.figure(5)
plt.plot(THRUST)
plt.plot(THRUST_TAR)
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













