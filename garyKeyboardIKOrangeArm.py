import numpy as np
from ikpy.chain import Chain
from ikpy.link import DHLink
import serial.tools.list_ports
from pynput import keyboard

ports = serial.tools.list_ports.comports()
serialInst = serial.Serial()
portsList = []

end = False

for one in ports:
    portsList.append(str(one))
    print(str(one))

serialInst.baudrate = 115200
serialInst.port = "/dev/cu.usbserial-1120"
serialInst.open()


Base = DHLink(name="Base", d=0, a=0, alpha=-1.57079633, theta=1.57079633, bounds = (-1.57079633, 1.57079633))
Shoulder = DHLink(name="Shoulder", d=0, a=48, alpha=0, theta=4.71238899, bounds=(-1.57079633, 1.57079633))
Elbow = DHLink(name="Elbow", d=0, a=47, alpha=0, theta=0, bounds=(-1.57079633, 1.57079633))
End = DHLink(name="End", d=0, a=80, alpha=0, theta=1.57079633, bounds=(-1.57079633, 1.57079633))

robot_arm = Chain(name = "Robot", links = [Base, Shoulder, Elbow, End])

Base.name = "Base"
Base.bounds = (-1.57079633, 1.57079633)
Shoulder.name = "Shoulder"
Shoulder.bounds = (-1.57079633, 1.57079633)
Elbow.name = "Elbow"
Elbow.bounds = (-1.57079633, 1.57079633)
End.name = "End"
End.bounds = (-1.57079633, 1.57079633)

x = 0
y = 75
z = 75

format_float = np.vectorize(lambda x: "%.6f" % x)
radVals = format_float(robot_arm.inverse_kinematics((x, y, z))) #Get rad values out of scientific notation
print("Rad vals ", radVals)
radVals = np.array(radVals, dtype=float) #Turn this into an np array
degVals = 57.2957795*radVals #Turn rad to deg
degVals = degVals+90 #Add 90 to make up for the 90 removed in the DH paramaters 
degVals = np.round(degVals).astype(int) #Cast to int
print(degVals)

#print("Proceed? y/n")

#response = input()

#if response == "y":
#    for i in range(len(degVals)):
#        current = str(i) + str(degVals[i])
#        command = current + "\n"
#        serialInst.write(command.encode('utf-8'))

def on_key_release(key):
    if key == 'q':
        end = True
    elif key == 'a':
        x -= 1
    elif key == 'd':
        x += 1
    elif key == 'w':
        y += 1
    elif key == 's':
        y -= 1
    elif key == keyboard.Key.up:
        z += 1
    elif key == keyboard.Key.down:
        z -= 1
    print(x, y, z)


with keyboard.Listener(on_release=on_key_release) as listener:
    listener.join()