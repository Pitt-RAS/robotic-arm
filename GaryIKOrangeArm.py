import numpy as np
from ikpy.chain import Chain
from ikpy.link import DHLink



Base = DHLink(name="Base", d=0, a=0, alpha=-1.57079633, theta=1.57079633, bounds = (-1.57079633, 1.57079633))
Shoulder = DHLink(name="Shoulder", d=0, a=48, alpha=0, theta=4.71238899, bounds=(-1.57079633, 1.57079633))
Elbow = DHLink(name="Elbow", d=0, a=47, alpha=0, theta=0, bounds=(-1.57079633, 1.57079633))
End = DHLink(name="End", d=0, a=80, alpha=0, theta=1.57079633, bounds=(-1.57079633, 1.57079633))

robot_arm = Chain(name = "Robot", links = [Base, Shoulder, Elbow, End])
#print(robot_arm.forward_kinematics([0, 1.57, 0, 0])[:3, 3])

format_float = np.vectorize(lambda x: "%.6f" % x)
radVals = format_float(robot_arm.inverse_kinematics((0, 75, 75))) #Get rad values out of scientific notation
radVals = np.array(radVals, dtype=float) #Turn this into an np array
degVals = 57.2957795*radVals #Turn rad to deg
degVals = degVals+90 #Add 90 to make up for the 90 removed in the DH paramaters 
degVals = np.round(degVals).astype(int) #Cast to int
print(degVals)
