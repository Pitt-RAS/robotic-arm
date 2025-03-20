import numpy as np
from ikpy.utils.plot import plot_chain
import matplotlib.pyplot as plt
import ikpy.utils.plot as plot_utils
from ikpy.chain import Chain
from ikpy.link import OriginLink
from ikpy.link import Link
from ikpy.link import DHLink
from ikpy.link import URDFLink



Base = DHLink(name="Base", d=10, a=0, alpha=-1.57079633, theta=0, bounds = (0, 3.1415926535))
Shoulder = DHLink(name="Shoulder", d=0, a=48, alpha=0, theta=-1.57079633, bounds=(0, 3.1415926535))
Elbow = DHLink(name="Elbow", d=0, a=47, alpha=0, theta=0, bounds=(0, 3.1415926535))
End = DHLink(name="End", d=0, a=80, alpha=0, theta=0, bounds=(0, 3.1415926535))

robot_arm = Chain(name = "Robot", links = [Base, Shoulder, Elbow, End])
print(robot_arm.forward_kinematics([0, 0, 0, 0])[:3, 3])
