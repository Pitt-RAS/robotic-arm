import numpy as np
from ikpy.utils.plot import plot_chain
import matplotlib.pyplot as plt
import ikpy.utils.plot as plot_utils
from ikpy.chain import Chain
from ikpy.link import OriginLink
from ikpy.link import URDFLink



# Define the robotic arm as a kinematic chain
robotic_arm = Chain(name="RoboticArm", links=[
    # Base motor
    URDFLink(
        name="Base",
        origin_translation=[0, 0, 0],  # Located at the origin
        origin_orientation=[0, 0, 0],        # No initial rotation
        rotation=[0, 0, 1],           # Rotates around the Z-axis
        bounds=(0, 3.14)              # Bounds: 0 to π radians
    ),
    # Shoulder motor
    URDFLink(
        name="Shoulder",
        origin_translation=[0, 0, 0],  # No distance between the base and shoulder
        origin_orientation=[0, 0, 0],        # No initial rotation
        rotation=[1, 0, 0],           # Rotates around the X-axis
        bounds=(0, 3.14)          # Bounds: -π/2 to π/2 radians
    ),
    # 48mm link
    URDFLink(
        name="UpperArm",
        origin_translation=[0.048, 0, 0],  # 48mm in the X-direction
        origin_orientation=[0, 0, 0],            # Static link, no rotation
        bounds=None,
        joint_type="fixed"
    ),
    # Elbow motor
    URDFLink(
        name="Elbow",
        origin_translation=[0, 0, 0],  # Mounted directly at the end of the previous link
        origin_orientation=[0, 0, 0],        # No initial rotation
        rotation=[1, 0, 0],           # Rotates around the X-axis
        bounds=(0, 3.14)          # Bounds: -π/2 to π/2 radians
    ),
    # 46.5mm link
    URDFLink(
        name="LowerArm",
        origin_translation=[0.0465, 0, 0],  # 46.5mm in the X-direction
        origin_orientation=[0, 0, 0],             # Static link, no rotation
        bounds=None,
        joint_type="fixed"
    ),
    # Wrist motor
    URDFLink(
        name="Wrist",
        origin_translation=[0, 0, 0],  # Mounted directly at the end of the previous link
        origin_orientation=[0, 0, 0],        # No initial rotation
        rotation=[1, 0, 0],           # Rotates around the X-axis
        bounds=(0, 3.14)          # Bounds: -π/2 to π/2 radians
    ),
    # Final 80mm link
    URDFLink(
        name="EndEffector",
        origin_translation=[0.08, 0, 0],  # 80mm in the X-direction
        origin_orientation=[0, 0, 0],           # Static link, no rotation
        bounds=None,
        joint_type="fixed"
    ),
])

# Print the complete chain
print("Robotic arm chain:")
for link in robotic_arm.links:
    print(link.name)

target_position = [0.16, 0, 0]  # Example target position (in meters)
joint_angles = robotic_arm.inverse_kinematics(target_position)
print("Calculated joint angles for target position:", joint_angles)

