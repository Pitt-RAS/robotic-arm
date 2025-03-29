import numpy as np
from ikpy.chain import Chain
from ikpy.link import DHLink
import serial.tools.list_ports
import cv2
import mediapipe as mp
from math import sqrt, acos
import time

mp_drawing = mp.solutions.drawing_utils
mp_drawing_styles = mp.solutions.drawing_styles
mp_hands = mp.solutions.hands
mp_pose = mp.solutions.pose
numPrevious = 3
filter = np.full((5, numPrevious), 90)


cap = cv2.VideoCapture(1)
counter = 0
img_width = 0
img_height = 0

ports = serial.tools.list_ports.comports()
serialInst = serial.Serial()
portsList = []

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

format_float = np.vectorize(lambda x: "%.6f" % x)

with mp_pose.Pose(min_detection_confidence=0.5, min_tracking_confidence=0.5) as pose:
    while cap.isOpened():
        counter += 1
        success, image = cap.read()
        if not success:
            print("Ignoring empty camera frame.")
            continue
        if counter == 0:
            img_width = image.shape[1]
            img_height = image.shape[0]
        image.flags.writeable = False
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        results = pose.process(image)
        image.flags.writeable = True
        if results.pose_landmarks:
          #time.sleep(0.1) #If we want to slow it down some.
          wristX = results.pose_landmarks.landmark[16].x
          wristY = results.pose_landmarks.landmark[16].y
          # wristZ = results.pose_landmarks.landmark[20].z Did some testing with this but it is not accurate. 
          #print("X: ", wristX, " Y: ", wristY, " Z: ", wristZ)
          centerX = results.pose_landmarks.landmark[0].x
          centerY = results.pose_landmarks.landmark[11].y - results.pose_landmarks.landmark[12].y
          centerY = centerY/2 + results.pose_landmarks.landmark[12].y

          #print("X: ", centerX - wristX)
          #print("Y: ", centerY - wristY)

          targetX = round(250 * (centerX - wristX))
          targetZ = round(250 * (centerY - wristY)) #Target Z since this is the up down of the arm
          if targetZ < 0:
              targetZ = 0
          #print("Target X: ", targetX)
          #print("Target Z: ", targetZ)
          targetY = 65 #Until we have a way to get depth informaiton

          radVals = format_float(robot_arm.inverse_kinematics((targetX, targetY, targetZ))) #Get rad values out of scientific notation
          radVals = np.array(radVals, dtype=float) #Turn this into an np array
          degVals = 57.2957795*radVals #Turn rad to deg
          degVals = degVals+90 #Add 90 to make up for the 90 removed in the DH paramaters 
          degVals = np.round(degVals).astype(int) #Cast to int
          #print(degVals)
          
          
        #Check IK orange arm for how to fix this

          for i in range(len(degVals)):
           filter[i][counter % numPrevious] = degVals[i]
           current = str(i) + str(int(np.mean(filter[i])))
           command = current + "\n"
           serialInst.write(command.encode('utf-8'))

          mp_drawing.draw_landmarks(
            image, 
            results.pose_landmarks, 
            mp_pose.POSE_CONNECTIONS, 
            mp_drawing_styles.get_default_pose_landmarks_style()
        )
          
        image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
        cv2.imshow('MediaPipe Pose and Hand Tracking', cv2.flip(image, 1))
        #Stop the loop on escape key
        if cv2.waitKey(5) & 0xFF == 27:
            break
cap.release()
serialInst.close()