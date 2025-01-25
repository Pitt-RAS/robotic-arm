import cv2
import mediapipe as mp
from math import sqrt, acos
import serial.tools.list_ports
import numpy as np

ports = serial.tools.list_ports.comports()
serialInst = serial.Serial()
portsList = []


for one in ports:
    portsList.append(str(one))
    print(str(one))

serialInst.baudrate = 115200
serialInst.port = "/dev/cu.usbserial-1110"
serialInst.open()

mp_drawing = mp.solutions.drawing_utils
mp_drawing_styles = mp.solutions.drawing_styles
mp_hands = mp.solutions.hands
mp_pose = mp.solutions.pose

cap = cv2.VideoCapture(0)
counter = 0
img_width = 0
img_height = 0

with mp_pose.Pose(
    min_detection_confidence=0.5, min_tracking_confidence=0.5) as pose, mp_hands.Hands(
    model_complexity=0, min_detection_confidence=0.5, min_tracking_confidence=0.5,
    max_num_hands=1) as hands:
    while cap.isOpened():
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
        handResults = hands.process(image)
        image.flags.writeable = True
        if results.pose_landmarks:
          wristX = results.pose_landmarks.landmark[16].x
          wristY = results.pose_landmarks.landmark[16].y
          elbowX = results.pose_landmarks.landmark[14].x
          elbowY = results.pose_landmarks.landmark[14].y
          shoulderX = results.pose_landmarks.landmark[12].x
          shoulderY = results.pose_landmarks.landmark[12].y

          #This is the calculation to get the "elbow" working on the orange arm
          A = sqrt((wristX-shoulderX)**2 + (wristY-shoulderY)**2)
          B = sqrt((elbowX-shoulderX)**2 + (elbowY-shoulderY)**2)
          C = sqrt((wristX-elbowX)**2 + (wristY-elbowY)**2)
          a = acos((C**2 + B**2 - A**2) / (2*C*B))
          a = (180 * a) / 3.14159
          #if counter % 5 == 0:
            #print(a)
          counter += 1
          elbowCommand = str(2) + str(int(180-a)) + "\n"
          #serialInst.write(command.encode('utf-8'))

          #This is the calculation to get the "shoulder" working on the orange arm
          #Still using B from above

          height = elbowY-shoulderY
          shoulderAngle = acos(height/B)
          shoulderAngle = (180 * shoulderAngle) / 3.14159
          mappedShoulderAngle = 15 + ((shoulderAngle * 105) / 180)
          shoulderCommand = str(1) + str(int(mappedShoulderAngle)) + "\n"
          #serialInst.write(shoulderCommand.encode('utf-8'))

          #forearm vector to be used later 
          

          mp_drawing.draw_landmarks(
            image, 
            results.pose_landmarks, 
            mp_pose.POSE_CONNECTIONS, 
            mp_drawing_styles.get_default_pose_landmarks_style()
        )
        
        if handResults.multi_hand_landmarks:
          for hand_landmarks in handResults.multi_hand_landmarks:
            tipWristX = abs(hand_landmarks.landmark[12].x-hand_landmarks.landmark[0].x)
            tipWristY = abs(hand_landmarks.landmark[12].y-hand_landmarks.landmark[0].y)
            knuckleWristX = abs(hand_landmarks.landmark[9].x-hand_landmarks.landmark[0].x)
            knuckleWristY = abs(hand_landmarks.landmark[9].y-hand_landmarks.landmark[0].y)
            tipWrist = sqrt(tipWristX**2 + tipWristY**2)
            knuckleWrist = sqrt(knuckleWristX**2 + knuckleWristY**2)
            percentOpen = tipWrist/(1.95*knuckleWrist)

            mappedPercentOpen = int((50 + 80*(percentOpen-0.2)))
            gripCommand = str(4) + str(mappedPercentOpen) + "\n"
            #serialInst.write(gripCommand.encode('utf-8'))
            if counter % 5 == 0:
              print(mappedPercentOpen)

            forearm = [hand_landmarks.landmark[0].x-elbowX, hand_landmarks.landmark[0].y-elbowY]
            forearmVector = np.array(forearm)
            wrist = [hand_landmarks.landmark[9].x-hand_landmarks.landmark[0].x, hand_landmarks.landmark[9].y-hand_landmarks.landmark[0].y]
            wristVector = np.array(wrist)
            dot_product = np.dot(forearmVector, wristVector)

            # Calculate the magnitudes of the vectors
            magnitude_forearm = np.linalg.norm(forearmVector)
            magnitude_wrist = np.linalg.norm(wristVector)

            # Calculate the cosine of the angle
            cos_angle = dot_product / (magnitude_forearm * magnitude_wrist)

            # Calculate the angle in radians
            angle_radians = np.arccos(cos_angle)

            # Convert the angle to degrees
            angle_degrees = np.degrees(angle_radians)
            cross_product = np.cross(forearmVector, wristVector)
            if cross_product < 0:
               angle_degrees = -angle_degrees
            

            wristValue = int(50 + 120/150*angle_degrees)
            wristCommand = str(3) + str(wristValue) + "\n"
            

            mp_drawing.draw_landmarks(
              image,
              hand_landmarks,
              mp_hands.HAND_CONNECTIONS,
              mp_drawing_styles.get_default_hand_landmarks_style(),
              mp_drawing_styles.get_default_hand_connections_style())
            
            if handResults.multi_hand_landmarks:
               #pass
               serialInst.write(gripCommand.encode('utf-8'))
               #serialInst.write(wristCommand.encode('utf-8'))
               #serialInst.write(elbowCommand.encode('utf-8'))
               #serialInst.write(shoulderCommand.encode('utf-8'))
            
        
        image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
        cv2.imshow('MediaPipe Pose', cv2.flip(image, 1))
        #Stop the loop on escape key
        if cv2.waitKey(5) & 0xFF == 27:
            break
cap.release()
serialInst.close()
'''
with mp_hands.Hands(
    model_complexity=0,
    min_detection_confidence=0.5,
    min_tracking_confidence=0.5) as hands:
  while cap.isOpened():
    success, image = cap.read()
    if not success:
      print("Ignoring empty camera frame.")
      # If loading a video, use 'break' instead of 'continue'.
      continue

    # To improve performance, optionally mark the image as not writeable to
    # pass by reference.
    image.flags.writeable = False
    image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    results = hands.process(image)

    # Draw the hand annotations on the image.
    image.flags.writeable = True
    image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
    counter = 0
    if results.multi_hand_landmarks:
      for hand_landmarks in results.multi_hand_landmarks:
        tipWristX = abs(hand_landmarks.landmark[12].x-hand_landmarks.landmark[0].x)
        tipWristY = abs(hand_landmarks.landmark[12].y-hand_landmarks.landmark[0].y)
        knuckleWristX = abs(hand_landmarks.landmark[9].x-hand_landmarks.landmark[0].x)
        knuckleWristY = abs(hand_landmarks.landmark[9].y-hand_landmarks.landmark[0].y)
        tipWrist = sqrt(tipWristX**2 + tipWristY**2)
        knuckleWrist = sqrt(knuckleWristX**2 + knuckleWristY**2)
        percentOpen = tipWrist/(1.95*knuckleWrist)
        wristX = hand_landmarks.landmark[0].x
        wristY = hand_landmarks.landmark[0].y
        if counter % 1000 == 0:
          print("Percent Open: " + str(percentOpen))
          print("Wrist pos: " + str(wristX) + " | " + str(wristY))
        counter+=1
        mp_drawing.draw_landmarks(
            image,
            hand_landmarks,
            mp_hands.HAND_CONNECTIONS,
            mp_drawing_styles.get_default_hand_landmarks_style(),
            mp_drawing_styles.get_default_hand_connections_style())
    # Flip the image horizontally for a selfie-view display.
    cv2.imshow('MediaPipe Hands', cv2.flip(image, 1))
    if cv2.waitKey(5) & 0xFF == 27:
      break
cap.release()
'''