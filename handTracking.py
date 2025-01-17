import cv2
import mediapipe as mp
from math import sqrt
mp_drawing = mp.solutions.drawing_utils
mp_drawing_styles = mp.solutions.drawing_styles
mp_hands = mp.solutions.hands

cap = cv2.VideoCapture(0)
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
