# AI Face Body and Hand Pose Detection with Python and Mediapipe
# importing the necessary libraries

import mediapipe as mp
import cv2

mp_drawing = mp.solutions.drawing_utils  # importing drawing utilities
mp_holistic = mp.solutions.holistic  #importing holistic model

# open camera

cap = cv2.VideoCapture(0)
while cap.isOpened():
    ret, frame = cap.read()
    cv2.imshow('Raw Webcam Feed', frame)
    
    if cv2.waitKey(10) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()

# close camera

cap.release()
cv2.destroyAllWindows()