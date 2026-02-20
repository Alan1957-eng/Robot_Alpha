# robot_eyes.py

import cv2
import mediapipe as mp

mp_face_detection = mp.solutions.face_detection
face_detection = mp_face_detection.FaceDetection(0.5)

cap = cv2.VideoCapture(0) # 0 is usually the USB camera

while cap.isOpened():
    success, image = cap.read()
    if not success:
        continue

    # Convert to RGB
    image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    results = face_detection.process(image)

    # Draw face box
    image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
    
    if results.detections:
        for detection in results.detections:
            # Draw a simple box around face
            mp.solutions.drawing_utils.draw_detection(image, detection)
            print("Face Detected!")

    cv2.imshow('Robot Vision', image)
    if cv2.waitKey(5) & 0xFF == 27: # Press Esc to exit
        break

cap.release()
cv2.destroyAllWindows()