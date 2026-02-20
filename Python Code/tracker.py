# tracker.py

import cv2
import mediapipe as mp
from gpiozero import AngularServo
from time import sleep

# --- CONFIGURATION ---
PAN_PIN = 18
TILT_PIN = 19
X_CENTER = 0.5  # Center of screen is 0.5
Y_CENTER = 0.5
DEAD_ZONE = 0.05 # How close to center before stopping
SPEED = 2       # Degrees to move per frame
# ---------------------

# Initialize Servos
pan = AngularServo(PAN_PIN, min_angle=-90, max_angle=90)
tilt = AngularServo(TILT_PIN, min_angle=-90, max_angle=90)

# Current Angles
pan_angle = 0
tilt_angle = 0

# Initialize Camera
mp_face_detection = mp.solutions.face_detection
face_detection = mp_face_detection.FaceDetection(min_detection_confidence=0.6)
cap = cv2.VideoCapture(0)

print("Face Tracker Started. Press 'q' to quit.")

def clamp(n, minn, maxn):
    return max(min(maxn, n), minn)

while cap.isOpened():
    success, image = cap.read()
    if not success: continue

    # Flip image for "mirror" effect (intuitive for testing)
    image = cv2.flip(image, 1)
    
    # Process Face
    image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    results = face_detection.process(image_rgb)

    if results.detections:
        for detection in results.detections:
            # Get bounding box
            bbox = detection.location_data.relative_bounding_box
            obj_x = bbox.xmin + (bbox.width / 2)
            obj_y = bbox.ymin + (bbox.height / 2)

            # --- PAN LOGIC (Left/Right) ---
            if obj_x < (X_CENTER - DEAD_ZONE):
                pan_angle += SPEED # Move Left
            elif obj_x > (X_CENTER + DEAD_ZONE):
                pan_angle -= SPEED # Move Right
            
            # --- TILT LOGIC (Up/Down) ---
            if obj_y < (Y_CENTER - DEAD_ZONE):
                tilt_angle += SPEED # Look Up
            elif obj_y > (Y_CENTER + DEAD_ZONE):
                tilt_angle -= SPEED # Look Down

            # Clamp and Apply
            pan_angle = clamp(pan_angle, -90, 90)
            tilt_angle = clamp(tilt_angle, -60, 60) # Limit tilt to avoid hitting robot body
            
            pan.angle = pan_angle
            tilt.angle = tilt_angle

    # Show feed
    cv2.imshow('Face Tracker', image)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()