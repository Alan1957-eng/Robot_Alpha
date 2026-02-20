# main_0.py

import threading
import time
import cv2
import mediapipe as mp
from flask import Flask, Response
from megapi import *
from gpiozero import AngularServo

# ==========================================
#              CONFIGURATION
# ==========================================
# --- HARDWARE PINS ---
MEGAPI_PORT = '/dev/ttyUSB0' # Check your actual port
ULTRA_PORT = 6
LINE_PORT = 7
SERVO_PAN_PIN = 18
SERVO_TILT_PIN = 19
AVOID_DISTANCE_OBJECT == 45
AVOID_DISTANCE_WALL == 25

# --- SERVO SETTINGS ---
PAN_LIMITS = (-90, 90)
TILT_LIMITS = (-60, 60)

# --- GLOBAL VARIABLES (Shared Data) ---
output_frame = None
lock = threading.Lock() # Prevents two threads reading image at same time
robot_mode = "IDLE"     # IDLE, TRACKING, AVOIDING
current_dist = 100      # Distance in cm
current_line = 3        # 3=Floor, 0=Cliff
target_pan = 0
target_tilt = 0

# ==========================================
#           HARDWARE INITIALIZATION
# ==========================================

# 1. Initialize MegaPi
print("Initializing MegaPi...")
bot = MegaPi()
bot.start(MEGAPI_PORT)

# 2. Initialize Servos
pan_servo = AngularServo(SERVO_PAN_PIN, min_angle=-90, max_angle=90)
tilt_servo = AngularServo(SERVO_TILT_PIN, min_angle=-90, max_angle=90)

# 3. Initialize Flask
app = Flask(__name__)

# ==========================================
#           THREAD 1: CAMERA & VISION
# ==========================================
def camera_thread():
    global output_frame, target_pan, target_tilt, robot_mode
    
    # Setup MediaPipe
    mp_face_detection = mp.solutions.face_detection
    face_detection = mp_face_detection.FaceDetection(min_detection_confidence=0.6)
    
    cap = cv2.VideoCapture(0)
    time.sleep(2.0) # Warmup

    while True:
        ret, frame = cap.read()
        if not ret: continue

        # Flip for mirror effect
        frame = cv2.flip(frame, 1)
        rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        
        # Detect Faces
        results = face_detection.process(rgb_frame)
        
        # Logic: If face found, calculate where it is
        if results.detections:
            robot_mode = "TRACKING"
            for detection in results.detections:
                bbox = detection.location_data.relative_bounding_box
                x_center = bbox.xmin + (bbox.width / 2)
                y_center = bbox.ymin + (bbox.height / 2)
                
                # Update Target Angles (Simple P-Controller)
                # If x_center < 0.5 (Left), increase angle
                err_x = 0.5 - x_center
                err_y = 0.5 - y_center
                
                # Adjust sensitivity (multiplier)
                target_pan += (err_x * 10) 
                target_tilt += (err_y * 10)
                
                # Draw box on frame
                mp.solutions.drawing_utils.draw_detection(frame, detection)
        else:
            robot_mode = "IDLE"

        # Update the global frame for the Web Server
        with lock:
            output_frame = frame.copy()
            
        time.sleep(0.01) # Yield to CPU

# ==========================================
#        THREAD 2: SENSORS & MOTORS
# ==========================================
def sensor_callback_dist(v):
    global current_dist
    current_dist = v

def sensor_callback_line(v):
    global current_line
    current_line = v

def control_thread():
    global target_pan, target_tilt
    
    print("Control Thread Started")
    
    current_state == "ROAMING"
  
    while True:
        # --- 1. READ SENSORS ---
        # MegaPi reads are non-blocking, we trigger them here
        bot.ultrasonicSensorRead(ULTRA_PORT, sensor_callback_dist)
        bot.lineFollowerRead(LINE_PORT, sensor_callback_line)
        
        # --- 2. SAFETY CHECKS ---
        if current_dist < AVOID_DISTANCE_WALL:
            # OBSTACLE! Stop motors
            bot.motorRun(M1, 0)
            bot.motorRun(M2, 0)
            # print("Obstacle!")
            
        elif current_line == 0: 
            # CLIFF! Stop and Back up (Simulated)
            bot.motorRun(M1, -50)
            bot.motorRun(M2, -50)
            time.sleep(0.5)
            bot.motorRun(M1, 0)
            bot.motorRun(M2, 0)
            # print("Cliff!")
            
        elif current_dist < AVOID_DISTANCE_OBJECT and current_state == "ROAMING":
            # Stop immediately
            bot.motorRun(M1, 0); bot.motorRun(M2, 0)
    
            # "The Wiggle Check" - Quick sweep to see if it's a real wall or just a leg
            # Turn slightly Left
            bot.motorRun(M1, -50); bot.motorRun(M2, 50)
            time.sleep(0.3)
            # Check Sensor... (Logic here)
    
            current_state = "AVOIDING"
            
        else:
            # Safe to move? (Add your roaming logic here)
            current_state == "ROAMING"
            
            pass

        # --- 3. UPDATE SERVOS ---
        # Clamp values to safe limits
        target_pan = max(PAN_LIMITS[0], min(PAN_LIMITS[1], target_pan))
        target_tilt = max(TILT_LIMITS[0], min(TILT_LIMITS[1], target_tilt))
        
        pan_servo.angle = target_pan
        tilt_servo.angle = target_tilt
        
        time.sleep(0.1) # Run at 10Hz

# ==========================================
#        THREAD 3: WEB SERVER (FLASK)
# ==========================================
def generate():
    global output_frame
    while True:
        with lock:
            if output_frame is None:
                continue
            # Encode frame to JPG
            (flag, encodedImage) = cv2.imencode(".jpg", output_frame)
            if not flag:
                continue
                
        yield(b'--frame\r\n' b'Content-Type: image/jpeg\r\n\r\n' + 
              bytearray(encodedImage) + b'\r\n')

@app.route("/")
def index():
    return "<html><body><h1>Robot Eye</h1><img src='/video_feed'></body></html>"

@app.route("/video_feed")
def video_feed():
    return Response(generate(), mimetype="multipart/x-mixed-replace; boundary=frame")

# ==========================================
#              MAIN ENTRY POINT
# ==========================================
if __name__ == '__main__':
    # Start Camera Thread
    t1 = threading.Thread(target=camera_thread)
    t1.daemon = True # Kills thread if main program quits
    t1.start()

    # Start Control Thread
    t2 = threading.Thread(target=control_thread)
    t2.daemon = True
    t2.start()

    # Start Web Server (This blocks the main thread, keeping script alive)
    print("Starting Web Server on port 5000...")
    app.run(host="0.0.0.0", port=5000, debug=False, use_reloader=False)