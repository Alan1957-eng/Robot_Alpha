# main.py

"""
The Implementation:

Separated the Audio into its own thread. Do not put Audio in the Camera Thread. 
Audio recording is "blocking" (it pauses the code to listen), which would ruin your 
video frame rate (FPS) and make the face tracking laggy.

Here is the complete, updated main.py with Voice Detection, Visual Scanning, and 
Approach Distance included.
"""

import threading
import time
import random
import cv2
import mediapipe as mp
import numpy as np
import pyaudio
import queue
import sys
import traceback
import cv2
from luma.core.interface.serial import i2c
from luma.core.render import canvas
from luma.oled.device import ssd1306
from PIL import Image
from flask import Flask, Response
from megapi import *
from gpiozero import AngularServo

# ==========================================
#              CONFIGURATION
# ==========================================
# --- HARDWARE PINS ---

# 1. MEGAPI (Body)
PORT_LEFT_MOTOR = 1
PORT_RIGHT_MOTOR = 2
PORT_ULTRASONIC = 6
PORT_LINE_FOLLOWER = 7

# 2. RASPBERRY PI GPIO (Head)
PIN_SERVO_PAN = 18  # GPIO 18 (Physical Pin 12)
PIN_SERVO_TILT = 19 # GPIO 19 (Physical Pin 35)

# 3. I2C DEVICES (OLED)
OLED_PORT = 1
OLED_ADDRESS = 0x3C

# 4. USB DEVICES
CAMERA_INDEX = 0
SERIAL_PORT = '/dev/ttyUSB0'

MEGAPI_PORT = '/dev/ttyUSB0' 
ULTRA_PORT = 6
LINE_PORT = 7
SERVO_PAN_PIN = 18
SERVO_TILT_PIN = 19
MIC_INDEX = 1 # Run your 'find_mics.py' to confirm this ID!
AVOID_DISTANCE_OBJECT == 45
AVOID_DISTANCE_WALL == 25

# --- GLOBAL SHARED OBJECTS ---
# 1. The "Error Bucket" - Any thread can drop a string here
error_queue = queue.Queue()

# 2. Global State Flags
system_running = True
current_state = "IDLE"

# --- HARDWARE SETUP ---
# Initialize OLED once globally so we don't fight over the I2C bus
try:
    serial = i2c(port=1, address=0x3C)
    oled_device = ssd1306(serial, rotate=0)
except Exception as e:
    print(f"CRITICAL: OLED Failed - {e}")
    # If OLED fails, we can't display errors, so just print
    sys.exit(1)

# --- HELPER: The "Whistleblower" Function ---
def log_error(source, message):
    """Any part of the code calls this to scream for help."""
    full_msg = f"[{source}] {message}"
    print(f"LOGGED ERROR: {full_msg}") # Still print to terminal for debugging
    error_queue.put(full_msg)

# --- BEHAVIOR SETTINGS ---
FACE_TARGET_SIZE = 0.15  # How big the face should be (0.0 to 1.0). 0.15 is roughly 1 meter away.
AUDIO_THRESHOLD = 1000   # Volume sensitivity (Adjust based on your mic)

# --- GLOBAL VARIABLES ---
output_frame = None
lock = threading.Lock()
robot_mode = "IDLE"     
face_detected_size = 0  # Width of the face relative to screen (0.0 - 1.0)
heard_voice_timestamp = 0 # When was the last loud sound?
target_pan = 0
target_tilt = 0

# --- FLASK APP ---
app = Flask(__name__)

# ==========================================
#           HARDWARE INIT
# ==========================================
print("Initializing Hardware...")
bot = MegaPi()
bot.start(MEGAPI_PORT)
pan_servo = AngularServo(SERVO_PAN_PIN, min_angle=-90, max_angle=90)
tilt_servo = AngularServo(SERVO_TILT_PIN, min_angle=-90, max_angle=90)

# ---------------------------------------------------------
# THREAD 1: THE SYSTEM MONITOR (OLED Manager)
# ---------------------------------------------------------
def oled_thread():
    """Controls the screen. Prioritizes Errors > Camera Feed."""
    print("OLED Thread Started")
    
    # Setup Camera for the "Fun" feed
    cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)

    while system_running:
        try:
            # --- PRIORITY 1: CHECK FOR ERRORS ---
            # If the queue is NOT empty, we have a problem.
            if not error_queue.empty():
                error_msg = error_queue.get()
                
                # Display the Error for 5 seconds
                with canvas(oled_device) as draw:
                    draw.rectangle(oled_device.bounding_box, outline="white", fill="black")
                    draw.text((0, 0), "!! SYSTEM ALERT !!", fill="white")
                    # Simple text wrapping for 2 lines
                    draw.text((0, 15), error_msg[:18], fill="white")
                    draw.text((0, 25), error_msg[18:36], fill="white")
                    draw.text((0, 35), error_msg[36:], fill="white")
                
                # Force the error to stay on screen for readability
                time.sleep(5) 
                
                # Mark task as done so queue is clear
                error_queue.task_done()
                continue # Skip the camera feed this loop

            # --- PRIORITY 2: THE "FUN" CAMERA FEED ---
            ret, frame = cap.read()
            if ret:
                # Process image for OLED (Resize -> Convert to Binary)
                img = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                pil_img = Image.fromarray(img)
                pil_img = pil_img.resize((oled_device.width, oled_device.height), Image.BILINEAR)
                pil_img = pil_img.convert("1")
                
                # Push to screen
                oled_device.display(pil_img)
            else:
                # If camera itself fails, log it to the queue!
                # This is self-healing: The loop will restart and pick up this error next time.
                log_error("OLED", "Camera feed lost")
                time.sleep(1)

        except Exception as e:
            print(f"OLED Thread Crash: {e}")
            time.sleep(1)

    cap.release()
# ==========================================
#        THREAD 2: AUDIO LISTENER
# ==========================================
def audio_thread():
    global heard_voice_timestamp
    
    CHUNK = 1024
    FORMAT = pyaudio.paInt16
    CHANNELS = 1
    RATE = 44100
    
    p = pyaudio.PyAudio()
    
    try:
        stream = p.open(format=FORMAT, channels=CHANNELS, rate=RATE, 
                        input=True, input_device_index=MIC_INDEX, 
                        frames_per_buffer=CHUNK)
    except Exception as e:
        print(f"Audio Error: {e}")
        return

    print("Audio Listening...")
    
    while True:
        try:
            data = stream.read(CHUNK, exception_on_overflow=False)
            # Convert raw data to integers to calculate volume
            audio_data = np.frombuffer(data, dtype=np.int16)
            peak = np.max(np.abs(audio_data))
            
            if peak > AUDIO_THRESHOLD:
                # Update timestamp of last sound
                heard_voice_timestamp = time.time()
                # print(f"Heard sound! Level: {peak}")
                
        except Exception:
            pass
            
        time.sleep(0.01)

# ==========================================
#        THREAD 3: CAMERA & VISION
# ==========================================
def camera_thread():
    global output_frame, target_pan, target_tilt, robot_mode, face_detected_size
    
    mp_face_detection = mp.solutions.face_detection
    face_detection = mp_face_detection.FaceDetection(min_detection_confidence=0.6)
    
    cap = cv2.VideoCapture(0)
    time.sleep(2.0)

    while True:
        ret, frame = cap.read()
        if not ret: continue

        frame = cv2.flip(frame, 1)
        rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        results = face_detection.process(rgb_frame)
        
        if results.detections:
            robot_mode = "TRACKING"
            # Get the first face found
            detection = results.detections[0]
            bbox = detection.location_data.relative_bounding_box
            
            x_center = bbox.xmin + (bbox.width / 2)
            y_center = bbox.ymin + (bbox.height / 2)
            face_detected_size = bbox.width # Save size for "Approach" logic
            
            # --- SERVO TRACKING LOGIC ---
            # Calculate error from center (0.5)
            err_x = 0.5 - x_center
            err_y = 0.5 - y_center
            
            # Proportional Control (Speed of tracking)
            target_pan += (err_x * 8) 
            target_tilt += (err_y * 8)
            
            mp.solutions.drawing_utils.draw_detection(frame, detection)
        else:
            robot_mode = "IDLE"
            face_detected_size = 0

        with lock:
            output_frame = frame.copy()
            
        time.sleep(0.01)


# ==========================================
#     THREAD 4: CONTROL (BRAIN + MUSCLES)
# ==========================================
# Variables for Sensor Callbacks
current_dist = 100
current_line = 3

def sensor_callback_dist(v):
    global current_dist
    current_dist = v

def sensor_callback_line(v):
    global current_line
    current_line = v

def control_thread():
    global target_pan, target_tilt
    global current_state, voice_command, panic_stop
    
    current_state = "ROAMING"
    avoid_start_time = 0
    avoid_duration = 0
    scan_start_time = 0
    
    # Scanning variables
    scan_dir = 1
    
    print("Robot Brain Active.")
    
    while True:
        
        # --- PRIORITY 1: PANIC STOP ---
        if panic_stop:
            bot.motorRun(0,0); bot.motorRun(1,0) # Kill motors
            current_state = "STOPPED"
            panic_stop = False # Reset flag
            continue # Skip the rest of the loop
            
        # --- PRIORITY 2: VOICE COMMANDS ---
        if voice_command:
            cmd = voice_command
            voice_command = None # Clear it so we don't repeat
            
            if cmd == "stop":
                current_state = "STOPPED"
                bot.motorRun(0,0); bot.motorRun(1,0)
            elif cmd == "go":
                current_state = "ROAMING"
            elif cmd == "turn left":
                bot.motorRun(0, -50); bot.motorRun(1, 50) # Spin Left
                time.sleep(1)
                bot.motorRun(0,0); bot.motorRun(1,0)
                
        # --- PRIORITY 3: EXISTING LOGIC (Roaming, Face Track, etc) ---
        if current_state == "ROAMING":  
        
        # 1. Trigger Sensors
        bot.ultrasonicSensorRead(ULTRA_PORT, sensor_callback_dist)
        bot.lineFollowerRead(LINE_PORT, sensor_callback_line)
        
        # 2. Check Inputs
        time_since_sound = time.time() - heard_voice_timestamp
        has_face = (robot_mode == "TRACKING")
        is_cliff = (current_line == 0)
        is_obstacle = (current_dist < 25)
        
        # ===========================
        #    STATE MACHINE LOGIC
        # ===========================
        
        # --- PRIORITY 1: SAFETY (Cliff) ---
        if is_cliff:
            current_state = "CLIFF_RECOVERY"
            
        # --- PRIORITY 2: FACE FOUND (Tracking) ---
        elif has_face and current_state != "CLIFF_RECOVERY":
            current_state = "TRACKING"
            
        # --- PRIORITY 3: HEARD SOUND? (Scan) ---
        elif time_since_sound < 1.0 and current_state == "ROAMING":
            print("Heard voice! Scanning...")
            current_state = "SCANNING"
            scan_start_time = time.time()
        
        elif current_dist < AVOID_DISTANCE_OBJECT and current_state == "ROAMING":
            # Stop immediately
            bot.motorRun(M1, 0); bot.motorRun(M2, 0)
    
            # "The Wiggle Check" - Quick sweep to see if it's a real wall or just a leg
            # Turn slightly Left
            bot.motorRun(M1, -50); bot.motorRun(M2, 50)
            time.sleep(0.3)
            # Check Sensor... (Logic here)
    
            current_state = "AVOIDING"   
        
        # --- PRIORITY 4: OBSTACLE (Avoid) ---
        elif is_obstacle and current_state == "ROAMING":
            current_state = "AVOIDING"
            avoid_start_time = time.time()
            avoid_duration = random.uniform(0.5, 1.2)
            turn_dir = random.choice([-1, 1])

        # ===========================
        #      MOTOR OUTPUTS
        # ===========================
        
        if current_state == "CLIFF_RECOVERY":
            bot.motorRun(M1, -60); bot.motorRun(M2, -60) # Backup
            time.sleep(0.4)
            bot.motorRun(M1, 80); bot.motorRun(M2, -80)  # Spin
            time.sleep(0.4)
            current_state = "ROAMING"

        elif current_state == "TRACKING":
            # --- APPROACH LOGIC ---
            # If face is small (< 15% of screen), we are too far. Drive closer.
            if face_detected_size < FACE_TARGET_SIZE:
                # Move forward slowly while tracking
                bot.motorRun(M1, 60)
                bot.motorRun(M2, 60)
            else:
                # Proper distance reached. Stop.
                bot.motorRun(M1, 0)
                bot.motorRun(M2, 0)
                
        elif current_state == "SCANNING":
            # Stop Motors
            bot.motorRun(M1, 0); bot.motorRun(M2, 0)
            
            # Pan head back and forth to find face
            # We use sin wave to oscillate the head
            elapsed = time.time() - scan_start_time
            target_pan = math.sin(elapsed * 3) * 60 # Swing +/- 60 degrees
            
            # Give up scanning after 4 seconds if no face found
            if elapsed > 4.0:
                current_state = "ROAMING"
                # Reset head
                target_pan = 0 
                target_tilt = 0

        elif current_state == "AVOIDING":
            bot.motorRun(M1, 80 * turn_dir)
            bot.motorRun(M2, -80 * turn_dir)
            
            if (time.time() - avoid_start_time) > avoid_duration:
                current_state = "ROAMING"

        elif current_state == "ROAMING":
            bot.motorRun(M1, 70)
            bot.motorRun(M2, 70)
            # Keep head centered
            target_pan = 0
            target_tilt = 0

        # Update Servos (Clamped)
        target_pan = max(-90, min(90, target_pan))
        target_tilt = max(-60, min(60, target_tilt))
        pan_servo.angle = target_pan
        tilt_servo.angle = target_tilt
        
        time.sleep(0.05)

# ==========================================
#     THREAD 5: WEB SERVER
# ==========================================
def generate():
    global output_frame
    while True:
        with lock:
            if output_frame is None: continue
            (flag, encodedImage) = cv2.imencode(".jpg", output_frame)
            if not flag: continue
        yield(b'--frame\r\n' b'Content-Type: image/jpeg\r\n\r\n' + bytearray(encodedImage) + b'\r\n')

@app.route("/")
def index(): return "<html><body><img src='/video_feed'></body></html>"

@app.route("/video_feed")
def video_feed(): return Response(generate(), mimetype="multipart/x-mixed-replace; boundary=frame")


# ==========================================
#     THREAD 6: VOICE THREAD
# ==========================================
d
def voice_thread():
    global voice_command, panic_stop
    
    # Initialize Vosk
    model = vosk.Model("models/model")
    rec = vosk.KaldiRecognizer(model, 16000, '["stop", "go", "turn left", "turn right"]')
    
    def callback(indata, frames, time, status):
        # 1. PANIC CHECK (Volume)
        volume = np.linalg.norm(np.frombuffer(indata, dtype=np.int16)) / np.sqrt(len(indata))
        if volume > 500: # Adjust this threshold!
            print("!!! PANIC STOP !!!")
            panic_stop = True

        # 2. VOICE CHECK (Words)
        if rec.AcceptWaveform(bytes(indata)):
            result = json.loads(rec.Result())
            text = result.get("text", "")
            if text:
                print(f"Heard: {text}")
                voice_command = text # Send to Main Thread

    # Start Listening
    with sd.InputStream(samplerate=16000, blocksize=8000, dtype='int16', 
                        channels=1, callback=callback):
        while True:
            time.sleep(0.1) # Just keep the thread alive

# ==========================================
#           MAIN START
# ==========================================
if __name__ == '__main__':
    import math # Needed for scanning sine wave
    
    # Create Threads
    t_audio = threading.Thread(target=audio_thread, daemon=True)
    t_cam = threading.Thread(target=camera_thread, daemon=True)
    t_ctrl = threading.Thread(target=control_thread, daemon=True)
    t_voice = threading.Thread(target=voice_thread)
    t_voice.daemon = True
    t_voice.start()
    
    # Start Threads
    t_audio.start()
    t_cam.start()
    t_ctrl.start()
    
    # Start Web Server
    app.run(host="0.0.0.0", port=5000, debug=False, use_reloader=False)