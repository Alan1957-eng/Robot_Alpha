# main.py / 15-Feb-2026

import threading
import time
import random
import math
import queue
import sys
import json
import cv2
import mediapipe as mp
import numpy as np
import sounddevice as sd
import vosk
from luma.core.interface.serial import i2c
from luma.core.render import canvas
from luma.oled.device import ssd1306
from PIL import Image
from flask import Flask, Response
from megapi import *
from gpiozero import AngularServo, PiGPIOFactory

# ==========================================
#              CONFIGURATION
# ==========================================

# --- 1. PIN & PORT ASSIGNMENTS ---
# MegaPi (Body)
PORT_LEFT_MOTOR = 1
PORT_RIGHT_MOTOR = 2
PORT_ULTRASONIC = 6
PORT_LINE_FOLLOWER = 7

# Raspberry Pi GPIO (Head)
PIN_SERVO_PAN = 18  # GPIO 18
PIN_SERVO_TILT = 19 # GPIO 19

# Devices
I2C_PORT = 1
I2C_ADDRESS = 0x3C
CAMERA_INDEX = 0
MIC_INDEX = 1        # Check 'arecord -l' if this doesn't work
SERIAL_PORT = '/dev/ttyUSB0'

# --- 2. BEHAVIOR SETTINGS ---
AVOID_DISTANCE_OBJECT = 45  # cm (Slow down/Check)
AVOID_DISTANCE_WALL = 25    # cm (Emergency Stop)
FACE_TARGET_SIZE = 0.15     # Target face width (0.0 - 1.0)
PANIC_VOLUME_THRESHOLD = 500 # Volume level to trigger panic stop

# --- 3. GLOBAL VARIABLES ---
# Thread Communication
output_frame = None
lock = threading.Lock()
error_queue = queue.Queue()

# Robot State
system_running = True
current_state = "IDLE"
robot_mode = "IDLE"          # TRACKING / IDLE
face_detected_size = 0       # 0.0 to 1.0
heard_voice_timestamp = 0    # Time of last loud noise
voice_command = None         # "stop", "go", etc.
panic_stop = False

# Head Position
target_pan = 0
target_tilt = 0

# --- FLASK APP ---
app = Flask(__name__)

# ==========================================
#           HARDWARE INITIALIZATION
# ==========================================
print("[INIT] Starting Hardware...")

# 1. OLED Display
try:
    serial = i2c(port=I2C_PORT, address=I2C_ADDRESS)
    oled_device = ssd1306(serial, rotate=0)
    print("[INIT] OLED Active.")
except Exception as e:
    print(f"[CRITICAL] OLED Failed: {e}")
    sys.exit(1)

# 2. MegaPi (Body)
bot = MegaPi()
bot.start(SERIAL_PORT)
print("[INIT] MegaPi Connected.")

# 3. Servos (Head)
# Using PiGPIOFactory for smoother movement (requires 'sudo pigpiod' running)
try:
    factory = PiGPIOFactory()
    pan_servo = AngularServo(PIN_SERVO_PAN, min_angle=-90, max_angle=90, pin_factory=factory)
    tilt_servo = AngularServo(PIN_SERVO_TILT, min_angle=-90, max_angle=90, pin_factory=factory)
except Exception:
    # Fallback if pigpio isn't running
    print("[WARN] Pigpio not found, using standard jittery servo control.")
    pan_servo = AngularServo(PIN_SERVO_PAN, min_angle=-90, max_angle=90)
    tilt_servo = AngularServo(PIN_SERVO_TILT, min_angle=-90, max_angle=90)

# ==========================================
#        HELPER FUNCTIONS
# ==========================================
def log_error(source, message):
    full_msg = f"[{source}] {message}"
    print(f"LOGGED ERROR: {full_msg}")
    error_queue.put(full_msg)

# ==========================================
#    THREAD 1: CAMERA & OLED (MERGED)
# ==========================================
def camera_thread():
    global output_frame, target_pan, target_tilt, robot_mode, face_detected_size
    
    print("[THREAD] Camera & Vision Started")
    
    # Vision Setup
    mp_face = mp.solutions.face_detection
    face_detection = mp_face.FaceDetection(min_detection_confidence=0.6)
    
    cap = cv2.VideoCapture(CAMERA_INDEX)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)
    
    oled_frame_counter = 0

    while system_running:
        # 1. Capture
        ret, frame = cap.read()
        if not ret:
            log_error("CAM", "Frame Capture Failed")
            time.sleep(1)
            continue

        # 2. Vision Processing (Face Track)
        frame = cv2.flip(frame, 1) # Mirror image
        rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        results = face_detection.process(rgb_frame)
        
        if results.detections:
            robot_mode = "TRACKING"
            detection = results.detections[0]
            bbox = detection.location_data.relative_bounding_box
            
            x_center = bbox.xmin + (bbox.width / 2)
            y_center = bbox.ymin + (bbox.height / 2)
            face_detected_size = bbox.width
            
            # Servo Tracking Logic
            err_x = 0.5 - x_center
            err_y = 0.5 - y_center
            target_pan += (err_x * 8) 
            target_tilt += (err_y * 8)
            
            mp.solutions.drawing_utils.draw_detection(frame, detection)
        else:
            robot_mode = "IDLE"
            face_detected_size = 0

        # Update Web Stream Variable
        with lock:
            output_frame = frame.copy()

        # 3. OLED Display Update
        # (Check for errors first, otherwise show camera feed)
        try:
            if not error_queue.empty():
                # --- ERROR MODE ---
                error_msg = error_queue.get()
                with canvas(oled_device) as draw:
                    draw.rectangle(oled_device.bounding_box, outline="white", fill="black")
                    draw.text((0, 0), "!! ERROR !!", fill="white")
                    draw.text((0, 15), error_msg[:18], fill="white")
                    draw.text((0, 25), error_msg[18:36], fill="white")
                time.sleep(3) # Pause to let user read
                error_queue.task_done()
            
            else:
                # --- CAMERA MODE (Every 3rd frame to save CPU) ---
                oled_frame_counter += 1
                if oled_frame_counter % 3 == 0:
                    # Convert to PIL, Resize to 128x64, Convert to 1-bit
                    pil_img = Image.fromarray(rgb_frame)
                    pil_img = pil_img.resize((oled_device.width, oled_device.height), Image.BILINEAR)
                    pil_img = pil_img.convert("1")
                    oled_device.display(pil_img)

        except Exception as e:
            print(f"OLED Error: {e}")

        time.sleep(0.01)
    
    cap.release()

# ==========================================
#    THREAD 2: VOICE & AUDIO (MERGED)
# ==========================================
def voice_thread():
    global voice_command, panic_stop, heard_voice_timestamp
    
    print("[THREAD] Voice & Audio Started")
    
    if not os.path.exists("models/model"):
        log_error("VOICE", "Model missing in /models")
        return

    model = vosk.Model("models/model")
    rec = vosk.KaldiRecognizer(model, 16000, '["stop", "go", "turn left", "turn right"]')
    
    def callback(indata, frames, time_info, status):
        global panic_stop, heard_voice_timestamp, voice_command
        
        # 1. Calculate Volume (RMS)
        audio_data = np.frombuffer(indata, dtype=np.int16)
        volume = np.linalg.norm(audio_data) / np.sqrt(len(audio_data))
        
        # 2. Panic Check
        if volume > PANIC_VOLUME_THRESHOLD:
            print(f"!!! PANIC TRIGGER (Vol: {volume:.0f}) !!!")
            panic_stop = True
            heard_voice_timestamp = time.time() # Register as a "sound" too

        # 3. Scanning Trigger (Medium Volume)
        if volume > 100: # Threshold for "Hearing a noise"
             heard_voice_timestamp = time.time()

        # 4. Word Recognition
        if rec.AcceptWaveform(bytes(indata)):
            result = json.loads(rec.Result())
            text = result.get("text", "")
            if text:
                print(f"Heard Command: {text}")
                voice_command = text 

    # Start stream
    with sd.InputStream(samplerate=16000, blocksize=8000, dtype='int16', 
                        channels=1, callback=callback, device=MIC_INDEX):
        while system_running:
            time.sleep(0.5)

# ==========================================
#    THREAD 3: CONTROL (BRAIN)
# ==========================================
current_dist = 100
current_line = 3

def sensor_callback_dist(v): global current_dist; current_dist = v
def sensor_callback_line(v): global current_line; current_line = v

def control_thread():
    global target_pan, target_tilt, current_state, voice_command, panic_stop
    
    print("[THREAD] Control Logic Started")
    
    scan_start_time = 0
    avoid_start_time = 0
    avoid_duration = 0
    turn_dir = 1
    
    while system_running:
        
        # --- PRIORITY 1: PANIC STOP ---
        if panic_stop:
            bot.motorRun(PORT_LEFT_MOTOR, 0); bot.motorRun(PORT_RIGHT_MOTOR, 0)
            current_state = "STOPPED"
            panic_stop = False
            time.sleep(0.5)
            continue
            
        # --- PRIORITY 2: VOICE COMMANDS ---
        if voice_command:
            cmd = voice_command
            voice_command = None
            if cmd == "stop":
                current_state = "STOPPED"
                bot.motorRun(PORT_LEFT_MOTOR, 0); bot.motorRun(PORT_RIGHT_MOTOR, 0)
            elif cmd == "go":
                current_state = "ROAMING"
            elif cmd == "turn left":
                bot.motorRun(PORT_LEFT_MOTOR, -50); bot.motorRun(PORT_RIGHT_MOTOR, 50)
                time.sleep(1)
                bot.motorRun(PORT_LEFT_MOTOR, 0); bot.motorRun(PORT_RIGHT_MOTOR, 0)

        # --- PRIORITY 3: AUTONOMOUS BEHAVIOR ---
        if current_state == "ROAMING" or current_state == "TRACKING" or current_state == "SCANNING":
            
            # Update Sensors
            bot.ultrasonicSensorRead(PORT_ULTRASONIC, sensor_callback_dist)
            bot.lineFollowerRead(PORT_LINE_FOLLOWER, sensor_callback_line)
            
            # Logic Flags
            time_since_sound = time.time() - heard_voice_timestamp
            has_face = (robot_mode == "TRACKING")
            is_cliff = (current_line == 0) # 0 usually means black line/drop
            is_wall = (current_dist < AVOID_DISTANCE_WALL)
            
            # --- STATE MACHINE ---
            
            # 1. Cliff Detected
            if is_cliff:
                current_state = "CLIFF_RECOVERY"

            # 2. Wall Detected
            elif is_wall:
                current_state = "AVOIDING"
                avoid_start_time = time.time()
                avoid_duration = random.uniform(0.5, 1.2)
                turn_dir = random.choice([-1, 1])
                
            # 3. Face Detected -> Approach
            elif has_face:
                current_state = "TRACKING"
                
            # 4. Heard Noise -> Scan
            elif time_since_sound < 1.0 and current_state != "TRACKING":
                if current_state != "SCANNING": # Only print once
                    print("Heard noise, scanning...")
                current_state = "SCANNING"
                scan_start_time = time.time()
                
            else:
                current_state = "ROAMING"

        # --- MOTOR OUTPUTS BASED ON STATE ---
        
        if current_state == "CLIFF_RECOVERY":
            bot.motorRun(PORT_LEFT_MOTOR, -60); bot.motorRun(PORT_RIGHT_MOTOR, -60)
            time.sleep(0.4)
            bot.motorRun(PORT_LEFT_MOTOR, 80); bot.motorRun(PORT_RIGHT_MOTOR, -80)
            time.sleep(0.4)
            current_state = "ROAMING"

        elif current_state == "TRACKING":
            # Approach Logic
            if face_detected_size < FACE_TARGET_SIZE:
                bot.motorRun(PORT_LEFT_MOTOR, 60)
                bot.motorRun(PORT_RIGHT_MOTOR, 60)
            else:
                # Close enough
                bot.motorRun(PORT_LEFT_MOTOR, 0)
                bot.motorRun(PORT_RIGHT_MOTOR, 0)

        elif current_state == "SCANNING":
            bot.motorRun(PORT_LEFT_MOTOR, 0); bot.motorRun(PORT_RIGHT_MOTOR, 0)
            # Oscillate Head
            elapsed = time.time() - scan_start_time
            target_pan = math.sin(elapsed * 4) * 60
            
            if elapsed > 4.0: # Stop scanning after 4s
                current_state = "ROAMING"
                target_pan = 0
                target_tilt = 0

        elif current_state == "AVOIDING":
            bot.motorRun(PORT_LEFT_MOTOR, 80 * turn_dir)
            bot.motorRun(PORT_RIGHT_MOTOR, -80 * turn_dir)
            if (time.time() - avoid_start_time) > avoid_duration:
                current_state = "ROAMING"

        elif current_state == "ROAMING":
            bot.motorRun(PORT_LEFT_MOTOR, 70)
            bot.motorRun(PORT_RIGHT_MOTOR, 70)
            target_pan = 0
            target_tilt = 0

        # Update Head Servos
        # Clamp values to safe ranges
        target_pan = max(-90, min(90, target_pan))
        target_tilt = max(-60, min(60, target_tilt))
        pan_servo.angle = target_pan
        tilt_servo.angle = target_tilt
        
        time.sleep(0.05)

# ==========================================
#    WEB SERVER (FLASK)
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
def index(): return "<html><body><h1>Robot Alpha Vision</h1><img src='/video_feed'></body></html>"

@app.route("/video_feed")
def video_feed(): return Response(generate(), mimetype="multipart/x-mixed-replace; boundary=frame")

# ==========================================
#           MAIN ENTRY POINT
# ==========================================
if __name__ == '__main__':
    # Thread Definitions
    t_cam = threading.Thread(target=camera_thread, daemon=True)
    t_voice = threading.Thread(target=voice_thread, daemon=True)
    t_ctrl = threading.Thread(target=control_thread, daemon=True)
    
    # Start Threads
    t_cam.start()
    t_voice.start()
    t_ctrl.start()
    
    # Start Web Server (Blocks main thread)
    print("[SYSTEM] All Systems Go. Web Server on port 5000.")
    app.run(host="0.0.0.0", port=5000, debug=False, use_reloader=False)