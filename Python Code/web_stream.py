from flask import Flask, Response
import cv2
from gpiozero import AngularServo
from time import sleep

app = Flask(__name__)

# FIX 1: Force OpenCV to use the native Raspberry Pi Linux driver (V4L2)
camera = cv2.VideoCapture(0, cv2.CAP_V4L2)

# Initialize the Servos (Axes are correct now)
pan_servo = AngularServo(18, min_pulse_width=0.0005, max_pulse_width=0.0025)
tilt_servo = AngularServo(19, min_pulse_width=0.0005, max_pulse_width=0.0025)

# Track the angles manually
current_pan = 0
current_tilt = 0

# Wake up, center, and go to sleep to prevent jitter
pan_servo.angle = current_pan
tilt_servo.angle = current_tilt
sleep(1)
pan_servo.angle = None
tilt_servo.angle = None

HTML_PAGE = """
<!DOCTYPE html>
<html>
<head><title>Robot Alpha Mission Control</title></head>
<body style="text-align:center; font-family:Arial; background-color:#222; color:white;">
    <h1>Robot Alpha Live Feed</h1>
    <img src="/video_feed" width="640" height="480" style="border:4px solid #4CAF50; border-radius:10px;"><br><br>
    
    <div>
        <button onclick="fetch('/tilt/up')" style="padding:15px; font-size:18px; margin:5px;">Tilt Up</button><br>
        <button onclick="fetch('/pan/left')" style="padding:15px; font-size:18px; margin:5px;">Pan Left</button>
        <button onclick="fetch('/pan/right')" style="padding:15px; font-size:18px; margin:5px;">Pan Right</button><br>
        <button onclick="fetch('/tilt/down')" style="padding:15px; font-size:18px; margin:5px;">Tilt Down</button>
    </div>
</body>
</html>
"""

@app.route('/')
def index():
    return HTML_PAGE

def generate_frames():
    while True:
        success, frame = camera.read()
        if not success:
            # FIX 2: Don't give up if a frame drops! Just wait a millisecond and try again.
            sleep(0.1)
            continue
        else:
            ret, buffer = cv2.imencode('.jpg', frame)
            frame = buffer.tobytes()
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')

@app.route('/video_feed')
def video_feed():
    return Response(generate_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/pan/<direction>')
def pan(direction):
    global current_pan
    # FIX 3: Inverted the math so Left goes Left and Right goes Right
    if direction == 'left' and current_pan > -90:
        current_pan -= 15
    elif direction == 'right' and current_pan < 90:
        current_pan += 15
    
    pan_servo.angle = current_pan
    sleep(0.3) 
    pan_servo.angle = None 
    return "OK"

@app.route('/tilt/<direction>')
def tilt(direction):
    global current_tilt
    # FIX 4: Inverted the math so Up goes Up and Down goes Down
    if direction == 'up' and current_tilt > -90:
        current_tilt -= 15
    elif direction == 'down' and current_tilt < 90:
        current_tilt += 15
    
    tilt_servo.angle = current_tilt
    sleep(0.3)
    tilt_servo.angle = None
    return "OK"

if __name__ == "__main__":
    app.run(host='0.0.0.0', port=5000)