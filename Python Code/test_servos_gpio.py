# test_servos_gpio.py

from gpiozero import AngularServo
from time import sleep

# --- CONFIGURATION ---
# We use min/max pulse widths that are standard for SG90 (9g) micro servos.
# If your servos buzz or don't move full range, adjust these values slightly.
PAN_PIN = 18
TILT_PIN = 19

print("Initializing Servos on GPIO 18 & 19...")

# Create Servo Objects
# min_angle/max_angle limits the software command (-90 to 90 degrees)
pan = AngularServo(PAN_PIN, min_angle=-90, max_angle=90, min_pulse_width=0.0005, max_pulse_width=0.0025)
tilt = AngularServo(TILT_PIN, min_angle=-90, max_angle=90, min_pulse_width=0.0005, max_pulse_width=0.0025)

def test_sequence():
    print(">>> Center (0)")
    pan.angle = 0
    tilt.angle = 0
    sleep(1)

    print(">>> Left / Up (-45)")
    pan.angle = -45
    tilt.angle = -45
    sleep(1)

    print(">>> Right / Down (+45)")
    pan.angle = 45
    tilt.angle = 45
    sleep(1)
    
    print(">>> Extreme Check (-90)")
    pan.angle = -90
    tilt.angle = -90
    sleep(1)
    
    print(">>> Return to Center")
    pan.angle = 0
    tilt.angle = 0
    sleep(1)

try:
    while True:
        test_sequence()
        
except KeyboardInterrupt:
    print("\nStopping...")
