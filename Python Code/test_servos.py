# test_servos.py

from gpiozero import AngularServo
from time import sleep

# Adjust min_pulse and max_pulse if servo hits limits
# Standard SG90 servos usually work well with these values
pan = AngularServo(18, min_angle=-90, max_angle=90, min_pulse_width=0.0005, max_pulse_width=0.0025)
tilt = AngularServo(19, min_angle=-90, max_angle=90, min_pulse_width=0.0005, max_pulse_width=0.0025)

print("Testing Pan (Left/Right)")
pan.angle = -45
sleep(1)
pan.angle = 45
sleep(1)
pan.angle = 0 # Center

print("Testing Tilt (Up/Down)")
tilt.angle = -20
sleep(1)
tilt.angle = 20
sleep(1)
tilt.angle = 0 # Center

print("Done.")