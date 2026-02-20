"""
test_servos2.py

Port 6 usually controls pins A12 (Slot 1) and A13 (Slot 2).
Port 7 usually controls pins A10 (Slot 1) and A11 (Slot 2).
Port 8 usually controls pins A8 (Slot 1) and A9 (Slot 2).

If you plug into "A6", the standard servo command might not work. 
Recommendation: Move your connectors to the pins labeled A12 and A13 (
which correspond to Port 6). If you must use A6, we have to use a different 
command, but Port 6 is the standard "Servo Home."

The Test Code: test_servos2.py
This script tells the MegaPi to wiggle the servos connected to Port 6.
"""
from megapi import *
import time

# --- CONFIGURATION ---
# If your robot doesn't move, try changing these to 7 or 8
SERVO_PORT = 6  
PAN_SLOT = 1    # Usually Pin A12
TILT_SLOT = 2   # Usually Pin A13

print(f"Testing Servos on MegaPi Port {SERVO_PORT}...")
print("Ensure MegaPi Switch is ON and Batteries are connected.")

def on_start():
    # Loop to wiggle servos
    while True:
        print("Looking Left (0 degrees)")
        # servoRun(port, slot, angle)
        bot.servoRun(SERVO_PORT, PAN_SLOT, 0)
        bot.servoRun(SERVO_PORT, TILT_SLOT, 0)
        time.sleep(1)

        print("Looking Center (90 degrees)")
        bot.servoRun(SERVO_PORT, PAN_SLOT, 90)
        bot.servoRun(SERVO_PORT, TILT_SLOT, 90)
        time.sleep(1)

        print("Looking Right (180 degrees)")
        bot.servoRun(SERVO_PORT, PAN_SLOT, 180)
        bot.servoRun(SERVO_PORT, TILT_SLOT, 180)
        time.sleep(1)

# --- INITIALIZATION ---
bot = MegaPi()
# Make sure this matches your USB connection
bot.start('/dev/ttyUSB0') 

# Execute the loop
on_start()

"""
How to Hook It Up for This Code:
Pan Motor (Left/Right): Plug into the 3-pin header labeled A12 (or Port 6 Slot 1).
Tilt Motor (Up/Down): Plug into the 3-pin header labeled A13 (or Port 6 Slot 2).

Run: python3 test_servos2.py

Troubleshooting:
Nothing happens? Check that the MegaPi switch is ON. The USB powers the brain, 
but the switch enables the muscles (servos).

Jittering? Your batteries might be low.

Wrong Pins? If you absolutely must use A6, let me know. We can try to force it,
but moving the plug to A12 (Port 6) is much easier.
"""