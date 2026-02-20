# test_screen.py

"""
If you haven't done this yet, the Pi won't see the screen.

Run: sudo raspi-config

Go to Interface Options -> I2C.

Select Yes to enable it.

Reboot: sudo reboot

Verify it is connected: After rebooting, run this command to see if the Pi
detects the screen address (usually 0x3c):

Bash
i2cdetect -y 1
"""
from luma.core.interface.serial import i2c
from luma.core.render import canvas
from luma.oled.device import ssd1306
from PIL import ImageFont
import time

# 1. Setup the I2C interface
serial = i2c(port=1, address=0x3C)

# 2. Initialize the Screen (SSD1306 driver)
# rotate=0 is standard. If upside down, change to 2.
device = ssd1306(serial, rotate=0)

print("Displaying text...")

# 3. Draw on the "Canvas"
# The 'canvas' object is a drawing context. Everything inside the 'with' block 
# gets drawn to the screen when the block ends.
try:
    while True:
        with canvas(device) as draw:
            # Draw a black box to clear previous frame (optional as canvas clears auto)
            # draw.rectangle(device.bounding_box, outline="white", fill="black")
            
            # Draw Text
            # (x, y), "Text", fill="white"
            draw.text((0, 0), "System: ONLINE", fill="white")
            draw.text((0, 15), "Mode: ROAMING", fill="white")
            draw.text((0, 30), "Battery: 98%", fill="white")
            draw.text((0, 45), "IP: 192.168.1.15", fill="white")
            
        time.sleep(1)
except KeyboardInterrupt:
    # Clear screen on exit
    device.cleanup()
    print("Screen off.")