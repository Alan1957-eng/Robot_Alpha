# test_camera_oled.py

import cv2
import time
from luma.core.interface.serial import i2c
from luma.core.render import canvas
from luma.oled.device import ssd1306
from PIL import Image

# --- CONFIGURATION ---
# OLED Setup (Same as your other scripts)
serial = i2c(port=1, address=0x3C)
device = ssd1306(serial, rotate=0) # Change rotate to 2 if upside down

def show_error(message):
    """Displays an error message on the OLED for 3 seconds."""
    with canvas(device) as draw:
        draw.rectangle(device.bounding_box, outline="white", fill="black")
        # Split message into lines to fit small screen
        draw.text((0, 0), "SYSTEM ERROR:", fill="white")
        draw.text((0, 15), message[:20], fill="white") # First 20 chars
        draw.text((0, 25), message[20:40], fill="white") # Next 20 chars
    
    print(f"Error displayed: {message}")
    time.sleep(3) # Pause so the user can read it

def main_loop():
    print("Initializing Camera for OLED Feed...")
    
    # 0 is usually the default USB webcam or Pi Camera
    cap = cv2.VideoCapture(0)
    
    # Lower resolution for the capture to speed up processing
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)

    try:
        while True:
            try:
                # 1. Capture Frame
                ret, frame = cap.read()
                
                if not ret:
                    raise IOError("Camera capture failed")

                # 2. Process Image for OLED
                # Convert OpenCV (BGR) to PIL (RGB)
                img = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                pil_img = Image.fromarray(img)
                
                # Resize to OLED dimensions (128x64)
                # Image.ANTIALIAS is deprecated in newer Pillow, use Image.LANCZOS or Image.BILINEAR
                pil_img = pil_img.resize((device.width, device.height), Image.BILINEAR)
                
                # Convert to 1-bit monochrome (dithered looks cool!)
                pil_img = pil_img.convert("1")

                # 3. Display on OLED
                device.display(pil_img)

            except Exception as e:
                # This catches camera glitches inside the loop
                show_error(str(e))
                # "Reverting" happens automatically because the loop restarts!
                print("Attempting to revert to camera feed...")
                time.sleep(1) # Short cool-down before retrying

    except KeyboardInterrupt:
        print("Stopping...")
    finally:
        cap.release()
        device.cleanup()
        print("Camera released and display cleared.")

if __name__ == "__main__":
    main_loop()