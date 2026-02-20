# main_add_display_error.py

import threading
import queue
import time
import sys
import traceback
import cv2
from luma.core.interface.serial import i2c
from luma.core.render import canvas
from luma.oled.device import ssd1306
from PIL import Image

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

# ---------------------------------------------------------
# THREAD 2: CONTROL / MOTORS (Example)
# ---------------------------------------------------------
def control_thread():
    while system_running:
        try:
            # Your normal robot logic here...
            # Example of a fake error:
            # if voltage < 10:
            #    raise ValueError("Low Battery")
            time.sleep(0.1)
            
        except Exception as e:
            # CATCH EVERYTHING and send to OLED
            log_error("MOTOR", str(e))
            time.sleep(2) # Don't spam the queue instantly

# ---------------------------------------------------------
# THREAD 3: VOICE (Example)
# ---------------------------------------------------------
def voice_thread():
    while system_running:
        try:
            # Your Vosk logic...
            time.sleep(0.1)
        except Exception as e:
            log_error("VOICE", str(e))
            time.sleep(2)

# ---------------------------------------------------------
# MAIN ENTRY POINT
# ---------------------------------------------------------
if __name__ == "__main__":
    
    # 1. THE GLOBAL CRASH CATCHER (The Safety Net)
    # If the main program crashes (syntax error, memory error), this catches it.
    def handle_exception(exc_type, exc_value, exc_traceback):
        if issubclass(exc_type, KeyboardInterrupt):
            sys.__excepthook__(exc_type, exc_value, exc_traceback)
            return
        
        # Send the crash report to the OLED queue
        error_msg = f"CRASH: {exc_type.__name__}"
        log_error("SYS", error_msg)
        
        # Wait for OLED to display it before dying
        time.sleep(6) 
        sys.__excepthook__(exc_type, exc_value, exc_traceback)

    sys.excepthook = handle_exception

    # 2. Start Threads
    t_oled = threading.Thread(target=oled_thread, daemon=True)
    t_control = threading.Thread(target=control_thread, daemon=True)
    t_voice = threading.Thread(target=voice_thread, daemon=True)

    t_oled.start()
    t_control.start()
    t_voice.start()

    # 3. Keep Main Alive
    try:
        while True:
            time.sleep(1)
            # You can even test it here:
            # if time.time() % 60 == 0:
            #     log_error("TEST", "Periodic Check")
            
    except KeyboardInterrupt:
        system_running = False
        print("\nShutting down...")
        time.sleep(1)