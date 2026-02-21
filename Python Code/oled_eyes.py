import cv2
import board
import busio
import adafruit_ssd1306
from PIL import Image

# Initialize I2C and the 128x64 OLED display
# i2c = busio.I2C(board.SCL, board.SDA)
i2c = board.I2C()
disp = adafruit_ssd1306.SSD1306_I2C(128, 64, i2c)

# Clear the display
disp.fill(0)
disp.show()

cap = cv2.VideoCapture(0)

print("Displaying camera feed on OLED. Press Ctrl+C to stop.")

try:
    while True:
        ret, frame = cap.read()
        if not ret:
            break
            
        # Convert the color video into flat grayscale
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        # Shrink to 128x64, convert to 1-bit color (pure black and white) for the OLED
        image = Image.fromarray(gray).resize((128, 64)).convert('1')
        
        # Send the image to the screen
        disp.image(image)
        disp.show()

except KeyboardInterrupt:
    # Clean up and turn the screen off when you quit
    disp.fill(0)
    disp.show()
    cap.release()