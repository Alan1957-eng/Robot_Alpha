# cliff_detection.py
 
from megapi import *

bot = MegaPi()
bot.start('/dev/ttyUSB0')

def check_hazards():
    # Read Ultrasonic on Port 6
    # megapi.ultrasonicSensorRead(port, callback)
    bot.ultrasonicSensorRead(6, on_dist)
    
    # Read Line Follower on Port 7
    bot.lineFollowerRead(7, on_line)

def on_dist(distance):
    if distance < 15: # Less than 15cm
        print("OBSTACLE! Stopping.")
        bot.motorRun(M1, 0)
        bot.motorRun(M2, 0)

def on_line(value):
    # Value 0 means black line OR sensor is over empty space (cliff)
    # Value 3 means white floor
    if value == 0: 
        print("CLIFF DETECTED! Backing up.")
        bot.motorRun(M1, -50)
        bot.motorRun(M2, -50)

# In your main loop, you call check_hazards() frequently.