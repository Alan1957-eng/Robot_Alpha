# test_motors.py

from megapi import *

# If connected via USB cable
bot = MegaPi()
bot.start('/dev/ttyUSB0') 

# If connected via GPIO pins, use:
# bot.start('/dev/ttyAMA0')

def onStart():
    print("Motors Forward!")
    # format: motorRun(motor_port, speed)
    # Speed is -255 to 255
    bot.motorRun(M1, 100) 
    bot.motorRun(M2, 100)
    sleep(2)
    
    print("Stopping...")
    bot.motorRun(M1, 0)
    bot.motorRun(M2, 0)

if __name__ == '__main__':
    onStart()