import cv2
import time

print("Waking up the camera...")
cap = cv2.VideoCapture(0)

if not cap.isOpened():
    print("Error: Could not find the webcam. Make sure it's plugged in!")
else:
    print("Camera connected! Warming up...")
    
    # Grab the default width and height from the camera
    width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    
    # Set up the Video Writer with a VLC-friendly codec (XVID / .avi)
    # We assume a standard 30 frames per second
    fourcc = cv2.VideoWriter_fourcc(*'XVID')
    out = cv2.VideoWriter('alpha_test_video.avi', fourcc, 30.0, (width, height))

    print("Recording for exactly 20 seconds. Smile!")
    
    # Start the stopwatch
    start_time = time.time()
    
    # Keep looping until 20 seconds have passed
    while (time.time() - start_time) < 20:
        success, frame = cap.read()
        
        if success:
            # 1. Save the frame to the video file
            out.write(frame)
            
            # 2. Pop open a window to show the live feed!
            #cv2.imshow("Robot Alpha - Live Feed", frame)
            
            # 3. Listen for a keystroke. If you press 'q', it stops early.
            # (The waitKey(1) is also required for imshow to actually draw the window)
            #if cv2.waitKey(1) & 0xFF == ord('q'):
            #    print("Recording stopped early by user.")
            #   break
        else:
            print("Warning: Camera dropped a frame.")
            break

    # Shut everything down cleanly
    cap.release()
    out.release()
    cv2.destroyAllWindows() # Closes the live video window
    print("Recording complete! Open 'alpha_test_video.avi' in VLC to watch.")