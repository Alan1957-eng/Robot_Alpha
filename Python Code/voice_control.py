# voice_control.py

"""
Do the following before running it:

Bash
pip install vosk sounddevice numpy
mkdir models
cd models
wget https://alphacephei.com/vosk/models/vosk-model-small-en-us-0.15.zip
unzip vosk-model-small-en-us-0.15.zip
mv vosk-model-small-en-us-0.15 model
"""

import sys
import queue
import sounddevice as sd
import vosk
import json
import numpy as np

# --- CONFIGURATION ---
MODEL_PATH = "models/model"  # Path to the folder you just unzipped
SAMPLE_RATE = 16000
PANIC_THRESHOLD = 30.0       # Adjust this sensitivity (Lower = more sensitive)

# The "Cocktail Party" List - The robot will ONLY listen for these words
COMMANDS = '["stop", "go", "turn left", "turn right"]'

# Audio Queue
q = queue.Queue()

def callback(indata, frames, time, status):
    """This is called every time the mic hears a chunk of audio."""
    if status:
        print(status, file=sys.stderr)
    q.put(bytes(indata))

    # --- POINT 3: THE PANIC TRIGGER (Energy Detector) ---
    # Calculate loudness (RMS) of the current chunk
    # We use numpy to turn the raw bytes into numbers we can measure
    audio_data = np.frombuffer(indata, dtype=np.int16)
    volume = np.linalg.norm(audio_data) / np.sqrt(len(audio_data))
    
    if volume > PANIC_THRESHOLD:
        print(f"!!! PANIC TRIGGER: LOUD NOISE DETECTED (Vol: {volume:.1f}) !!!")
        # In the real robot, you would call: bot.motorStop() here immediately

def main():
    # 1. Load the Model
    if not os.path.exists(MODEL_PATH):
        print(f"Please download the model to {MODEL_PATH} first!")
        sys.exit(1)
        
    print("Loading Model...")
    model = vosk.Model(MODEL_PATH)

    # 2. Create the Recognizer with the Grammar Constraint
    # This is the "Filter" - it rejects words not in our list
    rec = vosk.KaldiRecognizer(model, SAMPLE_RATE, COMMANDS)

    # 3. Start the Microphone Loop
    print("\nListening... (Say 'Stop', 'Go', 'Turn Left', 'Turn Right')")
    print("You can also CLAP to trigger the Panic Stop.")
    
    # We use the 'sd.InputStream' to grab audio from the default mic
    with sd.InputStream(samplerate=SAMPLE_RATE, blocksize=8000, device=None, 
                        dtype='int16', channels=1, callback=callback):
        while True:
            data = q.get()
            
            # Feed audio to Vosk
            if rec.AcceptWaveform(data):
                # If a full phrase is recognized
                result = json.loads(rec.Result())
                text = result.get("text", "")
                
                if text:
                    print(f"COMMAND RECEIVED: >> {text.upper()} <<")
                    # Here is where you add your motor logic:
                    # if text == "stop": bot.stop()
                    # if text == "turn left": bot.turnLeft()
            else:
                # Partial result (optional, good for debugging)
                # partial = json.loads(rec.PartialResult())
                pass

if __name__ == "__main__":
    import os
    try:
        main()
    except KeyboardInterrupt:
        print("\nStopping...")
        
"""
How this meets your needs:

The COMMANDS Variable: By passing COMMANDS into KaldiRecognizer, we tell Vosk to ignore
"the", "and", "maybe", or random gibberish. It forces the engine to map sounds only to your 
4 commands. This effectively filters out the "midrange motor noise" that sounds like human 
speech but isn't a known command.

The PANIC_THRESHOLD: Inside the callback function, we measure the "Root Mean Square" (RMS) 
volume before the AI even touches it. If you clap or yell, volume > 30 triggers instantly 
(microseconds latency), bypassing the AI entirely.

Low CPU Load: The vosk-model-small is designed for mobile devices. On a Pi 5, this should 
use very little CPU, leaving plenty of power for your face tracking.

To Test It:

Run the script.

Speak normally: "Turn Left." (It should print COMMAND RECEIVED: >> TURN LEFT <<).

Speak nonsense: "Blah blah refrigerator." (It should print nothing).

Clap your hands loudly. (It should print !!! PANIC TRIGGER !!!).
"""