# identify_people.py

import face_recognition
import cv2
import os
import numpy as np

# --- LOAD KNOWN FACES ---
path = "known_people"
known_face_encodings = []
known_face_names = []

print("Loading known faces...")
for filename in os.listdir(path):
    if filename.endswith(".jpg") or filename.endswith(".png"):
        # Load image
        img = face_recognition.load_image_file(f"{path}/{filename}")
        # Encode face (get the 128 measurements)
        encoding = face_recognition.face_encodings(img)[0]
        
        known_face_encodings.append(encoding)
        # Use filename as name (e.g., "alain.jpg" -> "Alain")
        known_face_names.append(os.path.splitext(filename)[0].capitalize())

print(f"Loaded: {known_face_names}")

# --- START CAMERA ---
video_capture = cv2.VideoCapture(0)

while True:
    ret, frame = video_capture.read()
    # Resize frame to 1/4 size for faster processing on Pi
    small_frame = cv2.resize(frame, (0, 0), fx=0.25, fy=0.25)
    rgb_small_frame = cv2.cvtColor(small_frame, cv2.COLOR_BGR2RGB)

    # Find faces in current frame
    face_locations = face_recognition.face_locations(rgb_small_frame)
    face_encodings = face_recognition.face_encodings(rgb_small_frame, face_locations)

    face_names = []
    for face_encoding in face_encodings:
        # Compare found face to known faces
        matches = face_recognition.compare_faces(known_face_encodings, face_encoding)
        name = "Stranger"

        # Find best match
        face_distances = face_recognition.face_distance(known_face_encodings, face_encoding)
        best_match_index = np.argmin(face_distances)
        if matches[best_match_index]:
            name = known_face_names[best_match_index]

        face_names.append(name)
        print(f"I see: {name}")

        # --- LOGIC TRIGGER HERE ---
        # if name == "Alain":
        #    play_sound("hello_alain.mp3")

    # Display results (Optional)
    for (top, right, bottom, left), name in zip(face_locations, face_names):
        top *= 4; right *= 4; bottom *= 4; left *= 4
        cv2.rectangle(frame, (left, top), (right, bottom), (0, 0, 255), 2)
        cv2.putText(frame, name, (left + 6, bottom - 6), cv2.FONT_HERSHEY_DUPLEX, 0.5, (255, 255, 255), 1)

    cv2.imshow('Video', frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

video_capture.release()
cv2.destroyAllWindows()