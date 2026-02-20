# web_control.py

from flask import Flask, render_template, Response
import cv2

app = Flask(__name__)
camera = cv2.VideoCapture(0)

def generate_frames():
    while True:
        success, frame = camera.read()
        if not success:
            break
        else:
            # Encode frame to JPEG
            ret, buffer = cv2.imencode('.jpg', frame)
            frame = buffer.tobytes()
            # Stream to browser
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')

@app.route('/')
def index():
    # Simple HTML page
    return """
    <html>
      <head><title>Robot Monitor</title></head>
      <body>
        <h1>Robot Camera Feed</h1>
        <img src="/video_feed" width="640" height="480">
      </body>
    </html>
    """

@app.route('/video_feed')
def video_feed():
    return Response(generate_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

if __name__ == '__main__':
    # Host='0.0.0.0' makes it accessible from other computers
    app.run(host='0.0.0.0', port=5000)