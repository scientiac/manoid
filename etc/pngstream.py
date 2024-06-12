#! /usr/bin/env python

from flask import Flask, Response
import cv2

app = Flask(__name__)


def generate_frames():
    img = cv2.imread("sample.png")  # Replace with your image path
    _, buffer = cv2.imencode(".jpg", img)
    frame = buffer.tobytes()
    while True:
        yield (b"--frame\r\n" b"Content-Type: image/jpeg\r\n\r\n" + frame + b"\r\n")


@app.route("/video_feed")
def video_feed():
    return Response(
        generate_frames(), mimetype="multipart/x-mixed-replace; boundary=frame"
    )


if __name__ == "__main__":
    app.run(debug=True)
