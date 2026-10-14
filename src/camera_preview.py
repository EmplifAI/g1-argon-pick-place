#!/usr/bin/env python3
"""Display the G1 video hub JPEG stream in a local browser."""
import threading
import time
import webbrowser
import socket
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.go2.video.video_client import VideoClient

latest = None
latest_at = 0.0
frame_count = 0
import json
condition = threading.Condition()


def fetch_frames():
    global latest, latest_at, frame_count
    client = VideoClient()
    client.SetTimeout(3.0)
    client.Init()
    while True:
        try:
            code, data = client.GetImageSample()
            if code == 0 and data:
                with condition:
                    latest = bytes(data)
                    latest_at = time.monotonic()
                    frame_count += 1
                    condition.notify_all()
                time.sleep(0.03)
            else:
                print(f"Video hub returned code {code}", flush=True)
                time.sleep(0.25)
        except Exception as exc:
            print(f"Video hub request failed: {exc}", flush=True)
            time.sleep(1)


class Handler(BaseHTTPRequestHandler):
    def do_GET(self):
        if self.path == "/health":
            body=json.dumps({"frame_age_s": time.monotonic()-latest_at, "frame_count": frame_count, "has_frame": latest is not None}).encode()
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
            return
        if self.path == "/" or self.path.startswith("/?"):
            page = b'''<!doctype html><meta charset="utf-8"><title>G1 RealSense</title>
<style>body{margin:0;background:#171717;color:#ddd;font:16px sans-serif}header{padding:12px}img{display:block;max-width:100vw;max-height:90vh;margin:auto}</style>
<header>G1 RealSense video hub</header><img src="/stream">
'''
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Content-Length", str(len(page)))
            self.end_headers()
            self.wfile.write(page)
            return
        if self.path != "/stream":
            self.send_error(404)
            return
        self.send_response(200)
        self.send_header("Cache-Control", "no-store")
        self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
        self.end_headers()
        last = None
        try:
            while True:
                with condition:
                    condition.wait_for(lambda: latest is not None and latest is not last, timeout=5)
                    frame = latest
                if frame is None or frame is last:
                    continue
                self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: " + str(len(frame)).encode() + b"\r\n\r\n" + frame + b"\r\n")
                self.wfile.flush()
                last = frame
        except (BrokenPipeError, ConnectionResetError):
            pass

    def log_message(self, *args):
        pass


ChannelFactoryInitialize(0, "enp12s0")
with socket.socket() as probe:
    if probe.connect_ex(("127.0.0.1", 8765)) == 0:
        print("G1 RealSense preview is already running at http://127.0.0.1:8765/", flush=True)
        webbrowser.open("http://127.0.0.1:8765/")
        raise SystemExit(0)
threading.Thread(target=fetch_frames, daemon=True).start()
print("Open http://127.0.0.1:8765/ in Ubuntu Browser; Ctrl+C stops preview.", flush=True)
threading.Timer(1.0, lambda: webbrowser.open("http://127.0.0.1:8765/")).start()
ThreadingHTTPServer(("127.0.0.1", 8765), Handler).serve_forever()
