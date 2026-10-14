"""Record RGB MJPEG at <=5 fps and robot telemetry at 10 Hz."""
import json
import threading
import time
import urllib.request
from datetime import datetime
from pathlib import Path
root = Path(__file__).resolve().parents[1]
session = root / "records" / datetime.now().strftime("%Y%m%d-%H%M%S")
session.mkdir(parents=True)
(root / "config/recording-session.json").write_text(json.dumps({"directory": str(session)}))
(session / "metadata.json").write_text(json.dumps({"goal": "green bottle to B in green ROBO",
 "model": "gemini-4-argon", "thinking_level": "high", "rgb": [1920,1080],
 "robot_clock_synchronized": False, "timestamps": "PC receive time", "credentials_recorded": False}))

def video():
    segment=0
    while True:
        try:
            with urllib.request.urlopen('http://127.0.0.1:8765/stream',timeout=5) as r:
                buf=b'';last=0;begin=time.monotonic()
                path=session/('rgb-%04d.mjpeg'%segment);frames=path.open('ab')
                with (session/'rgb-frames.jsonl').open('a') as index:
                    try:
                        while True:
                            buf+=r.read(4096);s=buf.find(b'\xff\xd8');e=buf.find(b'\xff\xd9',max(0,s)+2)
                            if s<0 or e<0:
                                if len(buf)>10000000:raise ValueError('oversized frame')
                                continue
                            frame=buf[s:e+2];buf=buf[e+2:];now=time.monotonic()
                            if now-last<.2:continue
                            if now-begin>=60:
                                frames.close();segment+=1;path=session/('rgb-%04d.mjpeg'%segment)
                                frames=path.open('ab');begin=now
                            offset=frames.tell();frames.write(frame);frames.flush()
                            index.write(json.dumps({'pc_time_s':time.time(),'pc_monotonic_s':now,
                              'file':path.name,'offset':offset,'bytes':len(frame)})+'\n');index.flush();last=now
                    finally:frames.close()
        except Exception as exc:
            print('video recording error:',type(exc).__name__,flush=True);time.sleep(1);segment+=1

threading.Thread(target=video,daemon=True).start()
print('Recording:',session,flush=True)
with (session/'telemetry.jsonl').open('a') as f:
    while True:
        begin=time.monotonic()
        try:
            with urllib.request.urlopen('http://192.168.123.164:8767/state',timeout=.3) as r:state=json.load(r)
            data={'pc_time_s':time.time(),'pc_monotonic_s':time.monotonic(),'robot':state}
        except Exception as exc:data={'pc_time_s':time.time(),'error':type(exc).__name__}
        f.write(json.dumps(data)+'\n');f.flush();time.sleep(max(0,.1-(time.monotonic()-begin)))
