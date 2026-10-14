"""Fresh Argon RGB feasibility trial. No robot mutation endpoints are used."""
import base64
import json
import sys
import time
import urllib.request
from pathlib import Path

root = Path(__file__).resolve().parents[1]
key = sys.stdin.readline().strip()
if not key:
    raise SystemExit('Missing credential on stdin')
def get(url):
    with urllib.request.urlopen(url, timeout=3) as response:
        return json.load(response)
state = get('http://192.168.123.164:8767/state')
health = get('http://127.0.0.1:8765/health')
if not health['has_frame'] or health['frame_age_s'] > .5:
    raise SystemExit('Camera is stale')
with urllib.request.urlopen('http://127.0.0.1:8765/stream', timeout=5) as response:
    buf = b''
    while len(buf) < 10000000:
        buf += response.read(4096)
        start = buf.find(b'\xff\xd8'); end = buf.find(b'\xff\xd9', max(0,start)+2)
        if start >= 0 and end >= 0:
            image = buf[start:end+2]; break
    else:
        raise SystemExit('No complete frame')
prompt = '''Analyze this fresh RGB frame for a suspended Unitree G1 LEFT RH56DFQ
hand to grasp the green PET bottle and place it at B in the green ROBO lettering.
No depth is used. No measured RGB-to-robot calibration is available. You have no
motion tools. Return JSON with observation_ja, bottle_bbox_px, B_pixel,
left_hand_bbox_px, visible_obstacles_ja, approach_ja, ready_for_cartesian_motion,
blocking_reasons_ja. Pixel coordinates use the native 1920x1080 image. Describe
visible facts separately from hypotheses. Never invent metric positions or
claim a successful grasp. ready_for_cartesian_motion must be false while measured
camera/tool calibration is absent.'''
config = json.loads((root/'config/argon.json').read_text())
body = dict(config,input=[{'type':'text','text':prompt},
    {'type':'image','mime_type':'image/jpeg','data':base64.b64encode(image).decode()}])
request = urllib.request.Request('https://generativelanguage.googleapis.com/v1beta/interactions',
    data=json.dumps(body).encode(),headers={'Content-Type':'application/json','x-goog-api-key':key})
began = time.monotonic()
with urllib.request.urlopen(request,timeout=60) as response:
    result = json.load(response)
outputs = [s for s in result.get('steps',[]) if s.get('type')=='model_output']
report = {'observed_at':time.time(),'motion_sent':False,'depth_used':False,
    'boot_id':state['boot_id'],'inference_seconds':time.monotonic()-began,
    'model':result.get('model'),'status':result.get('status'),'outputs':outputs}
(root/'captures/rgb-trial.jpg').write_bytes(image)
(root/'logs/rgb-trial.json').write_text(json.dumps(report,ensure_ascii=False,indent=2))
print(json.dumps(report,ensure_ascii=False))
