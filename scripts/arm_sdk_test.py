"""One visible left-elbow test over Unitree's upper-body SDK, with PC STOP."""
import json
import signal
import sys
import threading
import time
import urllib.request
from pathlib import Path
from http.server import BaseHTTPRequestHandler,ThreadingHTTPServer
from unitree_sdk2py.core.channel import ChannelFactoryInitialize,ChannelPublisher
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_
root=Path(__file__).resolve().parents[1];sys.path.insert(0,str(root/'src'))
from arm_sdk_command import build_command
cancel=threading.Event();quit_guard=threading.Event();status='上半身SDKの準備中'
directory=Path(json.loads((root/'config/recording-session.json').read_text())['directory'])
def log(event,**data):
    with (directory/'argon-actions.jsonl').open('a') as f:f.write(json.dumps(dict(event=event,pc_time_s=time.time(),**data),ensure_ascii=False)+'\n')
def state():
    with urllib.request.urlopen('http://192.168.123.164:8767/state',timeout=.5) as r:return json.load(r)
initial=state()
if initial['controller_active'] or initial['state_age']>.1:raise SystemExit('Low-level controller active or stale state')
if initial['mode_machine']!=5:raise SystemExit('Expected G1 29-axis mode5')
ChannelFactoryInitialize(0,'enp12s0')
publisher=ChannelPublisher('rt/arm_sdk',LowCmd_);publisher.Init()
class Handler(BaseHTTPRequestHandler):
    def do_POST(self):
        if self.path!='/stop':self.send_error(404);return
        cancel.set();self.reply({'cancel_requested':True})
    def do_GET(self):self.reply({'active':True,'status':status,'mode':'upper_body_sdk','cancel_requested':cancel.is_set()})
    def reply(self,data):
        b=json.dumps(data,ensure_ascii=False).encode();self.send_response(200);self.send_header('Content-Type','application/json');self.send_header('Content-Length',str(len(b)));self.end_headers();self.wfile.write(b)
    def log_message(self,*args):pass
server=ThreadingHTTPServer(('127.0.0.1',8769),Handler);threading.Thread(target=server.serve_forever,daemon=True).start()
def terminate(*args):cancel.set();quit_guard.set()
signal.signal(signal.SIGTERM,terminate);signal.signal(signal.SIGINT,terminate)
before=state();boot=before['boot_id'];generation=before['generation'];q=[m['q'] for m in before['motors']]
target=q.copy();target[18]-=.35
if target[18]<-1.0272:raise SystemExit('Elbow limit')
weight=0.;start=time.monotonic();motion_started=None;max_delta=[0.]*29;moving=True
log('upper_sdk_test_started',joint_id=18,delta_rad=-.35,duration_s=5.83,topic='rt/arm_sdk',leg_targets=False,hand_writes=False)
try:
    while not quit_guard.is_set():
        tick=time.monotonic();s=state()
        if s['boot_id']!=boot or s['generation']!=generation or s['state_age']>.1:cancel.set()
        if any(m['error'] or max(m['temperature'])>=65 for m in s['motors']):cancel.set()
        if any(abs(s['motors'][i]['tau_est'])>8 for i in range(15,29)):cancel.set()
        if cancel.is_set():
            status='上半身SDK停止：移動目標を固定して保持中'
            if moving:log('upper_sdk_test_stopped',max_joint_delta_rad=max_delta)
            moving=False
        if moving:
            if weight<1:weight=min(1,weight+.02)
            elif motion_started is None:motion_started=tick
            else:
                step=max(-.0012,min(.0012,target[18]-q[18]));q[18]+=step
                status='上半身SDK：左肘を約20度持ち上げる動作中'
                if abs(q[18]-target[18])<1e-5 and tick-motion_started>6.8:
                    moving=False;status='上半身SDK確認終了：保持中（停止ボタン有効）'
                    log('upper_sdk_test_complete',max_joint_delta_rad=max_delta)
                    print(json.dumps({'max_joint_delta_rad':max_delta,'legs_commanded':False,'finger_commands':False}),flush=True)
        for i,m in enumerate(s['motors']):max_delta[i]=max(max_delta[i],abs(m['q']-before['motors'][i]['q']))
        publisher.Write(build_command(q,weight))
        time.sleep(max(0,.02-(time.monotonic()-tick)))
finally:
    # Hand control remains untouched. Return SDK authority gradually, with a
    # fixed arm target, following the factory example's weight release.
    for _ in range(100):
        weight=max(0,weight-.01);publisher.Write(build_command(q,weight));time.sleep(.02)
    server.shutdown();server.server_close()
