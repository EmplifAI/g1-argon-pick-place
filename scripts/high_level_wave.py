"""One explicitly authorized factory-wave test, with a PC cancel endpoint."""
import argparse
import json
import signal
import sys
import threading
import time
import urllib.request
from http.server import BaseHTTPRequestHandler,ThreadingHTTPServer
from pathlib import Path
from unitree_sdk2py.core.channel import ChannelFactoryInitialize,ChannelSubscriber
from unitree_sdk2py.idl.std_msgs.msg.dds_ import String_
from unitree_sdk2py.g1.arm.g1_arm_action_client import G1ArmActionClient
from unitree_sdk2py.g1.arm.g1_arm_action_api import ROBOT_API_ID_ARM_ACTION_EXECUTE_ACTION
from unitree_sdk2py.g1.loco.g1_loco_client import LocoClient

parser=argparse.ArgumentParser()
parser.add_argument('--allow-all',action='store_true',required=True)
parser.add_argument('--start-fsm',action='store_true')
parser.add_argument('--keep-guard',action='store_true')
parser.add_argument('--prepare-standing',action='store_true')
args=parser.parse_args()
root=Path(__file__).resolve().parents[1]
directory=Path(json.loads((root/'config/recording-session.json').read_text())['directory'])
cancel=threading.Event();released=threading.Event();active=False;release_code=None
quit_guard=threading.Event()
test_status='純正waveの準備中';action_state=None
def log(event,**fields):
    with (directory/'argon-actions.jsonl').open('a') as f:
        f.write(json.dumps(dict(event=event,pc_time_s=time.time(),**fields),ensure_ascii=False)+'\n')
def state():
    with urllib.request.urlopen('http://192.168.123.164:8767/state',timeout=2) as r:return json.load(r)
initial=state()
if initial['controller_active'] or not initial['stopped']:raise SystemExit('Low-level controller must be inactive')
ChannelFactoryInitialize(0,'enp12s0')
def action_feedback(message):
    global action_state
    try:action_state=json.loads(message.data)
    except ValueError:action_state={'raw':message.data}
subscriber=ChannelSubscriber('rt/arm/action/state',String_);subscriber.Init(action_feedback,1)
client=G1ArmActionClient();client.SetTimeout(1);client.Init()
stop_client=G1ArmActionClient();stop_client.SetTimeout(1);stop_client.Init()
loco=LocoClient();loco.SetTimeout(1);loco.Init()
stop_loco=LocoClient();stop_loco.SetTimeout(1);stop_loco.Init()
def release(stop_all=False):
    global release_code,active
    cancel.set()
    if stop_all:
        code=stop_loco.SetFsmId(1)
        log('high_level_damp_request',api_code=code)
        active=False
    if released.is_set():return
    released.set()
    release_code=stop_client.ExecuteAction(99)
    log('high_level_release',api_code=release_code)
class Handler(BaseHTTPRequestHandler):
    def do_POST(self):
        if self.path!='/stop':self.send_error(404);return
        cancel.set();threading.Thread(target=release,args=(True,),daemon=True).start()
        body=b'{"cancel_requested":true}'
        self.send_response(200);self.send_header('Content-Type','application/json');self.send_header('Content-Length',str(len(body)));self.end_headers();self.wfile.write(body)
    def do_GET(self):
        body=json.dumps({'active':active,'cancel_requested':cancel.is_set(),'release_code':release_code,'status':test_status,'action_state':action_state},ensure_ascii=False).encode()
        self.send_response(200);self.send_header('Content-Type','application/json');self.send_header('Content-Length',str(len(body)));self.end_headers();self.wfile.write(body)
    def log_message(self,*args):pass
server=ThreadingHTTPServer(('127.0.0.1',8769),Handler)
threading.Thread(target=server.serve_forever,daemon=True).start()
def terminate(*args):
    cancel.set();quit_guard.set();threading.Thread(target=release,args=(True,),daemon=True).start()
signal.signal(signal.SIGTERM,terminate)
signal.signal(signal.SIGINT,terminate)
before=state();boot=before['boot_id'];generation=before['generation']
try:
    active=True
    fsm_code,fsm=loco.GetFsmId()
    log('high_level_fsm_query',api_code=fsm_code,fsm=fsm)
    if fsm_code:raise RuntimeError('Cannot read current FSM: '+str(fsm_code))
    if fsm not in (500,501,801):
        if not args.start_fsm:raise RuntimeError('Factory wave requires FSM 500/501/801')
        if cancel.is_set():raise RuntimeError('PC STOP before FSM transition')
        sequence=[1,4,500] if args.prepare_standing else [500]
        for target_fsm in sequence:
            test_status='純正姿勢の初期化：FSM '+str(target_fsm)
            if cancel.is_set():raise RuntimeError('PC STOP during preparation')
            start_code=loco.SetFsmId(target_fsm)
            log('high_level_start_request',fsm_target=target_fsm,api_code=start_code)
            print(json.dumps({'fsm_target':target_fsm,'api_code':start_code}),flush=True)
            if start_code:raise RuntimeError('Pure controller start rejected: '+str(start_code))
            deadline=time.monotonic()+3
            while time.monotonic()<deadline and not cancel.is_set():
                current=state()
                if current['state_age']>.1 or any(m['error'] or max(m['temperature'])>=65 for m in current['motors']):
                    raise RuntimeError('Hardware fault or stale state during preparation')
                fsm_code,fsm=loco.GetFsmId()
                if fsm_code==0 and (fsm==target_fsm or target_fsm==501 and fsm in (500,501,801)):break
                cancel.wait(.1)
            log('high_level_fsm_observed',fsm_target=target_fsm,api_code=fsm_code,fsm=fsm)
            if cancel.is_set() or fsm_code or not (fsm==target_fsm or target_fsm==501 and fsm in (500,501,801)):
                raise RuntimeError('FSM did not enter requested state: '+str(target_fsm)+'; observed '+str(fsm))
            cancel.wait(.5)
    test_status='純正waveを実行・関節変位を確認中'
    code=client.ExecuteAction(25)
    log('high_level_wave_request',action_id=25,name='wave_under_head',api_code=code,
        scope='all authorized for this test',low_level_publishing=False)
    print(json.dumps({'action_id':25,'api_code':code}),flush=True)
    if code:raise RuntimeError('Factory wave API rejected the request: '+str(code))
    end=time.monotonic()+5
    max_delta=[0.0]*29
    while time.monotonic()<end and not cancel.is_set():
        current=state()
        if current['boot_id']!=boot or current['generation']!=generation:
            cancel.set();break
        if current['state_age']>.1 or any(m['error'] or max(m['temperature'])>=65 for m in current['motors']):
            cancel.set();break
        for i,m in enumerate(current['motors']):max_delta[i]=max(max_delta[i],abs(m['q']-before['motors'][i]['q']))
        cancel.wait(.05)
    log('high_level_wave_observed',max_joint_delta_rad=max_delta,cancelled=cancel.is_set(),action_state=action_state)
    print(json.dumps({'observed_joint_max_delta_rad':max_delta,'cancelled':cancel.is_set()}),flush=True)
    test_status='純正wave確認終了。PC停止は全身ダンピングを要求します'
except Exception as exc:
    test_status='純正wave停止：'+str(exc)
    log('high_level_wave_error',reason=str(exc))
    print(json.dumps({'error':str(exc)},ensure_ascii=False),flush=True)
    release(True)
finally:
    release();active=bool(args.keep_guard)
    # Keep the cancellation response visible until release returns.
    if not args.keep_guard:
        active=False;server.shutdown();server.server_close()
if args.keep_guard:
    print('Factory test complete; PC stop guardian remains available on 8769',flush=True)
    while not quit_guard.wait(.5):pass
    server.shutdown();server.server_close()
