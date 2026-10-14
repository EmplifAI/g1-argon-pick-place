"""Argon high decides every primitive; robot-local gateway enforces limits.

API key comes from stdin, lives in memory only, and is never logged.
"""
import base64
import json
import sys
import threading
import time
import urllib.error
import urllib.request
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

root=Path(__file__).resolve().parents[1]
key=sys.stdin.readline().strip()
if not key:raise SystemExit('Missing API credential on stdin')
lock=threading.RLock()
cancel=threading.Event()
running=False
status='Argon low 接続済み・開始待ち'
step_counter=0
ROBOT='http://192.168.123.164:8767'

def robot(path,data=None):
    req=urllib.request.Request(ROBOT+path,data=None if data is None else json.dumps(data).encode(),headers={'Content-Type':'application/json'})
    try:
        with urllib.request.urlopen(req,timeout=2) as r:return json.load(r)
    except urllib.error.HTTPError as e:
        body=json.load(e);raise ValueError(body.get('error',str(e))) from None

def session_dir():
    try:return Path(json.loads((root/'config/recording-session.json').read_text())['directory'])
    except Exception:return root/'logs'

def record(event,**data):
    with (session_dir()/'argon-actions.jsonl').open('a') as f:
        f.write(json.dumps(dict(event=event,pc_time_s=time.time(),**data),ensure_ascii=False)+'\n')

def frame():
    health=json.load(urllib.request.urlopen('http://127.0.0.1:8765/health',timeout=1))
    if not health['has_frame'] or health['frame_age_s']>0.5:
        raise ValueError('Camera frame is stale')
    with urllib.request.urlopen('http://127.0.0.1:8765/stream',timeout=5) as r:
        buf=b''
        while len(buf)<10000000:
            buf+=r.read(4096);s=buf.find(b'\xff\xd8');e=buf.find(b'\xff\xd9',max(0,s)+2)
            if s>=0 and e>=0:
                health=json.load(urllib.request.urlopen('http://127.0.0.1:8765/health',timeout=1))
                if health['frame_age_s']>0.5:raise ValueError('Camera stalled')
                return buf[s:e+2]
    raise ValueError('Camera frame unavailable')

PROMPT='''You control a REAL suspended Unitree G1 29-axis with LEFT RH56DFQ Inspire Hand.
Task: grasp the green bottle with the left hand and place the bottle base at the B
in the GREEN ROBO text (not the Os). Every movement decision is yours, not another policy.
You receive a fresh RGB image, robot state, and available depth/geometry metadata.
Output ONLY one JSON primitive at a time with action and observation_ja.
Actions: enable, observe, open_hand, move_left_hand, grasp_bottle,
verify_grasp, place_at, release_at_B, stop, done.
enable has no extra fields. It takes over the LEFT arm at measured position only;
it does not move to home, and it sets the left hand position to its measured grip.
move_left_hand and place_at require xyz_m in robot-base coordinates and a 3x3
rotation matrix. They perform left-arm IK, collision checks and a continuous
7-joint trajectory locally at <=0.06 rad/s; you do NOT micromanage individual joints.
grasp_bottle requires xyz_m, rotation, and grip_q (6 normalized finger positions).
open_hand opens the hand smoothly and measures a stable unloaded relative-force
baseline. verify_grasp requires evidence with opposing_fingers_visible and
bottle_held_visible. release_at_B requires bottle_base_on_table and target_error_m.
Tools return measured results, not assumed success. Observe and verify after each
coarse primitive. One tool call performs the entire movement/grip, not tiny steps.
Order: pinky, ring, middle, index, thumb-bend, thumb-rotation. 1=open, 0=closed.
Do not command right arm/hand, waist, legs, mode changes, or arbitrary code.
Observe visually after every primitive. Never push the bottle over. Opening and
retracting may be needed because the fingers are initially against the bottle side
while the thumb is outside the grasp. Do not close from an unsuitable hand pose.
Cartesian motion is available only when the empirical camera/tool calibration and
fresh RGB-D scene report calibration_verified=true. Never invent coordinates or
claim calibration. If the scene is unverified, you may observe or open the empty
hand, then stop with the specific missing calibration evidence.
Raw finger force is NOT calibrated; absolute values are not reliable yet. Do not
claim a stable grasp or lift without fresh visual and state evidence. Stop freezes
the arm target and existing grip; it never opens the hand. PC heartbeat is required.
After stop/done, no further movement is sent. done only when the bottle is upright,
released, and verified at B. Use observe rather than inventing observations.
'''

def ask(image,state,history):
    compact={k:state.get(k) for k in ['stopped','reason','generation','controller_active',
              'heartbeat_age','state_age','left_hand','hand_state_age','hand_error','target_q']}
    compact['left_arm']=state['motors'][15:22]
    compact['other_active_joints']=[m['id'] for m in state['motors'] if m['id'] not in range(15,22) and m['mode']!=0]
    content=PROMPT+'\nState:'+json.dumps(compact)+'\nPrevious decisions:'+json.dumps(history[-2:],ensure_ascii=False)
    targets=root/'captures/scene-targets.json'
    if targets.exists():content+='\nMeasured depth targets:'+targets.read_text()
    config=json.loads((root/'config/argon.json').read_text())
    body=dict(config,input=[{'type':'text','text':content},
       {'type':'image','mime_type':'image/jpeg','data':base64.b64encode(image).decode()}],
       store=False)
    req=urllib.request.Request('https://generativelanguage.googleapis.com/v1beta/interactions',data=json.dumps(body).encode(),headers={'Content-Type':'application/json','x-goog-api-key':key})
    began=time.monotonic()
    try:
        with urllib.request.urlopen(req,timeout=180) as r:result=json.load(r)
    except urllib.error.HTTPError as e:
        raise ValueError('Argon API HTTP '+str(e.code)) from None
    if result.get('status')!='completed':raise ValueError('Argon response incomplete')
    text=''.join(c.get('text','') for step in result.get('steps',[]) if step.get('type')=='model_output'
                 for c in step.get('content',[]) if c.get('type')=='text')
    text=text.strip()
    if text.startswith('```'):text=text.split('\n',1)[1].rsplit('```',1)[0].strip()
    action=json.loads(text)
    if not isinstance(action,dict):raise ValueError('Invalid Argon primitive')
    record('model_response',model=result.get('model'),usage=result.get('usage'),action=action,
           inference_seconds=time.monotonic()-began,thinking_level=config['generation_config']['thinking_level'],
           service_tier=result.get('service_tier'))
    return action

def work(initial_generation):
    global running,status,step_counter
    history=[]
    expected_generation=initial_generation
    heartbeat_done=threading.Event()
    def maintain_heartbeat():
        while not heartbeat_done.is_set() and not cancel.is_set():
            try:robot('/heartbeat',{})
            except Exception:
                cancel.set()
                try:robot('/stop',{})
                except Exception:pass
                return
            heartbeat_done.wait(0.1)
    threading.Thread(target=maintain_heartbeat,daemon=True).start()
    try:
        from coarse_primitives import PickPlaceTools
        tools=PickPlaceTools(root,robot,cancel,frame,record)
        initial_state=robot('/state')
        expected_boot=initial_state['boot_id']
        for step in range(60):
            if cancel.is_set():break
            state=robot('/state')
            if state.get('boot_id') != expected_boot:
                raise ValueError('G1が再起動しました。古い指令を破棄しました')
            if state['generation']!=expected_generation:
                raise ValueError('停止または制御世代が変わりました')
            if state['heartbeat_age']>.4:raise ValueError('PC画面の監視信号が途切れています')
            image=frame();step_counter+=1
            observed_at=time.time()
            name='argon-input-%04d.jpg'%step_counter
            (session_dir()/name).write_bytes(image)
            record('observation',image=name,state=state)
            status='Argon low 判断中（ステップ%d）'%(step+1)
            action=ask(image,state,history)
            if cancel.is_set():break
            # Detect STOP even if it arrived while the model was thinking.
            current=robot('/state')
            if current['generation']!=expected_generation:raise ValueError('Argon判断中に停止しました')
            op=action.get('action')
            status=action.get('observation_ja','Argon指令確認中')
            if op=='enable':
                if current['controller_active'] and not current['stopped']:raise ValueError('Already enabled')
                ack=robot('/enable',{'generation':expected_generation,'boot_id':expected_boot})
                expected_generation=ack['generation']
            elif op=='move_left_hand':
                ack=tools.move_left_hand(action['xyz_m'],action['rotation'],expected_generation)
            elif op=='open_hand':
                ack=tools.open_hand(expected_generation)
            elif op=='grasp_bottle':
                ack=tools.grasp_bottle(action['xyz_m'],action['rotation'],expected_generation,action['grip_q'])
            elif op=='verify_grasp':
                evidence=dict(action['evidence'],observed_at_pc_s=observed_at)
                ack=tools.verify_grasp(evidence,expected_generation)
            elif op=='place_at':
                ack=tools.place_at(action['xyz_m'],action['rotation'],expected_generation)
            elif op=='release_at_B':
                evidence=dict(action['evidence'],observed_at_pc_s=observed_at)
                ack=tools.release_at_B(evidence,expected_generation)
            elif op=='observe':
                tools.observe()
            elif op in ('stop','done'):
                robot('/stop',{})
                status=('完了：' if op=='done' else 'Argon停止：')+action.get('blocking_reason_ja',action.get('observation_ja',''))
                record(op,action=action);return
            else:raise ValueError('Unsupported Argon action '+str(op))
            history.append(action)
            record('primitive_accepted',action=action)
        if cancel.is_set():status='PC停止：Argonの遅延応答は実行しません'
        else:status='ステップ上限で停止。完了は未確認。'
    except Exception as exc:
        status='停止：'+str(exc)
        record('error',reason=str(exc))
    finally:
        heartbeat_done.set()
        try:robot('/stop',{})
        except Exception:pass
        with lock:running=False

class Handler(BaseHTTPRequestHandler):
    def reply(self,data,status_code=200):
        body=json.dumps(data,ensure_ascii=False).encode();self.send_response(status_code)
        self.send_header('Content-Type','application/json');self.send_header('Content-Length',str(len(body)))
        self.end_headers();self.wfile.write(body)
    def do_GET(self):
        self.reply({'ready':not running,'running':running,'status':status})
    def do_POST(self):
        global running,status
        try:
            if self.path=='/stop':
                cancel.set();self.reply({'stopped':True});return
            if self.path!='/start':self.reply({'error':'Not found'},404);return
            n=int(self.headers.get('Content-Length','0'))
            if n>4096:raise ValueError('Request too large')
            data=json.loads(self.rfile.read(n))
            with lock:
                if running:raise ValueError('Argon is already running')
                # Explicit start is the authority for the PC executor lease.
                # It is maintained by the process, not a throttled browser tab.
                robot('/heartbeat',{})
                state=robot('/state')
                if state['generation']!=data['generation']:raise ValueError('STOP superseded start')
                running=True;cancel.clear();status='Argon low 開始'
                threading.Thread(target=work,args=(data['generation'],),daemon=True).start()
            self.reply({'started':True,'model':'gemini-4-argon','thinking_level':'low'})
        except Exception as exc:self.reply({'error':str(exc)},409)
    def log_message(self,*args):pass

ThreadingHTTPServer(('127.0.0.1',8768),Handler).serve_forever()
