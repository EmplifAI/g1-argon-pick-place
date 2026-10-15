"""One Argon-selected, bounded left-arm calibration trial; never hand commands."""
import base64
import json
import sys
import threading
import time
import urllib.request
from pathlib import Path
import numpy as np
import pinocchio as pin

root=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(root/'src'))
from left_arm_kinematics import LeftArmKinematics
key=sys.stdin.readline().strip()
if not key:raise SystemExit('Missing credential')
def call(path,data=None):
    request=urllib.request.Request('http://192.168.123.164:8767'+path,
        data=None if data is None else json.dumps(data).encode(),headers={'Content-Type':'application/json'})
    with urllib.request.urlopen(request,timeout=2) as response:return json.load(response)
def frame():
    with urllib.request.urlopen('http://127.0.0.1:8765/health',timeout=2) as r:health=json.load(r)
    if not health['has_frame'] or health['frame_age_s']>.5:raise ValueError('Stale RGB')
    with urllib.request.urlopen('http://127.0.0.1:8765/stream',timeout=3) as r:
        data=b''
        while len(data)<10000000:
            data+=r.read(4096);a=data.find(b'\xff\xd8');b=data.find(b'\xff\xd9',max(0,a)+2)
            if a>=0 and b>=0:return data[a:b+2]
    raise ValueError('No RGB frame')
session=json.loads((root/'config/recording-session.json').read_text())
directory=Path(session['directory'])
trial=directory/('arm-trial-'+time.strftime('%H%M%S'));trial.mkdir()
def log(event,**fields):
    entry=dict(event=event,pc_time_s=time.time(),**fields)
    with (directory/'argon-actions.jsonl').open('a') as f:f.write(json.dumps(entry,ensure_ascii=False)+'\n')
    with (trial/'events.jsonl').open('a') as f:f.write(json.dumps(entry,ensure_ascii=False)+'\n')
state=call('/state');boot=state['boot_id'];generation=state['generation']
if state.get('hand_write_enabled') is not False:raise SystemExit('Arm-only gateway is not installed')
if state.get('disabled_left_hand_channels')!=[0,1]:raise SystemExit('Damaged finger isolation missing')
if state['state_age']>.1 or state['hand_state_age']>.3:raise SystemExit('Stale robot state')
kin=LeftArmKinematics(root);full=kin.configuration(state)
def positions(q):
    pin.framesForwardKinematics(kin.model,kin.data,q)
    return np.array([kin.data.oMf[kin.model.getFrameId(n)].translation.copy() for n in
        ['left_elbow_link','left_wrist_yaw_link','L_hand_base_link']])
initial=positions(full);candidates=[]
for joint,delta in [(18,-.35)]:
    q=full.copy();q[kin.indices[joint]]+=delta
    movement=positions(q)-initial
    # User authorized a larger upward probe. Check the complete segment.
    safe=True
    for fraction in np.linspace(0,1,21):
        intermediate=full.copy();intermediate[kin.indices[joint]]+=fraction*delta
        segment_delta=positions(intermediate)-initial
        if np.min(segment_delta[:,2]) < -1e-5 or np.max(np.linalg.norm(segment_delta,axis=1))>.12:
            safe=False;break
    if safe and kin.lower[joint-15]<=q[kin.indices[joint]]<=kin.upper[joint-15]:
        candidates.append({'joint_id':joint,'delta_rad':delta,'speed_rad_s':.06,
                           'predicted_landmark_delta_m':movement.tolist()})
image=frame();(trial/'before.jpg').write_bytes(image)
prompt='''Choose ONE short LEFT ARM calibration trial or stop. Real suspended G1,
left forearm/elbow resting on tabletop; user allows slight pressure and wants to
learn from bounded physical trials. Pinky/ring are broken. NO HAND command is
available. The arm begins at measured pose. Candidates below predict nominal
upward/non-downward elbow, wrist, palm motion under 120 mm along the full segment.
User explicitly says space above the arm is clear and requests visibly larger
upward motion: elbow 0.35 rad, about 20 degrees, at 0.06 rad/s, taking about 6 seconds.
Prior elbow trial of -0.10 rad succeeded with measured change -0.067 rad, hand
writes zero, left-only control. Shoulder-roll joint16 tracked poorly; it is excluded.
Nominal gravity torque is about -1.95 Nm at joint15 and -1.55 Nm at joint18.
Extrinsic calibration
is unverified, so this is NOT a grasp. Inspect visible bottle, cables and obstacles.
Choose stop if even this trial risks snagging or collision. Output ONLY JSON:
{"action":"trial" or "stop","joint_id":candidate ID,"observation_ja":"reason"}.
Select only from the candidates. Never claim actual movement before execution.
Candidates: '''+json.dumps(candidates)
body=dict(json.loads((root/'config/argon.json').read_text()),input=[{'type':'text','text':prompt},
    {'type':'image','mime_type':'image/jpeg','data':base64.b64encode(image).decode()}])
req=urllib.request.Request('https://generativelanguage.googleapis.com/v1beta/interactions',data=json.dumps(body).encode(),headers={'Content-Type':'application/json','x-goog-api-key':key})
with urllib.request.urlopen(req,timeout=60) as r:answer=json.load(r)
text=''.join(c.get('text','') for s in answer.get('steps',[]) if s.get('type')=='model_output' for c in s.get('content',[]) if c.get('type')=='text').strip()
if text.startswith('```'):text=text.split('\n',1)[1].rsplit('```',1)[0]
decision=json.loads(text);log('calibration_decision',action=decision,model=answer.get('model'))
print(json.dumps(decision,ensure_ascii=False),flush=True)
if decision.get('action')!='trial':raise SystemExit('Argon selected stop')
selected=next((c for c in candidates if c['joint_id']==decision.get('joint_id')),None)
if selected is None:raise SystemExit('Invalid candidate')
done=threading.Event();fault=threading.Event()
def heartbeat():
    while not done.is_set():
        try:call('/heartbeat',{})
        except Exception:fault.set();return
        done.wait(.1)
threading.Thread(target=heartbeat,daemon=True).start()
try:
    now=call('/state')
    if now['boot_id']!=boot or now['generation']!=generation:raise ValueError('STOP/reboot invalidated trial')
    if max(abs(a['q']-b['q']) for a,b in zip(now['motors'],state['motors']))>.015:raise ValueError('Pose changed during inference')
    if not now['controller_active']:call('/prepare-lowlevel',{})
    call('/heartbeat',{})
    enabled=call('/enable',{'generation':generation,'boot_id':boot})
    generation=enabled['generation']
    start=np.array(enabled['target_q']);target=start.copy();target[selected['joint_id']-15]+=selected['delta_rad']
    duration=abs(selected['delta_rad'])/selected['speed_rad_s']
    segments=max(1,int(np.ceil(abs(selected['delta_rad'])/.1)))
    waypoints=[{'t_s':duration*f,'q':(start+f*(target-start)).tolist()} for f in np.linspace(0,1,segments+1)]
    call('/trajectory',{'waypoints':waypoints,
        'generation':generation,'boot_id':boot})
    log('calibration_trial_started',candidate=selected,before=state)
    deadline=time.monotonic()+duration+1
    while time.monotonic()<deadline:
        current=call('/state');log('calibration_sample',state=current)
        if fault.is_set() or current['stopped'] or current['generation']!=generation or current['boot_id']!=boot:raise ValueError('Trial stopped')
        if current['hand_writes']!=state['hand_writes']:raise ValueError('Unexpected hand write')
        if any(m['mode']!=0 for i,m in enumerate(current['motors']) if i not in range(15,22)):raise ValueError('Non-left motor became active')
        if max(abs(current['motors'][i]['tau_est']-state['motors'][i]['tau_est']) for i in range(15,22))>4.0:raise ValueError('Trial torque increase exceeded 4 Nm')
        if any(abs(current['motors'][i]['tau_est'])>(6.0 if i<19 else 2.0) for i in range(15,22)):raise ValueError('Absolute trial torque limit')
        if max(abs(current['motors'][i]['q']-state['motors'][i]['q']) for i in range(15,22))>.40:raise ValueError('Unexpected arm displacement')
        time.sleep(.04)
    (trial/'after.jpg').write_bytes(frame())
    log('calibration_trial_complete',after=current)
    print(json.dumps({'completed':True,'joint_id':selected['joint_id'],'measured_delta_rad':current['motors'][selected['joint_id']]['q']-state['motors'][selected['joint_id']]['q'],'recording':str(trial)}),flush=True)
except Exception as exc:
    log('calibration_trial_error',reason=str(exc));raise
finally:
    try:call('/stop',{})
    finally:done.set()
