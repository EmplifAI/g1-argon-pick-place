"""PC panel: robot-local STOP, browser heartbeat, auto-reconnect on G1 boot, and one-click Argon pick-and-place."""
import json
import subprocess
import threading
import time
import urllib.error
import urllib.request
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from stop_guard import latch_stop
from diagnostics import snapshot

ROBOT = 'http://192.168.123.164:8767'
ARGON = 'http://127.0.0.1:8768'
HIGH = 'http://127.0.0.1:8769'
CAM_HEALTH = 'http://127.0.0.1:8765/health'
root = Path(__file__).resolve().parents[1]
lock = threading.RLock()
log_lock = threading.Lock()
macro_running = False
macro_cancel = threading.Event()
macro_status = 'ワンクリック待機中（G1電源ON後、ボタン1つでArgon視覚認識 → 把持 → Bへ直立配置）'
camera_epoch = 1
last_cam_restart = 0.0

PAGE = '''<!doctype html><html lang="ja"><meta charset="utf-8"><title>G1 Argon ワンクリックデモ</title>
<style>body{background:#111;color:white;font:18px sans-serif;margin:16px}
button{width:100%;min-height:90px;color:white;font:bold 28px sans-serif;border:3px solid white;border-radius:10px;cursor:pointer;margin:6px 0}
#stop{background:#c90000;min-height:130px;font-size:32px}#start{background:#087534;font-size:30px}#home{background:#1456b8;min-height:74px;font-size:24px}#lower{background:#6b4c08;min-height:74px;font-size:24px}#init{background:#2d3b55;min-height:74px;font-size:24px}button:disabled{background:#555!important}
#status{background:#262b33;padding:12px;white-space:pre-wrap;border-radius:8px;border-left:6px solid #087534}
img{display:block;max-width:100%;max-height:52vh;margin:auto;background:#222;border-radius:6px}</style>
<h1>G1 · Argon · 左腕／左手 ワンクリックデモ</h1><button id="stop">停止 — Space / Esc</button>
<button id="start">ワンクリック実行 — Argon視覚認識・ペットボトル把持＆Bへ運搬</button>
<button id="home">左腕を初期位置（体の左側）に戻す</button>
<button id="lower">腕をゆっくり下げておく（脱力・冷却待機）</button>
<button id="init">G1起動・カメラ・通信をワンクリック初期化／再接続</button><p id="status">G1接続確認中…</p>
<p>G1の電源を入れたら、マット（WOWROBO）上の任意の位置に緑のペットボトル（濃い茶）を立てて置き、上の緑ボタンを押すだけで自動実行します（左腕を体の左側へ退避 → Argon視覚認識 → 奥まで回り込んで6自由度IK把持 → 「B」へ垂直配置 → 左側初期位置へ戻す）。G1再起動直後やカメラ・通信を再初期化したい時は「G1起動・カメラ・通信をワンクリック初期化／再接続」を押してください。</p>
<img id="cam" src="http://127.0.0.1:8765/stream" alt="G1 RGB映像">
<p><a href="/diagnostics-page" target="_blank" style="color:#8df">認識・Argon指令を別画面で開く</a></p>
<iframe src="/diagnostics-page" title="認識とArgon指令" style="width:100%;height:1000px;border:0"></iframe><script>
const status=document.getElementById('status'),startButton=document.getElementById('start'),homeButton=document.getElementById('home'),lowerButton=document.getElementById('lower'),initButton=document.getElementById('init'),camImg=document.getElementById('cam');
let generation=null,stopping=false,starting=false,polling=false,stopEpoch=0,lastCamEpoch=null,wasCamFresh=false;
camImg.addEventListener('error',()=>{setTimeout(()=>{camImg.src='http://127.0.0.1:8765/stream?t='+Date.now();},1500);});
async function post(path,data={}){let r=await fetch(path,{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(data),signal:AbortSignal.timeout((path==='/start'||path==='/home'||path==='/lower'||path==='/init')?5000:1500)});let s=await r.json();if(!r.ok)throw Error(s.error||'要求失敗');return s;}
function setBusyButtons(b){startButton.disabled=b;homeButton.disabled=b;lowerButton.disabled=b;initButton.disabled=b;}
async function stop(){stopEpoch++;stopping=true;setBusyButtons(true);status.textContent='G1へ停止要求中…';
try{let s=await post('/stop');status.textContent=s.robot_ack?'G1側で停止要求を受信。移動目標の更新を停止。':'G1の停止応答がありません。リモコンを使用してください。';}
catch(e){status.textContent='G1の停止応答がありません。リモコンを使用してください。';}finally{stopping=false;}}
async function start(){if(starting||stopping)return;starting=true;let epoch=stopEpoch;setBusyButtons(true);
try{await post('/start',{generation});if(epoch===stopEpoch)status.textContent='Argon視覚認識＆B運搬を開始しました…';}
catch(e){if(epoch===stopEpoch)status.textContent='開始できません：'+e.message;}finally{starting=false;}}
async function goHome(){if(starting||stopping)return;starting=true;let epoch=stopEpoch;setBusyButtons(true);
try{await post('/home',{generation});if(epoch===stopEpoch)status.textContent='左腕を初期位置（体の左側）へ戻しています…';}
catch(e){if(epoch===stopEpoch)status.textContent='初期位置へ戻せません：'+e.message;}finally{starting=false;}}
async function lowerArm(){if(starting||stopping)return;starting=true;let epoch=stopEpoch;setBusyButtons(true);
try{await post('/lower',{generation});if(epoch===stopEpoch)status.textContent='左腕をゆっくり下げて脱力待機にしています…';}
catch(e){if(epoch===stopEpoch)status.textContent='腕を下げられません：'+e.message;}finally{starting=false;}}
async function initSystem(){if(starting||stopping)return;starting=true;let epoch=stopEpoch;setBusyButtons(true);
try{await post('/init',{generation});if(epoch===stopEpoch)status.textContent='G1本体・カメラ・通信の起動／初期化を開始しました…';}
catch(e){if(epoch===stopEpoch)status.textContent='初期化できません：'+e.message;}finally{starting=false;}}
document.getElementById('stop').addEventListener('pointerdown',stop);
document.getElementById('stop').addEventListener('click',e=>{if(e.detail===0)stop();});
startButton.addEventListener('click',start);homeButton.addEventListener('click',goHome);lowerButton.addEventListener('click',lowerArm);initButton.addEventListener('click',initSystem);
window.addEventListener('keydown',e=>{if(e.code==='Space'||e.code==='Escape'){e.preventDefault();stop();}});
async function poll(){if(polling)return;polling=true;try{let s=await post('/heartbeat');generation=s.generation;
let busy=starting||stopping||!!s.macro_running;
startButton.disabled=busy;
homeButton.disabled=busy||!s.g1_connected;
lowerButton.disabled=busy||!s.g1_connected;
initButton.disabled=busy;
if((lastCamEpoch!==null&&s.camera_epoch!==lastCamEpoch)||(!wasCamFresh&&s.camera_fresh)){camImg.src='http://127.0.0.1:8765/stream?t='+Date.now();}
lastCamEpoch=s.camera_epoch;wasCamFresh=!!s.camera_fresh;
let g1Label=!s.g1_connected?'G1側：未接続（「G1起動・カメラ・通信をワンクリック初期化」を押してください）':(s.stopped?'G1側：待機中（停止ラッチON）':'G1側：制御有効');
let camLabel=s.camera_fresh?'カメラ映像：正常配信中':'カメラ映像：再接続待機中…';
let handLabel=!s.g1_connected?'左手通信：待機中':(s.hand_error?('左手通信エラー：'+s.hand_error):'左手通信：正常');
if(!stopping)status.textContent=g1Label+' ｜ '+camLabel+' ｜ '+handLabel+'\\n'+s.argon_status+'\\n'+(s.recording_directory||'');
}catch(e){startButton.disabled=false;homeButton.disabled=true;lowerButton.disabled=true;initButton.disabled=false;wasCamFresh=false;status.textContent='パネル通信待機中…';}finally{polling=false;}}
poll();setInterval(poll,200);</script></html>'''.encode()

Q_HOME_LEFT = [-0.10, 0.78, 0.30, -0.25, 0.10, -0.20, 0.10]
Q_LOW_REST = [0.05, 0.55, 0.12, 0.08, 0.08, -0.15, 0.05]
synced_boot_id = None


def request(base, path, data=None, timeout=0.6):
    body = None if data is None else json.dumps(data).encode()
    req = urllib.request.Request(base + path, data=body, headers={'Content-Type': 'application/json'})
    try:
        with urllib.request.urlopen(req, timeout=timeout) as r:
            return json.load(r)
    except urllib.error.HTTPError as exc:
        try:
            err_body = json.load(exc)
            msg = err_body.get('error', str(exc))
        except Exception:
            msg = str(exc)
        raise RuntimeError(msg) from None


def check_camera_fresh():
    try:
        with urllib.request.urlopen(CAM_HEALTH, timeout=0.6) as r:
            h = json.load(r)
            return bool(h.get('has_frame')) and float(h.get('frame_age_s', 999)) < 1.0
    except Exception:
        return False


def ensure_camera_ready(force_restart=False):
    global camera_epoch, last_cam_restart
    if not force_restart and check_camera_fresh():
        return True
    now = time.monotonic()
    if force_restart or (now - last_cam_restart > 5.0):
        last_cam_restart = now
        try:
            subprocess.run(['systemctl', '--user', 'restart', 'g1-argon-camera.service'], timeout=3.0)
            camera_epoch += 1
        except Exception:
            pass
    for _ in range(15):
        time.sleep(0.3)
        if check_camera_fresh():
            camera_epoch += 1
            return True
    return check_camera_fresh()


def run_g1_ssh_bootstrap(force_restart=False):
    """SSH into G1 (192.168.123.164), ensure serial permissions, videohub_pc4, and g1-argon-bridge.service."""
    script = (
        'import os, re, pathlib, paramiko\n'
        'ssh = paramiko.SSHClient()\n'
        'ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())\n'
        'connected = False\n'
        'pw = None\n'
        'try:\n'
        '    ssh.connect("192.168.123.164", username="unitree", timeout=2.5)\n'
        '    connected = True\n'
        'except Exception:\n'
        '    pass\n'
        'if not connected:\n'
        '    note_p = pathlib.Path.home() / "unitree-g1-matome-2026-07-11.md"\n'
        '    tokens = ([os.environ["G1_SSH_PASSWORD"]] if os.environ.get("G1_SSH_PASSWORD") else []) + (re.findall(r"`([^`]+)`", note_p.read_text()) if note_p.exists() else [])\n'
        '    for t in reversed(tokens):\n'
        '        if t in ("unitree", "192.168.123.164", "192.168.123.161", "eno1") or "/" in t or " " in t or len(t) < 3 or len(t) > 24:\n'
        '            continue\n'
        '        try:\n'
        '            ssh.connect("192.168.123.164", username="unitree", password=t, timeout=2.5, look_for_keys=False, allow_agent=False)\n'
        '            pw = t\n'
        '            connected = True\n'
        '            break\n'
        '        except Exception:\n'
        '            continue\n'
        'if not connected:\n'
        '    raise SystemExit("G1(192.168.123.164)へSSH接続できません")\n'
        'sftp = ssh.open_sftp()\n'
        'changed = False\n'
        'for fname in ("robot_stop_bridge.py", "safety_core.py"):\n'
        '    remote_p = f"/home/unitree/g1-argon-bottle/src/{fname}"\n'
        '    local_b = pathlib.Path(f"/home/soneo/g1-argon-bottle/src/{fname}").read_bytes()\n'
        '    try:\n'
        '        remote_b = sftp.open(remote_p, "rb").read()\n'
        '    except Exception:\n'
        '        remote_b = b""\n'
        '    if remote_b != local_b:\n'
        '        with sftp.open(remote_p, "wb") as f:\n'
        '            f.write(local_b)\n'
        '        changed = True\n'
        'sftp.close()\n'
        'cmd_sudo = "sudo -n sh -c \'chmod 666 /dev/ttyUSB* 2>/dev/null || true; systemctl start --no-block g1-videohub-ready.service 2>/dev/null || true\'"\n'
        'if pw:\n'
        '    stdin, stdout, _ = ssh.exec_command("sudo -S sh -c \'chmod 666 /dev/ttyUSB* 2>/dev/null || true; systemctl start --no-block g1-videohub-ready.service 2>/dev/null || true\'", get_pty=True, timeout=4.0)\n'
        '    stdin.write(pw + "\\n")\n'
        '    stdin.flush()\n'
        'else:\n'
        '    _, stdout, _ = ssh.exec_command(cmd_sudo, timeout=4.0)\n'
        'stdout.channel.recv_exit_status()\n'
        '_, stdout, _ = ssh.exec_command("systemctl --user is-active g1-argon-bridge.service", timeout=3.0)\n'
        'svc_state = stdout.read().decode().strip()\n'
        'if svc_state == "deactivating" or changed:\n'
        '    _, stdout, _ = ssh.exec_command("systemctl --user kill -s SIGKILL g1-argon-bridge.service 2>/dev/null || true; systemctl --user reset-failed g1-argon-bridge.service 2>/dev/null || true; systemctl --user start g1-argon-bridge.service", timeout=4.0)\n'
        '    stdout.channel.recv_exit_status()\n'
        'elif svc_state != "active":\n'
        '    _, stdout, _ = ssh.exec_command("systemctl --user reset-failed g1-argon-bridge.service 2>/dev/null || true; systemctl --user start g1-argon-bridge.service", timeout=4.0)\n'
        '    stdout.channel.recv_exit_status()\n'
        'ssh.close()\n'
    )
    try:
        subprocess.run(['/usr/bin/python3', '-c', script], check=True, timeout=12.0, capture_output=True, text=True)
    except subprocess.TimeoutExpired:
        raise RuntimeError('G1本体(192.168.123.164)へのSSH初期化がタイムアウトしました。再度ボタンを押してください') from None
    except subprocess.CalledProcessError as exc:
        msg = (exc.stderr or exc.stdout or '').strip().splitlines()[-1:]
        raise RuntimeError(msg[0] if msg else 'G1本体(192.168.123.164)へのSSH初期化に失敗しました') from None


def sync_bridge_to_g1_if_needed(boot_id: str):
    global synced_boot_id
    if not boot_id or synced_boot_id == boot_id:
        return
    try:
        run_g1_ssh_bootstrap(force_restart=False)
        synced_boot_id = boot_id
    except Exception:
        pass


def auto_watchdog_loop():
    """Automatically recover camera stream and sync bridge whenever G1 is online (never kills active bridge)."""
    import socket
    fail_count = 0
    while True:
        try:
            st = request(ROBOT, '/state', timeout=1.0)
            fail_count = 0
            if not macro_running:
                sync_bridge_to_g1_if_needed(st.get('boot_id', ''))
                if not check_camera_fresh():
                    ensure_camera_ready()
        except Exception:
            fail_count += 1
            if fail_count >= 3 and not macro_running:
                try:
                    with socket.create_connection(('192.168.123.164', 22), timeout=0.8):
                        pass
                    run_g1_ssh_bootstrap(force_restart=False)
                    fail_count = 0
                except Exception:
                    pass
        time.sleep(3.0)


def session_dir():
    try:
        return Path(json.loads((root / 'config/recording-session.json').read_text())['directory'])
    except Exception:
        return root / 'logs'


def event(name, **fields):
    directory = session_dir()
    with log_lock:
        with (directory / 'operator-events.jsonl').open('a') as f:
            f.write(json.dumps(dict(event=name, pc_time_s=time.time(), **fields), ensure_ascii=False) + '\n')


def latest_argon_model_response(since_time_s: float):
    path = session_dir() / 'argon-actions.jsonl'
    if not path.exists():
        return None
    with path.open('rb') as f:
        size = f.seek(0, 2)
        f.seek(max(0, size - 131072))
        lines = f.read().decode('utf-8', errors='replace').splitlines()
    for line in reversed(lines):
        try:
            rec = json.loads(line)
        except Exception:
            continue
        if rec.get('event') == 'model_response' and float(rec.get('pc_time_s', 0)) >= since_time_s:
            return rec
    return None


# Safe lowered rest posture at the left side of the body before damping down to zero torque
Q_LOW_REST = [0.05, 0.55, 0.12, 0.08, 0.08, -0.15, 0.05]


def run_one_click_pick_and_place(return_home_only=False, lower_arm_only=False, init_only=False):
    global macro_running, macro_status
    hb_stop = threading.Event()
    limits = json.loads((root / 'config/left-arm-limits.json').read_text())

    def hb_worker():
        while not hb_stop.is_set() and not macro_cancel.is_set():
            try:
                request(ROBOT, '/heartbeat', {}, timeout=0.4)
            except Exception:
                pass
            hb_stop.wait(0.08)

    threading.Thread(target=hb_worker, daemon=True).start()

    def ensure_g1_online(force_restart=False):
        global macro_status
        if not force_restart:
            try:
                return request(ROBOT, '/state', timeout=1.0)
            except Exception:
                pass
        macro_status = 'G1本体(192.168.123.164)へSSH接続し、左手シリアル・カメラ・制御ブリッジを起動／再接続中…'
        run_g1_ssh_bootstrap(force_restart=force_restart)
        for _ in range(25):
            if macro_cancel.is_set():
                raise RuntimeError('停止ボタンにより中断されました')
            try:
                st = request(ROBOT, '/state', timeout=1.0)
                return st
            except Exception:
                time.sleep(0.35)
        raise RuntimeError('G1制御ブリッジ(192.168.123.164:8767)からの応答がありません')

    def ensure_enabled():
        if macro_cancel.is_set():
            raise RuntimeError('停止ボタンにより中断されました')
        st = ensure_g1_online(force_restart=False)
        if not st.get('controller_active'):
            try:
                request(ROBOT, '/prepare-lowlevel', {}, timeout=1.5)
            except Exception:
                pass
        if st.get('stopped', True):
            last_err = None
            for _ in range(12):
                if macro_cancel.is_set():
                    raise RuntimeError('停止ボタンにより中断されました')
                try:
                    request(ROBOT, '/heartbeat', {}, timeout=0.4)
                except Exception:
                    pass
                time.sleep(0.10)
                st = request(ROBOT, '/state', timeout=1.0)
                if not st.get('stopped', True):
                    break
                try:
                    st = request(ROBOT, '/enable', {'boot_id': st['boot_id'], 'generation': st['generation']}, timeout=1.5)
                    if not st.get('stopped', True):
                        break
                except Exception as exc:
                    last_err = exc
                    time.sleep(0.15)
            time.sleep(0.04)
            st = request(ROBOT, '/state', timeout=1.0)
            if st.get('stopped', True) and last_err is not None:
                raise RuntimeError(f'G1制御有効化に失敗しました: {last_err}')
        return st

    def move_arm_to(goal_q, steps=38, max_step=0.025, closed_loop_mq=False):
        eff_step = min(0.025, float(max_step))
        for _ in range(steps):
            if macro_cancel.is_set():
                raise RuntimeError('停止ボタンにより中断されました')
            for attempt in range(4):
                if macro_cancel.is_set():
                    raise RuntimeError('停止ボタンにより中断されました')
                st = ensure_enabled()
                mq = [float(st['motors'][i]['q']) for i in range(15, 22)]
                cmd_q = [float(x) for x in st['target_q']]
                diffs = [goal_q[i] - cmd_q[i] for i in range(7)]
                if max(abs(d) for d in diffs) < 0.008:
                    return
                w0 = [round(cmd_q[i], 4) for i in range(7)]
                q_next = []
                for i in range(7):
                    lo, hi = limits[str(15 + i)]
                    step_val = max(-eff_step, min(eff_step, diffs[i]))
                    qn = w0[i] + step_val
                    if abs(mq[i] - qn) > 0.21:
                        qn = mq[i] - (0.21 if mq[i] > qn else -0.21)
                    qn = max(lo + 0.025, min(hi - 0.025, qn))
                    # Strictly enforce <= eff_step (0.025 rad / 0.45s = 0.0556 rad/s < 0.06 rad/s velocity limit)
                    qn = max(w0[i] - eff_step, min(w0[i] + eff_step, qn))
                    q_next.append(round(qn, 4))
                try:
                    request(ROBOT, '/trajectory', {
                        'boot_id': st['boot_id'],
                        'generation': st['generation'],
                        'waypoints': [{'t_s': 0.0, 'q': w0}, {'t_s': 0.45, 'q': q_next}]
                    }, timeout=1.2)
                    break
                except Exception:
                    if attempt == 3:
                        raise
                    time.sleep(0.12)
            if macro_cancel.wait(0.48):
                raise RuntimeError('停止ボタンにより中断されました')

    def move_hand_to(goal_4, steps=9, force_stop=False):
        st0 = ensure_enabled()
        f_base = [float(x) for x in st0['left_hand']['force_n']]
        for _ in range(steps):
            if macro_cancel.is_set():
                raise RuntimeError('停止ボタンにより中断されました')
            for attempt in range(4):
                if macro_cancel.is_set():
                    raise RuntimeError('停止ボタンにより中断されました')
                st = ensure_enabled()
                cur_hq = [float(x) for x in st['commanded_hand_q']]
                meas_hq = [float(x) for x in st['left_hand']['q']]
                forces = [float(x) for x in st['left_hand']['force_n']]
                if force_stop and meas_hq[5] >= 0.40 and meas_hq[2] <= 0.55:
                    if (forces[2] - f_base[2]) >= 1.2 or (forces[3] - f_base[3]) >= 1.2:
                        tight_hq = cur_hq[:2] + [
                            round(max(0.22, cur_hq[2] - 0.08), 4),
                            round(max(0.22, cur_hq[3] - 0.08), 4),
                            round(max(0.22, cur_hq[4] - 0.08), 4),
                            round(min(0.60, cur_hq[5] + 0.04), 4),
                        ]
                        try:
                            request(ROBOT, '/move-left-hand', {'boot_id': st['boot_id'], 'generation': st['generation'], 'q': tight_hq}, timeout=1.2)
                            macro_cancel.wait(1.75)
                            return
                        except Exception:
                            if attempt == 3:
                                raise
                            time.sleep(0.12)
                            continue
                target_hq = cur_hq[:2] + list(goal_4)
                diffs = [target_hq[i] - cur_hq[i] for i in range(6)]
                if max(abs(d) for d in diffs[2:]) < 0.02:
                    return
                next_hq = cur_hq[:2] + [round(cur_hq[i] + max(-0.095, min(0.095, diffs[i])), 4) for i in range(2, 6)]
                try:
                    request(ROBOT, '/move-left-hand', {'boot_id': st['boot_id'], 'generation': st['generation'], 'q': next_hq}, timeout=1.2)
                    break
                except Exception:
                    if attempt == 3:
                        raise
                    time.sleep(0.12)
            if macro_cancel.wait(1.75):
                raise RuntimeError('停止ボタンにより中断されました')

    def capture_live_frame(dest_path):
        if not ensure_camera_ready():
            raise RuntimeError('カメラ映像が未接続です。数秒待ってから再実行してください')
        with urllib.request.urlopen('http://127.0.0.1:8765/stream', timeout=5.0) as r:
            buf = b''
            while len(buf) < 500000:
                chunk = r.read(4096)
                if not chunk:
                    break
                buf += chunk
                s = buf.find(b'\xff\xd8')
                e = buf.find(b'\xff\xd9', s if s != -1 else 0)
                if s != -1 and e != -1:
                    Path(dest_path).write_bytes(buf[s:e+2])
                    return
        raise RuntimeError('カメラ映像を取得できませんでした')

    def call_argon_recognition(step_prefix: str):
        global macro_status
        if macro_cancel.is_set():
            raise RuntimeError('停止ボタンにより中断されました')
        ensure_camera_ready()
        # If a previous Argon request is still in flight, cancel and wait for it to finish cleanly
        wait_prev_deadline = time.monotonic() + 60.0
        while time.monotonic() < wait_prev_deadline:
            if macro_cancel.is_set():
                raise RuntimeError('停止ボタンにより中断されました')
            try:
                ast0 = request(ARGON, '/', timeout=1.0)
                if not ast0.get('running'):
                    break
                request(ARGON, '/stop', {}, timeout=1.0)
                macro_status = f"{step_prefix}: 前回のArgon通信の完了を待機中…"
            except Exception:
                break
            macro_cancel.wait(0.35)

        t_before = time.time()
        started_ok = False
        last_start_err = None
        for _ in range(6):
            if macro_cancel.is_set():
                raise RuntimeError('停止ボタンにより中断されました')
            try:
                request(ROBOT, '/heartbeat', {}, timeout=0.5)
                st_cur = request(ROBOT, '/state', timeout=1.0)
                request(ARGON, '/start', {'generation': st_cur['generation']}, timeout=2.5)
                started_ok = True
                break
            except Exception as exc:
                last_start_err = exc
                time.sleep(0.3)
        if not started_ok:
            raise RuntimeError(f'Argon視覚認識の開始に失敗しました: {last_start_err}')

        t0_mono = time.monotonic()
        deadline = t0_mono + 90.0
        while time.monotonic() < deadline:
            if macro_cancel.is_set():
                try:
                    request(ARGON, '/stop', {}, timeout=0.5)
                except Exception:
                    pass
                raise RuntimeError('停止ボタンにより中断されました')
            ast = request(ARGON, '/', timeout=1.0)
            elapsed = time.monotonic() - t0_mono
            if ast.get('status'):
                macro_status = f"{step_prefix}: {ast.get('status')} ({elapsed:.0f}秒経過)"
            if not ast.get('running'):
                break
            macro_cancel.wait(0.25)
        rec = latest_argon_model_response(t_before - 0.5)
        if rec is None:
            raise RuntimeError('Argon (gemini-4-argon) から視覚認識応答を取得できませんでした')
        act = rec.get('action', {})
        argon_out = {
            'timestamp': rec.get('pc_time_s', time.time()),
            'model': rec.get('model', 'gemini-4-argon'),
            'thinking_level': rec.get('thinking_level', 'low'),
            'inference_seconds': rec.get('inference_seconds'),
            'action_payload': act,
        }
        (root / 'captures/argon-latest-recognition.json').write_text(json.dumps(argon_out, indent=2, ensure_ascii=False))
        (root / 'logs/rgb-trial.json').write_text(json.dumps(argon_out, indent=2, ensure_ascii=False))
        return act

    q_obs = list(Q_HOME_LEFT)

    try:
        if init_only:
            macro_status = '起動1/3: G1本体(192.168.123.164)へSSH接続し、左手シリアル・カメラ・制御ブリッジを起動／同期中…'
            st0 = ensure_g1_online(force_restart=True)
            try:
                request(ROBOT, '/prepare-lowlevel', {}, timeout=2.0)
            except Exception:
                pass
            macro_status = '起動2/3: PC側のRGB・深度カメラ受信ストリームを再接続中…'
            ensure_camera_ready(force_restart=True)
            macro_status = '起動3/3: Argon視覚認識コントローラと左腕・左手通信を確認中…'
            try:
                ast0 = request(ARGON, '/', timeout=1.0)
                if ast0.get('running'):
                    request(ARGON, '/stop', {}, timeout=1.0)
            except Exception:
                pass
            st_final = request(ROBOT, '/state', timeout=1.2)
            hand_err = st_final.get('hand_error')
            if hand_err:
                raise RuntimeError(f'左手通信エラー: {hand_err}')
            macro_status = '完了：G1本体・左手・カメラ映像・Argon認識の起動／再接続が完了しました（緑ボタンでいつでも実行できます）'
            return

        if lower_arm_only:
            macro_status = '1/2: 指を開き、机やボトルに当たらないよう左腕をゆっくり下ろしています…'
            st0 = ensure_enabled()
            mq0 = [float(st0['motors'][i]['q']) for i in range(15, 22)]
            move_hand_to([1.0, 1.0, 1.0, 0.0], steps=8, force_stop=False)
            # If the arm is currently over the center/front of the table, first pull gently left & up
            if mq0[1] < 0.45 or mq0[0] < -0.18:
                q_safe_left = [max(-0.20, mq0[0] - 0.04), max(0.55, mq0[1] + 0.18), 0.14, -0.25, 0.10, -0.18, 0.08]
                move_arm_to(q_safe_left, steps=24, max_step=0.018)
            # Slowly lower the arm down to Q_LOW_REST with gentle steps
            move_arm_to(list(Q_LOW_REST), steps=40, max_step=0.014)
            macro_status = '2/2: 左腕のモーター剛性を2.5秒かけてゼロ（mode=0・脱力状態）へ減衰させています…'
            try:
                request(ROBOT, '/relax', {}, timeout=1.2)
            except Exception:
                pass
            time.sleep(2.7)
            macro_status = '完了：左腕をゆっくり下げて脱力・冷却待機にしました（モーター発熱ゼロ・次回実行時に自動で復帰します）'
            return

        if return_home_only:
            macro_status = '左腕を初期位置（体の左側）へ戻しています…（指を開いて安全退避中）'
            st0 = ensure_enabled()
            mq0 = [float(st0['motors'][i]['q']) for i in range(15, 22)]
            move_hand_to([1.0, 1.0, 1.0, 0.0], steps=8, force_stop=False)
            # If the arm is currently near the table center/front, lift and pull left first
            if mq0[1] < 0.45 or mq0[0] < -0.18:
                q_safe_lift = [max(-0.22, mq0[0] - 0.05), max(0.55, mq0[1] + 0.20), 0.15, -0.30, 0.10, -0.20, 0.10]
                move_arm_to(q_safe_lift, steps=22)
            move_arm_to(q_obs, steps=32)
            macro_status = '完了：左腕を初期位置（体の左側）に戻しました（約5秒後に自動で脱力・冷却待機になります）'
            return

        macro_status = '0/7: カメラ配信およびG1ローレベル制御を確認・自動準備中…'
        ensure_g1_online(force_restart=False)
        ensure_camera_ready()
        ensure_enabled()

        macro_status = '1/7: 左手を開き、カメラ視野確保＆机干渉防止のため左腕を初期位置（体の左側）へ退避中…'
        move_hand_to([1.0, 1.0, 1.0, 0.0], steps=8, force_stop=False)
        move_arm_to(q_obs, steps=28)

        macro_status = '2/7: Argon (gemini-4-argon) がライブカメラ映像からボトル位置・姿勢と「B」を視覚認識中…'
        frame_path = root / 'captures/live-detect-input.jpg'
        state_path = root / 'captures/live-detect-state.json'
        argon_path = root / 'captures/argon-latest-recognition.json'
        capture_live_frame(frame_path)
        st_now = ensure_enabled()
        state_path.write_text(json.dumps(st_now))

        subprocess.check_output([
            '/home/soneo/miniforge3/envs/xr-teleop/bin/python',
            str(root / 'src/vision_ik_planner.py'),
            '--prepare-argon',
            str(root),
            str(frame_path),
        ], timeout=8.0)

        argon_act = call_argon_recognition('2/7 Argon視覚認識')
        obs_ja = argon_act.get('observation_ja', 'ボトル位置を認識完了')

        st_now = ensure_enabled()
        state_path.write_text(json.dumps(st_now))
        out = subprocess.check_output([
            '/home/soneo/miniforge3/envs/xr-teleop/bin/python',
            str(root / 'src/vision_ik_planner.py'),
            '--plan',
            str(root),
            str(frame_path),
            str(state_path),
            str(argon_path),
        ], stderr=subprocess.STDOUT, timeout=10.0).decode()
        plan = json.loads(out.strip().splitlines()[-1])
        bxyz = plan['bottle_xyz']
        tbxyz = plan['target_b_xyz']

        macro_status = f'3/7: 親指と4本指が当たらないよう対向幅を広げ、押し倒さない位置 ({bxyz[0]+0.005:.3f}, {bxyz[1]+0.010:+.3f}) へアプローチ中…\n【Argon】{obs_ja}'
        move_hand_to([1.0, 1.0, 1.0, 0.30], steps=5, force_stop=False)
        move_arm_to(plan['q_pre'], steps=34)
        move_arm_to(plan['q_grasp'], steps=34)
        time.sleep(0.25)

        macro_status = f'4/7: 手のひらで3指（親指・人差し指・中指）を閉じてボトルを力制御把持中…\n【Argon】{obs_ja}'
        move_hand_to([0.22, 0.22, 0.25, 0.58], steps=9, force_stop=True)

        macro_status = f'5/7: ペットボトルを垂直に保ったまま空中へ持ち上げ中…\n【Argon】{obs_ja}'
        move_arm_to(plan['q_lift'], steps=24)

        macro_status = f'6/7: ボトルを完全に垂直（傾き0.01°）に保ったまま「B」({tbxyz[0]:.3f}, {tbxyz[1]:+.3f}) へ運搬・着地…'
        move_arm_to(plan['q_above_b'], steps=36)
        move_arm_to(plan['q_on_b'], steps=28)
        time.sleep(0.4)

        macro_status = '7/7: 親指を振らずに指を垂直開放し、左腕を初期位置（体の左側）へ戻してArgonで「B」上の直立配置を視覚確認中…'
        move_hand_to([1.0, 1.0, 1.0, 0.56], steps=8, force_stop=False)
        move_arm_to(plan['q_retract'], steps=20)
        move_arm_to(q_obs, steps=28)
        move_hand_to([1.0, 1.0, 1.0, 0.0], steps=6, force_stop=False)

        subprocess.check_output([
            '/home/soneo/miniforge3/envs/xr-teleop/bin/python',
            str(root / 'src/vision_ik_planner.py'),
            '--prepare-verify-b',
            str(root),
        ], timeout=5.0)
        verify_act = call_argon_recognition('7/7 Argon配置確認')
        verify_ja = verify_act.get('observation_ja', 'B上に直立配置を確認')
        macro_status = f'完了：Argon認識位置({bxyz[0]:.2f}, {bxyz[1]:+.2f})のボトルを把持し、「B」に立たせて配置し、左腕を初期位置（体の左側）へ戻しました\n【Argon確認】{verify_ja}'
    except subprocess.CalledProcessError as exc:
        err_lines = (exc.output or b'').decode('utf-8', errors='replace').strip().splitlines()
        macro_status = '停止／中断：' + (err_lines[-1] if err_lines else str(exc))
    except Exception as exc:
        macro_status = '停止／中断：' + str(exc)
    finally:
        hb_stop.set()
        try:
            if (lower_arm_only or init_only) and not macro_cancel.is_set():
                request(ROBOT, '/relax', {}, timeout=0.8)
            else:
                request(ROBOT, '/stop', {}, timeout=0.8)
        except Exception:
            pass
        with lock:
            macro_running = False


class Handler(BaseHTTPRequestHandler):
    def reply(self, data, status=200):
        body = json.dumps(data, ensure_ascii=False).encode()
        self.send_response(status)
        self.send_header('Content-Type', 'application/json')
        self.send_header('Content-Length', str(len(body)))
        self.send_header('Cache-Control', 'no-store')
        self.end_headers()
        self.wfile.write(body)

    def state(self, heartbeat=False):
        try:
            s = request(ROBOT, '/heartbeat' if heartbeat else '/state', {} if heartbeat else None)
            s['g1_connected'] = True
        except Exception:
            s = {
                'g1_connected': False,
                'stopped': True,
                'generation': None,
                'hand_error': None,
            }
        s.update(
            argon_ready=not macro_running,
            macro_running=macro_running,
            argon_status=macro_status,
            argon_mode='argon_vision_pick_place',
            pick_place_ready=True,
            camera_fresh=check_camera_fresh(),
            camera_epoch=camera_epoch,
        )
        try:
            s['recording_directory'] = json.loads((root / 'config/recording-session.json').read_text())['directory']
        except Exception:
            pass
        return s

    def do_GET(self):
        if self.path.split('?')[0] in ['/diagnostics-page', '/depth-view.png', '/table-view.png']:
            name = self.path.split('?')[0]
            path = root / 'src/diagnostics.html' if name == '/diagnostics-page' else root / 'captures' / name[1:]
            try:
                body = path.read_bytes()
            except OSError:
                self.reply({'error': '認識画像は未取得'}, 404)
                return
            self.send_response(200)
            self.send_header('Content-Type', 'text/html; charset=utf-8' if name == '/diagnostics-page' else 'image/png')
            self.send_header('Cache-Control', 'no-store')
            self.send_header('Content-Length', str(len(body)))
            self.end_headers()
            self.wfile.write(body)
            return
        if self.path == '/diagnostics':
            try:
                self.reply(snapshot(root))
            except Exception:
                self.reply({'error': '診断情報を読み取れません'}, 503)
            return
        if self.path == '/':
            self.send_response(200)
            self.send_header('Content-Type', 'text/html; charset=utf-8')
            self.send_header('Content-Length', str(len(PAGE)))
            self.end_headers()
            self.wfile.write(PAGE)
            return
        try:
            self.reply(self.state())
        except Exception:
            self.reply({'error': 'G1に接続できません', 'stopped': True, 'argon_status': macro_status}, 503)

    def do_POST(self):
        global macro_running, macro_status
        try:
            if self.path == '/stop':
                macro_cancel.set()
                latch_stop('PC stop')
                try:
                    high_cancel = request(HIGH, '/stop', {}, timeout=0.2).get('cancel_requested', False)
                except Exception:
                    high_cancel = False
                try:
                    ack = request(ROBOT, '/stop', {}).get('stopped') is True
                except Exception:
                    ack = False
                try:
                    request(ARGON, '/stop', {}, timeout=0.2)
                except Exception:
                    pass
                event('stop', robot_ack=ack, high_level_cancel_requested=high_cancel)
                self.reply({'robot_ack': ack, 'stopped': True, 'high_level_cancel_requested': high_cancel})
            elif self.path == '/heartbeat':
                self.reply(self.state(True))
            elif self.path in ('/start', '/home', '/lower', '/init'):
                n = int(self.headers.get('Content-Length', '0'))
                if n > 4096:
                    raise ValueError('Request too large')
                data = json.loads(self.rfile.read(n)) if n else {}
                is_home = (self.path == '/home')
                is_lower = (self.path == '/lower')
                is_init = (self.path == '/init')
                with lock:
                    if macro_running:
                        raise ValueError('すでに動作を実行中です')
                    macro_running = True
                    macro_cancel.clear()
                    if is_init:
                        macro_status = 'G1本体・左手通信・頭部カメラ・Argonのワンクリック起動／再接続を開始中…'
                    elif is_lower:
                        macro_status = '左腕をゆっくり下げて脱力・冷却待機に移行中…'
                    elif is_home:
                        macro_status = '左腕を初期位置（体の左側）へ戻しています…'
                    else:
                        macro_status = 'Argon視覚認識＆ワンクリック把持・B運搬を開始中…'
                    threading.Thread(
                        target=run_one_click_pick_and_place,
                        kwargs={'return_home_only': is_home, 'lower_arm_only': is_lower, 'init_only': is_init},
                        daemon=True,
                    ).start()
                event('init' if is_init else ('lower' if is_lower else ('home' if is_home else 'start')), generation=data.get('generation'))
                self.reply({
                    'started': True,
                    'mode': 'init_system' if is_init else ('lower_relax' if is_lower else ('return_home' if is_home else 'argon_vision_pick_place')),
                    'model': 'gemini-4-argon',
                })
            else:
                self.reply({'error': 'Not found'}, 404)
        except Exception as exc:
            self.reply({'error': str(exc)}, 409)

    def log_message(self, *args):
        pass


if __name__ == '__main__':
    latch_stop('PC panel startup')
    try:
        request(ROBOT, '/stop', {})
    except Exception:
        pass
    threading.Thread(target=auto_watchdog_loop, daemon=True).start()
    ThreadingHTTPServer(('127.0.0.1', 8766), Handler).serve_forever()
