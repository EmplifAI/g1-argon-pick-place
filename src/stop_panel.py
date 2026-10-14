"""PC panel: robot-local STOP, browser heartbeat, explicit Argon START."""
import json
import threading
import time
import urllib.request
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from stop_guard import latch_stop
from diagnostics import snapshot
ROBOT='http://192.168.123.164:8767'
ARGON='http://127.0.0.1:8768'
root=Path(__file__).resolve().parents[1]
log_lock=threading.Lock()
PAGE='''<!doctype html><html lang="ja"><meta charset="utf-8"><title>G1 Argon</title>
<style>body{background:#111;color:white;font:18px sans-serif;margin:16px}
button{width:100%;min-height:100px;color:white;font:bold 32px sans-serif;border:3px solid white;border-radius:10px;cursor:pointer;margin:6px 0}
#stop{background:#c90000;min-height:145px}#start{background:#087534}button:disabled{background:#555!important}
#status{background:#333;padding:12px;white-space:pre-wrap}img{display:block;max-width:100%;max-height:52vh;margin:auto}</style>
<h1>G1 · Argon · 左腕／左手</h1><button id="stop">停止 — Space / Esc</button>
<button id="start" disabled>開始 — Argonで実行</button><p id="status">接続確認中・動作禁止</p>
<p>緑のボトル → 緑のROBOのB。停止で手を開きません。吊り具を維持してください。</p>
<img src="http://127.0.0.1:8765/stream" alt="G1 RGB映像">
<p><a href="/diagnostics-page" target="_blank" style="color:#8df">認識・Argon指令を別画面で開く</a></p>
<iframe src="/diagnostics-page" title="認識とArgon指令" style="width:100%;height:1000px;border:0"></iframe><script>
const status=document.getElementById('status'),startButton=document.getElementById('start');
let generation=null,stopping=false,starting=false,polling=false,stopEpoch=0;
async function post(path,data={}){let r=await fetch(path,{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(data),signal:AbortSignal.timeout(path==='/start'?3000:1200)});let s=await r.json();if(!r.ok)throw Error(s.error||'要求失敗');return s;}
async function stop(){stopEpoch++;stopping=true;startButton.disabled=true;status.textContent='G1へ停止要求中…';
try{let s=await post('/stop');status.textContent=s.robot_ack?'G1側で停止要求を受信。移動目標の更新を停止。':'G1の停止応答がありません。リモコンを使用してください。';}
catch(e){status.textContent='G1の停止応答がありません。リモコンを使用してください。';}finally{stopping=false;}}
async function start(){if(starting||stopping||generation===null)return;starting=true;let epoch=stopEpoch;startButton.disabled=true;
try{await post('/start',{generation});if(epoch===stopEpoch)status.textContent='Argonが最新映像と状態を確認中…';}
catch(e){if(epoch===stopEpoch)status.textContent='開始できません：'+e.message;}finally{starting=false;}}
document.getElementById('stop').addEventListener('pointerdown',stop);
document.getElementById('stop').addEventListener('click',e=>{if(e.detail===0)stop();});startButton.addEventListener('click',start);
window.addEventListener('keydown',e=>{if(e.code==='Space'||e.code==='Escape'){e.preventDefault();stop();}});
async function poll(){if(polling)return;polling=true;try{let s=await post('/heartbeat');generation=s.generation;
startButton.disabled=starting||stopping||!s.argon_ready||s.controller_active&&!s.stopped;
if(!stopping)status.textContent=(s.stopped?'G1側：停止ラッチON':'G1側：制御有効')+'\\n'+s.reason+'\\n'+s.argon_status+'\\n'+(s.recording_directory||'');
}catch(e){startButton.disabled=true;status.textContent='G1監視信号を確認できません。通信断ではG1側で停止します。';}finally{polling=false;}}
poll();setInterval(poll,100);</script></html>'''.encode()

def request(base,path,data=None,timeout=.4):
    body=None if data is None else json.dumps(data).encode()
    req=urllib.request.Request(base+path,data=body,headers={'Content-Type':'application/json'})
    with urllib.request.urlopen(req,timeout=timeout) as r:return json.load(r)

def event(name,**fields):
    try:directory=Path(json.loads((root/'config/recording-session.json').read_text())['directory'])
    except Exception:directory=root/'logs'
    with log_lock:
        with (directory/'operator-events.jsonl').open('a') as f:
            f.write(json.dumps(dict(event=name,pc_time_s=time.time(),**fields))+'\n')

class Handler(BaseHTTPRequestHandler):
    def reply(self,data,status=200):
        body=json.dumps(data).encode();self.send_response(status)
        self.send_header('Content-Type','application/json');self.send_header('Content-Length',str(len(body)))
        self.send_header('Cache-Control','no-store');self.end_headers();self.wfile.write(body)
    def state(self,heartbeat=False):
        s=request(ROBOT,'/heartbeat' if heartbeat else '/state',{} if heartbeat else None)
        try:
            a=request(ARGON,'/state',timeout=.1);s.update(argon_ready=a.get('ready',False),argon_status=a.get('status',''))
        except Exception:s.update(argon_ready=False,argon_status='Argon制御プロセス準備中')
        try:s['recording_directory']=json.loads((root/'config/recording-session.json').read_text())['directory']
        except Exception:pass
        return s
    def do_GET(self):
        if self.path.split('?')[0] in ['/diagnostics-page','/depth-view.png','/table-view.png']:
            name=self.path.split('?')[0]
            path=root/'src/diagnostics.html' if name=='/diagnostics-page' else root/'captures'/name[1:]
            try:body=path.read_bytes()
            except OSError:self.reply({'error':'認識画像は未取得'},404);return
            self.send_response(200);self.send_header('Content-Type','text/html; charset=utf-8' if name=='/diagnostics-page' else 'image/png')
            self.send_header('Cache-Control','no-store');self.send_header('Content-Length',str(len(body)));self.end_headers();self.wfile.write(body);return
        if self.path=='/diagnostics':
            try:self.reply(snapshot(root))
            except Exception:self.reply({'error':'診断情報を読み取れません'},503)
            return
        if self.path=='/':
            self.send_response(200);self.send_header('Content-Type','text/html; charset=utf-8');self.send_header('Content-Length',str(len(PAGE)));self.end_headers();self.wfile.write(PAGE);return
        try:self.reply(self.state())
        except Exception:self.reply({'error':'G1に接続できません','stopped':True},503)
    def do_POST(self):
        try:
            if self.path=='/stop':
                latch_stop('PC stop')
                try:ack=request(ROBOT,'/stop',{}).get('stopped') is True
                except Exception:ack=False
                try:request(ARGON,'/stop',{},timeout=.1)
                except Exception:pass
                event('stop',robot_ack=ack);self.reply({'robot_ack':ack,'stopped':True})
            elif self.path=='/heartbeat':self.reply(self.state(True))
            elif self.path=='/start':
                n=int(self.headers.get('Content-Length','0'))
                if n>4096:raise ValueError('Request too large')
                data=json.loads(self.rfile.read(n));result=request(ARGON,'/start',data,timeout=1.5)
                event('start',generation=data.get('generation'));self.reply(result)
            else:self.reply({'error':'Not found'},404)
        except Exception as exc:self.reply({'error':str(exc)},409)
    def log_message(self,*args):pass

if __name__=='__main__':
    latch_stop('PC panel startup')
    try:request(ROBOT,'/stop',{})
    except Exception:pass
    ThreadingHTTPServer(('127.0.0.1',8766),Handler).serve_forever()
