"""Deploy an observing-only gateway. Refuses to replace an active controller."""
import argparse
import getpass
import json
import urllib.error
import urllib.request
from pathlib import Path
import paramiko

parser=argparse.ArgumentParser()
parser.add_argument('--host',default='192.168.123.164')
parser.add_argument('--user',default='unitree')
parser.add_argument('--ask-password',action='store_true')
args=parser.parse_args()
try:
    state=json.load(urllib.request.urlopen('http://'+args.host+':8767/state',timeout=1))
    if state.get('controller_active'):
        raise SystemExit('Active controller: do not restart or remove holding control')
except urllib.error.URLError:
    pass
password=getpass.getpass('G1 SSH password: ') if args.ask_password else None
client=paramiko.SSHClient();client.load_system_host_keys()
client.connect(args.host,username=args.user,password=password,look_for_keys=password is None,allow_agent=password is None,timeout=5)
root=Path(__file__).resolve().parents[1]
remote='/home/'+args.user+'/g1-argon-bottle'
sftp=client.open_sftp()
for directory in [remote,remote+'/src',remote+'/config',remote+'/captures',remote+'/logs','/home/'+args.user+'/.config','/home/'+args.user+'/.config/systemd','/home/'+args.user+'/.config/systemd/user']:
    try:sftp.mkdir(directory)
    except OSError:pass
for name in ['robot_stop_bridge.py','safety_core.py','inspire_serial.py','capture_depth.py','holding_transfer.py']:
    sftp.put(str(root/'src'/name),remote+'/src/'+name)
for name in ['robot.json','left-arm-limits.json']:
    sftp.put(str(root/'config'/name),remote+'/config/'+name)
unit='''[Unit]
Description=G1 Argon left-arm gateway (startup stopped)
After=network-online.target
[Service]
ExecStart=/usr/bin/python3 {remote}/src/robot_stop_bridge.py
WorkingDirectory={remote}
Restart=on-failure
RestartSec=2
StandardOutput=append:{remote}/logs/bridge-console.log
StandardError=append:{remote}/logs/bridge-console.log
[Install]
WantedBy=default.target
'''.format(remote=remote)
with sftp.file('/home/'+args.user+'/.config/systemd/user/g1-argon-bridge.service','w') as f:f.write(unit)
sftp.close()
def run(command,sudo=False):
    stdin,stdout,stderr=client.exec_command(command,timeout=10)
    if sudo:
        secret=password or getpass.getpass('G1 sudo password: ')
        stdin.write(secret+'\n');stdin.flush()
    code=stdout.channel.recv_exit_status()
    output=stdout.read().decode();error=stderr.read().decode()
    stdin.close();stdout.close();stderr.close()
    if code:raise SystemExit('Remote command failed: '+error)
    return output
run('sudo -S -p "" setfacl -m u:'+args.user+':rw /dev/ttyUSB2',True)
run('sudo -S -p "" loginctl enable-linger '+args.user,True)
env='XDG_RUNTIME_DIR=/run/user/1000 DBUS_SESSION_BUS_ADDRESS=unix:path=/run/user/1000/bus '
run(env+'systemctl --user daemon-reload')
run(env+'systemctl --user enable --now g1-argon-bridge')
client.close()
print('Deployed observing-only gateway. No automatic motion or mode change.')
