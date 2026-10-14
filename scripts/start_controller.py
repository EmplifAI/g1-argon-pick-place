"""Start Argon with credentials in an anonymous pipe, never a file/argument."""
import getpass
import os
import subprocess
import sys
from pathlib import Path
root=Path(__file__).resolve().parents[1]
key=os.environ.get('GEMINI_API_KEY') or getpass.getpass('Gemini API key: ')
if not key:raise SystemExit('Missing API key')
(root/'logs').mkdir(exist_ok=True)
with (root/'logs/argon-controller-console.log').open('ab',buffering=0) as log:
    child=subprocess.Popen([sys.executable,str(root/'src/argon_controller.py')],stdin=subprocess.PIPE,stdout=log,stderr=log,start_new_session=True)
    child.stdin.write((key+'\n').encode());child.stdin.close()
(root/'config/argon-controller.pid').write_text(str(child.pid))
print('Argon PID:',child.pid,'Operator panel: http://127.0.0.1:8766/')
