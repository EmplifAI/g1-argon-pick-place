"""Check new PR commit sign-offs without executing contributor-supplied commands."""
import os
import re
import subprocess

base, head = os.environ['BASE_SHA'], os.environ['HEAD_SHA']
if not all(re.fullmatch(r'[0-9a-f]{40}', sha) for sha in (base, head)):
    raise SystemExit('Invalid commit IDs')
commits = subprocess.check_output(['git', 'rev-list', base + '..' + head], text=True).splitlines()
failed = []
for commit in commits:
    message = subprocess.check_output(['git', 'show', '-s', '--format=%B', commit], text=True)
    author = subprocess.check_output(['git', 'show', '-s', '--format=%an <%ae>', commit], text=True).strip()
    signers = re.findall(r'^Signed-off-by:\s*(.+ <[^<>\s]+@[^<>\s]+>)\s*$', message, re.M | re.I)
    if author.casefold() not in [s.casefold() for s in signers]:
        failed.append(commit[:12])
if failed:
    raise SystemExit('Missing author DCO sign-off: ' + ', '.join(failed) + '. See CONTRIBUTING.md.')
print('All new commits have author sign-offs.')
