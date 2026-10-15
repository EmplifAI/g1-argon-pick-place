'use strict';
const $ = id => document.getElementById(id);
let token = '', imageData = '', imageMime = 'image/png', imageName = 'Illustrated reference scene';
let current = {}, working = false, observing = false, actionEpoch = 0, toastTimer;
function notify(message) { $('toast').textContent = message; $('toast').hidden = false; clearTimeout(toastTimer); toastTimer = setTimeout(() => $('toast').hidden = true, 8000); }
async function api(path, data) {
  const response = await fetch(path, data === undefined ? {cache:'no-store'} : {
    method:'POST', headers:{'Content-Type':'application/json','X-Console-Token':token}, body:JSON.stringify(data)
  });
  const result = await response.json();
  if (!response.ok) throw new Error(result.error || 'Request failed. Please retry.');
  return result;
}
function controls() {
  $('observe').disabled = !current.key_ready || observing || working || current.observing || !!current.controller?.running;
  $('connect').disabled = working || observing;
  $('disconnect').hidden = !current.key_ready;
  $('capture').disabled = !current.camera_fresh || observing;
  $('start').disabled = working || observing || !current.key_ready || !current.motion_available || !current.connected || !!current.robot?.error || !current.robot?.stopped || !current.camera_fresh || !!current.controller?.running || !$('suspended').checked || !$('clearance').checked;
  $('observe-help').textContent = observing ? 'Thinking… this can take a minute.' : current.key_ready ? 'One image · one observation · no motion' : 'Connect a key to begin.';
}
async function refresh() {
  try {
    current = await api('/api/status');
    $('api-badge').textContent = current.key_ready ? 'KEY CONNECTED' : 'NOT CONNECTED';
    $('robot-badge').textContent = current.connected && !current.robot.error ? 'CONNECTED' : 'OFFLINE';
    $('bridge-state').textContent = current.connected ? (current.robot.error ? 'Unavailable' : 'Connected') : 'Not connected';
    $('camera-state').textContent = current.camera_fresh ? 'Fresh frame' : 'Unavailable';
    $('control-state').textContent = current.controller.running ? 'Practice active' : current.motion_available ? 'Stopped / ready' : 'Locked';
    $('motion-help').textContent = current.motion_available ? 'Practice is limited to three small left-arm trials. No hand, waist, leg or right-arm actions.' : 'Observation mode. Restart with --enable-motion after completing hardware setup.';
    $('events').replaceChildren(...current.events.slice().reverse().map(item => {
      const li = document.createElement('li'), time = document.createElement('time'), message = document.createElement('span');
      time.textContent = item.time; message.textContent = item.message; li.append(time, message); return li;
    }));
  } catch (error) {
    current = {};
    $('api-badge').textContent = 'CONSOLE OFFLINE'; $('robot-badge').textContent = 'UNKNOWN';
    $('bridge-state').textContent = 'Unknown'; $('camera-state').textContent = 'Unknown'; $('control-state').textContent = 'Unknown';
  }
  controls();
}
async function run(fn) { working = true; controls(); try { await fn(); } catch (error) { notify(error.message); } finally { working = false; await refresh(); } }
$('connect-form').addEventListener('submit', event => { event.preventDefault(); run(async () => {
  const key = $('key').value; $('key').value = ''; $('key').type = 'password'; $('reveal').textContent = 'Show';
  await api('/api/connect', {api_key:key, model:$('model').value.trim(), thinking:$('thinking').value});
  notify('Gemini connected. Choose an image and click Observe scene.');
}); });
$('reveal').addEventListener('click', () => { const show = $('key').type === 'password'; $('key').type = show ? 'text' : 'password'; $('reveal').textContent = show ? 'Hide' : 'Show'; $('reveal').setAttribute('aria-label', show ? 'Hide API key' : 'Show API key'); });
$('disconnect').addEventListener('click', () => { actionEpoch++; run(async () => { await api('/api/disconnect', {}); notify('Disconnected. Your key has been cleared.'); }); });
function setImage(data, mime, name, source) {
  imageData = data.split(',')[1]; imageMime = mime; imageName = name;
  $('scene').src = data; $('scene').alt = name; $('source-label').textContent = source;
  $('frame-label').textContent = source === 'CAPTURED ROBOT FRAME' ? 'CAPTURED FRAME · NOT LIVE' : 'UPLOADED IMAGE';
  $('scene-name').textContent = name; $('image-info').textContent = name + ' · Sent to Gemini only when you click Observe.';
}
function readFile(file) { return new Promise((resolve, reject) => { const reader = new FileReader(); reader.onload = () => resolve(reader.result); reader.onerror = () => reject(new Error('Could not read the image.')); reader.readAsDataURL(file); }); }
$('upload').addEventListener('change', async event => { const file = event.target.files[0]; if (!file) return;
  if (!['image/jpeg','image/png','image/webp'].includes(file.type) || file.size > 4000000) { notify('Choose a JPEG, PNG or WebP smaller than 4 MB.'); return; }
  try { setImage(await readFile(file), file.type, file.name, 'UPLOADED IMAGE'); } catch (error) { notify(error.message); }
});
$('sample').addEventListener('click', () => {
  imageData = ''; imageName = 'Illustrated reference scene'; $('scene').src = '/sample.svg';
  $('scene').alt = 'Illustrated tabletop, not real robot footage'; $('source-label').textContent = 'ILLUSTRATION · NOT LIVE';
  $('frame-label').textContent = 'REFERENCE SCENE'; $('scene-name').textContent = 'A starting point for your next experiment.';
  $('image-info').textContent = 'Illustrated scene, not sensor data. Images are sent to Gemini only when you click Observe.'; $('upload').value = '';
});
$('capture').addEventListener('click', () => run(async () => {
  const response = await fetch('/api/frame'); if (!response.ok) throw new Error('Fresh camera frame unavailable.');
  const file = await response.blob(); setImage(await readFile(file), 'image/jpeg', 'Robot frame captured at ' + new Date().toLocaleTimeString(), 'CAPTURED ROBOT FRAME');
}));
async function referenceImage() {
  const img = new Image(); img.src = '/sample.svg'; await img.decode();
  const canvas = document.createElement('canvas'); canvas.width = img.naturalWidth; canvas.height = img.naturalHeight;
  canvas.getContext('2d').drawImage(img, 0, 0); return canvas.toDataURL('image/png').split(',')[1];
}
$('observe').addEventListener('click', async () => {
  if (observing) return;
  observing = true; const epoch = actionEpoch; controls();
  $('response').classList.add('has-output'); $('response').textContent = 'Inspecting the scene… No motion commands will be sent.';
  try {
    const result = await api('/api/observe', {image:imageData || await referenceImage(), mime_type:imageData ? imageMime : 'image/png', prompt:$('prompt').value + (imageData ? '' : '\nThis input is an illustration, not real sensor data.')});
    if (epoch === actionEpoch) $('response').textContent = result.text;
  } catch (error) { if (epoch === actionEpoch) { $('response').textContent = error.message; notify(error.message); } }
  finally { observing = false; await refresh(); }
});
$('robot-connect').addEventListener('click', () => run(async () => { await api('/api/robot', {}); notify('Robot connected. No motion or mode changes sent.'); }));
$('start').addEventListener('click', () => run(async () => {
  await api('/api/start', {suspended:$('suspended').checked, clearance:$('clearance').checked});
  $('suspended').checked = false; $('clearance').checked = false; notify('Bounded left-arm practice started.');
}));
async function stop() {
  actionEpoch++;
  if (observing) $('response').textContent = 'Session stopped. Any pending observation will be discarded.';
  $('stop-result').textContent = 'Requesting stop…';
  try { const result = await api('/api/stop', {}); $('stop-result').textContent = result.robot_ack ? 'Robot acknowledged stop. Movement target updates stopped.' : 'No robot acknowledgement. Use the physical remote if the robot is moving.'; }
  catch { $('stop-result').textContent = 'Stop could not be confirmed. Use the physical remote.'; }
  $('suspended').checked = false; $('clearance').checked = false; await refresh();
}
$('stop').addEventListener('click', stop);
window.addEventListener('keydown', event => { if (event.code === 'Escape') { event.preventDefault(); stop(); } });
$('suspended').addEventListener('change', controls); $('clearance').addEventListener('change', controls);
(async () => { try { token = (await api('/api/session')).token; await refresh(); } catch { notify('Could not connect to the local console. Reload to retry.'); } finally { setInterval(refresh, 2500); } })();
