"""Read-only operator diagnostics; no actuator calls."""
import json
import time
from pathlib import Path

def read_json(path):
    try:
        return json.loads(path.read_text())
    except (OSError, ValueError):
        return None

def snapshot(root):
    root = Path(root)
    scene = read_json(root/'captures/scene-targets.json')
    session = read_json(root/'config/recording-session.json') or {}
    directory = Path(session.get('directory', str(root/'logs')))
    events = []
    path = directory/'argon-actions.jsonl'
    if path.exists():
        with path.open('rb') as f:
            size = f.seek(0, 2)
            start = max(0, size-256000)
            f.seek(start)
            if start:
                f.readline()
            for line in f:
                try:
                    event = json.loads(line)
                    if event.get('event') != 'observation':
                        events.append(event)
                except ValueError:
                    pass
    capture = root/'captures/depth.npy'
    return {'scene': scene, 'depth_info': read_json(root/'captures/depth-info.json'),
        'contact_state': read_json(root/'config/contact-state.json'),
        'contact_clearance': read_json(root/'captures/contact-clearance.json'),
        'depth_age_s': time.time()-capture.stat().st_mtime if capture.exists() else None,
        'scene_age_s': time.time()-scene['captured_at_pc_s'] if scene else None,
        'events': events[-40:], 'rgb_trial': read_json(root/'logs/rgb-trial.json'),
        'recording_directory': str(directory)}

def render(root):
    import numpy as np
    from PIL import Image
    from scene_reconstruction import color_points, project
    root = Path(root); cap = root/'captures'
    info = read_json(cap/'depth-info.json')
    depth = np.load(cap/'depth.npy').astype(float)*info['depth_scale_m']
    valid = (depth > .1)&(depth < 2)
    scale = np.clip((depth-.2)/1.3,0,1)
    rgb = np.stack([255*(1-scale),255*(1-np.abs(2*scale-1)),255*scale],axis=-1).astype('uint8')
    rgb[~valid] = 0
    Image.fromarray(rgb).save(cap/'depth-view.png')
    scene = read_json(cap/'scene-targets.json')
    points = color_points(np.load(cap/'depth-points.npy'),info)
    uv = np.rint(project(points,info['color_intrinsics'])).astype(int)
    plane = np.array(scene['table_plane_camera'])
    mask = (np.abs(points@plane[:3]+plane[3])<.004)&(uv[:,0]>=0)&(uv[:,0]<1920)&(uv[:,1]>=0)&(uv[:,1]<1080)
    # Use a black canvas: the RGB stream is newer than this depth snapshot.
    canvas = np.zeros((1080,1920,3),dtype='uint8')
    canvas[uv[mask,1],uv[mask,0]] = [40,230,120]
    Image.fromarray(canvas).save(cap/'table-view.png')

if __name__ == '__main__':
    render(Path(__file__).resolve().parents[1])
