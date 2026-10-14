"""Use official DFQ URDF and real depth calibration; no robot commands."""
import json
import urllib.request
from pathlib import Path
import numpy as np
import pinocchio as pin
root=Path(__file__).resolve().parents[1]
config=json.loads((root/'config/robot.json').read_text())
model=pin.buildModelFromUrdf(str(root/config['urdf']))
q=pin.neutral(model)
state=json.load(urllib.request.urlopen('http://192.168.123.164:8767/state',timeout=2))
names=['left_hip_pitch_joint','left_hip_roll_joint','left_hip_yaw_joint','left_knee_joint','left_ankle_pitch_joint','left_ankle_roll_joint',
 'right_hip_pitch_joint','right_hip_roll_joint','right_hip_yaw_joint','right_knee_joint','right_ankle_pitch_joint','right_ankle_roll_joint',
 'waist_yaw_joint','waist_roll_joint','waist_pitch_joint','left_shoulder_pitch_joint','left_shoulder_roll_joint','left_shoulder_yaw_joint',
 'left_elbow_joint','left_wrist_roll_joint','left_wrist_pitch_joint','left_wrist_yaw_joint','right_shoulder_pitch_joint','right_shoulder_roll_joint',
 'right_shoulder_yaw_joint','right_elbow_joint','right_wrist_roll_joint','right_wrist_pitch_joint','right_wrist_yaw_joint']
for i,name in enumerate(names):
    jid=model.getJointId(name)
    if jid==model.njoints:raise ValueError('Missing joint '+name)
    q[model.joints[jid].idx_q]=state['motors'][i]['q']
data=model.createData();pin.framesForwardKinematics(model,data,q)
camera=data.oMf[model.getFrameId('d435_link')]
# Camera optical axes: right/down/forward -> robot forward/left/up.
optical=pin.SE3(np.array([[0,0,1],[-1,0,0],[0,-1,0]],dtype=float),np.zeros(3))
world_camera=camera*optical
info=json.loads((root/'captures/depth-info.json').read_text())
intr=info['color_intrinsics']
result={'urdf':config['urdf'],'hand_model':config['hand_model'],
 'T_base_camera_optical':world_camera.homogeneous.tolist(),'q':q.tolist(),
 'left_arm_limits':{},'projected_frames':{},'calibration_verified':False}
for name in names[15:22]:
    j=model.joints[model.getJointId(name)];result['left_arm_limits'][name]=[float(model.lowerPositionLimit[j.idx_q]),float(model.upperPositionLimit[j.idx_q])]
for name in ['left_wrist_yaw_link','L_hand_base_link','L_index_proximal','L_index_intermediate','L_thumb_proximal']:
    fid=model.getFrameId(name)
    if fid>=model.nframes:continue
    pose=data.oMf[fid];p=world_camera.actInv(pose.translation)
    uv=[intr['fx']*p[0]/p[2]+intr['ppx'],intr['fy']*p[1]/p[2]+intr['ppy']] if p[2]>0 else None
    result['projected_frames'][name]={'base_xyz_m':pose.translation.tolist(),'camera_xyz_m':p.tolist(),'pixel':uv}
points=np.load(root/'captures/depth-points.npy')
ex=info['depth_to_color'];rotation=np.array(ex['rotation_column_major']).reshape(3,3,order='F')
color_points=points@rotation.T+np.array(ex['translation_m'])
valid=(points[:,2]>.1)&(points[:,2]<2)
color_points=color_points[valid]
uv=np.column_stack([intr['fx']*color_points[:,0]/color_points[:,2]+intr['ppx'],intr['fy']*color_points[:,1]/color_points[:,2]+intr['ppy']]).round().astype(int)
aligned=np.zeros((1080,1920),dtype=np.float32)
mask=(uv[:,0]>=0)&(uv[:,0]<1920)&(uv[:,1]>=0)&(uv[:,1]<1080)
uv=uv[mask];cp=color_points[mask]
# Closest depth wins at each color pixel.
order=np.argsort(cp[:,2])[::-1]
aligned[uv[order,1],uv[order,0]]=cp[order,2]
np.save(root/'captures/depth-aligned.npy',aligned)
np.save(root/'captures/points-color.npy',cp)
(root/'captures/geometry.json').write_text(json.dumps(result,indent=2))
print(json.dumps(result['projected_frames'],indent=2))
