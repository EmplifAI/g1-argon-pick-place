"""Passive CAD/depth alignment diagnostics, not a calibration certificate."""
import json
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
import pinocchio as pin
import trimesh
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation
from scipy.optimize import least_squares
from left_arm_kinematics import LeftArmKinematics
from scene_reconstruction import color_points,project

def register(root):
    root=Path(root);cap=root/'captures'
    cfg=json.loads((root/'config/robot.json').read_text())
    state=json.loads((cap/'contact-state-robot.json').read_text())
    geometry=json.loads((cap/'geometry.json').read_text())
    info=json.loads((cap/'depth-info.json').read_text())
    scene=json.loads((cap/'scene-targets.json').read_text())
    kin=LeftArmKinematics(root);q=kin.configuration(state)
    pin.framesForwardKinematics(kin.model,kin.data,q)
    urdf=root/cfg['urdf'];doc=ET.parse(urdf)
    samples=[]
    # Deterministic evenly indexed mesh vertices avoid stochastic changes.
    for link in doc.findall('link'):
        name=link.get('name')
        if not name.startswith('L_'):continue
        mesh_node=link.find('visual/geometry/mesh')
        if mesh_node is None:continue
        mesh=trimesh.load_mesh(urdf.parent/mesh_node.get('filename'),process=False)
        vertices=np.unique(np.array(mesh.vertices),axis=0)
        vertices=vertices[::max(1,len(vertices)//1000)]
        origin=link.find('visual/origin')
        if origin is not None:
            rot=Rotation.from_euler('xyz',[float(x) for x in origin.get('rpy','0 0 0').split()]).as_matrix()
            vertices=vertices@rot.T+np.array([float(x) for x in origin.get('xyz','0 0 0').split()])
        pose=kin.data.oMf[kin.model.getFrameId(name)]
        samples.append(vertices@pose.rotation.T+pose.translation)
    model=np.concatenate(samples)
    points=color_points(np.load(cap/'depth-points.npy'),info)
    uv=project(points,info['color_intrinsics']);plane=np.array(scene['table_plane_camera'])
    mask=(uv[:,0]>0)&(uv[:,0]<340)&(uv[:,1]>550)&(uv[:,1]<1080)&(np.abs(points@plane[:3]+plane[3])>.008)
    observed=points[mask][::2]
    if len(observed)<100:raise ValueError('Not enough isolated hand depth points')
    T=np.array(geometry['T_base_camera_optical'])
    nominal=(model-T[:3,3])@T[:3,:3]
    tree=cKDTree(nominal)
    # A candidate may only correct a small nominal mounting error. Metrics
    # cannot establish landmark identity or independent holdout validation.
    def moved(x):return observed@Rotation.from_rotvec(x[:3]).as_matrix().T+x[3:]
    def residual(x):return np.minimum(tree.query(moved(x))[0],.08)
    fit=least_squares(residual,np.zeros(6),bounds=(np.r_[[-.25]*3,[-.08]*3],np.r_[[.25]*3,[.08]*3]),loss='soft_l1',f_scale=.008,max_nfev=60)
    distance=tree.query(moved(fit.x))[0]
    report={'verified':False,'method':'passive hand CAD nearest-surface candidate',
        'observed_points':len(observed),'median_surface_error_m':float(np.median(distance)),
        'p90_surface_error_m':float(np.percentile(distance,90)),
        'candidate_rotation_vector':fit.x[:3].tolist(),'candidate_translation_m':fit.x[3:].tolist(),
        'blocked_reasons':['surface matching is not independent landmark validation','grasp tool offset remains unmeasured'],
        'finger_angle_mapping':'normalized linear approximation; not measured joint calibration'}
    (cap/'hand-registration.json').write_text(json.dumps(report,indent=2))
    return report
if __name__=='__main__':print(json.dumps(register(Path(__file__).resolve().parents[1]),indent=2))
