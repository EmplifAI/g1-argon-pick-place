"""RGB-D tabletop reconstruction; never certifies an unmeasured transform."""
import json
import time
from pathlib import Path
import numpy as np

def color_points(points, info):
    ex=info['depth_to_color']
    R=np.array(ex['rotation_column_major']).reshape(3,3,order='F')
    valid=(points[:,2]>.1)&(points[:,2]<2)
    return points[valid]@R.T+np.array(ex['translation_m'])

def project(points, intr):
    return np.column_stack([intr['fx']*points[:,0]/points[:,2]+intr['ppx'],
                            intr['fy']*points[:,1]/points[:,2]+intr['ppy']])

def fit_plane(points, trials=160, tolerance=.004):
    if len(points)<100:raise ValueError('Insufficient tabletop depth')
    rng=np.random.default_rng(7);best=None;best_count=0
    for _ in range(trials):
        a,b,c=points[rng.choice(len(points),3,replace=False)]
        n=np.cross(b-a,c-a);norm=np.linalg.norm(n)
        if norm<1e-8:continue
        n/=norm;d=-n@a
        mask=np.abs(points@n+d)<tolerance
        count=int(mask.sum())
        if count>best_count:best_count=count;best=mask
    if best is None or best_count/len(points)<.75:raise ValueError('No reliable table plane')
    p=points[best];center=p.mean(axis=0);_,_,vt=np.linalg.svd(p-center,full_matrices=False)
    n=vt[-1];d=float(-n@center)
    residual=float(np.sqrt(np.mean((p@n+d)**2)))
    return np.r_[n,d],{'inlier_fraction':best_count/len(points),'rmse_m':residual}

def reconstruct(root):
    root=Path(root);cap=root/'captures'
    info=json.loads((cap/'depth-info.json').read_text())
    geometry=json.loads((cap/'geometry.json').read_text())
    cloud=color_points(np.load(cap/'depth-points.npy'),info)
    uv=project(cloud,info['color_intrinsics'])
    # Clear portion of this tabletop. The ROI is recorded, not a hidden
    # privileged object pose. It must be reselected for another setup.
    roi=(uv[:,0]>850)&(uv[:,0]<1300)&(uv[:,1]>450)&(uv[:,1]<950)
    plane,quality=fit_plane(cloud[roi][::3])
    T=np.array(geometry['T_base_camera_optical'])
    verified=False;method='URDF nominal only'
    calibration=root/'config/calibration.json'
    if calibration.exists():
        c=json.loads(calibration.read_text())
        if c.get('verified') and c.get('holdout_rmse_m',1) <= .01 and c.get('tool_offset_verified'):
            T=np.array(c['T_base_camera_optical']);verified=True;method=c['method']
    if T.shape!=(4,4) or not np.isfinite(T).all():raise ValueError('Invalid camera transform')
    nbase=T[:3,:3]@plane[:3];dbase=float(plane[3]-nbase@T[:3,3])
    if nbase[2]<0:nbase=-nbase;dbase=-dbase
    # Non-table points include robot/object points. Cartesian planners must
    # explicitly segment those before deriving a free-space obstacle cloud.
    scene={'captured_at_pc_s':time.time(),'calibration_verified':verified,'calibration_method':method,
       'T_base_camera_optical':T.tolist(),'table_plane_camera':plane.tolist(),
       'table_plane_base':np.r_[nbase,dbase].tolist(),'table_quality':quality,
       'color_intrinsics':info['color_intrinsics'],'depth_scale_m':info['depth_scale_m'],
       'scene_segmentation_verified':False,'obstacle_points_base':[],
       'blocked_reasons':[] if verified else ['camera/tool transform needs empirical holdout verification']}
    np.save(cap/'scene-points-color.npy',cloud)
    (cap/'scene-targets.json').write_text(json.dumps(scene,indent=2))
    return scene

def locate_pixel(root, pixel):
    root=Path(root);s=json.loads((root/'captures/scene-targets.json').read_text())
    p=np.load(root/'captures/scene-points-color.npy');uv=project(p,s['color_intrinsics'])
    distance=np.linalg.norm(uv-np.array(pixel),axis=1);near=p[distance<6]
    if len(near)<5:raise ValueError('No reliable depth near target pixel')
    point=np.median(near,axis=0);T=np.array(s['T_base_camera_optical'])
    return {'pixel':pixel,'camera_xyz_m':point.tolist(),
       'base_xyz_m':(T[:3,:3]@point+T[:3,3]).tolist(),'samples':len(near),
       'calibration_verified':s['calibration_verified']}

if __name__=='__main__':
    result=reconstruct(Path(__file__).resolve().parents[1])
    print(json.dumps(result,indent=2))
