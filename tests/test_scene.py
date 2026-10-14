import sys
import unittest
from pathlib import Path
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'src'))
from scene_reconstruction import fit_plane,project
class SceneTests(unittest.TestCase):
    def test_plane_recovers_table_with_outliers(self):
        rng=np.random.default_rng(3);xy=rng.uniform(-.4,.4,(1500,2));z=.7+rng.normal(0,.001,1500)
        p=np.column_stack([xy,z]);p[:100,2]+=.15
        plane,q=fit_plane(p)
        self.assertGreater(q['inlier_fraction'],.9);self.assertLess(q['rmse_m'],.002)
        self.assertLess(abs(abs(plane[3])-.7),.003)
    def test_projection_uses_real_intrinsics(self):
        uv=project(np.array([[0,0,1],[.1,.1,1]]),{'fx':100,'fy':100,'ppx':50,'ppy':60})
        np.testing.assert_allclose(uv,[[50,60],[60,70]])
if __name__=='__main__':unittest.main()
