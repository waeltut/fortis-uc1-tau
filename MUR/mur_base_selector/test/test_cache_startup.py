import tempfile
from pathlib import Path
import unittest
import numpy as np
from mur_base_selector.cache_startup import CacheStartupError,load_selector_volume
from mur_base_selector.volume import save_volume


class StartupTests(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory();self.addCleanup(self.tmp.cleanup)
        self.path=Path(self.tmp.name)/'volume.npz'

    def write(self,complete):
        cfg={'tool_axis':[1,0,0],'min_inward_angle_deg':0,'workspace_fov_deg':250,'workspace_front_offset_deg':0,'inward_reference':'horizontal'}
        save_volume(self.path,{'schema':2,'reachability_policy':'self_body_ignore_other_arm_v1','generation':cfg},np.array([[.6,0,.8]]),np.array([[0,0,0,1.]]),np.ones((2,1,1),bool),np.full((2,1),complete))

    def test_missing_cache_clean_exit_and_exact_path(self):
        with self.assertRaises(CacheStartupError) as e:load_selector_volume(self.path)
        self.assertEqual(e.exception.exit_code,0);self.assertIn(str(self.path),str(e.exception));self.assertIn('build_volume',str(e.exception));self.assertFalse(self.path.exists())

    def test_partial_cache_suggests_resume_without_modification(self):
        self.write(False);before=self.path.read_bytes()
        with self.assertRaises(CacheStartupError) as e:load_selector_volume(self.path)
        self.assertEqual(e.exception.exit_code,0);self.assertIn('--resume',str(e.exception));self.assertEqual(before,self.path.read_bytes())

    def test_corrupt_file_reports_nonzero_without_modification(self):
        self.path.write_bytes(b'broken')
        with self.assertRaises(CacheStartupError) as e:load_selector_volume(self.path)
        self.assertEqual(e.exception.exit_code,1);self.assertEqual(self.path.read_bytes(),b'broken')

    def test_complete_cache_loads(self):
        self.write(True);self.assertTrue(load_selector_volume(self.path).done.all())


if __name__=='__main__':unittest.main()

class PolicyTests(unittest.TestCase):
    setUp=StartupTests.setUp
    write=StartupTests.write

    def test_old_policy_rejected_for_selection_and_resume(self):
        from mur_base_selector.volume import Volume
        import json
        self.write(True)
        with np.load(self.path,allow_pickle=False) as f:data={k:f[k] for k in f.files}
        meta=json.loads(str(data['metadata']));meta['schema']=1;meta.pop('reachability_policy')
        data['metadata']=np.array(json.dumps(meta));np.savez_compressed(self.path,**data)
        for complete in (True,False):
            with self.assertRaisesRegex(ValueError,'Incompatible cache policy'):Volume(self.path,require_complete=complete)
        with self.assertRaises(CacheStartupError) as e:load_selector_volume(self.path)
        self.assertEqual(e.exception.exit_code,1)

    def test_collision_free_policy_is_rejected(self):
        import json
        from mur_base_selector.volume import Volume
        self.write(True)
        with np.load(self.path,allow_pickle=False) as f:data={k:f[k] for k in f.files}
        meta=json.loads(str(data['metadata']));meta['reachability_policy']='kinematic_only_v1'
        data['metadata']=np.array(json.dumps(meta));np.savez_compressed(self.path,**data)
        for complete in (True,False):
            with self.assertRaisesRegex(ValueError,'Incompatible cache policy'):Volume(self.path,require_complete=complete)
