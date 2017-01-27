#!/usr/bin/env python

import sys, os, glob
import unittest
abspath = os.path.dirname(os.path.abspath(__file__))
sys.path.append(abspath+'/../')
from mvbb.box_db import MVBBLoader
import numpy as np

class MVBBLoaderTest(unittest.TestCase):
    def setUp(self):
        for fname in glob.glob(abspath+"/data/test_db_*.csv"):
            os.remove(fname)

    def tearDown(self):
        for fname in glob.glob(abspath+"/data/test_db_*.csv"):
            os.remove(fname)

    def test_save_simulation(self):
        db = MVBBLoader(abspath+'/data/test_db', 19, 15)
        k = db.db.keys()[0]
        p = db.db[k][0]
        self.assertFalse(db.has_simulation(k,p))
        nan = float('nan')
        db.save_simulation(k, p, np.eye(4), [nan]*19, [nan]*15, [nan]*15)
        self.assertTrue(db.has_simulation(k, p))

        print "Reloading..."
        db2 = MVBBLoader(abspath+'/data/test_db', 19, 15)
        self.assertTrue(db2.has_simulation(k, p))

    def test_split_join(self):
        db = MVBBLoader(abspath+'/data/test_db', 19, 15)
        filenames = db.split_db()
        self.assertEqual(len(filenames), 6)
        for fname in filenames:
            db_i = MVBBLoader(fname, 19, 15)
            for k, ps in db_i.db.items():
                for p in ps:
                    nan = float('nan')
                    db_i.save_simulation(k, p, np.eye(4), [nan] * 19, [nan] * 15, [nan] * 15)
        db.join_results(filenames)
        poses = db.get_all_simulated_poses()
        pose_count = 0
        for i in poses.keys():
            pose_count+=len(poses[i])
        self.assertEqual(6,len(db.get_all_simulated_poses()))
        self.assertEqual(12,pose_count)


if __name__ == '__main__':
    unittest.main()