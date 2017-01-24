#!/usr/bin/env python

import sys, os
import unittest
sys.path.append('../')
from mvbb.box_db import MVBBLoader
import numpy as np

class MVBBLoaderTest(unittest.TestCase):
    def setUp(self):
        os.remove('data/test_db_simulated.csv')

    def test_save_simulation(self):
        db = MVBBLoader('data/test_db', 19, 15)
        k = db.db.keys()[0]
        p = db.db[k][0]
        self.assertFalse(db.has_simulation(k,p))
        nan = float('nan')
        db.save_simulation(k, p, np.eye(4), [nan]*19, [nan]*15, [nan]*15)
        self.assertTrue(db.has_simulation(k, p))

        print "Reloading..."
        db2 = MVBBLoader('data/test_db', 19, 15)
        self.assertTrue(db2.has_simulation(k, p))

if __name__ == '__main__':
    unittest.main()