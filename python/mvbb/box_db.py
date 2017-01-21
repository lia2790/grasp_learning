#!/usr/bin/env python

import csv
from klampt.math import se3, so3
import numpy as np

class MVBBLoader(object):
    def __init__(self, filename_no_ext, n_dofs, n_l):
        self.filename = '%s.csv'%filename_no_ext
        self.filename_simulated = '%s_simulated.csv' % filename_no_ext
        self.db = {}
        self.db_simulated = {}
        self._load_mvbbs()
        self._load_mvbbs_simulated()

        self.n_dofs = n_dofs
        self.n_l = n_l

    def _load_mvbbs(self):
        try:
            f = open(self.filename)
            self.db = {}
            reader = csv.reader(f)
            for row in reader:
                object_dims = tuple([float(v) for i, v in enumerate(row) if i in range(3)])
                t = [float(v) for i, v in enumerate(row) if i in range(3, 6)]
                q = [float(row[9])] + [float(v) for i, v in enumerate(row) if i in range(6, 9)]
                print q

                T = (so3.from_quaternion(q),t)
                if tuple(object_dims) not in self.db:
                    self.db[object_dims] = []
                self.db[object_dims].append(np.array(se3.homogeneous(T)))

        except:
            print "Error loading file", self.filename

    def _load_mvbbs_simulated(self):
        try:
            f = open(self.filename_simulated)
            reader = csv.reader(f)
            self.db_simulated = {}

            for row in reader:
                object_dims = tuple([float(v) for i, v in enumerate(row) if i in range(3)])
                t = [float(v) for i, v in enumerate(row) if i in range(3, 6)]
                q = [float(v) for i, v in enumerate(row) if i in range(6, 10)]

                T = (so3.from_quaternion(q), t)

                q_hand = [float(v) for i, v in enumerate(row) if i in range(10, 10 + n_dofs)]
                c_p = [float(v) for i, v in enumerate(row) if i in range(10 + n_dofs, 10 + n_dofs + n_l)]
                c_f = [float(v) for i, v in enumerate(row) if i in range(10 + n_dofs + n_l, 10 + n_dofs + 2 * n_l)]

                if tuple(object_dims) not in self.db_simulated:
                    self.db_simulated[tuple(object_dims)] = []
                obj_pose_q_c_p_c_f = {'T' :  T,
                                      'q':   q_hand,
                                      'c_p': c_p,
                                      'c_f': c_f}
                self.db_simulated[tuple(object_dims)].append(obj_pose_q_c_p_c_f)
        except:
            print "Error loading file", self.filename_simulated

    def save_simulation(self, box_dims, pose, h_T_o, q, c_p, c_f):
        if not self.has_simulation(object_name, pose):
            f = open(self.filename_simulated, 'a')
            values = list(box_dims)
            if isinstance(pose, np.ndarray):
                pose = se3.from_homogeneous(pose)
            # saving pose.t
            values += pose[1]
            # saving pose.q
            q = np.array(so3.quaternion(pose[0]))
            value += list(q[1:4])
            values.append(q[0])

            if isinstance(h_T_o, np.ndarray):
                h_T_o = se3.from_homogeneous(h_T_o)
            # saving h_T_o.t
            values += h_T_o[1]
            # saving h_T_o.q
            q = np.array(so3.quaternion(h_T_o[0]))
            value += list(q[1:4])
            values.append(q[0])

            # saving q
            values += list(q)
            values += list(c_p)
            values += list(c_f)
            f.write(','.join([str(v) for v in values]))
            f.write('\n')
            f.close()
        self._load_mvbbs_simulated()

    def has_simulation(self, box_dims, pose):
        self._load_mvbbs_simulated()
        box_dims = tuple(box_dims)
        if box_dims in self.db_simulated:
            if not isinstance(pose, np.ndarray):
                pose = np.array(se3.homogeneous(pose))
            poses = [p['T'] for p in self.db_simulated[box_dims]]
            for p in poses:
                if np.all((pose - p) < 1e-12):
                    return True
        return False

    def get_poses(self, box_dims):
        if self.db == {}:
            self._load_mvbbs()
        if tuple(box_dims) in self.db:
            return self.db[object_name]
        return []

    def get_simulated_poses(self, box_dims, only_successful = True):
        if self.db_simulated == {}:
            self._load_mvbbs_simulated()
        if tuple(box_dims) in self.db_simulated:
            return [ pose['T'] for pose in self.db_simulated[object_name]]
        return []

    def get_all_simulated_poses(self):
        if self.db_simulated == {}:
            self._load_mvbbs_simulated()
        obj_poses = {}
        for box_dims in self.db_simulated:
            poses = [ pose['T'] for pose in self.db_simulated[box_dims]]
            if len(poses) > 0:
                obj_poses[object_name] = poses
        return obj_poses