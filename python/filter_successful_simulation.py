#!/usr/bin/env python

import pkg_resources
pkg_resources.require("klampt>=0.7.0")
import os, sys, math
from mvbb.box_db import MVBBLoader
import traceback
from shutil import copyfile, move


if __name__ == '__main__':
    box_db = None
    try:
        filename = os.path.splitext(sys.argv[1])[0]
    except:
        filename = 'box_db'

    if not os.path.isfile(filename+'.csv'):
        print "Error: file", filename, "doesn't exist"
        exit()

    try:
        n_dofs = int(sys.argv[2])
        n_l = int(sys.argv[3])
    except:
        n_dofs = 19
        n_l = 16

    box_db = MVBBLoader(filename, n_dofs, n_l)
    fout = filename + '_filtered'
    copyfile(filename+'.csv', fout+'.csv')
    box_db_filtered = MVBBLoader(fout, n_dofs, n_l)

    poses = box_db.db_simulated
    suc_pose_count = 0
    pose_count = 0
    for k in poses.keys():
        pose_count += len(poses[k])
        for pose in poses[k]:
            if not math.isnan(pose['q'][0]):
                box_db_filtered.save_simulation(k, pose['T'], pose['h_T_o'], pose['q'], pose['c_p'], pose['c_f'])
                suc_pose_count+=1
    print "Found", suc_pose_count, "successful grasps out of", pose_count, "grasps  "
    move(fout+'_simulated.csv', fout+'.csv')
