#!/usr/bin/env python

import csv
from klampt.math import se3
import numpy as np
import pydany_bb

object_geom_file_patterns = {
	'ycb':['data/objects/ycb/%s/meshes/tsdf_mesh.stl','data/objects/ycb/%s/meshes/poisson_mesh.stl'],
	'apc2015':['data/objects/apc2015/%s/textured_meshes/optimized_tsdf_textured_mesh.ply']
}

class MVBBFilter(object):
    def __init__(self, hand_name):
        self.hand_name = hand_name

    def filter_poses(self, object_name):
        if self.db == {}:
            self._load_mvbbs()
        if object_name in self.db:
            return self.db[object_name]
        return []