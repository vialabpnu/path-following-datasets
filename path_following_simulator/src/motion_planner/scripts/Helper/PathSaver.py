
import numpy as np
# Description: This file contains the class PathSaver, which is used to save the path information

class PathSaver(object):
    def __init__(self):
        self.x = []
        self.y = []
        self.yaw = []
        self.ds = []
        self.curv = []
        self.s = []
        self.s_v = []
        self.s_a = []
        self.s_jerk = []
        self.l = []
        self.l_v = []
        self.l_a = []
        self.l_jerk = []
        self.t = []
        self.cost = 0.0
        
    def extract_curvature_based_on_distance(self, distance_list=[2.5, 5.0, 7.5]):
        curv = []
        dist_cumulative = np.cumsum(self.ds)
        # Extract the curvature based on the distance (s)
        for d in distance_list:
            # Check if the distance is within the path length
            if d <= dist_cumulative[-1]:
                idx = np.where(dist_cumulative >= d)[0][0]
                curv.append(self.curv[idx])
            else:
                curv.append(self.curv[-1])
        return curv        
