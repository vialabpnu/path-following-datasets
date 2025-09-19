import math
import numpy as np


class PathUtils(object):
    '''This class contains the methods for path planning and path generation including spline fitting, frenet path generation, and path cost calculation.'''
    def __init__(self):
        self.current_path_idx = 0
        self.speed_threshold_yaw = 0.1 # m/s
        self.closest_distance = 1000000.0
        self.closest_idx = 0
    
    def extract_optimal_path(self, paths, MAX_SPEED, MAX_ACCEL, MAX_CURVATURE):
        if len(paths) == 0:
            return None

        while len(paths) > 1:
            path = min(paths, key=paths.get)
            paths.pop(path)
            if self.verify_path(path, MAX_SPEED=MAX_SPEED, MAX_ACCEL=MAX_ACCEL, MAX_CURVATURE=MAX_CURVATURE) is False:
                continue
            else:
                return path
        last_path = paths.popitem()[0]
        if self.verify_path(last_path, MAX_SPEED=MAX_SPEED, MAX_ACCEL=MAX_ACCEL, MAX_CURVATURE=MAX_CURVATURE) is False:
            return None
        return last_path
        # return paths.popitem()[0]
        #return paths[-1]
        
    def calculate_curvature_using_circle(self,x,y):
        curvatures = []
        x = x
        y = y
        for i in range(1, len(x) - 1):
            x1 = x[i - 1]
            y1 = y[i - 1]
            x2 = x[i]
            y2 = y[i]
            x3 = x[i + 1]
            y3 = y[i + 1]
            R = self.find_circle(x1, y1, x2, y2, x3, y3)
            if R is None:
                curvatures.append(0)
            else:
                curvatures.append(1.0 / R)
        curvatures.append(0)
        # Round to the 2nd decimal place
        # curvatures = np.round(curvatures, 2).tolist()
        curvatures = [math.floor(c * 100) / 100 for c in curvatures]
        return curvatures
    
    @staticmethod
    def find_circle(x1, y1, x2, y2, x3, y3):
        # Calculate A, B, C, D
        A = x1*(y2 - y3) - y1*(x2 - x3) + x2*y3 - x3*y2
        B = (x1**2 + y1**2)*(y3 - y2) + (x2**2 + y2**2)*(y1 - y3) + (x3**2 + y3**2)*(y2 - y1)
        C = (x1**2 + y1**2)*(x2 - x3) + (x2**2 + y2**2)*(x3 - x1) + (x3**2 + y3**2)*(x1 - x2)
        D = (x1**2 + y1**2)*(x3*y2 - x2*y3) + (x2**2 + y2**2)*(x1*y3 - x3*y1) + (x3**2 + y3**2)*(x2*y1 - x1*y2)
        
        # Check if points are collinear (A should not be zero)
        if A == 0:
            return None
        
        radius_limit = 1.5
        
        # Calculate radius R
        R = math.sqrt(np.abs(B**2 + C**2 - 4*A*D) / (4*A**2))
        
        if R < radius_limit:
            return None
        return R  
    
    def calc_yaw_curv(self, x, y, s, s_max, ref_x, ref_y, ref_yaw, current_speed):
        yaw, curv, ds = [], [], []

        current_speed = float(current_speed)
        # print("Current Speed: {}".format(current_speed))
        # print("Flag Distance Above Maximum: {}".format(s[0] >= s_max))
        # print("Flag Below Speed Threshold: {}".format(np.abs(current_speed) < self.speed_threshold_yaw))
        # First yaw in reference
        first_yaw_in_reference = ref_yaw[0]
        # Last yaw in reference
        last_yaw_in_reference = ref_yaw[-1]
        if s[0] >= s_max:
            # Set the yaw to the last yaw in reference
            for i in range(len(x)):
                yaw.append(last_yaw_in_reference)
            # Set the curvature to zero
            for i in range(len(x)):
                curv.append(0.0)
            # Set the distance to zero
            for i in range(len(x)):
                ds.append(0.0)
            return yaw, curv, ds
        elif np.abs(current_speed) < self.speed_threshold_yaw:
            # print("Speed is less than threshold")
            # Get the closest point to the reference path
            ref_x_temp, ref_y_temp = [], []
            self.closest_distance = 1000000.0
            # for i in range(len(x)):
            #     distance = math.hypot(x[i] - ref_x[0], y[i] - ref_y[0])
            #     if distance < self.closest_distance:
            #         self.closest_distance = distance
            #         # If the closest index is less than the current index, set the closest index to the current index
            #         if self.closest_idx < i:
            #             self.closest_idx = i
            # Check if the closest index is large or equal to the previous closest index
            self.closest_idx = self.get_closest_index(x[0], y[0], ref_x, ref_y)
            if self.closest_idx >= self.current_path_idx:
                # Set the current path index to the closest index
                self.current_path_idx = self.closest_idx
            # Get the yaw from the reference patH and from the current closest index get the yaw for the rest of the path
            # Based on the distance from the current closest index get the yaw from the reference path
            # Calculate distance change
            ds = np.diff(s)
            ds = np.insert(ds, 0, 0.0)
            # Distance on the x and y reference path from the current closest index
            dx_path = np.diff(ref_x)
            dy_path = np.diff(ref_y)
            # Distance on the x and y path
            distances = np.hypot(dx_path, dy_path)
            # Calculate the distance change on the reference path
            s_ref = np.cumsum(distances)
            # Append 0.0 to the beginning of the distance change
            s_ref = np.insert(s_ref, 0, 0.0)
            # print("Current Path Idx: {}".format(self.current_path_idx))
            # Get the closest index based on the distance change
            closest_indices = self.get_closest_idx_s(s, s_ref, len(s)-1)
            # print("First Yaw in Reference: {}".format(first_yaw_in_reference))
            # print("Closest Indices: {}".format(closest_indices))
            # print("Length of Closest Indices: {}".format(len(closest_indices)))
            # print("Distance From Current Path: {}".format(s_ref))
            # print("Distance From Traj Path: {}".format(s))
            # Get the yaw from the reference path
            # print("Length of Distance Array: {}".format(len(s)))
            # print("Length of Reference X: {}".format(len(x)))
            for i in range(len(s)):
                if i == 0:
                    yaw.append(ref_yaw[self.current_path_idx])
                    ref_x_temp.append(ref_x[self.current_path_idx])
                    ref_y_temp.append(ref_y[self.current_path_idx])
                else:
                    yaw.append(ref_yaw[closest_indices[i]])
                    ref_x_temp.append(ref_x[closest_indices[i]])
                    ref_y_temp.append(ref_y[closest_indices[i]])
                    
            # Get the curvature from the reference path
            # curv = self.calculate_curvature_using_circle(ref_x_temp, ref_y_temp)
            curv = self.calculate_curvature_using_derivative(ref_x_temp, ref_y_temp)
        else:   
            for i in range(len(x) - 1):
                dx = x[i + 1] - x[i]
                dy = y[i + 1] - y[i]
                ds.append(math.hypot(dx, dy))
                if dx == 0.0:
                    if dy > 0:
                        yaw.append(math.pi / 2)
                    elif dy < 0:
                        yaw.append(-math.pi / 2)
                    else:
                        yaw.append(np.arctan2(dy, dx))
                else:
                    yaw.append(np.arctan2(dy, dx))
            
            if len(yaw) == 0:
                return None, None, None
            
            yaw.append(yaw[-1])
            ds.append(ds[-1])
            
            # If the distance is around 0 set the yaw to the first yaw in reference (not distance change)
            threshold_yaw_distance = 0.1
            for i in range(len(x)):
                if s[i] < threshold_yaw_distance:
                    yaw[i] = first_yaw_in_reference

            # Old curvature calculation
            # for i in range(len(yaw) - 1):
            #     # Avoid division by zero
            #     if ds[i] == 0.0:
            #         curv.append(0.0)
            #     else:
            #         curv.append((yaw[i + 1] - yaw[i]) / ds[i])
            # curv.append(curv[-1])
            
            # New curvature calculation
            curv = self.calculate_curvature_using_circle(x, y)
            if len(curv) != len(yaw):
                # Append the last curvature value to make the length equal to yaw
                curv.append(curv[-1])
        return yaw, curv, ds
    
    def get_closest_index(self, x, y, ref_x, ref_y):
        closest_idx = 0
        # Convert the reference path to numpy array
        ref_x = np.array(ref_x)
        ref_y = np.array(ref_y)
        distances = np.sqrt((ref_x - x)**2 + (ref_y - y)**2)
        closest_idx = np.argmin(distances)
        # IF the closest index is less than the current index, set the closest index to the current index
        if closest_idx < self.current_path_idx:
            closest_idx = self.current_path_idx
        return closest_idx
    
    def get_closest_idx_s(self, s, s_ref, total_idx):
        # Calcualte absolute difference between the distance change of the path and the reference path
        diff = np.abs(np.expand_dims(s, axis=1) - s_ref)
        # Get the index of the minimum difference
        closest_indices = np.argmin(diff, axis=1)
        # If any of the closest indices is less than the current closest index, set the closest index to the current closest index
        for i in range(len(closest_indices)):
            if closest_indices[i] < self.closest_idx:
                closest_indices[i] = self.closest_idx
        return closest_indices
    
    @staticmethod   
    def unwrap_angle(angle):
        return math.atan2(math.sin(angle), math.cos(angle))

    @staticmethod
    def calculate_curvature_using_derivative(x, y):
        # Calculate the first derivatives
        dx = np.gradient(x)
        dy = np.gradient(y)

        # Calculate the second derivatives
        ddx = np.gradient(dx)
        ddy = np.gradient(dy)

        # Calculate the curvature
        curvature = np.zeros_like(x)
        threshold = 1e-4  # Define a small threshold value
        nonzero = (np.abs(dx) > threshold) & (np.abs(dy) > threshold)
        curvature[nonzero] = np.abs(dx[nonzero] * ddy[nonzero] - dy[nonzero] * ddx[nonzero]) / (dx[nonzero]**2 + dy[nonzero]**2)**1.5
        # Floor to two decimal places
        curvature = [math.floor(c * 100) / 100 for c in curvature]
        return curvature
    
    @staticmethod
    def calculate_curvature(heading_angles, distances):
        # Calculate the differences between the heading angles
        d_heading_angles = np.diff(heading_angles)

        # Calculate the differences between the distances
        ds = np.diff(distances)

        # Add a small constant to the denominator to prevent division by zero
        ds += 1e-10

        # Calculate the curvature
        curvature = d_heading_angles / ds

        return curvature
    
    @staticmethod
    def is_path_collision(path, RF, RB, W, obs):
        index = range(0, len(path.x), 5)
        x = [path.x[i] for i in index]
        y = [path.y[i] for i in index]
        yaw = [path.yaw[i] for i in index]

        for ix, iy, iyaw in zip(x, y, yaw):
            d = 1.8
            dl = (RF - RB) / 2.0
            r = math.hypot((RF + RB) / 2.0, W / 2.0) + d

            cx = ix + dl * math.cos(iyaw)
            cy = iy + dl * math.sin(iyaw)

            for i in range(len(obs)):
                xo = obs[i][0] - cx
                yo = obs[i][1] - cy
                dx = xo * math.cos(iyaw) + yo * math.sin(iyaw)
                dy = -xo * math.sin(iyaw) + yo * math.cos(iyaw)

                if abs(dx) < r and abs(dy) < W / 2 + d:
                    return 1.0

        return 0.0

    @staticmethod
    def verify_path(path, MAX_SPEED, MAX_ACCEL, MAX_CURVATURE):
        # print("PATH VELOCITY {}".format(path.s_v))
        # print("PATH ACCEL {}".format(path.s_a))
        # if any([abs(v) > MAX_SPEED for v in path.s_v]) or any([abs(a) > MAX_ACCEL for a in path.s_a]): 
        #     return False
        return True
