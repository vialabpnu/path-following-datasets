import os
import math
import numpy as np
import time
import logging

from shapely.geometry import LineString
from Helper import PathSaver, PathUtils, env
from CurvesGenerator import cubic_spline, quintic_polynomial, quartic_polynomial


class MotionPlanner():
    def __init__(self):
        self.generated_path = None
        self.generated_path_length = None
        self.current_pose = None
        self.goal_pose = None
        self.DT = 0.1  # Time Step
        self.TARGET_SPEED = 4.4 / 3.6
        self.ACCEL_SCALE = 1.5
        self.MAX_SPEED = self.TARGET_SPEED + self.DT * self.ACCEL_SCALE
        self.MAX_ACCEL = 1.5  # m/s^2
        self.MAX_CURVATURE = 0.2  # 1/m
        self.MAX_STEER = 0.444  # rad
        self.SPEED_THRES_STOP = self.TARGET_SPEED + self.DT * self.TARGET_SPEED

        # Lattice Planner Parameters
        self.K_SIZE = 0.9
        self.RF = 2.68 * self.K_SIZE  # distance from rear to vehicle front end of vehicle
        self.RB = 1.06 * self.K_SIZE  # distance from rear to vehicle back end of vehicle
        self.W = 1.2 * self.K_SIZE  # width of vehicle
        self.WD = 1.1 * self.W  # distance between left-right wheels
        self.WB = 2.48 * self.K_SIZE  # Wheel base
        self.TR = 0.5 * self.K_SIZE  # Tyre radius
        self.TW = 1 * self.K_SIZE  # Tyre width
        self.ROAD_WIDTH = 1.2
        self.ROAD_SAMPLE_STEP = 0.3
        self.obs = []

    def __str__(self):
        return "Helper class for generating a trajectory for the vehicle to follow"

    def set_goal(self, goal_x, goal_y):
        self.goal_pose = [goal_x, goal_y]


class LatticePlanner(MotionPlanner):
    def __init__(self):
        MotionPlanner.__init__(self)
        self.K_JERK = 0.1
        self.K_TIME = 0.1
        self.K_V_DIFF = 1.0
        self.K_OFFSET = 1.5
        self.K_COLLISION = 500
        self.MAX_T = 9.6
        self.MIN_T = 9.0
        self.T_STEP = self.DT
        self.first_time = False
        self.DEBUG = False
        self.prev_head = None
        self.ref_x_from_path = None
        self.ref_y_from_path = None
        self.ref_yaw_from_path = None
        self.ref_curv_linear = None
        self.ref_path_linear = None
        self.ref_path_max_dist = None
        self.use_linear_spline = True

        self.logger = logging.getLogger(__name__)
        self.logger.setLevel(logging.INFO)
        self.logger.addHandler(logging.StreamHandler())

        self.path_utils = PathUtils.PathUtils()

    def sampling_paths_for_cruising(self, l0, l0_v, l0_a, s0, s0_v, s0_a, ref_path, cur_speed=None, prev_head=None):
        if self.first_time:
            l0 = 0.0
            s0 = 0.0
            s0_v = 0.0
            s0_a = 0.0
            l0_v = 0.0
            l0_a = 0.0
            self.first_time = False
        PATHS = dict()
        for s1_v in np.arange(self.TARGET_SPEED * 0.4, self.TARGET_SPEED * 1.0, self.TARGET_SPEED * 0.1):
            for t1 in np.arange(self.MIN_T, self.MAX_T, self.T_STEP):
                path_lon = quartic_polynomial.QuarticPolynomial(s0, s0_v, s0_a, s1_v, 0.0, t1)
                for l1 in [0.0]:
                    path = PathSaver.PathSaver()
                    path.t = list(np.arange(0.0, t1, self.DT))
                    path_lat = quintic_polynomial.QuinticPolynomial(l0, l0_v, l0_a, l1, 0.0, 0.0, t1)
                    for t in path.t:
                        path.s.append(path_lon.calc_xt(t))
                        path.s_v.append(path_lon.calc_dxt(t))
                        path.s_a.append(path_lon.calc_ddxt(t))
                        path.s_jerk.append(path_lon.calc_dddxt(t))
                        path.l.append(path_lat.calc_xt(t))
                        path.l_v.append(path_lat.calc_dxt(t))
                        path.l_a.append(path_lat.calc_ddxt(t))
                        path.l_jerk.append(path_lat.calc_dddxt(t))
                    path.s = self.distance_non_decreasing(path.s)
                    if not self.use_linear_spline:
                        path.x, path.y = self.SL_2_XY(path.s, path.l, ref_path)
                    else:
                        path.x, path.y = self.SL2XY_using_linear_spline(path.s, path.l)
                    path.yaw, path.curv, path.ds = self.path_utils.calc_yaw_curv(path.x, path.y, path.s, self.ref_path_max_dist, self.ref_x_from_path, self.ref_y_from_path, self.ref_yaw_from_path, current_speed=cur_speed)

                    l_jerk_sum = sum(np.abs(path.l_jerk))
                    s_jerk_sum = sum(np.abs(path.s_jerk))
                    v_diff = abs(self.TARGET_SPEED - path.s_v[-1])

                    path.cost = self.K_JERK * (l_jerk_sum + s_jerk_sum) + \
                                self.K_V_DIFF * v_diff + \
                                self.K_OFFSET * abs(path.l[-1]) + \
                                self.K_TIME * t1 * 2

                    PATHS[path] = path.cost
        return PATHS

    def sampling_paths_for_stopping(self, l0, l0_v, l0_a, s0, s0_v, s0_a, ref_path, goal_in_frenet, cur_speed=None, prev_head=None):
        PATHS = dict()
        for s1_v in np.arange(self.TARGET_SPEED * 0.0, self.TARGET_SPEED * 0.2, self.TARGET_SPEED * 0.1):
            for t1 in np.arange(self.MIN_T, self.MAX_T, self.T_STEP * 2):
                path_lon = quintic_polynomial.QuinticPolynomial(s0, s0_v, s0_a, goal_in_frenet, s1_v, 0.0, t1)
                for l1 in [0.0]:
                    path = PathSaver.PathSaver()
                    path.t = list(np.arange(0.0, t1, self.DT))
                    path_lat = quintic_polynomial.QuinticPolynomial(l0, l0_v, l0_a, l1, 0.0, 0.0, t1)
                    for t in path.t:
                        path.s.append(path_lon.calc_xt(t))
                        path.s_v.append(path_lon.calc_dxt(t))
                        path.s_a.append(path_lon.calc_ddxt(t))
                        path.s_jerk.append(path_lon.calc_dddxt(t))
                        path.l.append(path_lat.calc_xt(t))
                        path.l_v.append(path_lat.calc_dxt(t))
                        path.l_a.append(path_lat.calc_ddxt(t))
                        path.l_jerk.append(path_lat.calc_dddxt(t))
                    path.s = self.distance_non_decreasing(path.s)
                    if not self.use_linear_spline:
                        path.x, path.y = self.SL_2_XY(path.s, path.l, ref_path)
                    else:
                        path.x, path.y = self.SL2XY_using_linear_spline(path.s, path.l)
                    path.yaw, path.curv, path.ds = self.path_utils.calc_yaw_curv(path.x, path.y, path.s, self.ref_path_max_dist, self.ref_x_from_path, self.ref_y_from_path, self.ref_yaw_from_path, current_speed=cur_speed)

                    if path.yaw is None:
                        continue

                    l_jerk_sum = sum(np.abs(path.l_jerk))
                    s_jerk_sum = sum(np.abs(path.s_jerk))
                    v_diff = (path.s_v[-1]) ** 2

                    path.cost = self.K_JERK * (l_jerk_sum + s_jerk_sum) + \
                                self.K_V_DIFF * v_diff + \
                                self.K_TIME * t1 * 2 + \
                                self.K_OFFSET * abs(path.l[-1]) + \
                                50.0 * sum(np.abs(path.s_v))

                    PATHS[path] = path.cost

        return PATHS

    def lattice_planner_for_cruising(self, l0, l0_v, l0_a, s0, s0_v, s0_a, ref_path, cur_speed=None, ref_path_without_spline=None):
        times = time.time()
        paths = self.sampling_paths_for_cruising(l0, l0_v, l0_a, s0, s0_v, s0_a, ref_path, cur_speed)
        if self.DEBUG:
            print("Time to compute: ", time.time() - times)
        try:
            path = self.path_utils.extract_optimal_path(paths, self.MAX_SPEED, self.MAX_ACCEL, self.MAX_CURVATURE)
        except Exception as e:
            print("Error: ", e)
            return None

        if not self.path_utils.is_path_increasing_in_distance(path.s):
            self.logger.info("Path is not increasing in distance: %s", path.s)
        return path

    def lattice_planner_for_stopping(self, l0, l0_v, l0_a, s0, s0_v, s0_a, ref_path, goal_in_frenet, cur_speed=None, ref_path_without_spline=None):
        paths = self.sampling_paths_for_stopping(l0, l0_v, l0_a, s0, s0_v, s0_a, ref_path, goal_in_frenet, cur_speed)
        try:
            path = self.path_utils.extract_optimal_path(paths, self.MAX_SPEED, self.MAX_ACCEL, self.MAX_CURVATURE)
        except Exception as e:
            print("Error: ", e)
            return None

        if not self.path_utils.is_path_increasing_in_distance(path.s):
            self.logger.info("Path is not increasing in distance: %s", path.s)
        return path

    def get_reference_line(self, x, y):
        index = range(0, len(x), 3)
        x = [x[i] for i in index]
        y = [y[i] for i in index]

        cubicspline = cubic_spline.Spline2D(x, y)
        distance_step = 0.05
        s = np.arange(0, cubicspline.s[-1], distance_step)
        rx, ry, ryaw, rk = [], [], [], []
        for i_s in s:
            ix, iy = cubicspline.calc_position(i_s)
            rx.append(ix)
            ry.append(iy)
            ryaw.append(cubicspline.calc_yaw(i_s))
        rk = self.path_utils.calculate_curvature_using_circle(rx, ry)
        if len(rk) != len(rx):
            rk.append(rk[-1])
            if len(rk) != len(rx):
                print("Length of Curvature and X are not the same")
                print("Length of Curvature: ", len(rk))
                print("Length of X: ", len(rx))
        return rx, ry, ryaw, rk, cubicspline

    def set_the_maximum_distance_cubic_spline(self, ref_path):
        self.ref_path_max_dist = ref_path.s[-1]
        print("Reference Path Max Distance: ", self.ref_path_max_dist)

    def create_line_string_from_waypoints(self, waypoints):
        self.ref_x_from_path, self.ref_y_from_path, self.ref_yaw_from_path = waypoints[0], waypoints[1], waypoints[2]
        self.ref_path_linear = LineString((self.ref_x_from_path[i], self.ref_y_from_path[i]) for i in range(len(self.ref_x_from_path)))
        self.ref_path_max_dist = self.ref_path_linear.length
        print("Reference Path Max Distance: ", self.ref_path_max_dist)

    def SL2XY_using_linear_spline(self, s, l):
        ref_x, ref_y = [], []
        for i in range(len(s)):
            ref_points = list(self.ref_path_linear.interpolate(s[i]).coords)
            x, y = ref_points[0][0], ref_points[0][1]
            ref_x.append(x)
            ref_y.append(y)
        return ref_x, ref_y

    @staticmethod
    def SL_2_XY(s_set, l_set, ref_path):
        pathx, pathy = [], []
        for i in range(len(s_set)):
            x_ref, y_ref = ref_path.calc_position(s_set[i])
            if x_ref is None:
                break
            pathx.append(x_ref)
            pathy.append(y_ref)
        return pathx, pathy

    @staticmethod
    def pi_2_pi(theta):
        if theta > math.pi:
            return theta - 2.0 * math.pi

        if theta < -math.pi:
            return theta + 2.0 * math.pi

        return theta

    @staticmethod
    def distance_non_decreasing(s):
        for i in range(1, len(s)):
            if s[i] < s[i - 1]:
                s[i] = s[i - 1]
        return s

if "__main__" == __name__:
    planner = LatticePlanner()
    ENV = env.ENVStopping()
    wx, wy = ENV.ref_line

    planner.ROAD_WIDTH = ENV.road_width
    rx, ry, ryaw, rk, ref_path = planner.get_reference_line(wx, wy)
    times = time.time()
    path = planner.lattice_planner_for_cruising(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, ref_path)
    print("Time: ", time.time() - times)
    