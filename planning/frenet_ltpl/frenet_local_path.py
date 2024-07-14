#!/usr/bin/env python3

import configparser
import copy
import math
import numpy as np


import frenet_ltpl 


class FrenetLocalPath:
    def __init__(self):
        self.set_configs()
    
    def set_configs(self):
        config_file_path = './params/config.ini'
        config = configparser.ConfigParser()
        config.read(config_file_path)
        fot_config = config['FrenetOptimalTrajectory']
        self.max_speed = float(fot_config['max_speed'])/3.6
        self.max_accel = float(fot_config['max_accel'])
        self.max_curvature = float(fot_config['max_curvature']) 
        self.max_road_width = float(fot_config['max_road_width'])
        self.d_road_w = float(fot_config['d_road_w']) 
        self.dt = float(fot_config['dt']) 
        self.max_t = float(fot_config['max_t']) 
        self.min_t = float(fot_config['min_t'])
        self.target_speed = float(fot_config['target_speed'])/3.6
        self.d_t_s = float(fot_config['d_t_s'])/3.6 
        self.n_s_sample = float(fot_config['n_s_sample']) 
        self.robot_radius = float(fot_config['robot_radius']) 
        cost_config = config['Cost']
        self.k_j = float(cost_config['k_j'])
        self.k_t = float(cost_config['k_t'])
        self.k_d = float(cost_config['k_d'])
        self.k_lat = float(cost_config['k_lat'])
        self.k_lon = float(cost_config['k_lon'])
    
    def calc_frenet_paths(self, c_speed, c_accel, c_d, c_d_d, c_d_dd, s0, road_widths, max_speed):
        frenet_paths = []
        # generate path to each offset goal
        for di in np.arange(-road_widths['w_left'], road_widths['w_right'], self.d_road_w):

            # Lateral motion planning
            for Ti in np.arange(self.min_t, self.max_t, self.dt):
                fp = frenet_ltpl.frenet_path.FrenetPath()

                lat_qp = frenet_ltpl.quintic_polynomials_planner.QuinticPolynomial(c_d, c_d_d, c_d_dd, di, 0.0, 0.0, Ti)
                fp.t = [t for t in np.arange(0.0, Ti, self.dt)]
                fp.d = [lat_qp.calc_point(t) for t in fp.t]
                fp.d_d = [lat_qp.calc_first_derivative(t) for t in fp.t]
                fp.d_dd = [lat_qp.calc_second_derivative(t) for t in fp.t]
                fp.d_ddd = [lat_qp.calc_third_derivative(t) for t in fp.t]

                # Longitudinal motion planning (Velocity keeping)
                for tv in np.arange(self.max_speed - self.d_t_s * self.n_s_sample,
                                    self.max_speed + self.d_t_s * self.n_s_sample, self.d_t_s):
                    tfp = copy.deepcopy(fp)
                    lon_qp = frenet_ltpl.quartic_polynomial.QuarticPolynomial(s0, c_speed, c_accel, tv, 0.0, Ti)

                    tfp.s = [lon_qp.calc_point(t) for t in fp.t]
                    tfp.s_d = [lon_qp.calc_first_derivative(t) for t in fp.t]
                    tfp.s_dd = [lon_qp.calc_second_derivative(t) for t in fp.t]
                    tfp.s_ddd = [lon_qp.calc_third_derivative(t) for t in fp.t]
                    
                    Jp = sum(np.power(tfp.d_ddd, 2))  # square of jerk
                    Js = sum(np.power(tfp.s_ddd, 2))  # square of jerk

                    # square of diff from target speed
                    ds = (self.max_speed - tfp.s_d[-1]) ** 2

                    tfp.cd = self.k_j * Jp + self.k_t * Ti + self.k_d * tfp.d[-1] ** 2
                    tfp.cv = self.k_j * Js + self.k_t * Ti + self.k_d * ds
                    tfp.cf = self.k_lat * tfp.cd + self.k_lon * tfp.cv

                    frenet_paths.append(tfp)
        return frenet_paths


    def calc_global_paths(self, fplist, csp):
        # transform trajectory from frenet to global
        for fp in fplist:
            # calc global positions
            for i in range(len(fp.s)):
                ix, iy = csp.calc_position(fp.s[i])
                if ix is None:
                    break
                i_yaw = csp.calc_yaw(fp.s[i])
                di = fp.d[i]
                fx = ix + di * math.cos(i_yaw + math.pi / 2.0)
                fy = iy + di * math.sin(i_yaw + math.pi / 2.0)
                fp.x.append(fx)
                fp.y.append(fy)

            # calc yaw and ds
            for i in range(len(fp.x) - 1):
                dx = fp.x[i + 1] - fp.x[i]
                dy = fp.y[i + 1] - fp.y[i]
                fp.yaw.append(math.atan2(dy, dx))
                fp.ds.append(math.hypot(dx, dy))

            fp.yaw.append(fp.yaw[-1])
            fp.ds.append(fp.ds[-1])

            # calc curvature
            for i in range(len(fp.yaw) - 1):
                fp.c.append((fp.yaw[i + 1] - fp.yaw[i]) / fp.ds[i])

        return fplist


    def check_collision(self, fp, ob):
        if ob.size == 0:
            return True
        for i in range(len(ob[:, 0])):
            d = [((ix - ob[i, 0]) ** 2 + (iy - ob[i, 1]) ** 2)
                for (ix, iy) in zip(fp.x, fp.y)]

            collision = any([di <= self.robot_radius ** 2 for di in d])

            if collision:
                return False

        return True


    def check_paths(self, fplist, ob):
        ok_ind = []
        for i, _ in enumerate(fplist):
            if any([v > self.max_speed for v in fplist[i].s_d]):  # Max speed check
                continue
            elif any([abs(a) > self.max_accel for a in fplist[i].s_dd]):  # Max accel check
                continue
            elif any([abs(c) > self.max_accel for c in fplist[i].c]):  # Max curvature check
                continue
            elif not self.check_collision(fplist[i], ob):
                continue

            ok_ind.append(i)

        return [fplist[i] for i in ok_ind]


    def frenet_optimal_planning(self, csp, s0, c_speed, c_accel, c_d, c_d_d, c_d_dd, ob, road_widths, max_speed):
        fplist = self.calc_frenet_paths(c_speed, c_accel, c_d, c_d_d, c_d_dd, s0, road_widths, max_speed)
        fplist = self.calc_global_paths(fplist, csp)
        fplist = self.check_paths(fplist, ob)
        # find minimum cost path
        min_cost = float("inf")
        best_path = None
        for fp in fplist:
            if min_cost >= fp.cf:
                min_cost = fp.cf
                best_path = fp
        return best_path


    def generate_target_course(self, x, y, k):
        csp = frenet_ltpl.cubic_spline_planner.CubicSpline2D(x, y, k)
        s = np.arange(0, csp.s[-1], 0.1)

        rx, ry, ryaw, rk = [], [], [], []
        for i_s in s:
            ix, iy = csp.calc_position(i_s)
            rx.append(ix)
            ry.append(iy)
            ryaw.append(csp.calc_yaw(i_s))
            rk.append(csp.calc_curvature(i_s))

        return rx, ry, ryaw, rk, csp