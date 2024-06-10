
import libs.gp_utils as gput
import copy

KPH_TO_MPS = 1 / 3.6
MPS_TO_KPH = 3.6

class LocalPathTest:
    def __init__(self, ros_handler, map):
        self.RH = ros_handler
        self.MAP = map
        self.setting_values()
    
    def setting_values(self):
        gput.lanelets = self.MAP.lanelets
        gput.tiles = self.MAP.tiles
        gput.tile_size = self.MAP.tile_size

        self.precision = 1
        self.local_path = None
        self.temp_signal = 0
        self.x_p = 3
        self.x_c = 50
        self.x2_i = 30
        self.x2_v_th = 9
        self.x3_c = 8

    def need_update(self, local_pos):
        if self.local_path == None:
            return 0, -1

        idx = gput.find_nearest_idx(self.local_path, local_pos)
        if self.temp_signal != self.RH.current_signal and self.RH.current_signal != 0:
            self.temp_signal = self.RH.current_signal
            return 2, idx
        threshold = (self.RH.current_velocity*MPS_TO_KPH)*2.5
        if len(self.local_path)-idx <= threshold:
            return 1, idx
        else:
            return -1, idx

    def get_change_path(self, sni,  path_len, to=1):
        wps, uni = gput.get_straight_path( sni, path_len)
        c_pt = wps[-1]
        l_id, r_id = gput.get_neighbor(uni[0])
        n_id = l_id if to==1 else r_id

        if n_id != None:
            r = self.MAP.lanelets[n_id]['waypoints']
            u_n = n_id
            u_i = gput.find_nearest_idx(r, c_pt)
            uni = [u_n, u_i]
        else:
            r = wps
        return r, uni
    
    def make_path(self, update_type, local_pos):
        r0 = []
        if update_type == 0 or update_type == 2 or update_type == 3:
            start_pose = local_pos
        else:
            start_pose = self.local_path[-1]
            idx = gput.find_nearest_idx(self.local_path, local_pos)
            r0 = self.local_path[idx:]        
       
        ego_ni = gput.lanelet_matching(start_pose)
        if ego_ni == None:
            return None
        
        if self.RH.current_velocity < 0.5:
            x1 = 50
        else:
            x1 = 30 if self.RH.current_velocity < self.x2_v_th else self.RH.current_velocity * MPS_TO_KPH

        r1, ni1 = gput.get_straight_path(ego_ni, x1)

        if self.RH.current_signal == 0 or self.RH.current_signal == 4 or update_type == 3 or update_type == 1:
            local_path = r0+r1
        else:
            x2 = self.x2_i if self.RH.current_velocity < self.x2_v_th else self.RH.current_velocity * self.x_p
            _, ni2 = self.get_change_path(ni1, x2, self.RH.current_signal)
            x3 = self.x3_c + self.RH.current_velocity * self.x_p
            r3, _ = gput.get_straight_path(ni2, x3)
            local_path = r0+r1+r3

        return local_path

    def calc_kappa(self, idx):
        copy_local_path = copy.deepcopy(self.local_path)
        ratio = self.RH.current_velocity/22.0 if self.RH.current_velocity > 0 else 0.1
        start = int(idx+30*ratio)
        end = start+60 if len(copy_local_path[start:])>60 else len(copy_local_path)-1
        if start < len(copy_local_path):
            copy_local_path = copy_local_path[start:end]
        else:
            pass
        ccopy_local_path = copy.deepcopy(copy_local_path)
        #self.RH.publish_kappa_viz(copy_local_path)
        copy_local_path.insert(0, copy_local_path[0])
        copy_local_path.append(copy_local_path[-1])
        local_kappa = []
        for i, f in enumerate(ccopy_local_path):
            before_after_pts = [copy_local_path[i], copy_local_path[i+2]]
            Rk = gput.calc_kappa(f, before_after_pts)
            local_kappa.append(Rk)
        return local_kappa

    def get_velocity(self, max_vel):
        if self.RH.system_mode < 1:
            return 0
        ev = self.RH.current_velocity * MPS_TO_KPH
        acceleration = (0.5 * (max_vel - ev))
        out_vel = min(ev+acceleration, max_vel)
        return out_vel*KPH_TO_MPS

    def execute(self, local_pos):
        if local_pos == None:
            return None
        local_path = []
        need_update, idx = self.need_update(local_pos)
        if need_update != -1:
            local_path = self.make_path(need_update, local_pos)
            if local_path == None or len(local_path) <= 0:
                return
            self.local_path = gput.smooth_interpolate(local_path, self.precision)

        self.local_kappa = self.calc_kappa(idx)
        return self.local_path, self.local_kappa
